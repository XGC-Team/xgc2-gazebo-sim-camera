// XGC Gazebo Classic camera source.
//
// The normal frame path performs one GPU encode:
//   OGRE render texture -> OpenGL RGBA texture -> NVENC H264 Annex-B AU.
// Each AU is fanned out to loopback RTP and a bounded asynchronous ROS
// publisher queue. ROS serialization and transport never run in the render
// callback. Explicit calibration snapshots use an asynchronous OpenGL PBO and
// a depth-one JPEG worker, with the former synchronous RGB readback retained
// only as the compatibility fallback.

#include "fresh_render_gate.h"
#include "snapshot_jpeg_backend.h"
#include "camera_source_control.h"

#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/Distortion.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/sensors/CameraSensor.hh>

#include <OgreHardwarePixelBuffer.h>
#include <OgrePixelFormat.h>
#include <OgreRenderTarget.h>
#include <OgreTexture.h>
#include <RenderSystems/GL/OgreGLTexture.h>

#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES 1
#endif
#include <GL/gl.h>
#include <GL/glext.h>

#include <ffnvcodec/nvEncodeAPI.h>

#include <foxglove_msgs/CompressedVideo.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/CameraInfo.h>
#include <xgc_camera_msgs/FrameTiming.h>
#include <xgc_camera_msgs/StreamInfo.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <deque>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace gazebo {
namespace {

// A single data callback owner for all cameras in one Gazebo process. Keeping
// this queue separate permits native worlds to publish ROS user data without
// any gazebo_ros_api_plugin management services.
class CameraROSDataRuntime {
 public:
  static std::shared_ptr<CameraROSDataRuntime> Acquire() {
    static std::mutex mutex;
    static std::weak_ptr<CameraROSDataRuntime> owner;
    std::lock_guard<std::mutex> lock(mutex);
    auto result = owner.lock();
    if (!result) { result = std::shared_ptr<CameraROSDataRuntime>(new CameraROSDataRuntime); owner = result; }
    return result;
  }
  ros::CallbackQueue queue;
 private:
  CameraROSDataRuntime() {
    if (!ros::isInitialized()) {
      int argc = 0; char **argv = nullptr;
      ros::init(argc, argv, "xgc_native_camera_data", ros::init_options::NoSigintHandler);
    }
    spinner_ = std::make_unique<ros::AsyncSpinner>(1, &queue);
    spinner_->start();
  }
  std::unique_ptr<ros::AsyncSpinner> spinner_;
};

constexpr std::size_t kMaximumRtpPayloadBytes = 1'200;
constexpr std::size_t kMinimumPacedQueueBytes = 256 * 1024;
constexpr const char *kSourceTimestampClockDomain = "simulation";
constexpr std::uint8_t kH264PayloadType = 96;
constexpr std::uint32_t kRtpClockRate = 90'000;
constexpr std::uint32_t kRtpSSRC = 0x58474332;  // "XGC2"
constexpr std::size_t kDefaultROSPublisherQueueCapacity = 8;
constexpr std::size_t kMaximumPendingEncodedFrames = 4;
constexpr std::size_t kMaximumPendingDiagnostics = 64;

using NvEncodeAPICreateInstance = NVENCSTATUS (NVENCAPI *)(NV_ENCODE_API_FUNCTION_LIST *);

template <typename Value>
Value SDFValue(const sdf::ElementPtr &sdf, const std::string &name, Value fallback) {
  if (sdf && sdf->HasElement(name)) {
    return sdf->Get<Value>(name);
  }
  return fallback;
}

bool IsSafeIdentifier(const std::string &value) {
  if (value.empty() || value.size() > 128) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return std::isalnum(character) || character == '-' || character == '_' || character == '.';
  });
}

std::vector<std::vector<std::uint8_t>> SplitAnnexB(const std::uint8_t *data, std::size_t size) {
  auto startCodeLength = [data, size](std::size_t offset) -> std::size_t {
    if (offset + 3 <= size && data[offset] == 0 && data[offset + 1] == 0 && data[offset + 2] == 1) {
      return 3;
    }
    if (offset + 4 <= size && data[offset] == 0 && data[offset + 1] == 0 && data[offset + 2] == 0 && data[offset + 3] == 1) {
      return 4;
    }
    return 0;
  };

  std::vector<std::vector<std::uint8_t>> nalUnits;
  std::size_t start = 0;
  while (start < size && startCodeLength(start) == 0) {
    ++start;
  }
  while (start < size) {
    const std::size_t prefix = startCodeLength(start);
    if (prefix == 0) {
      break;
    }
    const std::size_t payloadStart = start + prefix;
    std::size_t next = payloadStart;
    while (next < size && startCodeLength(next) == 0) {
      ++next;
    }
    if (next > payloadStart) {
      nalUnits.emplace_back(data + payloadStart, data + next);
    }
    start = next;
  }
  return nalUnits;
}

}  // namespace

class XGCMediaCameraPlugin final : public SensorPlugin, private Ogre::RenderTargetListener {
 public:
  XGCMediaCameraPlugin() = default;

  ~XGCMediaCameraPlugin() override {
    stopping_.store(true);
    postRenderConnection_.reset();
    // CameraSensor::Fini tears down its rendering::Camera and OGRE render
    // target before Sensor::Fini releases sensor plugins. renderTarget_ is a
    // non-owning cache, so it is already dangling when this destructor runs
    // and must not be dereferenced here. Destruction of the target also
    // destroys its listener registry.
    renderTarget_ = nullptr;
    StopControlServer();
    StopSnapshotEncoder();
    if (captureSlotHeld_ && controlHost_) {
      controlHost_->ReleaseCapture();
      captureSlotHeld_ = false;
    }
    StopROSPublisher();
    StopRTPPacer();
    if (rtpSocket_ >= 0) {
      close(rtpSocket_);
      rtpSocket_ = -1;
    }
    StopDiagnosticReporter();
    // GL/NVENC resources are intentionally released from OnPostRender while a
    // valid Gazebo render context is current. Process exit owns any final
    // teardown that occurs after Gazebo has removed that context.
  }

  void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override {
    sensor_ = std::dynamic_pointer_cast<sensors::CameraSensor>(parent);
    if (!sensor_) {
      gzerr << "xgc_media_camera requires a Gazebo CameraSensor\n";
      return;
    }

    sourceID_ = SDFValue<std::string>(sdf, "sourceId", "usb_cam");
    frameID_ = SDFValue<std::string>(sdf, "frameId", "usb_cam_optical_frame");
    snapshotPoseFrameID_ =
        SDFValue<std::string>(sdf, "snapshotPoseFrameId", "world");
    rtpHost_ = SDFValue<std::string>(sdf, "rtpHost", "127.0.0.1");
    rtpPort_ = SDFValue<int>(sdf, "rtpPort", 5004);
    controlSocketPath_ = SDFValue<std::string>(sdf, "controlEndpoint", "");
    controlTargetID_ = SDFValue<std::string>(sdf, "controlTargetId", "");
    controlInstanceID_ = SDFValue<std::string>(sdf, "controlInstanceId", "");
    bitrate_ = SDFValue<int>(sdf, "bitrate", 6'000'000);
    maxBitrate_ = SDFValue<int>(sdf, "maxBitrate", bitrate_ + bitrate_ / 2);
    pacingBitrate_ = SDFValue<int>(sdf, "pacingBitrate", maxBitrate_);
    vbvBufferMilliseconds_ = std::clamp(SDFValue<int>(sdf, "vbvBufferMilliseconds", 500), 50, 2'000);
    jpegQuality_.store(SDFValue<int>(sdf, "jpegQuality", 90));
    const std::string snapshotJpegPolicyValue =
        SDFValue<std::string>(sdf, "snapshotJpegBackend", "auto");
    const auto snapshotJpegPolicy =
        gazebo_sim_camera::ParseSnapshotJpegPolicy(snapshotJpegPolicyValue);
    if (!snapshotJpegPolicy) {
      gzerr << "xgc_media_camera snapshotJpegBackend must be auto, hardware, or cpu\n";
      return;
    }
    snapshotJpegPolicy_ = *snapshotJpegPolicy;
    // The portable plugin intentionally has no CUDA dependency. Optional GPU
    // JPEG support is a versioned C-ABI module; only a successfully created
    // encoder counts as hardware preflight. NVENC itself cannot encode JPEG.
    const bool hardwareAvailable =
        snapshotJpegPolicy_ != gazebo_sim_camera::SnapshotJpegPolicy::kCPU &&
        snapshotJpegHardware_.Open();
    snapshotJpegHardwarePreflightError_ = snapshotJpegHardware_.error();
    snapshotJpegBackend_ = gazebo_sim_camera::SelectSnapshotJpegBackend(
        snapshotJpegPolicy_, hardwareAvailable,
        hardwareAvailable ? snapshotJpegHardware_.backend() : "nvjpeg-cuda");
    if (!snapshotJpegBackend_.available) {
      gzerr << "xgc_media_camera " << snapshotJpegBackend_.error;
      if (!snapshotJpegHardwarePreflightError_.empty()) {
        gzerr << ": " << snapshotJpegHardwarePreflightError_;
      }
      gzerr << "\n";
      return;
    }
    rosPublishingEnabled_ = SDFValue<bool>(sdf, "rosPublishEnabled", true);
    rosCameraInfoTopic_ = SDFValue<std::string>(sdf, "rosCameraInfoTopic", "/xgc/camera/world/camera_info");
    rosVideoTopic_ = SDFValue<std::string>(
        sdf, "rosVideoTopic", "/xgc/camera/world/video_h264");
    rosFrameTimingTopic_ = SDFValue<std::string>(
        sdf, "rosFrameTimingTopic", "/xgc/camera/world/frame_timing");
    rosStreamInfoTopic_ = SDFValue<std::string>(
        sdf, "rosStreamInfoTopic", "/xgc/camera/world/stream_info");
    rosPublisherQueueCapacity_ = static_cast<std::size_t>(std::clamp(
        SDFValue<int>(
            sdf,
            "rosPublisherQueueCapacity",
            static_cast<int>(kDefaultROSPublisherQueueCapacity)),
        1,
        128));

    if (!IsSafeIdentifier(sourceID_) || !IsSafeIdentifier(controlTargetID_) || controlSocketPath_.empty() || frameID_.empty() ||
        snapshotPoseFrameID_.empty() || jpegQuality_.load() < 50 || jpegQuality_.load() > 100 ||
        sensor_->ImageWidth() > 4096 || sensor_->ImageHeight() > 2160 ||
        sensor_->ImageWidth() < 16 || sensor_->ImageHeight() < 16 || rtpPort_ < 1 || rtpPort_ > 65535 ||
        (rtpHost_ != "127.0.0.1" && rtpHost_ != "localhost") || bitrate_ < 128'000 ||
        (rosPublishingEnabled_ &&
         (rosVideoTopic_.empty() || rosFrameTimingTopic_.empty() ||
          rosStreamInfoTopic_.empty()))) {
      gzerr << "xgc_media_camera has invalid private media source configuration\n";
      return;
    }
    if (maxBitrate_ < bitrate_ || pacingBitrate_ < maxBitrate_) {
      gzerr << "xgc_media_camera requires bitrate <= maxBitrate <= pacingBitrate\n";
      return;
    }

    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(static_cast<std::uint16_t>(rtpPort_));
    if (inet_pton(AF_INET, "127.0.0.1", &destination.sin_addr) != 1) {
      gzerr << "xgc_media_camera could not resolve loopback RTP target\n";
      return;
    }
    rtpSocket_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (rtpSocket_ < 0) {
      gzerr << "xgc_media_camera could not create RTP socket: " << std::strerror(errno) << "\n";
      return;
    }
    rtpDestination_ = destination;

    if (!StartRTPPacer()) {
      close(rtpSocket_);
      rtpSocket_ = -1;
      return;
    }

    if (!StartSnapshotEncoder()) {
      StopRTPPacer();
      close(rtpSocket_);
      rtpSocket_ = -1;
      return;
    }

    if (!StartDiagnosticReporter()) {
      StopControlServer();
      StopSnapshotEncoder();
      StopRTPPacer();
      close(rtpSocket_);
      rtpSocket_ = -1;
      return;
    }

    streamWidth_ = sensor_->ImageWidth();
    streamHeight_ = sensor_->ImageHeight();
    streamFrameRate_ = std::max(0.1, static_cast<double>(sensor_->UpdateRate()));
    streamKeyframeIntervalFrames_ = std::max<std::uint32_t>(
        1, static_cast<std::uint32_t>(std::llround(streamFrameRate_ * 2.0)));
    streamEpoch_ = InitialEpochToken();
    pendingDiscontinuity_ =
        xgc_camera_msgs::FrameTiming::DISCONTINUITY_STREAM_START;
    rosWaitingForIDR_ = true;
    const auto nativeProjection = sensor_->Camera()->ProjectionMatrix();
    calibrationMetadata_.width = streamWidth_;
    calibrationMetadata_.height = streamHeight_;
    calibrationMetadata_.matrix = {nativeProjection(0,0) * streamWidth_ / 2., nativeProjection(0,1) * streamWidth_ / 2., (1. - nativeProjection(0,2)) * streamWidth_ / 2.,
        0., nativeProjection(1,1) * streamHeight_ / 2., (1. + nativeProjection(1,2)) * streamHeight_ / 2., 0., 0., 1.};
    if (const auto distortion = sensor_->Camera()->LensDistortion()) calibrationMetadata_.distortion = {distortion->K1(), distortion->K2(), distortion->P1(), distortion->P2(), distortion->K3()};
    calibrationMetadata_.distortionCount = 5;
    if (!StartROSPublisher()) {
      gzwarn << "xgc_media_camera will continue RTP service without ROS encoded-video publication\n";
    }

    // Render only on demand. The global post-render callback remains available
    // to turn a camera back on for a new WebRTC consumer or one snapshot.
    sensor_->SetActive(false);
    postRenderConnection_ = event::Events::ConnectPostRender(
        std::bind(&XGCMediaCameraPlugin::OnPostRender, this));
    if (!StartControlServer()) {
      StopSnapshotEncoder();
      StopRTPPacer();
      close(rtpSocket_);
      rtpSocket_ = -1;
      return;
    }
    std::ostringstream startupMessage;
    startupMessage << "xgc_media_camera source " << sourceID_
                   << " serves H264/RTP through 127.0.0.1:" << rtpPort_
                   << " and " << snapshotJpegBackend_.backend
                   << " snapshots (policy "
                   << gazebo_sim_camera::SnapshotJpegPolicyName(snapshotJpegPolicy_)
                   << ")";
    if (rosPublishingEnabled_) {
      startupMessage << " and H264/Annex-B on " << rosVideoTopic_;
    }
    gzmsg << startupMessage.str() << "\n";
  }

 private:
  struct NativeControlCommand;
  struct NativeCalibrationMetadata {
    std::uint64_t revision = 1;
    unsigned int width = 0, height = 0;
    int model = 0;
    std::size_t distortionCount = 0;
    std::array<double, 9> matrix{};
    std::array<double, 8> distortion{};
  };
  struct SnapshotResult {
    bool completed = false;
    bool failed = false;
    bool captureSubmitted = false;
    bool includeRGB = true;
    int jpegQuality = 90;
    std::string error;
    std::string id;
    std::uint64_t generation = 0;
    std::uint64_t frameSequence = 0;
    bool calibrationValid = false;
    std::int64_t timestampNanoseconds = 0;
    unsigned int width = 0;
    unsigned int height = 0;
    std::array<double, 9> cameraMatrix{};
    std::array<double, 5> distortion{};
    std::array<double, 3> renderPosition{};
    std::array<double, 4> renderOrientation{};
    std::string poseFrameID;
    bool renderPoseValid = false;
    std::string jpegBackend;
    std::string jpegReadback;
    std::string jpegFallbackReason;
    double jpegReadbackMilliseconds = 0.0;
    double jpegEncodeMilliseconds = 0.0;
    std::vector<std::uint8_t> rgb;
    std::vector<std::uint8_t> jpeg;
  };

  struct QueuedRTPPacket {
    std::vector<std::uint8_t> bytes;
  };

  struct QueuedRTPAccessUnit {
    std::vector<QueuedRTPPacket> packets;
    std::size_t bytes = 0;
    std::uint64_t generation = 0;
    std::chrono::nanoseconds framePeriod{};
  };

  struct PendingEncodedFrame {
    std::uint64_t encoderTimestamp = 0;
    std::int64_t sourceTimeNanoseconds = 0;
    std::uint64_t sourceSequence = 0;
    std::uint64_t keyframeControlRevision = 0;
    std::uint64_t captureKeyframeGeneration = 0;
  };

  struct QueuedROSFrame {
    std::vector<std::uint8_t> annexB;
    std::int64_t sourceTimeNanoseconds = 0;
    std::uint64_t sourceSequence = 0;
    std::uint64_t epoch = 0;
    std::uint64_t frameSequence = 0;
    std::uint64_t generation = 0;
    std::uint32_t rtpTimestamp = 0;
    std::uint32_t droppedFramesBefore = 0;
    std::uint8_t discontinuity =
        xgc_camera_msgs::FrameTiming::DISCONTINUITY_NONE;
    bool keyframe = false;
  };

  struct StreamInfoSnapshot {
    std::uint64_t epoch = 0;
    std::uint64_t generation = 0;
    unsigned int width = 0;
    unsigned int height = 0;
    double frameRate = 0.0;
    std::uint32_t keyframeIntervalFrames = 0;
  };

  std::uint64_t InitialEpochToken() const {
    const auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    std::uint64_t token = static_cast<std::uint64_t>(now);
    token ^= static_cast<std::uint64_t>(getpid()) << 32U;
    token ^= static_cast<std::uint64_t>(
        reinterpret_cast<std::uintptr_t>(this));
    return token == 0 ? 1 : token;
  }

  std::uint64_t NextEpochToken(std::uint64_t current) const {
    ++current;
    return current == 0 ? 1 : current;
  }

  bool StartROSPublisher() {
    if (!rosPublishingEnabled_) {
      return true;
    }
    // Native worlds need the user data-plane ROS node, without the retired
    // gazebo_ros_api_plugin control bus. ROS remains a process-wide data runtime.
    try {
      rosDataRuntime_ = CameraROSDataRuntime::Acquire();
      rosNode_ = std::make_unique<ros::NodeHandle>();
      rosNode_->setCallbackQueue(&rosDataRuntime_->queue);
      rosCameraInfoPublisher_ = rosNode_->advertise<sensor_msgs::CameraInfo>(rosCameraInfoTopic_, 1, true);
      {
        std::lock_guard<std::mutex> lock(rosPublisherMutex_);
        rosPublisherStopping_ = false;
      }
      {
        std::lock_guard<std::mutex> lock(rosConsumerMutex_);
        rosSubscriberCallbacksEnabled_ = true;
        rosSubscriberConnectionCount_ = 0;
      }
      const ros::SubscriberStatusCallback connected =
          [this](const ros::SingleSubscriberPublisher &) {
            OnROSSubscriberConnected();
          };
      const ros::SubscriberStatusCallback disconnected =
          [this](const ros::SingleSubscriberPublisher &) {
            OnROSSubscriberDisconnected();
          };
      ros::AdvertiseOptions videoOptions;
      videoOptions.init<foxglove_msgs::CompressedVideo>(
          rosVideoTopic_,
          static_cast<std::uint32_t>(rosPublisherQueueCapacity_),
          connected,
          disconnected);
      rosVideoPublisher_ = rosNode_->advertise(videoOptions);
      ros::AdvertiseOptions timingOptions;
      timingOptions.init<xgc_camera_msgs::FrameTiming>(
          rosFrameTimingTopic_,
          static_cast<std::uint32_t>(rosPublisherQueueCapacity_),
          connected,
          disconnected);
      rosFrameTimingPublisher_ = rosNode_->advertise(timingOptions);
      rosStreamInfoPublisher_ =
          rosNode_->advertise<xgc_camera_msgs::StreamInfo>(
              rosStreamInfoTopic_, 1, true);
      {
        std::lock_guard<std::mutex> lock(rosPublisherMutex_);
        rosStreamInfoPending_ = true;
        rosCameraInfoPending_ = true;
      }
      rosPublisherThread_ =
          std::thread(&XGCMediaCameraPlugin::ROSPublisherLoop, this);
      rosPublisherCondition_.notify_one();
      return true;
    } catch (const std::exception &error) {
      gzwarn << "xgc_media_camera could not start its ROS publisher: "
             << error.what() << "\n";
      rosVideoPublisher_.shutdown();
      rosFrameTimingPublisher_.shutdown();
      rosStreamInfoPublisher_.shutdown();
      rosCameraInfoPublisher_.shutdown();
      rosNode_.reset();
      rosDataRuntime_.reset();
      {
        std::lock_guard<std::mutex> lock(rosConsumerMutex_);
        rosSubscriberCallbacksEnabled_ = false;
        rosSubscriberConnectionCount_ = 0;
        rosConsumersActive_.store(false);
      }
      rosPublishingEnabled_ = false;
      return false;
    }
  }

  void StopROSPublisher() {
    {
      std::lock_guard<std::mutex> lock(rosPublisherMutex_);
      rosPublisherStopping_ = true;
      rosFrameQueue_.clear();
      rosStreamInfoPending_ = false;
      ++rosPublisherGeneration_;
    }
    {
      std::lock_guard<std::mutex> lock(rosConsumerMutex_);
      rosSubscriberCallbacksEnabled_ = false;
      rosSubscriberConnectionCount_ = 0;
      rosConsumersActive_.store(false);
    }
    rosPublisherCondition_.notify_all();
    if (rosPublisherThread_.joinable()) {
      rosPublisherThread_.join();
    }
    rosVideoPublisher_.shutdown();
    rosFrameTimingPublisher_.shutdown();
    rosStreamInfoPublisher_.shutdown();
    rosCameraInfoPublisher_.shutdown();
    rosNode_.reset();
    rosDataRuntime_.reset();
  }

  void AdvanceROSEpochLocked(
      std::uint8_t discontinuity,
      std::uint64_t additionallyDroppedFrames = 0) {
    rosDroppedFramesBeforeIDR_ +=
        static_cast<std::uint64_t>(rosFrameQueue_.size()) +
        additionallyDroppedFrames;
    rosFrameQueue_.clear();
    streamEpoch_ = NextEpochToken(streamEpoch_);
    nextPublishedFrameSequence_ = 0;
    pendingDiscontinuity_ = discontinuity;
    rosWaitingForIDR_ = true;
    ++rosPublisherGeneration_;
    rosStreamInfoPending_ = true;
  }

  void BeginROSEpoch(std::uint8_t discontinuity) {
    if (!rosPublishingEnabled_) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(rosPublisherMutex_);
      if (rosPublisherStopping_) {
        return;
      }
      AdvanceROSEpochLocked(discontinuity);
    }
    forceKeyframe_.store(true);
    rosPublisherCondition_.notify_one();
  }

  bool ROSHasPendingEpoch() const {
    std::lock_guard<std::mutex> lock(rosPublisherMutex_);
    return rosWaitingForIDR_ && nextPublishedFrameSequence_ == 0;
  }

  void OnROSSubscriberConnected() {
    bool firstConnection = false;
    {
      std::lock_guard<std::mutex> lock(rosConsumerMutex_);
      if (!rosSubscriberCallbacksEnabled_) {
        return;
      }
      firstConnection = rosSubscriberConnectionCount_ == 0;
      ++rosSubscriberConnectionCount_;
      if (!firstConnection) {
        // Set this before releasing the connection lock so the render thread
        // cannot begin another delta frame after observing the new consumer.
        forceKeyframe_.store(true);
      }
    }
    if (firstConnection) {
      BeginROSEpoch(
          xgc_camera_msgs::FrameTiming::DISCONTINUITY_STREAM_START);
      if (!desiredActive_.load()) {
        // A dormant CameraSensor can first replay its pre-deactivation render
        // target. Arm the render-thread freshness gate before making ROS
        // consumption visible to OnPostRender.
        rosFreshRenderGeneration_.fetch_add(1);
      }
      std::lock_guard<std::mutex> lock(rosConsumerMutex_);
      if (rosSubscriberCallbacksEnabled_ &&
          rosSubscriberConnectionCount_ > 0) {
        // Activate only after the epoch transition requested its IDR. This
        // prevents a dormant but not-yet-destroyed encoder from publishing one
        // old-epoch P-frame during a first-connection race.
        rosConsumersActive_.store(true);
      }
      return;
    }
    // ROS1 does not replay the previous GOP to a late subscriber. Request an
    // IDR for every additional connection so it becomes independently
    // decodable without disrupting the epoch seen by existing recorders.
  }

  void OnROSSubscriberDisconnected() {
    bool lastConnection = false;
    {
      std::lock_guard<std::mutex> lock(rosConsumerMutex_);
      if (!rosSubscriberCallbacksEnabled_ ||
          rosSubscriberConnectionCount_ == 0) {
        return;
      }
      --rosSubscriberConnectionCount_;
      lastConnection = rosSubscriberConnectionCount_ == 0;
      if (lastConnection) {
        rosConsumersActive_.store(false);
      }
    }
    if (lastConnection) {
      std::lock_guard<std::mutex> lock(rosPublisherMutex_);
      if (rosPublisherStopping_) {
        return;
      }
      rosDroppedFramesBeforeIDR_ +=
          static_cast<std::uint64_t>(rosFrameQueue_.size());
      rosFrameQueue_.clear();
      ++rosPublisherGeneration_;
    }
  }

  bool EnqueueROSAccessUnit(
      const std::uint8_t *data,
      std::size_t size,
      std::uint32_t rtpTimestamp,
      bool keyframe,
      const PendingEncodedFrame &metadata) {
    if (!rosPublishingEnabled_ || !rosConsumersActive_.load() ||
        !data || size == 0) {
      return false;
    }

    QueuedROSFrame frame;
    frame.annexB.assign(data, data + size);
    frame.sourceTimeNanoseconds = metadata.sourceTimeNanoseconds;
    frame.sourceSequence = metadata.sourceSequence;
    frame.rtpTimestamp = rtpTimestamp;
    frame.keyframe = keyframe;

    {
      std::lock_guard<std::mutex> lock(rosPublisherMutex_);
      if (rosPublisherStopping_) {
        return false;
      }
      if (rosFrameQueue_.size() >= rosPublisherQueueCapacity_) {
        const std::uint64_t droppedCurrent = keyframe ? 0 : 1;
        AdvanceROSEpochLocked(
            xgc_camera_msgs::FrameTiming::DISCONTINUITY_QUEUE_OVERFLOW,
            droppedCurrent);
        if (!keyframe) {
          forceKeyframe_.store(true);
          rosPublisherCondition_.notify_one();
          return false;
        }
      }
      if (rosWaitingForIDR_ && !keyframe) {
        ++rosDroppedFramesBeforeIDR_;
        forceKeyframe_.store(true);
        return false;
      }

      frame.epoch = streamEpoch_;
      frame.frameSequence = nextPublishedFrameSequence_++;
      frame.generation = rosPublisherGeneration_;
      if (rosWaitingForIDR_) {
        frame.discontinuity = pendingDiscontinuity_;
        frame.droppedFramesBefore = static_cast<std::uint32_t>(
            std::min<std::uint64_t>(
                rosDroppedFramesBeforeIDR_,
                std::numeric_limits<std::uint32_t>::max()));
        rosDroppedFramesBeforeIDR_ = 0;
        pendingDiscontinuity_ =
            xgc_camera_msgs::FrameTiming::DISCONTINUITY_NONE;
        rosWaitingForIDR_ = false;
      }
      rosFrameQueue_.push_back(std::move(frame));
    }
    rosPublisherCondition_.notify_one();
    return true;
  }

  StreamInfoSnapshot StreamInfoLocked() const {
    StreamInfoSnapshot snapshot;
    snapshot.epoch = streamEpoch_;
    snapshot.generation = rosPublisherGeneration_;
    snapshot.width = streamWidth_;
    snapshot.height = streamHeight_;
    snapshot.frameRate = streamFrameRate_;
    snapshot.keyframeIntervalFrames = streamKeyframeIntervalFrames_;
    return snapshot;
  }

  void PublishStreamInfo(const StreamInfoSnapshot &snapshot) {
    {
      std::lock_guard<std::mutex> lock(rosPublisherMutex_);
      if (snapshot.generation != rosPublisherGeneration_) {
        return;
      }
    }
    xgc_camera_msgs::StreamInfo message;
    message.contract_version =
        xgc_camera_msgs::StreamInfo::CONTRACT_VERSION_CURRENT;
    message.stream_id = sourceID_;
    message.frame_id = frameID_;
    message.epoch = snapshot.epoch;
    message.codec = xgc_camera_msgs::StreamInfo::CODEC_H264;
    message.bitstream_format =
        xgc_camera_msgs::StreamInfo::BITSTREAM_FORMAT_ANNEX_B;
    message.clock_domain =
        xgc_camera_msgs::StreamInfo::CLOCK_DOMAIN_SIMULATION;
    message.timestamp_source =
        xgc_camera_msgs::StreamInfo::TIMESTAMP_SOURCE_SENSOR;
    message.timestamp_reference =
        xgc_camera_msgs::StreamInfo::TIMESTAMP_REFERENCE_RENDER_COMPLETE;
    message.transport_mask =
        xgc_camera_msgs::StreamInfo::TRANSPORT_ROS_COMPRESSED_VIDEO |
        xgc_camera_msgs::StreamInfo::TRANSPORT_RTP;
    message.width = snapshot.width;
    message.height = snapshot.height;
    message.nominal_frame_rate = snapshot.frameRate;
    message.rtp_clock_rate = kRtpClockRate;
    message.rtp_payload_type = kH264PayloadType;
    message.target_bitrate_bps = static_cast<std::uint32_t>(bitrate_);
    message.maximum_bitrate_bps = static_cast<std::uint32_t>(maxBitrate_);
    message.keyframe_interval_frames = snapshot.keyframeIntervalFrames;
    message.publisher_queue_capacity =
        static_cast<std::uint32_t>(rosPublisherQueueCapacity_);
    rosStreamInfoPublisher_.publish(message);
  }

  std::int64_t HostRealtimeNanoseconds() const {
    return static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
  }

  void PublishROSFrame(QueuedROSFrame frame) {
    {
      std::lock_guard<std::mutex> lock(rosPublisherMutex_);
      if (frame.generation != rosPublisherGeneration_) {
        return;
      }
    }
    const std::uint64_t sourceTimeNanoseconds = static_cast<std::uint64_t>(
        std::max<std::int64_t>(0, frame.sourceTimeNanoseconds));
    ros::Time sourceTime;
    sourceTime.fromNSec(sourceTimeNanoseconds);
    const std::uint32_t encodedSize =
        static_cast<std::uint32_t>(std::min<std::size_t>(
            frame.annexB.size(),
            std::numeric_limits<std::uint32_t>::max()));
    const std::int64_t hostPublishRealtimeNanoseconds =
        HostRealtimeNanoseconds();

    foxglove_msgs::CompressedVideo video;
    video.timestamp = sourceTime;
    video.frame_id = frameID_;
    video.data = std::move(frame.annexB);
    video.format = "h264";
    rosVideoPublisher_.publish(video);

    xgc_camera_msgs::FrameTiming timing;
    timing.source_time = sourceTime;
    timing.source_time_valid = true;
    timing.timestamp_reference =
        xgc_camera_msgs::FrameTiming::TIMESTAMP_REFERENCE_RENDER_COMPLETE;
    timing.frame_id = frameID_;
    timing.stream_id = sourceID_;
    timing.epoch = frame.epoch;
    timing.frame_sequence = frame.frameSequence;
    timing.source_sequence = frame.sourceSequence;
    timing.rtp_timestamp = frame.rtpTimestamp;
    timing.keyframe = frame.keyframe;
    timing.discontinuity = frame.discontinuity;
    timing.dropped_frames_before = frame.droppedFramesBefore;
    timing.encoded_size_bytes = encodedSize;
    timing.native_source_time_ns = frame.sourceTimeNanoseconds;
    timing.host_dequeue_monotonic_ns = 0;
    timing.host_publish_realtime_ns = hostPublishRealtimeNanoseconds;
    timing.source_to_ros_offset_ns = 0;
    timing.mapping_uncertainty_ns = 0;
    rosFrameTimingPublisher_.publish(timing);
  }

  void PublishCameraInfo(const NativeCalibrationMetadata &metadata) {
    sensor_msgs::CameraInfo message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = frameID_;
    message.width = metadata.width;
    message.height = metadata.height;
    message.distortion_model = metadata.model == 0 ? "plumb_bob" : metadata.model == 1 ? "rational_polynomial" : "equidistant";
    std::copy(metadata.matrix.begin(), metadata.matrix.end(), message.K.begin());
    message.D.assign(metadata.distortion.begin(), metadata.distortion.begin() + metadata.distortionCount);
    message.R = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    message.P = {metadata.matrix[0], metadata.matrix[1], metadata.matrix[2], 0.,
                 metadata.matrix[3], metadata.matrix[4], metadata.matrix[5], 0., 0., 0., 1., 0.};
    rosCameraInfoPublisher_.publish(message);
    publishedCalibrationRevision_.store(metadata.revision);
    WakeControl();
  }

  void ROSPublisherLoop() {
    while (true) {
      std::optional<QueuedROSFrame> frame;
      std::optional<StreamInfoSnapshot> streamInfo;
      std::optional<NativeCalibrationMetadata> calibration;
      {
        std::unique_lock<std::mutex> lock(rosPublisherMutex_);
        rosPublisherCondition_.wait(lock, [this] {
          return rosPublisherStopping_ || rosStreamInfoPending_ || rosCameraInfoPending_ ||
                 !rosFrameQueue_.empty();
        });
        if (rosPublisherStopping_) {
          return;
        }
        if (rosCameraInfoPending_) {
          calibration = calibrationMetadata_;
          rosCameraInfoPending_ = false;
        } else if (rosStreamInfoPending_) {
          streamInfo = StreamInfoLocked();
          rosStreamInfoPending_ = false;
        } else {
          frame = std::move(rosFrameQueue_.front());
          rosFrameQueue_.pop_front();
        }
      }
      if (calibration) {
        PublishCameraInfo(*calibration);
      } else if (streamInfo) {
        PublishStreamInfo(*streamInfo);
      } else if (frame) {
        PublishROSFrame(std::move(*frame));
      }
    }
  }

  // NVENC exposes an encoded access unit all at once. Sending all of its RTP
  // fragments synchronously from the render callback creates a microburst:
  // one 4K IDR frame can exceed the host UDP receive buffer before Media Edge
  // is scheduled. The pacer owns whole access units on a separate thread,
  // transmits them in bounded packet batches, and starts each AU immediately.
  // An IDR is allowed to borrow enough burst bandwidth to finish inside one
  // frame period; carrying its late completion phase into following delta
  // frames would turn normal scheduler jitter into a visible stall.
  bool StartRTPPacer() {
    try {
      rtpPacerStopping_ = false;
      rtpPacerThread_ = std::thread(&XGCMediaCameraPlugin::RTPPacerLoop, this);
      return true;
    } catch (const std::system_error &error) {
      gzerr << "xgc_media_camera could not start RTP pacer: " << error.what() << "\n";
      return false;
    }
  }

  void StopRTPPacer() {
    {
      std::lock_guard<std::mutex> lock(rtpPacerMutex_);
      rtpPacerStopping_ = true;
      rtpQueue_.clear();
      rtpQueuedBytes_ = 0;
      ++rtpQueueGeneration_;
    }
    rtpPacerCondition_.notify_all();
    if (rtpPacerThread_.joinable()) {
      rtpPacerThread_.join();
    }
  }

  void ClearRTPQueue() {
    {
      std::lock_guard<std::mutex> lock(rtpPacerMutex_);
      rtpQueue_.clear();
      rtpQueuedBytes_ = 0;
      ++rtpQueueGeneration_;
    }
    rtpPacerCondition_.notify_all();
  }

  std::size_t MaximumPacedAccessUnitBytes() const {
    // This is a defensive bound for one encoded frame, not permission to
    // accumulate seconds of media. Live viewing keeps only one pending AU.
    return std::max(kMinimumPacedQueueBytes, static_cast<std::size_t>(pacingBitrate_ / 8));
  }

  bool EnqueueRTPAccessUnit(std::vector<QueuedRTPPacket> packets, bool keyframe) {
    if (packets.empty()) {
      return false;
    }
    std::size_t bytes = 0;
    for (const auto &packet : packets) {
      bytes += packet.bytes.size();
    }
    {
      std::lock_guard<std::mutex> lock(rtpPacerMutex_);
      if (rtpPacerStopping_) {
        return false;
      }
      const std::size_t maximum = MaximumPacedAccessUnitBytes();
      if (keyframe && !rtpQueue_.empty()) {
        // An IDR is independently decodable, so it always replaces a stale
        // not-yet-sent delta frame.
        rtpQueue_.clear();
        rtpQueuedBytes_ = 0;
        ++rtpQueueGeneration_;
      }
      if (!keyframe && !rtpQueue_.empty()) {
        // P-frames form a reference chain: sending only the newer queued
        // delta would not produce a latest *decodable* frame. Discard the
        // stale pending chain and make the next capture an IDR instead.
        rtpQueue_.clear();
        rtpQueuedBytes_ = 0;
        ++rtpQueueGeneration_;
        forceKeyframe_.store(true);
        LogEncoderError("live RTP fell behind; replacing stale deltas with a fresh IDR");
        return false;
      }
      if (!keyframe && bytes > maximum) {
        forceKeyframe_.store(true);
        LogEncoderError("delta frame exceeds the RTP latency budget; requesting an IDR");
        return false;
      }
      if (keyframe && bytes > maximum) {
        LogEncoderError("IDR frame exceeds the paced RTP latency budget");
      }
      QueuedRTPAccessUnit accessUnit;
      accessUnit.packets = std::move(packets);
      accessUnit.bytes = bytes;
      accessUnit.generation = rtpQueueGeneration_;
      const double fps = std::max(0.1, static_cast<double>(sensor_->UpdateRate()));
      accessUnit.framePeriod = std::chrono::nanoseconds(std::max<std::int64_t>(
          1, static_cast<std::int64_t>(std::llround(1'000'000'000.0 / fps))));
      rtpQueuedBytes_ += bytes;
      rtpQueue_.push_back(std::move(accessUnit));
    }
    rtpPacerCondition_.notify_one();
    return true;
  }

  std::chrono::nanoseconds RTPAccessUnitSpread(const QueuedRTPAccessUnit &accessUnit) const {
    const double seconds = static_cast<double>(accessUnit.bytes) * 8.0 /
                           static_cast<double>(pacingBitrate_);
    const auto configured = std::chrono::nanoseconds(std::max<std::int64_t>(
        1, static_cast<std::int64_t>(std::ceil(seconds * 1'000'000'000.0))));
    const auto guard = std::min(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::milliseconds(4)),
        accessUnit.framePeriod / 4);
    const auto frameBudget = std::max(std::chrono::nanoseconds(1), accessUnit.framePeriod - guard);
    return std::min(configured, frameBudget);
  }

  void RTPPacerLoop() {
    while (true) {
      QueuedRTPAccessUnit accessUnit;
      std::chrono::nanoseconds spread{};
      {
        std::unique_lock<std::mutex> lock(rtpPacerMutex_);
        rtpPacerCondition_.wait(lock, [this] { return rtpPacerStopping_ || !rtpQueue_.empty(); });
        if (rtpPacerStopping_) {
          return;
        }
        accessUnit = std::move(rtpQueue_.front());
        rtpQueue_.pop_front();
        rtpQueuedBytes_ -= accessUnit.bytes;
        spread = RTPAccessUnitSpread(accessUnit);
        rtpInFlight_.store(true);
      }
      {
        std::lock_guard<std::mutex> lock(rtpPacerMutex_);
        if (rtpPacerStopping_) {
          rtpInFlight_.store(false);
          return;
        }
        if (accessUnit.generation != rtpQueueGeneration_) {
          rtpInFlight_.store(false);
          WakeControl();
          continue;
        }
      }
      const auto transmitAt = std::chrono::steady_clock::now();
      SendPacedRTPAccessUnit(accessUnit, transmitAt, spread);
      rtpInFlight_.store(false);
      WakeControl();
    }
  }

  void SendPacedRTPAccessUnit(QueuedRTPAccessUnit &accessUnit,
                              std::chrono::steady_clock::time_point transmitAt,
                              std::chrono::nanoseconds spread) {
    constexpr std::size_t kPacketBatchSize = 32;
    const std::size_t batchCount =
        (accessUnit.packets.size() + kPacketBatchSize - 1) / kPacketBatchSize;
    for (std::size_t batch = 0; batch < batchCount; ++batch) {
      const auto batchAt = batchCount == 1
          ? transmitAt + spread
          : transmitAt + spread * static_cast<std::int64_t>(batch) /
                             static_cast<std::int64_t>(batchCount - 1);
      std::this_thread::sleep_until(batchAt);
      {
        std::lock_guard<std::mutex> lock(rtpPacerMutex_);
        if (rtpPacerStopping_ || accessUnit.generation != rtpQueueGeneration_) return;
      }
      const std::size_t begin = batch * kPacketBatchSize;
      const std::size_t end = std::min(accessUnit.packets.size(), begin + kPacketBatchSize);
      for (std::size_t index = begin; index < end; ++index) {
        SendPacedRTPPacket(accessUnit.packets[index].bytes);
      }
    }
  }

  void SendPacedRTPPacket(std::vector<std::uint8_t> &packet) {
    if (rtpSocket_ < 0 || packet.empty()) {
      return;
    }
    const std::uint16_t sequence = htons(rtpSequence_++);
    std::memcpy(packet.data() + 2, &sequence, sizeof(sequence));
    const ssize_t sent = sendto(rtpSocket_, packet.data(), packet.size(), MSG_NOSIGNAL,
                                reinterpret_cast<const sockaddr *>(&rtpDestination_), sizeof(rtpDestination_));
    if (sent != static_cast<ssize_t>(packet.size())) {
      LogEncoderError(std::string("paced loopback RTP send failed: ") + std::strerror(errno));
    }
  }

  void WakeControl() noexcept {
    if (auto *host = controlWakeHost_.load()) host->Wake();
  }

  bool StartControlServer() {
    try {
      controlHost_ = gazebo_sim_camera::CameraSourceControlHost::Acquire(
          controlSocketPath_, controlTargetID_, controlInstanceID_);
      controlWakeHost_.store(controlHost_.get());
      desiredConfig_["active"] = false;
      desiredConfig_["jpeg_quality"] = jpegQuality_.load();
      desiredConfig_["rtp_host"] = rtpHost_;
      desiredConfig_["rtp_port"] = rtpPort_;
      desiredConfig_["bitrate"] = bitrate_;
      if (rosPublishingEnabled_) {
        auto &metadata = desiredConfig_["calibration"];
        metadata["scope"] = "calibration-metadata";
        metadata["model"] = "plumb_bob";
        metadata["width"] = calibrationMetadata_.width;
        metadata["height"] = calibrationMetadata_.height;
        for (auto value : calibrationMetadata_.matrix) metadata["camera_matrix"].append(value);
        for (std::size_t i = 0; i < calibrationMetadata_.distortionCount; ++i) metadata["distortion"].append(calibrationMetadata_.distortion[i]);
      }
      appliedConfig_ = desiredConfig_;
      controlHost_->Register(sourceID_,
          [this](const auto &operation, const auto &request, const auto &input, auto reply) {
            HandleControlRequest(operation, request, input, reply);
          }, [this] { FlushControl(); });
      controlRegistered_ = true;
      return true;
    } catch (const std::exception &error) {
      gzerr << "xgc_media_camera XRPC startup failed: " << error.what() << "\n";
      return false;
    }
  }

  void StopControlServer() {
    if (controlHost_ && controlRegistered_) {
      controlHost_->Unregister(sourceID_);
      controlRegistered_ = false;
    }
    if (pendingControl_) pendingControl_->reply.complete(
        gazebo_sim_camera::ControlError(503, "unavailable", "camera sensor was removed"));
    for (auto &watcher : controlWatchers_) if (watcher) {
      watcher->reply.complete(gazebo_sim_camera::ControlError(503, "unavailable", "camera sensor was removed"));
      watcher.reset();
    }
    if (captureReply_) {
      captureReply_->complete(gazebo_sim_camera::ControlError(503, "unavailable", "camera sensor was removed"));
      captureReply_.reset();
      // Keep the shared slot until the JPEG worker joins in the destructor.
      // Sensor removal cannot overlap old submitted work with a new capture.
    }
  }

  static void RequireFields(const Json::Value &input,
      std::initializer_list<const char *> allowed) {
    for (const auto &name : input.getMemberNames()) {
      if (std::none_of(allowed.begin(), allowed.end(), [&name](const char *field) { return name == field; }))
        throw std::invalid_argument("unknown camera request field: " + name);
    }
  }

  Json::Value CameraStatus() const {
    std::lock_guard<std::mutex> stream(rosPublisherMutex_);
    Json::Value result;
    result["ok"] = true;
    static const char *states[] = {"idle", "starting", "active", "stopping", "faulted"};
    result["state"] = states[nativeLifecycleState_.load()];
    result["desired_active"] = desiredConfig_["active"];
    result["applied_active"] = nativeAppliedActive_.load();
    result["configuration_revision"] = Json::UInt64(desiredRevision_);
    result["desired_revision"] = Json::UInt64(desiredRevision_);
    result["applied_revision"] = Json::UInt64(appliedRevision_);
    result["persisted_revision"] = Json::nullValue;
    result["last_error"] = nativeLifecycleState_.load() == 4 ? "native encoder did not accept a rendered frame" : "";
    result["native"]["encoder_ready"] = encoderResourcesActive_.load();
    result["native"]["rtp_in_flight"] = rtpInFlight_.load();
    result["source_id"] = sourceID_;
    result["service_ref"] = controlHost_->ServiceRef();
    result["protocol_version"] = 1;
    result["codec"] = "H264";
    result["rtp_payload_type"] = Json::UInt(kH264PayloadType);
    result["rtp_clock_rate"] = kRtpClockRate;
    result["rtp_host"] = rtpHost_;
    result["rtp_port"] = rtpPort_;
    result["width"] = streamWidth_;
    result["height"] = streamHeight_;
    result["fps"] = streamFrameRate_;
    result["frame_id"] = frameID_;
    result["timestamp_clock_domain"] = kSourceTimestampClockDomain;
    result["snapshot_jpeg_policy"] = gazebo_sim_camera::SnapshotJpegPolicyName(snapshotJpegPolicy_);
    result["snapshot_jpeg_backend"] = snapshotJpegBackend_.backend;
    result["snapshot_jpeg_hardware_state"] = snapshotJpegBackend_.hardwareState;
    result["capabilities"] = Json::Value(Json::arrayValue);
    for (const auto capability : {"start", "stop", "request-keyframe", "capture", "fresh-snapshot", "config", "observe", "receipts"})
      result["capabilities"].append(capability);
    if (rosPublishingEnabled_) result["capabilities"].append("calibration-metadata");
    result["camera_info_topic"] = rosCameraInfoTopic_;
    result["published_calibration_revision"] = Json::UInt64(publishedCalibrationRevision_.load());
    result["native_lifecycle_owner"] = "simulation-v1";
    result["desired"] = desiredConfig_;
    result["applied"] = appliedConfig_;
    result["native"]["effective_time"]["nanoseconds"] = Json::Int64(appliedTimeNanoseconds_);
    result["native"]["effective_time"]["clock_domain"] = "simulation";
    result["event_revision"] = Json::UInt64(controlEventRevision_);
    result["pending"] = bool(pendingControl_);
    if (nextControlReceipt_ > 0) result["last_receipt"] = controlReceipts_[(nextControlReceipt_ - 1) % controlReceipts_.size()];
    result["ros_consumers_active"] = rosConsumersActive_.load();
    result["native_sensor_active"] = nativeSensorActive_.load();
    result["rtp_requested_active"] = desiredActive_.load();
    result["configuration_schema"]["schema_version"] = 1;
    result["configuration_schema"]["live_fields"].append("jpeg_quality");
    if (rosPublishingEnabled_) result["configuration_schema"]["live_fields"].append("calibration");
    result["configuration_schema"]["persist_supported"] = false;
    result["configuration_schema"]["restart_required_fields"] = Json::Value(Json::arrayValue);
    for (const auto field : {"width", "height", "fps", "frame_id", "rtp_host", "rtp_port", "bitrate", "snapshot_jpeg_backend"})
      result["configuration_schema"]["restart_required_fields"].append(field);
    result["limits"]["pending_mutations_per_source"] = 1;
    result["limits"]["concurrent_captures_per_host"] = 1;
    result["limits"]["receipt_count"] = 32;
    result["limits"]["observers_per_source"] = 4;
    result["limits"]["capture_bytes"] = 64 * 1024 * 1024;
    result["limits"]["jpeg_bytes"] = 32 * 1024 * 1024;
    result["limits"]["rgb_bytes"] = 4096 * 2160 * 3;
    result["limits"]["capture_timeout_ms"] = 4000;
    result["storage"]["persistent_writes"] = Json::Value(Json::arrayValue);
    result["storage"]["runtime_endpoint"] = controlSocketPath_;
    return result;
  }

  Json::Value CameraDescriptor() const {
    std::lock_guard<std::mutex> stream(rosPublisherMutex_);
    Json::Value result;
    result["ok"] = true; result["protocolVersion"] = 1; result["sourceId"] = sourceID_;
    result["codec"] = "H264"; result["rtpPayloadType"] = Json::UInt(kH264PayloadType);
    result["rtpClockRate"] = kRtpClockRate; result["rtpHost"] = rtpHost_; result["rtpPort"] = rtpPort_;
    result["width"] = streamWidth_; result["height"] = streamHeight_; result["fps"] = streamFrameRate_;
    result["frameId"] = frameID_; result["keyframeRequestSupported"] = true;
    result["keyframePolicy"] = "native-force-idr";
    for (const auto capability : {"start", "stop", "request-keyframe", "capture", "fresh-snapshot", "config"}) result["capabilities"].append(capability);
    if (rosPublishingEnabled_) result["capabilities"].append("calibration-metadata");
    result["snapshotJpegPolicy"] = gazebo_sim_camera::SnapshotJpegPolicyName(snapshotJpegPolicy_);
    result["snapshotJpegBackend"] = snapshotJpegBackend_.backend;
    result["snapshotJpegHardwareState"] = snapshotJpegBackend_.hardwareState;
    return result;
  }

  Json::Value CameraConfiguration() const {
    auto result = CameraStatus();
    result["persistence"] = "ephemeral";
    result["mutable_fields"].append("jpeg_quality");
    if (rosPublishingEnabled_) result["mutable_fields"].append("calibration");
    return result;
  }

  Json::Value RememberReceipt(const std::string &requestID, const std::string &operation,
      const std::string &stage, const std::string &error = "") {
    Json::Value receipt;
    receipt["operation_id"] = requestID;
    receipt["request_id"] = requestID;
    receipt["operation"] = operation;
    receipt["stage"] = stage;
    receipt["desired_revision"] = Json::UInt64(desiredRevision_);
    receipt["applied_revision"] = Json::UInt64(appliedRevision_);
    receipt["persisted_revision"] = Json::Value();
    receipt["event_revision"] = Json::UInt64(++controlEventRevision_);
    receipt["effects"]["applied"] = stage == "completed";
    if (!error.empty()) { receipt["error"] = error; receipt["failure_stage"] = operation == "capture" ? "capture" : "apply"; }
    controlReceipts_[nextControlReceipt_++ % controlReceipts_.size()] = receipt;
    return receipt;
  }

  void HandleControlRequest(const std::string &operation,
      const gazebo_sim_camera::ControlRequest &request, const Json::Value &input,
      gazebo_sim_camera::ControlReply reply) {
    if (request.method == "GET" && (operation == "describe" || operation == "status" || operation == "config")) {
      RequireFields(input, {});
      reply.complete(gazebo_sim_camera::JSONResponse(operation == "describe" ? CameraDescriptor() : operation == "config" ? CameraConfiguration() : CameraStatus()));
      return;
    }
    if (request.method == "GET" && operation.compare(0, 9, "receipts/") == 0) {
      RequireFields(input, {});
      const auto id = operation.substr(9);
      for (const auto &receipt : controlReceipts_) if (receipt["operation_id"].asString() == id) {
        reply.complete(gazebo_sim_camera::JSONResponse(receipt));
        return;
      }
      reply.complete(gazebo_sim_camera::ControlError(404, "not_found", "receipt has expired or was not accepted"));
      return;
    }
    if (request.method == "GET" && operation.compare(0, 8, "observe/") == 0) {
      RequireFields(input, {});
      const auto sequence = operation.substr(8);
      std::uint64_t after = 0;
      if (sequence.empty() || sequence.size() > 20 || (sequence.size() > 1 && sequence[0] == '0'))
        throw std::invalid_argument("invalid observation revision");
      for (const auto c : sequence) {
        if (c < '0' || c > '9' || after > (std::numeric_limits<std::uint64_t>::max() - (c - '0')) / 10)
          throw std::invalid_argument("invalid observation revision");
        after = after * 10 + (c - '0');
      }
      if (after > controlEventRevision_) {
        reply.complete(gazebo_sim_camera::ControlError(409, "conflict", "observation revision is ahead of provider"));
        return;
      }
      if (after < controlEventRevision_) {
        reply.complete(gazebo_sim_camera::JSONResponse(CameraStatus()));
        return;
      }
      for (auto &watcher : controlWatchers_) if (!watcher) {
        watcher = ControlWatcher{reply, request.deadline, after};
        return;
      }
      reply.complete(gazebo_sim_camera::ControlError(429, "resource_exhausted", "camera observers are full"));
      return;
    }
    if (request.method == "POST" && operation == "capture") {
      for (const auto &receipt : controlReceipts_) if (receipt["request_id"].asString() == request.request_id) {
        reply.complete(gazebo_sim_camera::ControlError(409, "conflict", "request_id was already used; read its receipt"));
        return;
      }
      RequireFields(input, {"snapshotId", "includeRgb", "requireFresh", "requestKeyframe"});
      for (const auto field : {"includeRgb", "requireFresh", "requestKeyframe"})
        if (input.isMember(field) && !input[field].isBool()) throw std::invalid_argument(std::string(field) + " must be a boolean");
      if (input.isMember("snapshotId") && (!input["snapshotId"].isString() ||
          !gazebo_sim_camera::SafeSourceIdentifier(input["snapshotId"].asString())))
        throw std::invalid_argument("snapshotId must be a stable source identifier");
      if (input.get("requestKeyframe", false).asBool() && !nativeAppliedActive_.load()) {
        reply.complete(gazebo_sim_camera::ControlError(409, "conflict", "native force-IDR requires an active encoder")); return;
      }
      if (!controlHost_->ReserveCapture()) {
        reply.complete(gazebo_sim_camera::ControlError(429, "resource_exhausted", "camera capture slot is busy"));
        return;
      }
      {
        std::lock_guard<std::mutex> lock(snapshotMutex_);
        snapshot_ = SnapshotResult{};
        snapshot_.id = input.get("snapshotId", request.request_id).asString();
        snapshot_.generation = ++snapshotGeneration_;
        snapshot_.includeRGB = input.get("includeRgb", true).asBool();
        captureKeyframeGeneration_ = input.get("requestKeyframe", false).asBool() ? snapshot_.generation : 0;
        snapshotRenderPassesToSkip_ = 1;
      }
      if (captureKeyframeGeneration_) { nativeCaptureKeyframePending_.store(captureKeyframeGeneration_); forceKeyframe_.store(true); }
      captureSlotHeld_ = true;
      captureReply_ = reply;
      captureRequestID_ = request.request_id;
      captureDeadline_ = std::min(request.deadline, gazebo_sim_camera::ControlClock::now() + std::chrono::seconds(4));
      return;
    }
    const bool mutation = (request.method == "POST" && (operation == "start" || operation == "stop" || operation == "request-keyframe")) ||
        (request.method == "PATCH" && operation == "config");
    if (!mutation) {
      reply.complete(gazebo_sim_camera::ControlError(404, "not_found", "camera route or method not supported"));
      return;
    }
    RequireFields(input, operation == "config" ? std::initializer_list<const char *>{"expected_revision", "persist", "config"} :
        std::initializer_list<const char *>{});
    if (input.isMember("persist") && !input["persist"].isBool()) throw std::invalid_argument("persist must be a boolean");
    if (input.get("persist", false).asBool()) {
      reply.complete(gazebo_sim_camera::ControlError(400, "invalid_argument", "Gazebo camera configuration is ephemeral; persist is unsupported"));
      return;
    }
    if (operation == "config" && !input.isMember("expected_revision"))
      throw std::invalid_argument("expected_revision is required for config");
    if (input.isMember("expected_revision") && ((input["expected_revision"].type() != Json::intValue && input["expected_revision"].type() != Json::uintValue) || !input["expected_revision"].isUInt64()))
      throw std::invalid_argument("expected_revision must be an unsigned integer");
    if (input.isMember("expected_revision") && input["expected_revision"].asUInt64() != desiredRevision_) {
      reply.complete(gazebo_sim_camera::ControlError(409, "conflict", "camera configuration revision differs"));
      return;
    }
    if (pendingControl_) {
      reply.complete(gazebo_sim_camera::ControlError(429, "resource_exhausted", "camera native mutation slot is busy"));
      return;
    }
    if (operation == "request-keyframe" && !nativeAppliedActive_.load()) {
      reply.complete(gazebo_sim_camera::ControlError(409, "conflict", "native force-IDR requires an active encoder")); return;
    }
    for (const auto &receipt : controlReceipts_) if (receipt["request_id"].asString() == request.request_id) {
      reply.complete(gazebo_sim_camera::ControlError(409, "conflict", "request_id was already used; read its receipt"));
      return;
    }
    Json::Value desired = desiredConfig_;
    std::optional<NativeCalibrationMetadata> metadata;
    if (operation == "config") {
      if (!input["config"].isObject() || input["config"].empty()) throw std::invalid_argument("config must be a nonempty object");
      RequireFields(input["config"], {"jpeg_quality", "calibration", "rtp_host", "rtp_port", "bitrate"});
      for (const auto field : {"rtp_host", "rtp_port", "bitrate"}) if (input["config"].isMember(field)) {
        reply.complete(gazebo_sim_camera::ControlError(409, "restart_required", std::string(field) + " is an authored startup binding")); return;
      }
      if (input["config"].isMember("jpeg_quality")) {
        if ((input["config"]["jpeg_quality"].type() != Json::intValue && input["config"]["jpeg_quality"].type() != Json::uintValue) || !input["config"]["jpeg_quality"].isInt() || input["config"]["jpeg_quality"].asInt() < 50 || input["config"]["jpeg_quality"].asInt() > 100)
          throw std::invalid_argument("jpeg_quality must be 50 through 100");
        desired["jpeg_quality"] = input["config"]["jpeg_quality"];
      }
      if (input["config"].isMember("calibration")) {
        if (!rosPublishingEnabled_) {
          reply.complete(gazebo_sim_camera::ControlError(503, "unavailable", "CameraInfo data publication is disabled"));
          return;
        }
        const auto &calibration = input["config"]["calibration"];
        if (!calibration.isObject()) throw std::invalid_argument("calibration must be an object");
        RequireFields(calibration, {"scope", "model", "width", "height", "camera_matrix", "distortion"});
        if (!calibration["scope"].isString() || !calibration["model"].isString() || calibration["scope"].asString() != "calibration-metadata" ||
            (calibration["width"].type() != Json::intValue && calibration["width"].type() != Json::uintValue) ||
            (calibration["height"].type() != Json::intValue && calibration["height"].type() != Json::uintValue) || !calibration["width"].isUInt() || !calibration["height"].isUInt() ||
            calibration["width"].asUInt() != streamWidth_ || calibration["height"].asUInt() != streamHeight_)
          throw std::invalid_argument("calibration scope/size must match the native source");
        NativeCalibrationMetadata record;
        record.width = streamWidth_; record.height = streamHeight_;
        const auto model = calibration["model"].asString();
        if (model == "plumb_bob") { record.model = 0; record.distortionCount = 5; }
        else if (model == "rational_polynomial") { record.model = 1; record.distortionCount = 8; }
        else if (model == "equidistant") { record.model = 2; record.distortionCount = 4; }
        else throw std::invalid_argument("unsupported calibration distortion model");
        if (!calibration["camera_matrix"].isArray() || calibration["camera_matrix"].size() != 9 ||
            !calibration["distortion"].isArray() || calibration["distortion"].size() != record.distortionCount)
          throw std::invalid_argument("calibration matrix/distortion dimensions differ from its model");
        const auto number = [](const Json::Value &value) {
          if (!value.isNumeric() || !std::isfinite(value.asDouble())) throw std::invalid_argument("calibration coefficients must be finite numbers");
          return value.asDouble();
        };
        for (std::size_t i = 0; i < 9; ++i) record.matrix[i] = number(calibration["camera_matrix"][Json::ArrayIndex(i)]);
        for (std::size_t i = 0; i < record.distortionCount; ++i) record.distortion[i] = number(calibration["distortion"][Json::ArrayIndex(i)]);
        if (record.matrix[0] <= 0 || record.matrix[4] <= 0 || record.matrix[6] != 0 || record.matrix[7] != 0 || record.matrix[8] != 1)
          throw std::invalid_argument("calibration camera matrix must have positive focal lengths and homogeneous bottom row");
        metadata = record;
        desired["calibration"] = calibration;
      }
    } else if (operation != "request-keyframe") desired["active"] = operation == "start";
    PendingControl pending;
    pending.reply = reply;
    pending.requestID = request.request_id;
    pending.operation = operation;
    pending.deadline = request.deadline;
    pending.revision = ++desiredRevision_;
    desiredConfig_ = desired;
    pending.calibration = metadata.has_value();
    if (metadata) metadata->revision = pending.revision;
    {
      std::lock_guard<std::mutex> lock(nativeCommandMutex_);
      const int kind = operation == "start" ? 1 : operation == "stop" ? 2 : operation == "request-keyframe" ? 3 : 0;
      nativeCommand_ = NativeControlCommand{pending.revision, kind == 0 ? desiredActive_.load() : desired["active"].asBool(), desired["jpeg_quality"].asInt(), operation == "request-keyframe", metadata, kind};
      if (kind == 1) nativeLifecycleState_.store(1);
      if (kind == 2) nativeLifecycleState_.store(3);
    }
    pendingControl_ = std::move(pending);
  }

  // Render-thread handoff: fixed record, try-lock, no JSON or networking.
  void ApplyNativeControl() {
    std::unique_lock<std::mutex> lock(nativeCommandMutex_, std::try_to_lock);
    if (!lock || !nativeCommand_) return;
    const auto command = *nativeCommand_;
    std::unique_lock<std::mutex> publisher(rosPublisherMutex_, std::defer_lock);
    if (command.calibration && !publisher.try_lock()) return;
    nativeCommand_.reset();
    const bool oldActive = desiredActive_.exchange(command.active);
    jpegQuality_.store(command.jpegQuality);
    if (command.keyframe || (command.active && !oldActive)) forceKeyframe_.store(true);
    if (command.active && !oldActive && !rosConsumersActive_.load()) rosFreshRenderGeneration_.fetch_add(1);
    if (!command.active) ClearRTPQueue();
    // Demand includes independent ROS data consumers and explicit snapshots.
    if (command.calibration) {
      calibrationMetadata_ = *command.calibration;
      rosCameraInfoPending_ = true;
      rosPublisherCondition_.notify_one();
    }
    sensor_->SetActive(command.active || rosConsumersActive_.load() || SnapshotNeedsRender());
    nativeSensorActive_.store(sensor_->IsActive());
    if (command.kind == 0) CompleteNativeControl(command);
    else awaitingNativeControl_ = command;
  }

  void CompleteNativeControl(const NativeControlCommand &command) {
    if (command.kind == 1) { nativeAppliedActive_.store(true); nativeLifecycleState_.store(2); }
    if (command.kind == 2) { nativeAppliedActive_.store(false); nativeLifecycleState_.store(0); }
    nativeAppliedTime_.store(SourceTimeNanoseconds());
    nativeAppliedRevision_.store(command.revision);
    WakeControl();
  }

  void CompleteNativeStopIfDrained() {
    if (awaitingNativeControl_ && awaitingNativeControl_->kind == 2 && !rtpInFlight_.load() &&
        (rosConsumersActive_.load() || !encoderResourcesActive_.load())) {
      if (!rosConsumersActive_.load() && !SnapshotNeedsRender()) sensor_->SetActive(false);
      nativeSensorActive_.store(sensor_->IsActive());
      CompleteNativeControl(*awaitingNativeControl_); awaitingNativeControl_.reset();
    }
  }

  void FailNativeEncoderTransition() {
    const auto capture = nativeCaptureKeyframePending_.exchange(0);
    if (capture) nativeCaptureKeyframeFailed_.store(capture);
    if (awaitingNativeControl_ && (awaitingNativeControl_->kind == 1 || awaitingNativeControl_->kind == 3)) {
      nativeFailedRevision_.store(awaitingNativeControl_->revision);
      awaitingNativeControl_.reset(); desiredActive_.store(false);
      nativeAppliedActive_.store(false); nativeLifecycleState_.store(4);
      ClearRTPQueue(); DestroyEncoder(); WakeControl();
    }
  }

  void FlushControl() {
    const auto now = gazebo_sim_camera::ControlClock::now();
    if (pendingControl_ && nativeFailedRevision_.load() == pendingControl_->revision) {
      RememberReceipt(pendingControl_->requestID, pendingControl_->operation, "failed", "native encoder did not accept a rendered frame");
      pendingControl_->reply.complete(gazebo_sim_camera::ControlError(503, "unavailable", "native encoder did not accept a rendered frame"));
      pendingControl_.reset();
    } else if (pendingControl_ && nativeAppliedRevision_.load() == pendingControl_->revision &&
        (!pendingControl_->calibration || publishedCalibrationRevision_.load() == pendingControl_->revision)) {
      appliedRevision_ = pendingControl_->revision;
      appliedConfig_ = desiredConfig_;
      appliedConfig_["active"] = nativeAppliedActive_.load();
      appliedTimeNanoseconds_ = nativeAppliedTime_.load();
      auto pending = std::move(*pendingControl_);
      pendingControl_.reset();
      auto receipt = RememberReceipt(pending.requestID, pending.operation, "completed");
      Json::Value result = CameraConfiguration();
      result["receipt"] = std::move(receipt);
      if (pending.operation == "start" || pending.operation == "stop") {
        result["active"] = nativeAppliedActive_.load(); result["completion"] = "applied";
      }
      if (pending.operation == "request-keyframe") result["completion"] = "applied";
      pending.reply.complete(gazebo_sim_camera::JSONResponse(result));
    } else if (pendingControl_ && (pendingControl_->reply.cancelled() || now >= pendingControl_->deadline)) {
      // Remove only an unconsumed command. If the render owns it, keep the
      // record until its real acknowledgement; caller cancellation is not rollback.
      std::unique_lock<std::mutex> lock(nativeCommandMutex_, std::try_to_lock);
      if (lock && nativeCommand_ && nativeCommand_->revision == pendingControl_->revision) {
        nativeCommand_.reset();
        RememberReceipt(pendingControl_->requestID, pendingControl_->operation, "failed", "native application deadline exceeded");
        pendingControl_->reply.complete(gazebo_sim_camera::ControlError(504, "deadline_exceeded", "native application deadline exceeded"));
        pendingControl_.reset();
      }
    }
    if (captureReply_) FlushCapture(now);
    for (auto &watcher : controlWatchers_) if (watcher) {
      if (watcher->reply.cancelled()) watcher.reset();
      else if (watcher->after < controlEventRevision_) {
        watcher->reply.complete(gazebo_sim_camera::JSONResponse(CameraStatus()));
        watcher.reset();
      } else if (now >= watcher->deadline) {
        watcher->reply.complete(gazebo_sim_camera::ControlError(504, "deadline_exceeded", "no camera event before observation deadline"));
        watcher.reset();
      }
    }
  }

  void FlushCapture(gazebo_sim_camera::ControlClock::time_point now) {
    SnapshotResult response;
    bool completed = false;
    {
      std::lock_guard<std::mutex> lock(snapshotMutex_);
      completed = snapshot_.completed;
      if (completed && captureKeyframeGeneration_ && nativeCaptureKeyframeApplied_.load() != captureKeyframeGeneration_) {
        if (nativeCaptureKeyframeFailed_.load() != captureKeyframeGeneration_) return;
        snapshot_.failed = true; snapshot_.error = "native force-IDR did not accept the capture request";
      }
      if (completed) {
        response = std::move(snapshot_);
        snapshot_ = SnapshotResult{};
        snapshotRenderPassesToSkip_ = 0;
      } else if (captureReply_->cancelled() || now >= captureDeadline_) {
        // The one slot remains reserved until any submitted GPU/worker work
        // finishes. This prevents timeout followed by overlapping readbacks.
        if (!snapshot_.captureSubmitted && !snapshotReadbackPending_.load()) {
          snapshot_ = SnapshotResult{};
          snapshotRenderPassesToSkip_ = 0;
          completed = true;
          response.failed = true;
          response.error = "camera capture deadline exceeded";
        } else return;
      }
    }
    if (!completed) return;
    auto reply = *captureReply_;
    captureReply_.reset();
    const bool oversized = response.jpeg.size() > 32 * 1024 * 1024 || response.rgb.size() > 4096 * 2160 * 3;
    if (oversized) response.error = "camera capture payload exceeds limits";
    const auto receipt = RememberReceipt(captureRequestID_, "capture", response.failed || oversized ? "failed" : "completed", response.error);
    if (response.failed) {
      reply.complete(gazebo_sim_camera::ControlError(503, "unavailable", response.error));
    } else if (oversized) {
      reply.complete(gazebo_sim_camera::ControlError(429, "resource_exhausted", "camera capture payload exceeds limits"));
    } else {
      Json::Value metadata;
      metadata["ok"] = true;
      metadata["snapshotId"] = response.id;
      metadata["sourceId"] = sourceID_;
      metadata["frameId"] = frameID_;
      metadata["frameSequence"] = Json::UInt64(response.frameSequence);
      metadata["timestampNanoseconds"] = Json::Int64(response.timestampNanoseconds);
      metadata["timestampClockDomain"] = kSourceTimestampClockDomain;
      metadata["width"] = response.width;
      metadata["height"] = response.height;
      metadata["pixelFormat"] = "rgb8";
      metadata["jpegBytes"] = Json::UInt64(response.jpeg.size());
      metadata["rgbBytes"] = Json::UInt64(response.rgb.size());
      metadata["jpegBackend"] = response.jpegBackend;
      metadata["jpegReadback"] = response.jpegReadback;
      metadata["jpegReadbackMilliseconds"] = response.jpegReadbackMilliseconds;
      metadata["jpegEncodeMilliseconds"] = response.jpegEncodeMilliseconds;
      metadata["calibrationState"] = response.calibrationValid ? "available" : "unavailable";
      if (response.calibrationValid) metadata["calibrationSource"] = "native-projection";
      if (response.calibrationValid) {
        for (const auto value : response.cameraMatrix) metadata["cameraMatrix"].append(value);
        for (const auto value : response.distortion) metadata["distortion"].append(value);
      }
      if (!response.jpegFallbackReason.empty()) metadata["jpegFallbackReason"] = response.jpegFallbackReason;
      if (response.renderPoseValid) {
        const char *axes[] = {"x", "y", "z", "w"};
        for (std::size_t i = 0; i < 3; ++i) metadata["renderPose"]["position"][axes[i]] = response.renderPosition[i];
        for (std::size_t i = 0; i < 4; ++i) metadata["renderPose"]["orientation"][axes[i]] = response.renderOrientation[i];
        metadata["poseFrameId"] = response.poseFrameID;
      }
      metadata["receipt"] = receipt;
      const auto boundary = "xgc-capture-" + gazebo_sim_camera::NewControlIdentifier();
      gazebo_sim_camera::ControlResponse result;
      result.headers.emplace_back("Content-Type", "multipart/mixed; boundary=" + boundary);
      const auto appendPart = [&result, &boundary](const std::string &name, const std::string &type, const char *bytes, std::size_t count) {
        result.body += "--" + boundary + "\r\nContent-Type: " + type + "\r\nContent-Disposition: inline; name=\"" + name + "\"\r\n\r\n";
        result.body.append(bytes, count);
        result.body += "\r\n";
      };
      const auto json = gazebo_sim_camera::ControlJSON(metadata);
      result.body.reserve(response.jpeg.size() + response.rgb.size() + json.size() + 1024);
      appendPart("metadata", "application/json", json.data(), json.size());
      appendPart("jpeg", "image/jpeg", reinterpret_cast<const char *>(response.jpeg.data()), response.jpeg.size());
      if (!response.rgb.empty()) appendPart("rgb", "application/octet-stream", reinterpret_cast<const char *>(response.rgb.data()), response.rgb.size());
      result.body += "--" + boundary + "--\r\n";
      reply.complete(std::move(result));
    }
    controlHost_->ReleaseCapture();
    captureSlotHeld_ = false;
  }

  bool SnapshotNeedsRender() const {
    if (snapshotReadbackPending_.load()) {
      return true;
    }
    std::lock_guard<std::mutex> lock(snapshotMutex_);
    return !snapshot_.id.empty() && !snapshot_.completed &&
           !snapshot_.captureSubmitted;
  }

  void OnPostRender() {
    if (stopping_.load() || !sensor_) {
      return;
    }
    ApplyNativeControl();
    CompleteNativeStopIfDrained();
    nativeSensorActive_.store(sensor_->IsActive());
    const bool shouldEncode =
        desiredActive_.load() || rosConsumersActive_.load();
    if (!camera_) {
      camera_ = sensor_->Camera();
      if (!camera_) {
        if (shouldEncode || SnapshotNeedsRender()) {
          sensor_->SetActive(true);
        }
        return;
      }
    }
    if (!AttachRenderTargetListener()) {
      return;
    }
    const bool shouldRender = shouldEncode || SnapshotNeedsRender();
    if (shouldRender) {
      if (!sensor_->IsActive()) {
        sensor_->SetActive(true);
      }
      return;
    }
    if (encoderResourcesActive_.load()) {
      // Let the render-target listener run one final time while its GL context
      // is current. It releases NVENC/GL resources and the next post-render
      // pass disables the Gazebo sensor completely.
      cleanupEncoderRequested_.store(true);
      if (!sensor_->IsActive()) {
        sensor_->SetActive(true);
      }
      return;
    }
    if (sensor_->IsActive()) {
      sensor_->SetActive(false);
    }
  }

  bool AttachRenderTargetListener() {
    if (renderTarget_) {
      return true;
    }
    if (!camera_ || !camera_->RenderTexture()) {
      return false;
    }
    Ogre::RenderTarget *target = camera_->RenderTexture()->getBuffer()->getRenderTarget();
    if (!target) {
      return false;
    }
    target->addListener(this);
    renderTarget_ = target;
    return true;
  }

  void postRenderTargetUpdate(const Ogre::RenderTargetEvent &) override {
    if (stopping_.load()) {
      return;
    }
    ++renderFrameSequence_;
    const bool shouldEncode =
        desiredActive_.load() || rosConsumersActive_.load();
    if (cleanupEncoderRequested_.exchange(false) && !shouldEncode &&
        !SnapshotNeedsRender()) {
      DestroyEncoder();
      return;
    }
    if (!shouldEncode && !SnapshotNeedsRender()) {
      return;
    }
    if (SnapshotNeedsRender()) {
      CaptureSnapshot();
    }
    if (!shouldEncode) {
      return;
    }
    if (!ROSRenderIsFreshForCurrentActivation()) {
      return;
    }
    if (!EncodeRenderedFrame()) {
      FailNativeEncoderTransition();
      // Keep the source alive; a transient GPU context or driver reset should
      // be visible in Gazebo logs but must not turn into a CPU fallback path.
      return;
    }
    if (awaitingNativeControl_ && awaitingNativeControl_->kind == 1) {
      CompleteNativeControl(*awaitingNativeControl_); awaitingNativeControl_.reset();
    }
  }

  bool CaptureSnapshot() {
    if (snapshotReadbackPending_.load()) {
      return PollSnapshotPBOReadback();
    }
    SnapshotResult result;
    {
      std::lock_guard<std::mutex> lock(snapshotMutex_);
      if (snapshot_.id.empty() || snapshot_.completed ||
          snapshot_.captureSubmitted) {
        return false;
      }
      if (snapshotRenderPassesToSkip_ > 0) {
        --snapshotRenderPassesToSkip_;
        return false;
      }
      result.id = snapshot_.id;
      result.generation = snapshot_.generation;
      result.includeRGB = snapshot_.includeRGB;
    }
    if (!camera_ || !camera_->RenderTexture()) {
      CompleteSnapshotFailure("camera render texture is unavailable");
      return false;
    }
    const unsigned int width = camera_->ImageWidth();
    const unsigned int height = camera_->ImageHeight();
    if (width == 0 || height == 0 || width > 4096 || height > 2160) {
      CompleteSnapshotFailure("camera dimensions are invalid");
      return false;
    }
    const std::size_t bytes = static_cast<std::size_t>(width) * height * 3;
    result.width = width;
    result.height = height;
    result.frameSequence = renderFrameSequence_;
    result.jpegQuality = jpegQuality_.load();
    if (camera_->ProjectionType() == "perspective") {
      const auto projection = camera_->ProjectionMatrix();
      result.cameraMatrix = {projection(0,0) * width / 2., projection(0,1) * width / 2., (1. - projection(0,2)) * width / 2.,
                            0., projection(1,1) * height / 2., (1. + projection(1,2)) * height / 2., 0., 0., 1.};
      result.calibrationValid = std::all_of(result.cameraMatrix.begin(), result.cameraMatrix.end(), [](double value) { return std::isfinite(value); }) && result.cameraMatrix[0] > 0 && result.cameraMatrix[4] > 0;
      if (const auto distortion = camera_->LensDistortion()) result.distortion = {distortion->K1(), distortion->K2(), distortion->P1(), distortion->P2(), distortion->K3()};
    }
    // Freeze source time and pose for the render being read back. JPEG work is
    // deliberately deferred and must never relabel the frame with worker time.
    const ignition::math::Pose3d renderCameraPose = camera_->WorldPose();
    const ignition::math::Quaterniond linkToOptical(
        -1.5707963267948966, 0.0, -1.5707963267948966);
    ignition::math::Quaterniond opticalRotation =
        renderCameraPose.Rot() * linkToOptical;
    opticalRotation.Normalize();
    result.renderPosition = {
        renderCameraPose.Pos().X(),
        renderCameraPose.Pos().Y(),
        renderCameraPose.Pos().Z()};
    result.renderOrientation = {
        opticalRotation.X(),
        opticalRotation.Y(),
        opticalRotation.Z(),
        opticalRotation.W()};
    result.poseFrameID = snapshotPoseFrameID_;
    result.renderPoseValid = true;
    // Scene::SimTime is the time of the poses rendered into this texture.
    // CameraSensor resets its measurement timestamp on dormant activation,
    // so reading LastMeasurementTime here can relabel a fresh render as zero.
    const common::Time measurementTime = camera_->GetScene()->SimTime();
    result.timestampNanoseconds =
        static_cast<std::int64_t>(measurementTime.sec) * 1'000'000'000LL +
        measurementTime.nsec;

    if (snapshotPBOAvailable_ && IssueSnapshotPBOReadback(result, bytes)) {
      return true;
    }
    if (!snapshotPBOFailureReason_.empty()) {
      result.jpegFallbackReason = snapshotPBOFailureReason_;
    }
    return CaptureSnapshotSynchronously(std::move(result), bytes);
  }

  bool CaptureSnapshotSynchronously(
      SnapshotResult result,
      std::size_t bytes) {
    result.jpegReadback = "synchronous-cpu";
    result.rgb.resize(bytes);
    const auto readbackStarted = std::chrono::steady_clock::now();
    try {
      // PF_R8G8B8 is a word-ordered format and therefore writes B,G,R bytes
      // on little-endian hosts. The snapshot contract is explicitly rgb8, so
      // request OGRE's byte-ordered RGB format.
      Ogre::PixelBox destination(
          result.width, result.height, 1, Ogre::PF_BYTE_RGB,
          result.rgb.data());
      camera_->RenderTexture()->getBuffer()->blitToMemory(destination);
    } catch (const std::exception &error) {
      CompleteSnapshotFailure(std::string("camera snapshot readback failed: ") + error.what());
      return false;
    }
    result.jpegReadbackMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - readbackStarted).count();
    {
      std::lock_guard<std::mutex> lock(snapshotMutex_);
      if (snapshot_.id != result.id ||
          snapshot_.generation != result.generation || snapshot_.completed) {
        return false;
      }
      snapshot_.captureSubmitted = true;
    }
    if (!QueueSnapshotEncode(std::move(result))) {
      CompleteSnapshotFailure("snapshot JPEG worker queue is unavailable");
      return false;
    }
    return true;
  }

  bool EnsureGLEWDispatch() {
    if (glewInitialized_) {
      return true;
    }
    // SensorManager invokes its global post-render event after it releases the
    // OGRE context. RenderTargetListener runs while that context is current.
    glGetError();
    if (glewInit() != GLEW_OK) {
      LogEncoderError("GLEW could not initialize in the Gazebo render context");
      return false;
    }
    glGetError();
    glewInitialized_ = true;
    return true;
  }

  bool EnsureSnapshotPBO(std::size_t bytes) {
    if (!EnsureGLEWDispatch() || !GLEW_ARB_pixel_buffer_object ||
        !GLEW_ARB_sync) {
      snapshotPBOFailureReason_ =
          "asynchronous OpenGL PBO readback is unavailable";
      snapshotPBOAvailable_ = false;
      LogEncoderError(snapshotPBOFailureReason_ + "; using synchronous readback");
      return false;
    }
    if (snapshotReadbackBuffer_ == 0) {
      glGenBuffers(1, &snapshotReadbackBuffer_);
    }
    if (snapshotReadFramebuffer_ == 0) {
      glGenFramebuffers(1, &snapshotReadFramebuffer_);
    }
    if (snapshotReadbackBuffer_ == 0 || snapshotReadFramebuffer_ == 0) {
      snapshotPBOFailureReason_ =
          "OpenGL could not allocate snapshot PBO resources";
      snapshotPBOAvailable_ = false;
      LogEncoderError(snapshotPBOFailureReason_ + "; using synchronous readback");
      return false;
    }
    if (snapshotReadbackBufferBytes_ != bytes) {
      GLint previousBuffer = 0;
      glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &previousBuffer);
      while (glGetError() != GL_NO_ERROR) {
      }
      glBindBuffer(GL_PIXEL_PACK_BUFFER, snapshotReadbackBuffer_);
      glBufferData(
          GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(bytes), nullptr,
          GL_STREAM_READ);
      glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(previousBuffer));
      if (glGetError() != GL_NO_ERROR) {
        snapshotPBOFailureReason_ =
            "OpenGL could not size the snapshot PBO";
        snapshotPBOAvailable_ = false;
        LogEncoderError(snapshotPBOFailureReason_ + "; using synchronous readback");
        return false;
      }
      snapshotReadbackBufferBytes_ = bytes;
    }
    return true;
  }

  bool IssueSnapshotPBOReadback(
      SnapshotResult &result,
      std::size_t bytes) {
    if (!EnsureSnapshotPBO(bytes)) {
      return false;
    }
    auto *renderTexture = dynamic_cast<Ogre::GLTexture *>(
        camera_->RenderTexture());
    if (!renderTexture || renderTexture->getGLID() == 0) {
      snapshotPBOFailureReason_ =
          "Gazebo camera texture is not an OpenGL texture";
      snapshotPBOAvailable_ = false;
      LogEncoderError(snapshotPBOFailureReason_ + "; using synchronous readback");
      return false;
    }

    GLint previousFramebuffer = 0;
    GLint previousBuffer = 0;
    GLint previousPackAlignment = 0;
    GLint previousPackRowLength = 0;
    GLint previousPackSkipRows = 0;
    GLint previousPackSkipPixels = 0;
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previousFramebuffer);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &previousBuffer);
    glGetIntegerv(GL_PACK_ALIGNMENT, &previousPackAlignment);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &previousPackRowLength);
    glGetIntegerv(GL_PACK_SKIP_ROWS, &previousPackSkipRows);
    glGetIntegerv(GL_PACK_SKIP_PIXELS, &previousPackSkipPixels);
    auto restore = [&] {
      glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(previousBuffer));
      glPixelStorei(GL_PACK_ALIGNMENT, previousPackAlignment);
      glPixelStorei(GL_PACK_ROW_LENGTH, previousPackRowLength);
      glPixelStorei(GL_PACK_SKIP_ROWS, previousPackSkipRows);
      glPixelStorei(GL_PACK_SKIP_PIXELS, previousPackSkipPixels);
      glBindFramebuffer(
          GL_READ_FRAMEBUFFER, static_cast<GLuint>(previousFramebuffer));
    };

    while (glGetError() != GL_NO_ERROR) {
    }
    glBindFramebuffer(GL_READ_FRAMEBUFFER, snapshotReadFramebuffer_);
    glFramebufferTexture2D(
        GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
        renderTexture->getGLID(), 0);
    if (glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) !=
        GL_FRAMEBUFFER_COMPLETE) {
      restore();
      snapshotPBOFailureReason_ =
          "OpenGL could not bind the Gazebo texture for snapshot readback";
      snapshotPBOAvailable_ = false;
      LogEncoderError(snapshotPBOFailureReason_ + "; using synchronous readback");
      return false;
    }
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, snapshotReadbackBuffer_);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ROW_LENGTH, 0);
    glPixelStorei(GL_PACK_SKIP_ROWS, 0);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
    const auto started = std::chrono::steady_clock::now();
    glReadPixels(
        0, 0, static_cast<GLsizei>(result.width),
        static_cast<GLsizei>(result.height), GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();
    const GLenum issueError = glGetError();
    restore();
    if (issueError != GL_NO_ERROR || fence == nullptr) {
      if (fence != nullptr) {
        glDeleteSync(fence);
      }
      snapshotPBOFailureReason_ =
          "OpenGL could not issue asynchronous snapshot readback";
      snapshotPBOAvailable_ = false;
      LogEncoderError(snapshotPBOFailureReason_ + "; using synchronous readback");
      return false;
    }

    {
      std::lock_guard<std::mutex> lock(snapshotMutex_);
      if (snapshot_.id != result.id ||
          snapshot_.generation != result.generation || snapshot_.completed) {
        glDeleteSync(fence);
        return true;
      }
      snapshot_.captureSubmitted = true;
    }
    result.jpegReadback = "opengl-pbo";
    snapshotReadbackResult_ = std::move(result);
    snapshotReadbackFence_ = fence;
    snapshotReadbackStarted_ = started;
    snapshotReadbackPending_.store(true);
    return true;
  }

  bool PollSnapshotPBOReadback() {
    if (!snapshotReadbackResult_ || snapshotReadbackFence_ == nullptr) {
      snapshotReadbackPending_.store(false);
      return false;
    }
    const GLenum status = glClientWaitSync(snapshotReadbackFence_, 0, 0);
    if (status == GL_TIMEOUT_EXPIRED) {
      return false;
    }
    SnapshotResult result = std::move(*snapshotReadbackResult_);
    snapshotReadbackResult_.reset();
    glDeleteSync(snapshotReadbackFence_);
    snapshotReadbackFence_ = nullptr;
    snapshotReadbackPending_.store(false);
    if (status == GL_WAIT_FAILED) {
      RearmSnapshotAfterPBOFailure(
          result, "OpenGL snapshot PBO fence wait failed");
      return false;
    }

    GLint previousBuffer = 0;
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &previousBuffer);
    while (glGetError() != GL_NO_ERROR) {
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, snapshotReadbackBuffer_);
    const auto *mapped = static_cast<const std::uint8_t *>(glMapBufferRange(
        GL_PIXEL_PACK_BUFFER, 0,
        static_cast<GLsizeiptr>(snapshotReadbackBufferBytes_),
        GL_MAP_READ_BIT));
    if (!mapped) {
      glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(previousBuffer));
      RearmSnapshotAfterPBOFailure(
          result, "OpenGL could not map the completed snapshot PBO");
      return false;
    }
    // Ogre's Gazebo RenderTexture GL storage already matches the H264/live
    // image row order. Flipping mapped rows here makes snapshots disagree
    // with Live and mirrors every AprilTag payload.
    const bool copied = gazebo_sim_camera::CopyGazeboPBOToImageOrder(
        mapped, snapshotReadbackBufferBytes_, result.width, result.height,
        &result.rgb);
    const GLboolean unmapped = glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(previousBuffer));
    if (!copied || unmapped != GL_TRUE || glGetError() != GL_NO_ERROR) {
      RearmSnapshotAfterPBOFailure(
          result, "OpenGL could not complete the snapshot PBO mapping");
      return false;
    }
    result.jpegReadbackMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - snapshotReadbackStarted_).count();
    if (!QueueSnapshotEncode(std::move(result))) {
      CompleteSnapshotFailure("snapshot JPEG worker queue is unavailable");
      return false;
    }
    return true;
  }

  void RearmSnapshotAfterPBOFailure(
      const SnapshotResult &result,
      const std::string &reason) {
    snapshotPBOFailureReason_ = reason;
    snapshotPBOAvailable_ = false;
    LogEncoderError(reason + "; recapturing through synchronous readback");
    std::lock_guard<std::mutex> lock(snapshotMutex_);
    if (snapshot_.id == result.id &&
        snapshot_.generation == result.generation && !snapshot_.completed) {
      snapshot_.captureSubmitted = false;
      snapshotRenderPassesToSkip_ = 0;
    }
  }

  void CompleteSnapshotFailure(const std::string &error) {
    {
      std::lock_guard<std::mutex> lock(snapshotMutex_);
      if (snapshot_.id.empty() || snapshot_.completed) {
        return;
      }
      snapshot_.failed = true;
      snapshot_.completed = true;
      snapshot_.error = error;
    }
    WakeControl();
  }

  bool StartSnapshotEncoder() {
    try {
      snapshotEncoderStopping_ = false;
      snapshotEncoderThread_ =
          std::thread(&XGCMediaCameraPlugin::SnapshotEncoderLoop, this);
      return true;
    } catch (const std::system_error &error) {
      gzerr << "xgc_media_camera could not start snapshot JPEG worker: "
            << error.what() << "\n";
      return false;
    }
  }

  void StopSnapshotEncoder() {
    {
      std::lock_guard<std::mutex> lock(snapshotEncoderMutex_);
      snapshotEncoderStopping_ = true;
      snapshotEncodeJob_.reset();
    }
    snapshotEncoderCondition_.notify_all();
    if (snapshotEncoderThread_.joinable()) {
      snapshotEncoderThread_.join();
    }
  }

  bool QueueSnapshotEncode(SnapshotResult result) {
    {
      std::lock_guard<std::mutex> lock(snapshotEncoderMutex_);
      if (snapshotEncoderStopping_ || snapshotEncodeJob_) {
        return false;
      }
      snapshotEncodeJob_ = std::move(result);
    }
    snapshotEncoderCondition_.notify_one();
    return true;
  }

  void SnapshotEncoderLoop() {
    while (true) {
      SnapshotResult result;
      {
        std::unique_lock<std::mutex> lock(snapshotEncoderMutex_);
        snapshotEncoderCondition_.wait(lock, [this] {
          return snapshotEncoderStopping_ || snapshotEncodeJob_.has_value();
        });
        if (snapshotEncoderStopping_) {
          return;
        }
        result = std::move(*snapshotEncodeJob_);
        snapshotEncodeJob_.reset();
      }

      gazebo_sim_camera::SnapshotJpegEncodeResult encoded;
      const bool tryHardware = snapshotJpegBackend_.useHardware &&
          !snapshotJpegHardwareFusedOff_.load();
      const auto encodeStarted = std::chrono::steady_clock::now();
      if (tryHardware) {
        encoded = snapshotJpegHardware_.Encode(
            result.rgb.data(), result.rgb.size(), result.width, result.height,
            result.jpegQuality);
      }
      if (!tryHardware ||
          (!encoded.error.empty() && snapshotJpegBackend_.allowCPUFallback)) {
        if (tryHardware && !encoded.error.empty()) {
          snapshotJpegHardwareFusedOff_.store(true);
          result.jpegFallbackReason =
              "hardware JPEG backend failed and was fused off: " +
              encoded.error;
          LogEncoderError(result.jpegFallbackReason);
        }
        encoded = gazebo_sim_camera::EncodeSnapshotJpegCPU(
            result.rgb.data(), result.rgb.size(), result.width, result.height,
            result.jpegQuality);
      }
      result.jpegEncodeMilliseconds = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - encodeStarted).count();
      result.jpegBackend = encoded.backend;
      if (snapshotJpegPolicy_ == gazebo_sim_camera::SnapshotJpegPolicy::kAuto &&
          snapshotJpegBackend_.hardwareState == "unavailable") {
        if (!result.jpegFallbackReason.empty()) {
          result.jpegFallbackReason += "; ";
        }
        result.jpegFallbackReason += "hardware backend unavailable at runtime preflight";
        if (!snapshotJpegHardwarePreflightError_.empty()) {
          result.jpegFallbackReason += ": " +
              snapshotJpegHardwarePreflightError_;
        }
      }
      result.jpeg = std::move(encoded.bytes);
      if (!result.includeRGB) {
        result.rgb.clear();
        result.rgb.shrink_to_fit();
      }
      if (!encoded.error.empty() || result.jpeg.empty()) {
        result.failed = true;
        result.error = encoded.error.empty()
            ? "camera snapshot JPEG encoding failed"
            : encoded.error;
        LogEncoderError(result.error);
      }
      result.completed = true;

      bool delivered = false;
      {
        std::lock_guard<std::mutex> lock(snapshotMutex_);
        if (snapshot_.id == result.id &&
            snapshot_.generation == result.generation &&
            !snapshot_.completed) {
          snapshot_ = std::move(result);
          delivered = true;
        }
      }
      if (delivered) {
        WakeControl();
      }
    }
  }

  std::int64_t SourceTimeNanoseconds() const {
    const common::Time measurementTime = sensor_->LastMeasurementTime();
    return static_cast<std::int64_t>(measurementTime.sec) *
               1'000'000'000LL +
           static_cast<std::int64_t>(measurementTime.nsec);
  }

  bool ROSRenderIsFreshForCurrentActivation() {
    const std::int64_t sourceTimeNanoseconds = SourceTimeNanoseconds();
    const auto decision = rosFreshRenderGate_.Observe(
        rosFreshRenderGeneration_.load(), sourceTimeNanoseconds);
    if (decision ==
        gazebo_sim_camera::FreshRenderDecision::kDiscardAndReset) {
      // The first callback after CameraSensor activation may still expose the
      // dormant texture. It must not feed either NVENC's delayed-output state
      // or the new ROS epoch, even when its stale AU happens to be an IDR.
      ClearRTPQueue();
      DestroyEncoder();
      forceKeyframe_.store(true);
      return false;
    }
    return decision == gazebo_sim_camera::FreshRenderDecision::kAccept;
  }

  void ObserveSourceTime(std::int64_t sourceTimeNanoseconds) {
    if (lastSourceTimeNanoseconds_ &&
        sourceTimeNanoseconds < *lastSourceTimeNanoseconds_) {
      ClearRTPQueue();
      BeginROSEpoch(
          xgc_camera_msgs::FrameTiming::
              DISCONTINUITY_SOURCE_TIME_RESET);
      forceKeyframe_.store(true);
      LogEncoderError(
          "simulation time moved backwards; starting a new stream epoch");
      // A delayed NVENC output still belongs to the previous simulation
      // timeline. Recreating the encoder is the only safe way to guarantee
      // that it cannot later be paired with metadata from the new epoch.
      DestroyEncoder();
    }
    lastSourceTimeNanoseconds_ = sourceTimeNanoseconds;
  }

  std::optional<PendingEncodedFrame> TakePendingEncodedFrame(
      std::uint64_t encoderTimestamp) {
    const auto found = std::find_if(
        pendingEncodedFrames_.begin(),
        pendingEncodedFrames_.end(),
        [encoderTimestamp](const PendingEncodedFrame &pending) {
          return pending.encoderTimestamp == encoderTimestamp;
        });
    if (found != pendingEncodedFrames_.end()) {
      PendingEncodedFrame result = *found;
      pendingEncodedFrames_.erase(found);
      return result;
    }
    return std::nullopt;
  }

  bool EncodeRenderedFrame() {
    if (!camera_ || !camera_->RenderTexture()) {
      return false;
    }
    auto *renderTexture = dynamic_cast<Ogre::GLTexture *>(camera_->RenderTexture());
    if (!renderTexture || renderTexture->getGLTextureTarget() != GL_TEXTURE_2D) {
      LogEncoderError("Gazebo camera render texture is not a GL_TEXTURE_2D texture");
      return false;
    }
    const unsigned int width = camera_->ImageWidth();
    const unsigned int height = camera_->ImageHeight();
    const std::int64_t sourceTimeNanoseconds = SourceTimeNanoseconds();
    ObserveSourceTime(sourceTimeNanoseconds);
    if (!EnsureEncoder(width, height)) {
      return false;
    }
    if (!CopyTextureToEncoderInput(renderTexture->getGLID(), width, height)) {
      return false;
    }
    NV_ENC_MAP_INPUT_RESOURCE mapping{};
    mapping.version = NV_ENC_MAP_INPUT_RESOURCE_VER;
    mapping.registeredResource = registeredInput_;
    if (api_.nvEncMapInputResource(encoder_, &mapping) != NV_ENC_SUCCESS) {
      LogEncoderError("NVENC could not map the OpenGL input texture");
      return false;
    }
    const bool forceKeyframe = forceKeyframe_.exchange(false);
    NV_ENC_PIC_PARAMS picture{};
    picture.version = NV_ENC_PIC_PARAMS_VER;
    picture.inputWidth = width;
    picture.inputHeight = height;
    picture.inputPitch = width;
    picture.inputBuffer = mapping.mappedResource;
    picture.outputBitstream = bitstream_;
    picture.bufferFmt = mapping.mappedBufferFmt;
    picture.pictureStruct = NV_ENC_PIC_STRUCT_FRAME;
    const double fps = std::max(0.1, static_cast<double>(sensor_->UpdateRate()));
    const std::uint64_t inputTimestamp = static_cast<std::uint64_t>(
        std::llround(static_cast<double>(encodedFrameIndex_) *
                     static_cast<double>(kRtpClockRate) / fps));
    picture.inputTimeStamp = inputTimestamp;
    picture.inputDuration = std::max<std::uint64_t>(
        1, static_cast<std::uint64_t>(
               std::llround(static_cast<double>(kRtpClockRate) / fps)));
    PendingEncodedFrame pending;
    pending.encoderTimestamp = inputTimestamp;
    pending.sourceTimeNanoseconds = sourceTimeNanoseconds;
    pending.sourceSequence = sourceFrameSequence_++;
    if (forceKeyframe) {
      if (awaitingNativeControl_ && awaitingNativeControl_->kind == 3) pending.keyframeControlRevision = awaitingNativeControl_->revision;
      pending.captureKeyframeGeneration = nativeCaptureKeyframePending_.load();
    }
    pendingEncodedFrames_.push_back(pending);
    if (forceKeyframe) {
      picture.encodePicFlags = NV_ENC_PIC_FLAG_FORCEIDR | NV_ENC_PIC_FLAG_OUTPUT_SPSPPS;
    }
    const NVENCSTATUS encodeStatus = api_.nvEncEncodePicture(encoder_, &picture);
    if (encodeStatus != NV_ENC_SUCCESS && encodeStatus != NV_ENC_ERR_NEED_MORE_INPUT) {
      api_.nvEncUnmapInputResource(encoder_, mapping.mappedResource);
      LogEncoderError("NVENC could not encode the OpenGL frame");
      ClearRTPQueue();
      BeginROSEpoch(
          xgc_camera_msgs::FrameTiming::DISCONTINUITY_ENCODER_RESET);
      forceKeyframe_.store(true);
      DestroyEncoder();
      return false;
    }
    bool sent = true;
    bool recreateEncoder = false;
    if (encodeStatus == NV_ENC_SUCCESS) {
      NV_ENC_LOCK_BITSTREAM locked{};
      locked.version = NV_ENC_LOCK_BITSTREAM_VER;
      locked.outputBitstream = bitstream_;
      if (api_.nvEncLockBitstream(encoder_, &locked) != NV_ENC_SUCCESS) {
        sent = false;
        LogEncoderError("NVENC could not lock the H264 bitstream");
        ClearRTPQueue();
        BeginROSEpoch(
            xgc_camera_msgs::FrameTiming::DISCONTINUITY_ENCODER_RESET);
        forceKeyframe_.store(true);
        recreateEncoder = true;
      } else {
        const auto encodedMetadata =
            TakePendingEncodedFrame(locked.outputTimeStamp);
        if (!encodedMetadata) {
          sent = false;
          LogEncoderError(
              "NVENC output timestamp has no source-frame timing metadata");
          ClearRTPQueue();
          BeginROSEpoch(
              xgc_camera_msgs::FrameTiming::
                  DISCONTINUITY_ENCODER_RESET);
          forceKeyframe_.store(true);
          recreateEncoder = true;
        } else {
          sent = SendH264AccessUnit(
              static_cast<const std::uint8_t *>(locked.bitstreamBufferPtr),
              locked.bitstreamSizeInBytes,
              static_cast<std::uint32_t>(locked.outputTimeStamp),
              *encodedMetadata);
        }
        api_.nvEncUnlockBitstream(encoder_, bitstream_);
      }
    } else if (pendingEncodedFrames_.size() >
               kMaximumPendingEncodedFrames) {
      // This source is deliberately configured without B-frames, lookahead,
      // or asynchronous encode. A bounded grace window tolerates a transient
      // delayed output, but an unbounded metadata queue would eventually make
      // source-time pairing ambiguous and reuse a still-in-flight GL surface.
      sent = false;
      LogEncoderError(
          "NVENC retained too many delayed frames; rebuilding the encoder");
      ClearRTPQueue();
      BeginROSEpoch(
          xgc_camera_msgs::FrameTiming::DISCONTINUITY_ENCODER_RESET);
      forceKeyframe_.store(true);
      recreateEncoder = true;
    }
    api_.nvEncUnmapInputResource(encoder_, mapping.mappedResource);
    if (recreateEncoder) {
      DestroyEncoder();
      return false;
    }
    ++encodedFrameIndex_;
    return sent;
  }

  bool EnsureEncoder(unsigned int width, unsigned int height) {
    if (encoder_ && encoderWidth_ == width && encoderHeight_ == height) {
      return true;
    }
    DestroyEncoder();
    if (width == 0 || height == 0 || !EnsureConversionTexture(width, height)) {
      return false;
    }
    nvencLibrary_ = dlopen("libnvidia-encode.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!nvencLibrary_) {
      LogEncoderError("libnvidia-encode.so.1 is unavailable; no CPU fallback is permitted");
      return false;
    }
    const auto createInstance = reinterpret_cast<NvEncodeAPICreateInstance>(dlsym(nvencLibrary_, "NvEncodeAPICreateInstance"));
    if (!createInstance) {
      LogEncoderError("NvEncodeAPICreateInstance is unavailable");
      DestroyEncoder();
      return false;
    }
    api_ = {};
    api_.version = NV_ENCODE_API_FUNCTION_LIST_VER;
    if (createInstance(&api_) != NV_ENC_SUCCESS || !api_.nvEncOpenEncodeSessionEx || !api_.nvEncInitializeEncoder || !api_.nvEncGetEncodePresetConfigEx) {
      LogEncoderError("NVENC API initialization failed");
      DestroyEncoder();
      return false;
    }
    NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS session{};
    session.version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER;
    session.deviceType = NV_ENC_DEVICE_TYPE_OPENGL;
    session.device = nullptr;  // The current GL context owns the device on Linux.
    session.apiVersion = NVENCAPI_VERSION;
    if (api_.nvEncOpenEncodeSessionEx(&session, &encoder_) != NV_ENC_SUCCESS) {
      LogEncoderError("NVENC could not open an OpenGL encode session");
      DestroyEncoder();
      return false;
    }
    NV_ENC_PRESET_CONFIG preset{};
    preset.version = NV_ENC_PRESET_CONFIG_VER;
    preset.presetCfg.version = NV_ENC_CONFIG_VER;
    if (api_.nvEncGetEncodePresetConfigEx(encoder_, NV_ENC_CODEC_H264_GUID, NV_ENC_PRESET_P4_GUID, NV_ENC_TUNING_INFO_LOW_LATENCY, &preset) != NV_ENC_SUCCESS) {
      LogEncoderError("NVENC low-latency H264 preset is unavailable");
      DestroyEncoder();
      return false;
    }
    const double fps = std::max(0.1, static_cast<double>(sensor_->UpdateRate()));
    const std::uint32_t fpsNumerator = static_cast<std::uint32_t>(std::llround(fps * 1'000.0));
    NV_ENC_CONFIG configuration = preset.presetCfg;
    configuration.version = NV_ENC_CONFIG_VER;
    configuration.profileGUID = NV_ENC_H264_PROFILE_BASELINE_GUID;
    configuration.gopLength = std::max<std::uint32_t>(1, static_cast<std::uint32_t>(std::llround(fps * 2.0)));
    configuration.frameIntervalP = 1;
    configuration.rcParams.version = NV_ENC_RC_PARAMS_VER;
    // WebRTC needs a bounded *average* rate but an IDR legitimately needs
    // more bits than a delta frame. Strict one-frame CBR made every two-second
    // IDR visibly blur at 4K. VBR with an explicit peak, paired with the RTP
    // pacer above, preserves IDR quality without emitting UDP microbursts.
    configuration.rcParams.rateControlMode = NV_ENC_PARAMS_RC_VBR;
    configuration.rcParams.averageBitRate = static_cast<std::uint32_t>(bitrate_);
    configuration.rcParams.maxBitRate = static_cast<std::uint32_t>(maxBitrate_);
    const std::uint64_t vbvBits = static_cast<std::uint64_t>(maxBitrate_) * static_cast<std::uint64_t>(vbvBufferMilliseconds_) / 1'000ULL;
    configuration.rcParams.vbvBufferSize = static_cast<std::uint32_t>(std::min<std::uint64_t>(std::numeric_limits<std::uint32_t>::max(), std::max<std::uint64_t>(1, vbvBits)));
    configuration.rcParams.vbvInitialDelay = configuration.rcParams.vbvBufferSize;
    configuration.rcParams.enableAQ = 1;
    configuration.rcParams.aqStrength = 8;
    configuration.rcParams.strictGOPTarget = 0;
    configuration.rcParams.enableLookahead = 0;
    // WebRTC requires presentation timestamps to stay in decode order.  The
    // low-latency preset normally supplies this, but leaving it implicit lets
    // driver/preset changes advertise a reorder window in SPS/VUI even though
    // frameIntervalP=1 emits IPP only.
    configuration.rcParams.zeroReorderDelay = 1;
    configuration.encodeCodecConfig.h264Config.repeatSPSPPS = 1;
    configuration.encodeCodecConfig.h264Config.outputAUD = 1;
    configuration.encodeCodecConfig.h264Config.level = NV_ENC_LEVEL_H264_51;
    configuration.encodeCodecConfig.h264Config.idrPeriod = configuration.gopLength;
    configuration.encodeCodecConfig.h264Config.entropyCodingMode = NV_ENC_H264_ENTROPY_CODING_MODE_CAVLC;
    NV_ENC_INITIALIZE_PARAMS initialization{};
    initialization.version = NV_ENC_INITIALIZE_PARAMS_VER;
    initialization.encodeGUID = NV_ENC_CODEC_H264_GUID;
    initialization.presetGUID = NV_ENC_PRESET_P4_GUID;
    initialization.tuningInfo = NV_ENC_TUNING_INFO_LOW_LATENCY;
    initialization.encodeWidth = width;
    initialization.encodeHeight = height;
    initialization.darWidth = width;
    initialization.darHeight = height;
    initialization.frameRateNum = std::max<std::uint32_t>(1, fpsNumerator);
    initialization.frameRateDen = 1'000;
    initialization.enableEncodeAsync = 0;
    initialization.enablePTD = 1;
    initialization.encodeConfig = &configuration;
    if (api_.nvEncInitializeEncoder(encoder_, &initialization) != NV_ENC_SUCCESS) {
      LogEncoderError("NVENC could not initialize H264 encoding");
      DestroyEncoder();
      return false;
    }
    NV_ENC_INPUT_RESOURCE_OPENGL_TEX texture{};
    texture.texture = encoderTexture_;
    texture.target = GL_TEXTURE_2D;
    NV_ENC_REGISTER_RESOURCE resource{};
    resource.version = NV_ENC_REGISTER_RESOURCE_VER;
    resource.resourceType = NV_ENC_INPUT_RESOURCE_TYPE_OPENGL_TEX;
    resource.width = width;
    resource.height = height;
    resource.pitch = width * 4;
    resource.resourceToRegister = &texture;
    // GL_RGBA8 stores R,G,B,A bytes. NVENC names its packed formats by the
    // 32-bit word layout; on little-endian hosts A8B8G8R8 (ABGR) is therefore
    // the matching R,G,B,A byte layout. ARGB would reinterpret those bytes as
    // B,G,R,A and swap red with blue in the WebRTC stream.
    resource.bufferFormat = NV_ENC_BUFFER_FORMAT_ABGR;
    resource.bufferUsage = NV_ENC_INPUT_IMAGE;
    if (api_.nvEncRegisterResource(encoder_, &resource) != NV_ENC_SUCCESS) {
      LogEncoderError("NVENC could not register the OpenGL texture");
      DestroyEncoder();
      return false;
    }
    registeredInput_ = resource.registeredResource;
    NV_ENC_CREATE_BITSTREAM_BUFFER bitstream{};
    bitstream.version = NV_ENC_CREATE_BITSTREAM_BUFFER_VER;
    if (api_.nvEncCreateBitstreamBuffer(encoder_, &bitstream) != NV_ENC_SUCCESS) {
      LogEncoderError("NVENC could not allocate an H264 bitstream buffer");
      DestroyEncoder();
      return false;
    }
    bitstream_ = bitstream.bitstreamBuffer;
    encoderWidth_ = width;
    encoderHeight_ = height;
    // encodedFrameIndex_ is the transport clock, not encoder-local state.
    // Gazebo can move simulation time backwards while inserting a model. That
    // deliberately rebuilds NVENC above, but the loopback RTP source and its
    // WebRTC readers remain alive. Resetting this counter here would move the
    // RTP/PTS clock backwards and MediaMTX would terminate the first browser
    // session as an apparent H264 B-frame stream. Keep it monotonic for the
    // entire camera-plugin lifetime; a uint32 RTP wrap is handled normally by
    // the receiver.
    pendingEncodedFrames_.clear();
    const bool epochAlreadyPending = ROSHasPendingEpoch();
    if (encoderEverInitialized_ && !epochAlreadyPending) {
      ClearRTPQueue();
      BeginROSEpoch(
          xgc_camera_msgs::FrameTiming::DISCONTINUITY_ENCODER_RESET);
    }
    encoderEverInitialized_ = true;
    {
      std::lock_guard<std::mutex> lock(rosPublisherMutex_);
      streamWidth_ = width;
      streamHeight_ = height;
      streamFrameRate_ = fps;
      streamKeyframeIntervalFrames_ = configuration.gopLength;
      rosStreamInfoPending_ = rosPublishingEnabled_;
    }
    rosPublisherCondition_.notify_one();
    forceKeyframe_.store(true);
    encoderResourcesActive_.store(true);
    return true;
  }

  bool EnsureConversionTexture(unsigned int width, unsigned int height) {
    if (!EnsureGLEWDispatch()) {
      return false;
    }
    if (encoderTexture_ != 0 && encoderTextureWidth_ == width && encoderTextureHeight_ == height) {
      return true;
    }
    DestroyConversionTexture();
    glGenTextures(1, &encoderTexture_);
    glBindTexture(GL_TEXTURE_2D, encoderTexture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glBindTexture(GL_TEXTURE_2D, 0);
    glGenFramebuffers(1, &readFramebuffer_);
    glGenFramebuffers(1, &drawFramebuffer_);
    if (encoderTexture_ == 0 || readFramebuffer_ == 0 || drawFramebuffer_ == 0 || glGetError() != GL_NO_ERROR) {
      LogEncoderError("OpenGL could not allocate the GPU-only NVENC conversion texture");
      DestroyConversionTexture();
      return false;
    }
    encoderTextureWidth_ = width;
    encoderTextureHeight_ = height;
    return true;
  }

  bool CopyTextureToEncoderInput(GLuint sourceTexture, unsigned int width, unsigned int height) {
    glBindFramebuffer(GL_READ_FRAMEBUFFER, readFramebuffer_);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, sourceTexture, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, drawFramebuffer_);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, encoderTexture_, 0);
    if (glCheckFramebufferStatus(GL_READ_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE ||
        glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      LogEncoderError("OpenGL could not bind Gazebo and NVENC conversion framebuffers");
      return false;
    }
    // The source and destination stay entirely on the GPU. GL's framebuffer
    // coordinate convention also keeps this copy aligned with the OGRE pixel
    // readback used by the snapshot transaction. NVENC's registered-resource
    // map does not guarantee it waits for an unrelated GL blit submitted with
    // glFlush, so finish the copy before mapping; otherwise WebRTC remains one
    // rendered frame behind the snapshot/annotation path.
    glBlitFramebuffer(0, 0, width, height, 0, 0, width, height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glFinish();
    if (glGetError() != GL_NO_ERROR) {
      LogEncoderError("OpenGL GPU texture copy for NVENC failed");
      return false;
    }
    return true;
  }

  void DestroyEncoder() {
    if (encoder_) {
      if (registeredInput_ && api_.nvEncUnregisterResource) {
        api_.nvEncUnregisterResource(encoder_, registeredInput_);
      }
      registeredInput_ = nullptr;
      if (bitstream_ && api_.nvEncDestroyBitstreamBuffer) {
        api_.nvEncDestroyBitstreamBuffer(encoder_, bitstream_);
      }
      bitstream_ = nullptr;
      if (api_.nvEncDestroyEncoder) {
        api_.nvEncDestroyEncoder(encoder_);
      }
      encoder_ = nullptr;
    }
    api_ = {};
    if (nvencLibrary_) {
      dlclose(nvencLibrary_);
      nvencLibrary_ = nullptr;
    }
    encoderWidth_ = 0;
    encoderHeight_ = 0;
    pendingEncodedFrames_.clear();
    lastSourceTimeNanoseconds_.reset();
    encoderResourcesActive_.store(false);
    DestroyConversionTexture();
  }

  void DestroyConversionTexture() {
    if (readFramebuffer_ != 0) {
      glDeleteFramebuffers(1, &readFramebuffer_);
      readFramebuffer_ = 0;
    }
    if (drawFramebuffer_ != 0) {
      glDeleteFramebuffers(1, &drawFramebuffer_);
      drawFramebuffer_ = 0;
    }
    if (encoderTexture_ != 0) {
      glDeleteTextures(1, &encoderTexture_);
      encoderTexture_ = 0;
    }
    encoderTextureWidth_ = 0;
    encoderTextureHeight_ = 0;
  }

  bool SendH264AccessUnit(
      const std::uint8_t *data,
      std::size_t size,
      std::uint32_t timestamp,
      const PendingEncodedFrame &metadata) {
    if (!data || size == 0) {
      return false;
    }
    const auto nalUnits = SplitAnnexB(data, size);
    if (nalUnits.empty()) {
      LogEncoderError("NVENC produced a non-Annex-B H264 access unit");
      return false;
    }
    const bool keyframe = std::any_of(
        nalUnits.begin(), nalUnits.end(), [](const auto &nal) {
          return !nal.empty() && (nal.front() & 0x1f) == 5;
        });
    // Only a real native IDR output paired by NVENC outputTimeStamp can
    // acknowledge a force-IDR operation. Accepted input/NEED_MORE_INPUT alone
    // does not complete it; lock/output failure retains native ownership.
    if (keyframe) {
      if (metadata.keyframeControlRevision && awaitingNativeControl_ &&
          awaitingNativeControl_->kind == 3 && awaitingNativeControl_->revision == metadata.keyframeControlRevision) {
        CompleteNativeControl(*awaitingNativeControl_); awaitingNativeControl_.reset();
      }
      if (metadata.captureKeyframeGeneration && nativeCaptureKeyframePending_.load() == metadata.captureKeyframeGeneration) {
        nativeCaptureKeyframePending_.store(0); nativeCaptureKeyframeApplied_.store(metadata.captureKeyframeGeneration); WakeControl();
      }
    }
    EnqueueROSAccessUnit(data, size, timestamp, keyframe, metadata);
    if (!desiredActive_.load() || rtpSocket_ < 0) {
      return true;
    }

    std::vector<QueuedRTPPacket> packets;
    for (std::size_t index = 0; index < nalUnits.size(); ++index) {
      const auto &nal = nalUnits[index];
      if (nal.empty()) {
        continue;
      }
      const bool lastNAL = index + 1 == nalUnits.size();
      if (nal.size() <= kMaximumRtpPayloadBytes) {
        if (!AppendRTPPacket(packets, nal.data(), nal.size(), timestamp, lastNAL)) {
          return false;
        }
        continue;
      }
      const std::uint8_t nalHeader = nal.front();
      const std::uint8_t fuIndicator = static_cast<std::uint8_t>((nalHeader & 0xe0) | 28);
      const std::uint8_t nalType = static_cast<std::uint8_t>(nalHeader & 0x1f);
      std::size_t offset = 1;
      const std::size_t fragmentPayload = kMaximumRtpPayloadBytes - 2;
      bool first = true;
      while (offset < nal.size()) {
        const std::size_t count = std::min(fragmentPayload, nal.size() - offset);
        const bool last = offset + count == nal.size();
        std::vector<std::uint8_t> fragment;
        fragment.reserve(count + 2);
        fragment.push_back(fuIndicator);
        fragment.push_back(static_cast<std::uint8_t>(nalType | (first ? 0x80 : 0) | (last ? 0x40 : 0)));
        fragment.insert(fragment.end(), nal.begin() + static_cast<std::ptrdiff_t>(offset),
                        nal.begin() + static_cast<std::ptrdiff_t>(offset + count));
        if (!AppendRTPPacket(packets, fragment.data(), fragment.size(), timestamp, lastNAL && last)) {
          return false;
        }
        offset += count;
        first = false;
      }
    }
    return EnqueueRTPAccessUnit(std::move(packets), keyframe);
  }

  bool AppendRTPPacket(std::vector<QueuedRTPPacket> &packets, const std::uint8_t *payload,
                       std::size_t payloadSize, std::uint32_t timestamp, bool marker) {
    if (!payload || payloadSize == 0 || payloadSize > kMaximumRtpPayloadBytes) {
      return false;
    }
    QueuedRTPPacket packet;
    packet.bytes.resize(12 + payloadSize);
    packet.bytes[0] = 0x80;
    packet.bytes[1] = static_cast<std::uint8_t>(kH264PayloadType | (marker ? 0x80 : 0));
    const std::uint32_t networkTimestamp = htonl(timestamp);
    const std::uint32_t ssrc = htonl(kRtpSSRC);
    std::memcpy(packet.bytes.data() + 4, &networkTimestamp, sizeof(networkTimestamp));
    std::memcpy(packet.bytes.data() + 8, &ssrc, sizeof(ssrc));
    std::memcpy(packet.bytes.data() + 12, payload, payloadSize);
    packets.push_back(std::move(packet));
    return true;
  }

  bool StartDiagnosticReporter() {
    try {
      diagnosticStopping_ = false;
      diagnosticThread_ =
          std::thread(&XGCMediaCameraPlugin::DiagnosticReporterLoop, this);
      return true;
    } catch (const std::system_error &error) {
      gzerr << "xgc_media_camera could not start diagnostic reporter: "
            << error.what() << "\n";
      return false;
    }
  }

  void StopDiagnosticReporter() {
    {
      std::lock_guard<std::mutex> lock(diagnosticMutex_);
      diagnosticStopping_ = true;
      pendingDiagnostics_.clear();
    }
    diagnosticCondition_.notify_all();
    if (diagnosticThread_.joinable()) {
      diagnosticThread_.join();
    }
  }

  void DiagnosticReporterLoop() {
    while (true) {
      std::string message;
      {
        std::unique_lock<std::mutex> lock(diagnosticMutex_);
        diagnosticCondition_.wait(lock, [this] {
          return diagnosticStopping_ || !pendingDiagnostics_.empty();
        });
        if (diagnosticStopping_) {
          return;
        }
        message = std::move(pendingDiagnostics_.front());
        pendingDiagnostics_.pop_front();
      }
      const auto now = std::chrono::steady_clock::now();
      if (message != lastEncoderError_ ||
          now - lastEncoderErrorAt_ > std::chrono::seconds(5)) {
        // Gazebo console output can acquire global logging locks and perform
        // file I/O, so it is confined to this worker and never executed in
        // Ogre::RenderTargetListener.
        gzwarn << "xgc_media_camera: " << message << "\n";
        lastEncoderError_ = message;
        lastEncoderErrorAt_ = now;
      }
    }
  }

  void LogEncoderError(const std::string &message) {
    // This method is called from both worker threads and the render callback.
    // A contended diagnostics lock must never stall Gazebo rendering.
    std::unique_lock<std::mutex> lock(
        diagnosticMutex_, std::try_to_lock);
    if (!lock.owns_lock() || diagnosticStopping_) {
      return;
    }
    if (pendingDiagnostics_.size() >= kMaximumPendingDiagnostics) {
      pendingDiagnostics_.pop_front();
    }
    pendingDiagnostics_.push_back(message);
    lock.unlock();
    diagnosticCondition_.notify_one();
  }

  sensors::CameraSensorPtr sensor_;
  rendering::CameraPtr camera_;
  event::ConnectionPtr postRenderConnection_;
  Ogre::RenderTarget *renderTarget_ = nullptr;

  std::string sourceID_;
  std::string frameID_;
  std::string snapshotPoseFrameID_;
  std::string rtpHost_;
  int rtpPort_ = 0;
  std::string controlSocketPath_;
  int bitrate_ = 0;
  int maxBitrate_ = 0;
  int pacingBitrate_ = 0;
  int vbvBufferMilliseconds_ = 500;
  std::atomic<int> jpegQuality_{90};
  gazebo_sim_camera::SnapshotJpegPolicy snapshotJpegPolicy_ =
      gazebo_sim_camera::SnapshotJpegPolicy::kAuto;
  gazebo_sim_camera::SnapshotJpegBackendDecision snapshotJpegBackend_;
  gazebo_sim_camera::SnapshotJpegHardwareBackend snapshotJpegHardware_;
  std::string snapshotJpegHardwarePreflightError_;
  std::atomic<bool> snapshotJpegHardwareFusedOff_{false};
  bool rosPublishingEnabled_ = true;
  std::string rosVideoTopic_;
  std::string rosFrameTimingTopic_;
  std::string rosStreamInfoTopic_;
  std::size_t rosPublisherQueueCapacity_ =
      kDefaultROSPublisherQueueCapacity;

  std::atomic<bool> stopping_{false};
  std::atomic<bool> desiredActive_{false};
  std::atomic<bool> nativeAppliedActive_{false}, rtpInFlight_{false};
  std::atomic<unsigned int> nativeLifecycleState_{0};
  std::atomic<bool> forceKeyframe_{true};
  std::atomic<bool> cleanupEncoderRequested_{false};
  std::atomic<bool> encoderResourcesActive_{false};
  std::atomic<std::uint64_t> rosFreshRenderGeneration_{0};
  gazebo_sim_camera::FreshRenderGate rosFreshRenderGate_;
  struct NativeControlCommand {
    std::uint64_t revision;
    bool active;
    int jpegQuality;
    bool keyframe;
    std::optional<NativeCalibrationMetadata> calibration;
    int kind = 0;
  };
  struct PendingControl {
    gazebo_sim_camera::ControlReply reply;
    std::string requestID, operation;
    gazebo_sim_camera::ControlClock::time_point deadline;
    std::uint64_t revision = 0;
    bool calibration = false;
  };
  struct ControlWatcher {
    gazebo_sim_camera::ControlReply reply;
    gazebo_sim_camera::ControlClock::time_point deadline;
    std::uint64_t after;
  };
  std::atomic<gazebo_sim_camera::CameraSourceControlHost *> controlWakeHost_{nullptr};
  std::shared_ptr<gazebo_sim_camera::CameraSourceControlHost> controlHost_;
  std::string controlInstanceID_;
  std::string controlTargetID_;
  bool controlRegistered_ = false;
  std::optional<PendingControl> pendingControl_;
  std::array<std::optional<ControlWatcher>, 4> controlWatchers_{};
  std::array<Json::Value, 32> controlReceipts_{};
  std::size_t nextControlReceipt_ = 0;
  std::uint64_t desiredRevision_ = 1, appliedRevision_ = 1, controlEventRevision_ = 0;
  Json::Value desiredConfig_, appliedConfig_;
  std::int64_t appliedTimeNanoseconds_ = 0;
  std::mutex nativeCommandMutex_;
  std::optional<NativeControlCommand> nativeCommand_;
  std::optional<NativeControlCommand> awaitingNativeControl_;
  std::uint64_t renderFrameSequence_ = 0, captureKeyframeGeneration_ = 0;
  std::atomic<std::uint64_t> nativeFailedRevision_{0}, nativeCaptureKeyframePending_{0}, nativeCaptureKeyframeApplied_{0}, nativeCaptureKeyframeFailed_{0};
  std::atomic<std::uint64_t> nativeAppliedRevision_{1};
  std::atomic<std::int64_t> nativeAppliedTime_{0};
  std::atomic<bool> nativeSensorActive_{false};
  bool captureSlotHeld_ = false;
  std::optional<gazebo_sim_camera::ControlReply> captureReply_;
  std::string captureRequestID_;
  gazebo_sim_camera::ControlClock::time_point captureDeadline_;
  mutable std::mutex snapshotMutex_;
  SnapshotResult snapshot_;
  std::uint64_t snapshotGeneration_ = 0;
  unsigned int snapshotRenderPassesToSkip_ = 0;
  std::atomic<bool> snapshotReadbackPending_{false};
  bool snapshotPBOAvailable_ = true;
  std::string snapshotPBOFailureReason_;
  GLuint snapshotReadbackBuffer_ = 0;
  GLuint snapshotReadFramebuffer_ = 0;
  GLsync snapshotReadbackFence_ = nullptr;
  std::size_t snapshotReadbackBufferBytes_ = 0;
  std::optional<SnapshotResult> snapshotReadbackResult_;
  std::chrono::steady_clock::time_point snapshotReadbackStarted_;
  std::mutex snapshotEncoderMutex_;
  std::condition_variable snapshotEncoderCondition_;
  std::optional<SnapshotResult> snapshotEncodeJob_;
  std::thread snapshotEncoderThread_;
  bool snapshotEncoderStopping_ = false;

  int rtpSocket_ = -1;
  sockaddr_in rtpDestination_{};
  std::uint16_t rtpSequence_ = 0;
  std::mutex rtpPacerMutex_;
  std::condition_variable rtpPacerCondition_;
  std::deque<QueuedRTPAccessUnit> rtpQueue_;
  std::thread rtpPacerThread_;
  bool rtpPacerStopping_ = false;
  std::size_t rtpQueuedBytes_ = 0;
  std::uint64_t rtpQueueGeneration_ = 0;

  std::unique_ptr<ros::NodeHandle> rosNode_;
  ros::Publisher rosVideoPublisher_;
  ros::Publisher rosFrameTimingPublisher_;
  ros::Publisher rosStreamInfoPublisher_;
  ros::Publisher rosCameraInfoPublisher_;
  std::string rosCameraInfoTopic_;
  std::shared_ptr<CameraROSDataRuntime> rosDataRuntime_;
  NativeCalibrationMetadata calibrationMetadata_;
  bool rosCameraInfoPending_ = false;
  std::atomic<std::uint64_t> publishedCalibrationRevision_{0};
  std::atomic<bool> rosConsumersActive_{false};
  std::mutex rosConsumerMutex_;
  bool rosSubscriberCallbacksEnabled_ = false;
  std::size_t rosSubscriberConnectionCount_ = 0;
  mutable std::mutex rosPublisherMutex_;
  std::condition_variable rosPublisherCondition_;
  std::deque<QueuedROSFrame> rosFrameQueue_;
  std::thread rosPublisherThread_;
  bool rosPublisherStopping_ = false;
  bool rosStreamInfoPending_ = false;
  bool rosWaitingForIDR_ = true;
  std::uint64_t rosDroppedFramesBeforeIDR_ = 0;
  std::uint64_t rosPublisherGeneration_ = 0;
  std::uint64_t streamEpoch_ = 1;
  std::uint64_t nextPublishedFrameSequence_ = 0;
  std::uint8_t pendingDiscontinuity_ =
      xgc_camera_msgs::FrameTiming::DISCONTINUITY_STREAM_START;
  unsigned int streamWidth_ = 0;
  unsigned int streamHeight_ = 0;
  double streamFrameRate_ = 0.0;
  std::uint32_t streamKeyframeIntervalFrames_ = 0;

  void *nvencLibrary_ = nullptr;
  NV_ENCODE_API_FUNCTION_LIST api_{};
  void *encoder_ = nullptr;
  NV_ENC_REGISTERED_PTR registeredInput_ = nullptr;
  NV_ENC_OUTPUT_PTR bitstream_ = nullptr;
  GLuint encoderTexture_ = 0;
  GLuint readFramebuffer_ = 0;
  GLuint drawFramebuffer_ = 0;
  unsigned int encoderTextureWidth_ = 0;
  unsigned int encoderTextureHeight_ = 0;
  unsigned int encoderWidth_ = 0;
  unsigned int encoderHeight_ = 0;
  std::uint64_t encodedFrameIndex_ = 0;
  std::uint64_t sourceFrameSequence_ = 0;
  std::deque<PendingEncodedFrame> pendingEncodedFrames_;
  std::optional<std::int64_t> lastSourceTimeNanoseconds_;
  bool encoderEverInitialized_ = false;
  bool glewInitialized_ = false;

  std::mutex diagnosticMutex_;
  std::condition_variable diagnosticCondition_;
  std::deque<std::string> pendingDiagnostics_;
  std::thread diagnosticThread_;
  bool diagnosticStopping_ = true;
  std::string lastEncoderError_;
  std::chrono::steady_clock::time_point lastEncoderErrorAt_{};
};

GZ_REGISTER_SENSOR_PLUGIN(XGCMediaCameraPlugin)

}  // namespace gazebo
