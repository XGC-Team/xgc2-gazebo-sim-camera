#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace gazebo_sim_camera {

inline constexpr const char *kDefaultSnapshotJpegHardwareLibrary =
    "libxgc_gazebo_snapshot_jpeg_hardware.so";

// Operator intent stays vendor-neutral. "hardware" is strict so a deployment
// can prove acceleration instead of silently benchmarking the CPU fallback.
enum class SnapshotJpegPolicy {
  kAuto,
  kHardware,
  kCPU,
};

std::optional<SnapshotJpegPolicy> ParseSnapshotJpegPolicy(
    const std::string &value);
const char *SnapshotJpegPolicyName(SnapshotJpegPolicy policy);

struct SnapshotJpegBackendDecision {
  bool available = false;
  bool useHardware = false;
  bool allowCPUFallback = false;
  std::string backend;
  std::string hardwareState;
  std::string error;
};

SnapshotJpegBackendDecision SelectSnapshotJpegBackend(
    SnapshotJpegPolicy policy,
    bool hardwareAvailable,
    const std::string &hardwareBackend = "nvjpeg-cuda");

struct SnapshotJpegEncodeResult {
  std::vector<std::uint8_t> bytes;
  std::string backend;
  std::string error;
};

class SnapshotJpegHardwareBackend {
 public:
  SnapshotJpegHardwareBackend() = default;
  ~SnapshotJpegHardwareBackend();
  SnapshotJpegHardwareBackend(const SnapshotJpegHardwareBackend &) = delete;
  SnapshotJpegHardwareBackend &operator=(
      const SnapshotJpegHardwareBackend &) = delete;

  bool Open(
      const std::string &library = kDefaultSnapshotJpegHardwareLibrary);
  void Close();
  bool available() const;
  const std::string &backend() const;
  const std::string &error() const;
  SnapshotJpegEncodeResult Encode(
      const std::uint8_t *rgb,
      std::size_t size,
      unsigned int width,
      unsigned int height,
      int quality);

 private:
  void *library_ = nullptr;
  void *encoderStorage_ = nullptr;
  std::string backend_;
  std::string error_;
};

// Portable final fallback. Ubuntu's libjpeg ABI is backed by libjpeg-turbo on
// every supported Focal target, so this retains SIMD acceleration without a
// second runtime dependency or a CUDA requirement.
SnapshotJpegEncodeResult EncodeSnapshotJpegCPU(
    const std::uint8_t *rgb,
    std::size_t size,
    unsigned int width,
    unsigned int height,
    int quality);

bool CopyGazeboPBOToImageOrder(
    const std::uint8_t *renderTextureRows,
    std::size_t size,
    unsigned int width,
    unsigned int height,
    std::vector<std::uint8_t> *imageRows);

}  // namespace gazebo_sim_camera
