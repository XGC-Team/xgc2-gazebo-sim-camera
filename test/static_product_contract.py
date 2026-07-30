#!/usr/bin/env python3
from pathlib import Path

import yaml


root = Path(__file__).resolve().parents[1]
xacro = (root / "urdf/fixed_rgb_camera.urdf.xacro").read_text(encoding="utf-8")
launch = (root / "launch/static_camera.launch").read_text(encoding="utf-8")
intrinsic = (root / "launch/intrinsic_calibration_world.launch").read_text(
    encoding="utf-8"
)
extrinsic = (root / "launch/extrinsic_calibration_world.launch").read_text(
    encoding="utf-8"
)
keepalive = (root / "scripts/camera_lifecycle_keepalive.py").read_text(
    encoding="utf-8"
)
camera_contract = (root / "scripts/camera_contract_publisher.py").read_text(
    encoding="utf-8"
)
media_plugin = (root / "src/xgc_media_camera_plugin.cpp").read_text(
    encoding="utf-8"
)
cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
docker_build = (root / ".xgc2/scripts/build_debs_in_docker.sh").read_text(
    encoding="utf-8"
)
profiles = yaml.safe_load(
    (root / "config/world_camera_profiles.yaml").read_text(encoding="utf-8")
)

assert profiles["schema_version"] == 1
assert profiles["default_profile"] == "world_ultrawide_4k30_130"
assert list(profiles["profiles"]) == [
    "world_standard_720p30",
    "world_standard_1080p30",
    "world_ultrawide_4k30_130",
    "calibration_standard_720p20",
]

assert "<gazebo><static>true</static></gazebo>" in xacro
assert '<xacro:arg name="static" default="true"/>' in xacro
assert "<gravity>false</gravity>" in xacro
assert "xacro.load_yaml(xacro.arg('camera_profiles_file'))" in xacro
assert "radians(float(profile_lens['horizontal_fov_degrees']))" in xacro
assert "radians(float(xacro.arg('hfov_degrees')))" in xacro
assert 'filename="libxgc_gazebo_media_camera.so"' in xacro
assert "<sourceId>$(arg media_source_id)</sourceId>" in xacro
assert "<rtpPort>$(arg media_rtp_port)</rtpPort>" in xacro
assert "<controlSocket>$(arg media_control_socket)</controlSocket>" in xacro
assert "<snapshotPoseFrameId>$(arg snapshot_pose_frame_id)</snapshotPoseFrameId>" in xacro
assert "<rosPublishEnabled>$(arg publish_encoded_video)</rosPublishEnabled>" in xacro
assert "<rosVideoTopic>$(arg encoded_video_topic)</rosVideoTopic>" in xacro
assert "<rosFrameTimingTopic>$(arg frame_timing_topic)</rosFrameTimingTopic>" in xacro
assert "<rosStreamInfoTopic>$(arg stream_info_topic)</rosStreamInfoTopic>" in xacro
assert "libgazebo_ros_camera.so" not in xacro

assert 'name="camera_profile" default="world_ultrawide_4k30_130"' in launch
assert "config/world_camera_profiles.yaml" in launch
for argument in (
    "width",
    "height",
    "fps",
    "hfov",
    "near_clip",
    "far_clip",
    "noise_stddev",
    "media_bitrate",
    "media_max_bitrate",
    "media_pacing_bitrate",
    "media_vbv_buffer_milliseconds",
    "snapshot_jpeg_quality",
):
    assert f'<arg name="{argument}" default="profile"/>' in launch
assert '<arg name="hfov_degrees" default="profile"/>' in launch
assert "hfov_degrees:=$(arg hfov_degrees)" in launch
assert "camera_profile:=$(arg camera_profile)" in launch
assert "camera_profiles_file:=$(arg camera_profiles_file)" in launch
assert "static:=$(arg static)" in launch
assert '<arg name="snapshot_pose_frame_id" default="world"/>' in launch
assert "snapshot_pose_frame_id:=$(arg snapshot_pose_frame_id)" in launch
assert '<arg name="publish_encoded_video" default="true"/>' in launch
assert '<arg name="enable_continuous_jpeg_preview" default="false"/>' in launch
assert 'name="enable_continuous_jpeg_preview"' in launch

for calibration_launch in (intrinsic, extrinsic):
    assert 'name="camera_profile" default="world_ultrawide_4k30_130"' in calibration_launch
    assert '<arg name="camera_profile" value="$(arg camera_profile)"/>' in calibration_launch
    assert '<arg name="camera_profiles_file" value="$(arg camera_profiles_file)"/>' in calibration_launch
    assert '<arg name="publish_encoded_video" default="true"/>' in calibration_launch
    assert '<arg name="publish_encoded_video" value="$(arg publish_encoded_video)"/>' in calibration_launch
    assert '<arg name="enable_continuous_jpeg_preview" default="false"/>' in calibration_launch

assert '<arg name="static" value="$(arg camera_static)"/>' in intrinsic
assert "--gui-client-plugin libKeyboardGUIPlugin.so" in launch
assert 'type="keyboard_camera_teleop.py"' in intrinsic

assert "arg('mode') == 'truth'" in launch
assert "arg('mode') == 'validation'" in launch
assert "gt_camera_link_frame" in launch and "gt_optical_frame" in launch
assert "arg('mode') == 'calibration' or not arg('publish_truth_tf')" in launch
assert 'type="camera_lifecycle_keepalive.py"' in launch
assert 'name="$(arg model_name)_lifecycle_keepalive"' in launch
assert 'rospy.init_node("camera_lifecycle_keepalive")' in keepalive
assert "rospy.spin()" in keepalive
assert "scripts/camera_lifecycle_keepalive.py" in cmake
assert 'name="media_control_socket" value="$(arg media_control_socket)"' in launch
assert 'name="camera_profile" value="$(arg camera_profile)"' in launch
assert "input_compressed_image_topic" not in launch
assert "input_camera_info_topic" not in launch
assert "_configured_intrinsics()" in camera_contract
assert '"includeRgb": False' in camera_contract
assert '"requestKeyframe": False' in camera_contract
assert 'rospy.get_param("~enable_continuous_jpeg_preview", False)' in camera_contract
assert "self._image_timer = None" in camera_contract
assert "if self._continuous_jpeg_preview:" in camera_contract
assert "self._publish_camera_info(rospy.Time.now())" in camera_contract
assert "0.0, 0.0, 1.0," in camera_contract

assert "foxglove_msgs::CompressedVideo" in media_plugin
assert "xgc_camera_msgs::FrameTiming" in media_plugin
assert "xgc_camera_msgs::StreamInfo" in media_plugin
assert "sensor_->LastMeasurementTime()" in media_plugin
assert "timing.native_source_time_ns = frame.sourceTimeNanoseconds" in media_plugin
assert "timing.host_publish_realtime_ns = hostPublishRealtimeNanoseconds" in media_plugin
assert "timing.source_to_ros_offset_ns = 0" in media_plugin
assert "timing.mapping_uncertainty_ns = 0" in media_plugin
assert 'kSourceTimestampClockDomain = "simulation"' in media_plugin
assert "timestampClockDomain" in media_plugin
assert "camera_->WorldPose()" in media_plugin
assert '\\"renderPose\\"' in media_plugin
assert '\\"poseFrameId\\"' in media_plugin
assert "ROSPublisherLoop" in media_plugin
assert "DISCONTINUITY_QUEUE_OVERFLOW" in media_plugin
assert "if (rosWaitingForIDR_ && !keyframe)" in media_plugin
assert "forceKeyframe_.store(true)" in media_plugin
assert "getNumSubscribers()" not in media_plugin
assert "OnROSSubscriberConnected" in media_plugin
assert "std::try_to_lock" in media_plugin
assert "rosFreshRenderGeneration_.fetch_add(1)" in media_plugin
assert "ROSRenderIsFreshForCurrentActivation" in media_plugin
assert "FreshRenderDecision::kDiscardAndReset" in media_plugin
render_callback = media_plugin[
    media_plugin.index("void postRenderTargetUpdate"):
    media_plugin.index("bool CaptureSnapshot")
]
assert ".publish(" not in render_callback
assert "gzwarn" not in render_callback
source_time_observer = media_plugin[
    media_plugin.index("void ObserveSourceTime"):
    media_plugin.index("std::optional<PendingEncodedFrame>")
]
assert "DISCONTINUITY_SOURCE_TIME_RESET" in source_time_observer
assert "DestroyEncoder();" in source_time_observer
encode_frame = media_plugin[
    media_plugin.index("bool EncodeRenderedFrame"):
    media_plugin.index("bool EnsureEncoder")
]
assert "kMaximumPendingEncodedFrames" in encode_frame
assert "DISCONTINUITY_ENCODER_RESET" in encode_frame
assert "DestroyEncoder();" in encode_frame

source_build = docker_build.index("    catkin_make\n")
source_overlay = docker_build.index(
    "    source /workspace/work/devel/setup.bash\n"
)
run_contract = docker_build.index(
    "      catkin_make run_tests_gazebo_sim_camera\n"
)
assert source_build < source_overlay < run_contract
assert "LIBGL_ALWAYS_SOFTWARE=1 xvfb-run" in docker_build

assert '<arg name="vrpn_use_server_time" default="false"/>' in extrinsic
assert '<arg name="use_server_time" value="$(arg vrpn_use_server_time)"/>' in extrinsic

print("Static product contracts passed")
