#!/usr/bin/env python3
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


root = Path(__file__).resolve().parents[1]
xacro = (root / "urdf/fixed_rgb_camera.urdf.xacro").read_text(encoding="utf-8")
launch = (root / "launch/static_camera.launch").read_text(encoding="utf-8")
cli = (root / "scripts/place_world_camera").read_text(encoding="utf-8")
camera_contract = (root / "scripts/camera_contract_publisher.py").read_text(
    encoding="utf-8"
)
media_plugin = (root / "src/xgc_media_camera_plugin.cpp").read_text(
    encoding="utf-8"
)
control_host = (root / "src/camera_source_control.cpp").read_text(encoding="utf-8")
native_camera_test = (root / "test/native_camera_xrpc.py").read_text(encoding="utf-8")
camera_contract_test = (root / "scripts/camera_contract_test.py").read_text(encoding="utf-8")
cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
docker_build = (root / ".xgc2/scripts/build_debs_in_docker.sh").read_text(
    encoding="utf-8"
)
profiles = yaml.safe_load(
    (root / "config/world_camera_profiles.yaml").read_text(encoding="utf-8")
)

# With parallel starts the session ROS master may not be up yet; roslaunch must wait for it
# instead of starting a second master of its own.
definition = json.loads(
    (root / "process-definitions/gazebo-static-camera.json").read_text(encoding="utf-8")
)
for entry in definition["definitions"]:
    assert entry["readiness"] == {"kind": "none"}
    assert "services" not in entry and "publishAs" not in entry
    properties = entry["parameters"]["properties"]
    required = entry["parameters"]["required"]
    assert not any("serviceref" in name.lower() for name in properties)
    assert properties["mediaControlEndpoint"]["fixedOnly"] is True
    assert {"mediaControlEndpoint", "mediaControlTargetId", "pythonExecutable"} <= set(required)
    for removed in ("world", "simulationEndpoint", "targetId", "startGazebo", "gazeboMasterPort",
                    "cameraStatic", "mediaRtpPort", "mediaRtpHost", "snapshotJpegBackend", "gui"):
        assert removed not in properties
    # The truth transform publishes the authored pose: Core binds the same six values to the
    # placement command and to this runtime definition.
    for pose in ("x", "y", "z", "roll", "pitch", "yaw"):
        assert properties[pose]["type"] == "number"
    command = entry["command"]["args"]
    separator = command.index("--")
    assert command[separator + 1].endswith("/roslaunch"), command[separator + 1]
    assert command[separator + 2] == "--wait", command[separator + 2]
    assert not any(arg.startswith(("world:=", "simulation_endpoint:=", "media_rtp_port:=")) for arg in command)
    for pose in ("x", "y", "z", "roll", "pitch", "yaw"):
        assert pose + ":=${" + pose + "}" in command

launch_root = ET.fromstring(launch)
arguments = {arg.get("name"): arg for arg in launch_root.findall("arg")}
for removed in ("world", "simulation_endpoint", "target_id", "media_rtp_port", "start_gazebo", "static",
                "media_bitrate", "snapshot_jpeg_backend", "publish_encoded_video"):
    assert removed not in arguments
# The direct launch still needs these inputs for its existing TF nodes.
assert {"x", "y", "z", "roll", "pitch", "yaw"} <= set(arguments)
assert not launch_root.findall("include")
assert not any(param.get("command") for param in launch_root.findall("param"))
assert all(node.get("type") in ("camera_contract_publisher.py", "static_transform_publisher")
           for node in launch_root.iter("node"))
for removed in ("scripts/native_camera_entity.py", "scripts/keyboard_camera_teleop.py",
                "scripts/drive_intrinsic_calibration.py", "launch/intrinsic_calibration_world.launch",
                "launch/extrinsic_calibration_world.launch", "test/test_keyboard_camera_teleop.py",
                "test/test_intrinsic_sweep.py", ".xgc2/scripts/check_gazebo_shutdown.sh"):
    assert not (root / removed).exists()
assert "scripts/place_world_camera" in cmake
assert "test/test_place_world_camera.py" in cmake
assert "from xgc2_gazebo_scene.placement import place_entity" in cli
assert '"kind": "camera"' in cli and '"schema": "xgc2.placement.v1"' in cli
assert 'ReclaimUnreachable' in control_host
assert 'sdf->HasElement("rtpPort")' in media_plugin
assert 'SDFValue<int>(sdf, "rtpPort",' not in media_plugin

assert profiles["schema_version"] == 1
assert profiles["default_profile"] == "world_wide_4k30_110"
assert list(profiles["profiles"]) == ["world_wide_4k30_110"]
for profile in profiles["profiles"].values():
    assert profile["lens"]["near_clip_m"] > 0.065
assert "optical_origin_x" in xacro
assert "<pose>${optical_origin_x} 0 0 0 0 0</pose>" in xacro
assert 'xyz="${optical_origin_x} 0 0"' in xacro
assert "args=\"0.067 0 0 -1.5707963267948966 0 -1.5707963267948966" in launch
assert '#include "camera_source_control.h"' in media_plugin
assert "CameraSourceControlHost::Acquire" in media_plugin
assert "xgc2::xrpc::HttpServer" in control_host
assert "xgc2::xrpc::HttpLimits limits;" in control_host
assert 'xgc2::xrpc::HttpIdentity{instance_id_, {"/v1/describe"}}' in control_host
assert "request.target.substr(0, query_at)" in control_host
assert 'path == "/v1/describe"' in control_host
assert 'const std::string key = "wait_ready_ms=";' in control_host
assert "std::stoi(digits) > 30000" in control_host
for field, value in (
    ("service", '"camera-source"'),
    ("api_version", '"v1"'),
    ("instance_id", "instance_id_"),
    ("ready", "true"),
):
    assert f'result["{field}"] = {value};' in control_host
assert 'result["facts"]["target_id"] = target_id_;' in control_host
assert 'result["facts"]["service_ref"] = ServiceRef();' in control_host
assert 'describe ? result["facts"] : result' in control_host
assert '"/v1/describe?wait_ready_ms=1000"' in native_camera_test
assert 'facts = discovered["facts"]' in native_camera_test
assert 'ServiceRef.from_dict(facts["service_ref"])' in native_camera_test
assert 'description["facts"]["sources"]' in camera_contract
assert 'reference = ServiceRef.from_dict' in (root / "scripts/camera_peer.py").read_text()
assert 'instance_id=description["instance_id"]' in camera_contract_test
assert "rejectDupKeys" in control_host
assert "ReadControlLine" not in media_plugin
assert not (root / "src/unix_control_socket.h").exists()
assert "rosConsumersActive_.load()" in media_plugin
assert "rosFreshRenderGeneration_.fetch_add(1);" in media_plugin
assert {
    profile["lens"]["horizontal_fov_degrees"]
    for profile in profiles["profiles"].values()
} == {110.0}

assert "<gazebo><static>true</static></gazebo>" in xacro
assert '<xacro:arg name="static" default="true"/>' in xacro
assert "<gravity>false</gravity>" in xacro
assert "xacro.load_yaml(xacro.arg('camera_profiles_file'))" in xacro
assert "radians(float(profile_lens['horizontal_fov_degrees']))" in xacro
assert "radians(float(xacro.arg('hfov_degrees')))" in xacro
assert '<xacro:arg name="media_plugin_filename" default="libxgc_gazebo_media_camera.so"/>' in xacro
assert 'filename="$(arg media_plugin_filename)"' in xacro
assert "<sourceId>$(arg media_source_id)</sourceId>" in xacro
assert "<rtpPort>$(arg media_rtp_port)</rtpPort>" in xacro
assert "<controlEndpoint>$(arg media_control_endpoint)</controlEndpoint>" in xacro
assert "<snapshotPoseFrameId>$(arg snapshot_pose_frame_id)</snapshotPoseFrameId>" in xacro
assert "<rosPublishEnabled>$(arg publish_encoded_video)</rosPublishEnabled>" in xacro
assert "<rosVideoTopic>$(arg encoded_video_topic)</rosVideoTopic>" in xacro
assert "<rosFrameTimingTopic>$(arg frame_timing_topic)</rosFrameTimingTopic>" in xacro
assert "<rosStreamInfoTopic>$(arg stream_info_topic)</rosStreamInfoTopic>" in xacro
assert "libgazebo_ros_camera.so" not in xacro

assert 'name="camera_profile" default="world_wide_4k30_110"' in launch
assert "config/world_camera_profiles.yaml" in launch
assert 'command="xacro ' not in launch
for argument in ("width", "height", "hfov_degrees"):
    assert arguments[argument].get("default") == "profile"
assert '<arg name="hfov"' not in launch
assert "enable_continuous_jpeg_preview" not in launch
assert "xgc_compressed_image_topic" not in launch
assert "output_compressed_image_topic" not in launch
assert 'name="xgc_camera_link_frame"' not in launch
assert 'name="xgc_optical_frame"' not in launch
assert '<param name="camera_link_frame" value="$(arg camera_link_frame)"/>' in launch
assert '<param name="optical_frame" value="$(arg optical_frame)"/>' in launch

assert "arg('mode') == 'truth'" in launch
assert "arg('mode') == 'validation'" in launch
assert "gt_camera_link_frame" in launch and "gt_optical_frame" in launch
assert "camera_source_control_test" in cmake
assert "spawn_model" not in launch
assert 'name="camera_profile" value="$(arg camera_profile)"' in launch
assert '<arg name="camera_name" default="usb_cam"/>' in launch
assert 'name="camera_name" value="$(arg camera_name)"' in launch
assert "input_compressed_image_topic" not in launch
assert "input_camera_info_topic" not in launch
assert "_configured_intrinsics()" in camera_contract
assert "_selected_intrinsics" not in camera_contract
assert "~intrinsic_file" not in camera_contract
assert 'document.get("camera_name") != camera_name' in camera_contract
assert "_CAMERA_NAME_PATTERN.fullmatch(camera_name)" in camera_contract
assert "~intrinsic_json" in camera_contract
assert 'input.get("includeRgb", true)' in media_plugin
assert 'input.get("requestKeyframe", false)' in media_plugin
assert "fresh-snapshot" in media_plugin
assert 'SDFValue<std::string>(sdf, "snapshotJpegBackend", "auto")' in media_plugin
assert "ParseSnapshotJpegPolicy" in media_plugin
assert "IssueSnapshotPBOReadback" in media_plugin
assert "SnapshotEncoderLoop" in media_plugin
assert '<xacro:arg name="snapshot_jpeg_backend" default="auto"/>' in xacro
assert '<snapshotJpegBackend>$(arg snapshot_jpeg_backend)</snapshotJpegBackend>' in xacro
hardware_contract = (root / "test/static_camera_hardware_contract.test").read_text(
    encoding="utf-8"
)
assert '<arg name="snapshot_jpeg_backend"' not in hardware_contract
assert '<param name="expected_jpeg_backend" value="nvjpeg-cuda"/>' in hardware_contract
assert '<param name="expect_jpeg_fallback" value="false"/>' in hardware_contract
assert "CompressedImage" not in camera_contract
assert "_publish_media_snapshot" not in camera_contract
assert '"camera-source", self._apply_camera_info_metadata' in camera_contract
assert "CameraInfo()" not in camera_contract
assert "expected_revision" in camera_contract
assert "calibration-metadata" in camera_contract
assert "GAZEBO_LINK_TO_OPTICAL_TRANSLATION" in camera_contract

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
assert 'metadata["renderPose"]' in media_plugin
assert 'metadata["poseFrameId"]' in media_plugin
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

ensure_encoder = media_plugin[
    media_plugin.index("bool EnsureEncoder"):
    media_plugin.index("bool EnsureConversionTexture")
]
assert "configuration.frameIntervalP = 1" in ensure_encoder
assert "configuration.rcParams.enableLookahead = 0" in ensure_encoder
assert "configuration.rcParams.zeroReorderDelay = 1" in ensure_encoder
assert "encodedFrameIndex_ = 0" not in ensure_encoder
assert "transport clock, not encoder-local state" in ensure_encoder

source_build = docker_build.index("    catkin_make -j2\n")
source_overlay = docker_build.index(
    "    source /workspace/work/devel/setup.bash\n"
)
run_contract = docker_build.index(
    "      catkin_make -j2 run_tests_gazebo_sim_camera\n"
)
assert source_build < source_overlay < run_contract
assert "LIBGL_ALWAYS_SOFTWARE=1 xvfb-run" in docker_build


print("Static product contracts passed")
