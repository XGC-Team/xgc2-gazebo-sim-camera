#!/usr/bin/env python3
import json
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
cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
profiles = yaml.safe_load(
    (root / "config/world_camera_profiles.yaml").read_text(encoding="utf-8")
)
# Process definitions are owned by the xgc2 process-catalog (B1, current only).
catalog_definition = (
    root.parents[3]
    / "xgc2"
    / "xgc2"
    / "process-catalog"
    / "current"
    / "ros1"
    / "simulator"
    / "gazebo-sim"
    / "camera"
    / "gazebo-static-camera.json"
)
definition = json.loads(catalog_definition.read_text(encoding="utf-8"))["definitions"][0]

assert profiles["schema_version"] == 1
assert profiles["default_profile"] == "world_ultrawide_4k30_140"
assert list(profiles["profiles"]) == [
    "world_ultrawide_4k30_140",
    "calibration_standard_720p20",
]

assert "<gazebo><static>true</static></gazebo>" in xacro
assert '<xacro:arg name="static" default="true"/>' in xacro
assert "<gravity>false</gravity>" in xacro
assert "xacro.load_yaml(xacro.arg('camera_profiles_file'))" in xacro
assert "radians(float(profile_lens['horizontal_fov_degrees']))" in xacro
assert 'filename="libxgc_gazebo_media_camera.so"' in xacro
assert "<sourceId>$(arg media_source_id)</sourceId>" in xacro
assert "<rtpPort>$(arg media_rtp_port)</rtpPort>" in xacro
assert "<controlSocket>$(arg media_control_socket)</controlSocket>" in xacro
assert "libgazebo_ros_camera.so" not in xacro

assert 'name="camera_profile" default="world_ultrawide_4k30_140"' in launch
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
assert "camera_profile:=$(arg camera_profile)" in launch
assert "camera_profiles_file:=$(arg camera_profiles_file)" in launch
assert "static:=$(arg static)" in launch

for calibration_launch in (intrinsic, extrinsic):
    assert 'name="camera_profile" default="world_ultrawide_4k30_140"' in calibration_launch
    assert '<arg name="camera_profile" value="$(arg camera_profile)"/>' in calibration_launch
    assert '<arg name="camera_profiles_file" value="$(arg camera_profiles_file)"/>' in calibration_launch

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

assert '<arg name="vrpn_use_server_time" default="false"/>' in extrinsic
assert '<arg name="use_server_time" value="$(arg vrpn_use_server_time)"/>' in extrinsic

properties = definition["parameters"]["properties"]
profile_property = properties["cameraProfile"]
assert definition["version"] == "0.7.0"
assert profile_property["default"] == profiles["default_profile"]
assert profile_property["enum"] == list(profiles["profiles"])
assert definition["parameters"]["additionalProperties"] is False
for removed_fragment in (
    "width",
    "height",
    "fps",
    "hfov",
    "mediaBitrate",
    "mediaMaxBitrate",
    "mediaPacingBitrate",
    "mediaVbvBufferMilliseconds",
    "snapshotJpegQuality",
):
    assert removed_fragment not in properties
for instance_property in (
    "modelName",
    "cameraLinkFrame",
    "opticalFrame",
    "mediaSourceId",
    "mediaRtpPort",
    "mediaControlSocket",
    "x",
    "y",
    "z",
    "roll",
    "pitch",
    "yaw",
):
    assert instance_property in properties

command_arguments = definition["command"]["args"]
assert "camera_profile:=${cameraProfile}" in command_arguments
assert "media_source_id:=${mediaSourceId}" in command_arguments
assert "media_rtp_port:=${mediaRtpPort}" in command_arguments
assert "media_control_socket:=${mediaControlSocket}" in command_arguments
assert not any(argument.startswith("width:=") for argument in command_arguments)
assert not any(argument.startswith("media_bitrate:=") for argument in command_arguments)
assert definition["beforeStart"]["command"]["args"][1] == "/gazebo/delete_model"
assert definition["beforeStop"]["command"]["args"][1] == "/gazebo/delete_model"
assert definition["resourceClaims"][0]["namespace"] == "gazebo-model"
probe = definition["readiness"]
assert probe["kind"] == "exec"
assert probe["command"]["executable"] == "/usr/bin/test"
assert probe["command"]["args"] == ["-S", "${mediaControlSocket}"]

print("Static product contracts passed")
