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

for calibration_launch in (intrinsic, extrinsic):
    assert 'name="camera_profile" default="world_ultrawide_4k30_130"' in calibration_launch
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
