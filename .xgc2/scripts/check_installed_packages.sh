#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
SHARE="/opt/ros/${ROS_DISTRO}/share/gazebo_sim_camera"

dpkg -s ros-noetic-xgc2-gazebo-sim-camera >/dev/null
test "$(rospack find gazebo_sim_camera)" = "${SHARE}"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_camera/camera_contract_test.py"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_camera/camera_lifecycle_keepalive.py"
test -f "${SHARE}/urdf/fixed_rgb_camera.urdf.xacro"
test -f "${SHARE}/config/world_camera_profiles.yaml"
test -f "${PLUGIN}"
grep -q 'xacro.load_yaml' "${SHARE}/urdf/fixed_rgb_camera.urdf.xacro"
grep -q 'libxgc_gazebo_media_camera.so' "${SHARE}/urdf/fixed_rgb_camera.urdf.xacro"
grep -q 'type="camera_lifecycle_keepalive.py"' "${SHARE}/launch/static_camera.launch"
grep -q '"version": "0.7.0"' "${PLUGIN}"
grep -q '"cameraProfile"' "${PLUGIN}"
python3 -m json.tool "${PLUGIN}" >/dev/null

EXPANDED_PROFILE="$(mktemp)"
trap 'rm -f "${EXPANDED_PROFILE}"' EXIT
/opt/ros/noetic/bin/xacro "${SHARE}/urdf/fixed_rgb_camera.urdf.xacro" \
  camera_profile:=calibration_standard_720p20 >"${EXPANDED_PROFILE}"
grep -q '<width>1280</width>' "${EXPANDED_PROFILE}"
grep -q '<height>720</height>' "${EXPANDED_PROFILE}"
grep -q '<update_rate>20.0</update_rate>' "${EXPANDED_PROFILE}"

roslaunch --files gazebo_sim_camera static_camera.launch gui:=false >/dev/null
roslaunch --files gazebo_sim_camera intrinsic_calibration_world.launch gui:=false >/dev/null
roslaunch --files gazebo_sim_camera extrinsic_calibration_world.launch gui:=false start_vrpn:=false >/dev/null
echo "Installed package check passed"
