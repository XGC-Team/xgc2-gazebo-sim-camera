#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
: "${SELECTED_PYTHON:?explicit managed Python interpreter required}"
[[ "${SELECTED_PYTHON}" == /* && -x "${SELECTED_PYTHON}" ]]
"${SELECTED_PYTHON}" -c 'import sys; assert sys.version_info >= (3, 10); import xgc2_xrpc, rospy, tf'
SHARE="/opt/ros/${ROS_DISTRO}/share/gazebo_sim_camera"
PLUGIN="/opt/ros/${ROS_DISTRO}/lib/libxgc_gazebo_media_camera.so"

dpkg -s ros-noetic-xgc2-gazebo-sim-camera >/dev/null
dpkg -s ros-noetic-foxglove-msgs ros-noetic-xgc2-camera-msgs >/dev/null
dpkg -s libgl1 libglew2.1 libjpeg8 >/dev/null
test "$(rospack find gazebo_sim_camera)" = "${SHARE}"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_camera/camera_contract_test.py"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_camera/native_camera_entity.py"
test -f "${SHARE}/urdf/fixed_rgb_camera.urdf.xacro"
test -f "${SHARE}/config/world_camera_profiles.yaml"
test -f "${PLUGIN}"
PLUGIN_DEPENDENCIES="$(ldd "${PLUGIN}")"
! grep -q 'not found' <<<"${PLUGIN_DEPENDENCIES}"
grep -q 'libgazebo_sensors.so' <<<"${PLUGIN_DEPENDENCIES}"
grep -q 'libGLEW.so.2.1' <<<"${PLUGIN_DEPENDENCIES}"
grep -q 'libjpeg.so.8' <<<"${PLUGIN_DEPENDENCIES}"
grep -q 'libGL.so.1' <<<"${PLUGIN_DEPENDENCIES}"
grep -q 'libroscpp.so' <<<"${PLUGIN_DEPENDENCIES}"
grep -a -q 'libnvidia-encode.so.1' "${PLUGIN}"
grep -q 'xacro.load_yaml' "${SHARE}/urdf/fixed_rgb_camera.urdf.xacro"
grep -q 'libxgc_gazebo_media_camera.so' "${SHARE}/urdf/fixed_rgb_camera.urdf.xacro"
grep -q 'type="native_camera_entity.py"' "${SHARE}/launch/static_camera.launch"

EXPANDED_PROFILE="$(mktemp)"
trap 'rm -f "${EXPANDED_PROFILE}"' EXIT
/opt/ros/noetic/bin/xacro "${SHARE}/urdf/fixed_rgb_camera.urdf.xacro" \
  camera_profile:=world_wide_4k30_110 media_control_endpoint:=/granted/media.sock \
  media_control_target_id:=package-contract >"${EXPANDED_PROFILE}"
grep -q '<width>3840</width>' "${EXPANDED_PROFILE}"
grep -q '<height>2160</height>' "${EXPANDED_PROFILE}"
grep -q '<update_rate>30.0</update_rate>' "${EXPANDED_PROFILE}"

# --files only validates installed launch composition; it starts no process and
# therefore uses an explicit syntax-only ServiceRef, never a readiness probe.
CONTRACT_REF='{"target_id":"package-contract","service":"xgc2.simulation","api_version":"v1","instance_id":"syntax-only-instance","profile":"http.v1","endpoint":{"kind":"unix","address":"/granted/world.sock"}}'
CONTRACT_ARGS=(world:=/granted/prepared-world.world simulation_service_ref_json:="${CONTRACT_REF}"
  target_id:=package-contract media_control_endpoint:=/granted/media.sock
  media_control_target_id:=package-contract python_executable:="${SELECTED_PYTHON}")
for entry in static_camera intrinsic_calibration_world; do
  roslaunch --files gazebo_sim_camera "${entry}.launch" gui:=false "${CONTRACT_ARGS[@]}" >/dev/null
done
roslaunch --files gazebo_sim_camera extrinsic_calibration_world.launch gui:=false \
  vrpn_config:=/granted/vrpn.yaml "${CONTRACT_ARGS[@]}" >/dev/null
echo "Installed package check passed"
