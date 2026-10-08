#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"
export PYTHONPYCACHEPREFIX="${PYTHONPYCACHEPREFIX:-/tmp/xgc2-gazebo-sim-camera-pycache}"
bash -n .xgc2/scripts/*.sh
python3 -m py_compile scripts/native_camera_entity.py scripts/camera_contract_test.py scripts/drive_intrinsic_calibration.py scripts/keyboard_camera_teleop.py test/test_world_camera_profiles.py .xgc2/scripts/xgc2_artifact_manifest.py
python3 test/static_product_contract.py
python3 test/test_world_camera_profiles.py

required=(
  .github/workflows/ci.yml .github/workflows/release.yml .xgc2/product.yml
  .xgc2/scripts/build_debs_in_docker.sh .xgc2/scripts/check_installed_packages.sh
  .xgc2/scripts/check_gazebo_shutdown.sh
  .xgc2/scripts/check_package_compliance.sh .xgc2/scripts/package_debs.sh
  CMakeLists.txt LICENSE README.md package.xml
  docs/encoded_camera_recording.md
  launch/static_camera.launch launch/intrinsic_calibration_world.launch
  launch/extrinsic_calibration_world.launch launch/camera_ar_rviz.launch
  scripts/native_camera_entity.py
  urdf/fixed_rgb_camera.urdf.xacro config/extrinsic_markers_vrpn.yaml
  config/world_camera_profiles.yaml
  test/static_camera_contract.test
  test/static_product_contract.py
  test/test_world_camera_profiles.py
)
for path in "${required[@]}"; do test -f "${path}" || { echo "Missing ${path}" >&2; exit 1; }; done

grep -q 'id: xgc2-gazebo-sim-camera' .xgc2/product.yml
grep -Eq '^version: [0-9]+\.[0-9]+\.[0-9]+-[0-9]+$' .xgc2/product.yml
python3 - <<'PY'
import yaml
with open('.xgc2/product.yml') as stream:
    product = yaml.safe_load(stream)
assert product['version'] == product['release']['apt_versions']['focal']
PY
grep -q 'PACKAGE="ros-noetic-xgc2-gazebo-sim-camera"' .xgc2/scripts/package_debs.sh
grep -q 'PLUGIN="${PREFIX}/lib/libxgc_gazebo_media_camera.so"' .xgc2/scripts/package_debs.sh
grep -q '<name>gazebo_sim_camera</name>' package.xml
grep -q '<exec_depend>python3-yaml</exec_depend>' package.xml
grep -q '<exec_depend>rospy</exec_depend>' package.xml
grep -q '<depend>foxglove_msgs</depend>' package.xml
grep -q '<depend>xgc_camera_msgs</depend>' package.xml
grep -q '^Depends: libgl1, libglew2.1, libjpeg8, python3-numpy, python3-opencv, python3-yaml, ros-noetic-foxglove-msgs,' .xgc2/scripts/package_debs.sh
grep -q 'ros-noetic-xgc2-camera-msgs (>= 1.2.0-8)' .xgc2/scripts/package_debs.sh
grep -q '^  recommends:$' .xgc2/product.yml
grep -q '^Recommends: ros-noetic-xgc2-gazebo-sim-vrpn-bridge' .xgc2/scripts/package_debs.sh

for xml in launch/*.launch test/*.test; do xmllint --noout "${xml}"; done
# Direct expansion exercises every declared default.  This specifically guards
# against root-element substitutions that are evaluated before <xacro:arg>.
/opt/ros/noetic/bin/xacro urdf/fixed_rgb_camera.urdf.xacro \
  media_control_endpoint:=/granted/media.sock media_control_target_id:=package-contract >/dev/null

# Verify both a named profile and the advanced per-field compatibility
# overrides accepted by direct roslaunch users.
/opt/ros/noetic/bin/xacro urdf/fixed_rgb_camera.urdf.xacro camera_profile:=world_wide_4k30_110 \
  media_control_endpoint:=/granted/media.sock media_control_target_id:=package-contract >/dev/null
/opt/ros/noetic/bin/xacro urdf/fixed_rgb_camera.urdf.xacro model_name:=test_camera camera_link_frame:=usb_cam_link optical_frame:=usb_cam_optical_frame width:=320 height:=240 fps:=10 hfov_degrees:=110 near_clip:=0.05 far_clip:=20 noise_stddev:=0 \
  media_control_endpoint:=/granted/media.sock media_control_target_id:=package-contract >/dev/null
echo "Package compliance checks passed"
