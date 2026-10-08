#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE="ros-noetic-xgc2-gazebo-sim-camera"
ROS_PACKAGE="gazebo_sim_camera"
VERSION="$(awk -F': *' '/^version:/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml")"
VERSION="${PACKAGE_VERSION:-${VERSION}}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root) INSTALL_ROOT="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
[[ -n "${INSTALL_ROOT}" && -n "${OUTPUT_DIR}" ]] || { echo "--install-root and --output-dir are required" >&2; exit 1; }

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PKG_ROOT="$(mktemp -d)"
trap 'rm -rf "${PKG_ROOT}"' EXIT
mkdir -p "${OUTPUT_DIR}" "${PKG_ROOT}/DEBIAN" "${PKG_ROOT}/usr/share/doc/${PACKAGE}"
rm -f "${OUTPUT_DIR}"/*.deb

for relative in "share/${ROS_PACKAGE}" "lib/${ROS_PACKAGE}"; do
  source_path="${INSTALL_ROOT}${PREFIX}/${relative}"
  test -e "${source_path}" || {
    echo "missing installed ROS package path: ${source_path}" >&2
    exit 1
  }
  mkdir -p "${PKG_ROOT}${PREFIX}/$(dirname "${relative}")"
  cp -a "${source_path}" "${PKG_ROOT}${PREFIX}/$(dirname "${relative}")/"
done

PLUGIN="${PREFIX}/lib/libxgc_gazebo_media_camera.so"
PLUGIN_SOURCE="${INSTALL_ROOT}${PLUGIN}"
test -f "${PLUGIN_SOURCE}" || {
  echo "missing installed Gazebo media camera plugin: ${PLUGIN_SOURCE}" >&2
  exit 1
}
mkdir -p "${PKG_ROOT}$(dirname "${PLUGIN}")"
cp -a "${PLUGIN_SOURCE}" "${PKG_ROOT}${PLUGIN}"

# Resolve native runtime ABI requirements from this exact installed artifact.
# The build toolchain must install the SDK and JsonCpp as owned Debian packages;
# a private Noble .so must not be advertised as a Focal-compatible dependency.
mkdir -p "${PKG_ROOT}/debian"
printf 'Source: xgc2-gazebo-sim-camera\nSection: misc\nPriority: optional\nMaintainer: XGC2 <dev@xiaokang.ink>\n\nPackage: %s\nArchitecture: any\nDescription: XGC2 camera source\n' "${PACKAGE}" >"${PKG_ROOT}/debian/control"
NATIVE_DEPENDS="$(cd "${PKG_ROOT}" && dpkg-shlibdeps -O -e"${PKG_ROOT}${PLUGIN}")"
NATIVE_DEPENDS="${NATIVE_DEPENDS#shlibs:Depends=}"
[[ -n "${NATIVE_DEPENDS}" ]] || { echo 'missing native dependency evidence' >&2; exit 1; }
rm -r "${PKG_ROOT}/debian"

cat >"${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <dev@xiaokang.ink>
Depends: libgl1, libglew2.1, libjpeg8, python3-numpy, python3-opencv, python3-yaml, ros-noetic-foxglove-msgs, ros-noetic-gazebo-plugins, ros-noetic-gazebo-ros, ros-noetic-roscpp, ros-noetic-rospy, ros-noetic-roslaunch, ros-noetic-rostopic, ros-noetic-rviz, ros-noetic-sensor-msgs, ros-noetic-geometry-msgs, ros-noetic-tf2-msgs, ros-noetic-tf, ros-noetic-xacro, ros-noetic-xgc2-camera-msgs (>= 1.2.0-8), ros-noetic-xgc2-camera-calibration (>= 0.3.0-40), ros-noetic-xgc2-gazebo-sim-worlds (>= 1.4.1-4), libxgc2-xrpc1 (>= 0.1.0), ${NATIVE_DEPENDS}
Recommends: ros-noetic-xgc2-gazebo-sim-vrpn-bridge (>= 1.1.0-27)
Description: XGC2 independent Gazebo Classic fixed-site RGB camera
EOF

install -m 0644 "${REPO_ROOT}/LICENSE" "${PKG_ROOT}/usr/share/doc/${PACKAGE}/copyright"
find "${PKG_ROOT}" -type d -exec chmod 0755 {} +
find "${PKG_ROOT}" -type f -exec chmod 0644 {} +
if [[ -d "${PKG_ROOT}${PREFIX}/lib/${ROS_PACKAGE}" ]]; then
  find "${PKG_ROOT}${PREFIX}/lib/${ROS_PACKAGE}" -type f -exec chmod 0755 {} +
fi
chmod 0755 "${PKG_ROOT}/DEBIAN"
fakeroot dpkg-deb --build "${PKG_ROOT}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -name '*.deb' -type f -print
