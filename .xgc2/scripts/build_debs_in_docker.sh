#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --skip-install-check) INSTALL_CHECK=false; shift ;;
    *) echo "unknown argument: $1" >&2; exit 1 ;;
  esac
done
mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"
docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" bash -lc '
    set -euo pipefail

    sed -i \
      -e "s#http://archive.ubuntu.com/ubuntu#https://archive.ubuntu.com/ubuntu#g" \
      -e "s#http://security.ubuntu.com/ubuntu#https://archive.ubuntu.com/ubuntu#g" \
      -e "s#http://ports.ubuntu.com/ubuntu-ports#https://ports.ubuntu.com/ubuntu-ports#g" \
      /etc/apt/sources.list
    printf "%s\n" "Acquire::Retries \"5\";" \
      >/etc/apt/apt.conf.d/99-xgc2-retries
    apt_update() {
      local attempt
      for attempt in 1 2 3; do
        if apt-get update; then
          return 0
        fi
        [[ "${attempt}" -lt 3 ]] || return 1
        sleep "$((attempt * 5))"
      done
    }
    apt_install() {
      local attempt
      for attempt in 1 2 3; do
        if apt-get install "$@"; then
          return 0
        fi
        [[ "${attempt}" -lt 3 ]] || return 1
        sleep "$((attempt * 5))"
        apt_update
      done
    }
    apt_update
    apt_install -y --no-install-recommends ca-certificates
    echo "deb [trusted=yes arch=$(dpkg --print-architecture)] https://xgc2.apt.xiaokang.ink focal main" >/etc/apt/sources.list.d/xgc2.list
    if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
      sed "s#https://xgc2.apt.xiaokang.ink#${XGC2_APT_OVERLAY_URL%/}#g" /etc/apt/sources.list.d/xgc2.list >/etc/apt/sources.list.d/00-xgc2-release-train.list
    fi
    apt_update
    apt_install -y --no-install-recommends \
      build-essential cmake dpkg-dev fakeroot git libffmpeg-nvenc-dev libgl1-mesa-dev \
      libglew-dev libjpeg-dev libxml2-utils python3-numpy python3-opencv python3-yaml \
      rsync xauth xvfb ros-noetic-camera-calibration \
      ros-noetic-gazebo-dev \
      ros-noetic-gazebo-plugins ros-noetic-gazebo-ros ros-noetic-rostest \
      ros-noetic-rospack ros-noetic-rospy ros-noetic-roslaunch ros-noetic-rostopic ros-noetic-rviz \
      ros-noetic-sensor-msgs ros-noetic-tf ros-noetic-xacro \
      ros-noetic-xgc2-gazebo-sim-worlds
    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/gazebo-camera
    rsync -a --delete /workspace/repo/ /workspace/work/src/gazebo-camera/
    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    catkin_make
    # The contract spawns Gazebo from the just-built source workspace. Without
    # this overlay, Gazebo cannot resolve libxgc_gazebo_media_camera.so and the
    # model still inserts successfully while its control socket never appears.
    source /workspace/work/devel/setup.bash
    LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1280x720x24" \
      catkin_make run_tests_gazebo_sim_camera
    catkin_test_results
    DESTDIR=/workspace/work/install-root catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCATKIN_ENABLE_TESTING=OFF
    /workspace/repo/.xgc2/scripts/package_debs.sh --install-root /workspace/work/install-root --output-dir /workspace/out
    if [[ "${INSTALL_CHECK}" == true ]]; then
      apt_install -y /workspace/out/ros-noetic-xgc2-gazebo-sim-camera_*.deb
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    fi
  '
find "${OUTPUT_DIR}" -maxdepth 1 -name '*.deb' -type f -print
