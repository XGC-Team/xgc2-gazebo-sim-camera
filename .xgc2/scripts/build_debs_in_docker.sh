#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DOCKER_IMAGE="${DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-full-noetic:1.0.8@sha256:fce2d76fddf4f6439bf0a188249b731650febdc163befc360bed186b269d252a}"
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

    echo "deb [trusted=yes arch=$(dpkg --print-architecture)] https://xgc2.apt.xiaokang.ink focal main" >/etc/apt/sources.list.d/xgc2.list
    if [[ -n "${XGC2_APT_OVERLAY_URL:-}" ]]; then
      sed "s#https://xgc2.apt.xiaokang.ink#${XGC2_APT_OVERLAY_URL%/}#g" /etc/apt/sources.list.d/xgc2.list >/etc/apt/sources.list.d/00-xgc2-release-train.list
    fi
    apt-get update
    apt-get install -y --no-install-recommends \
      libxgc2-xrpc-dev \
      ros-noetic-xgc2-camera-msgs \
      ros-noetic-xgc2-gazebo-sim-worlds \
      ros-noetic-xgc2-gazebo-scene
    scene_version="$(dpkg-query -W -f '\''${Version}'\'' ros-noetic-xgc2-gazebo-scene)"
    dpkg --compare-versions "${scene_version}" ge 1.5.0-4
    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/gazebo-camera
    rsync -a --delete /workspace/repo/ /workspace/work/src/gazebo-camera/
    cd /workspace/work
    source /opt/ros/noetic/setup.bash
    # First-party official Python SDK 0.2.0 (third-party modules stay image-owned):
    # the released wheel is verified by checksum and placed on the interpreter path.
    sdk_wheel=/workspace/work/xgc2_xrpc-0.2.0-py3-none-any.whl
    curl -fsSL --retry 5 \
      https://github.com/XGC-Team/xgc2-xrpc/releases/download/v0.2.0-1/xgc2_xrpc-0.2.0-py3-none-any.whl \
      -o "$sdk_wheel"
    echo "fc81fe2ee5ef1abf2e7313ff4ae9401ce60f3dee14292b5228ea46e01f97c298  $sdk_wheel" | sha256sum -c -
    rm -rf /workspace/work/sdk
    python3 -m zipfile -e "$sdk_wheel" /workspace/work/sdk
    printf "%s\n" "{\"url\":\"https://github.com/XGC-Team/xgc2-xrpc/releases/download/v0.2.0-1/xgc2_xrpc-0.2.0-py3-none-any.whl\",\"archive_info\":{\"hashes\":{\"sha256\":\"fc81fe2ee5ef1abf2e7313ff4ae9401ce60f3dee14292b5228ea46e01f97c298\"}}}" \
      > /workspace/work/sdk/xgc2_xrpc-0.2.0.dist-info/direct_url.json
    export PYTHONPATH="/workspace/work/sdk:${PYTHONPATH:-}"
    catkin_make -j2
    # The Python tests import the just-built scripts through the source overlay.
    source /workspace/work/devel/setup.bash
    LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1280x720x24" \
      catkin_make -j2 run_tests_gazebo_sim_camera
    catkin_test_results
    DESTDIR=/workspace/work/install-root catkin_make -j2 install -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic -DCATKIN_ENABLE_TESTING=OFF
    /workspace/repo/.xgc2/scripts/package_debs.sh --install-root /workspace/work/install-root --output-dir /workspace/out
    if [[ "${INSTALL_CHECK}" == true ]]; then
      apt-get install -y /workspace/out/ros-noetic-xgc2-gazebo-sim-camera_*.deb
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    fi
  '
find "${OUTPUT_DIR}" -maxdepth 1 -name '*.deb' -type f -print
