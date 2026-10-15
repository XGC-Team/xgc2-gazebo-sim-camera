#!/usr/bin/env bash
# Pin the real Python dependency for source CI before its Debian publication.
set -euo pipefail
destination="${1:?pass an absolute directory outside the product checkout}"
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
case "${destination}" in
  /*) ;;
  *) echo "Calibration contract directory must be absolute" >&2; exit 1 ;;
esac
destination="$(realpath -m "${destination}")"
case "${destination}" in
  "${repo_root}"|"${repo_root}"/*) echo "Keep dependency source outside the product tree" >&2; exit 1 ;;
esac
revision=bbcc20e7559e7c742e50e48c4d97b61917eaa2bb
if [[ ! -d "${destination}/.git" ]]; then
  mkdir -p "${destination}"
  git -C "${destination}" init -q
  git -C "${destination}" fetch -q --depth 1 https://github.com/XGC-Team/xgc2-camera-calibration.git "${revision}"
  git -C "${destination}" checkout -q --detach FETCH_HEAD
fi
test "$(git -C "${destination}" rev-parse HEAD)" = "${revision}"
git -C "${destination}" diff --quiet
test -f "${destination}/xgc_camera_calibration/src/xgc_camera_calibration/camera_initial_pose.py"
printf 'Calibration contract revision: %s\n' "${revision}"
