#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUNDLE_DIR="${1:-}"
[[ -n "$BUNDLE_DIR" ]] || {
  echo "usage: $0 /absolute/path/to/nvjpeg-bundle" >&2
  exit 2
}
BUNDLE_DIR="$(cd "$BUNDLE_DIR" && pwd)"
MODULE="$BUNDLE_DIR/libxgc_gazebo_snapshot_jpeg_hardware.so"
[[ -r "$MODULE" ]] || {
  echo "nvJPEG module is unavailable: $MODULE" >&2
  exit 1
}
command -v nvidia-smi >/dev/null
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader

SMOKE_BINARY="$(mktemp /tmp/xgc-nvjpeg-smoke.XXXXXX)"
trap 'rm -f -- "$SMOKE_BINARY"' EXIT
g++ -std=c++17 -O2 \
  "$REPO_ROOT/test/snapshot_jpeg_hardware_smoke.cpp" \
  "$REPO_ROOT/src/snapshot_jpeg_backend.cpp" \
  -I"$REPO_ROOT/src" -ljpeg -ldl -o "$SMOKE_BINARY"
"$SMOKE_BINARY" "$MODULE"
