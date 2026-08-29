#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CUDA_IMAGE="${CUDA_IMAGE:-nvidia/cuda@sha256:03681bbd11fea044ff3e3a5d65e190a6f935af30c56e03e740b27e4563f61e0f}"
OUTPUT_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image) CUDA_IMAGE="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ -n "$OUTPUT_DIR" ]] || {
  echo "--output-dir is required" >&2
  exit 2
}
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(cd "$OUTPUT_DIR" && pwd)"

docker pull "$CUDA_IMAGE" >/dev/null
docker run --rm --entrypoint bash \
  -v "$REPO_ROOT:/src:ro" \
  -v "$OUTPUT_DIR:/out" \
  "$CUDA_IMAGE" -lc '
    set -euo pipefail
    g++ -std=c++17 -O2 -fPIC -shared \
      /src/src/snapshot_jpeg_nvjpeg.cpp \
      -I/src/src -I/usr/local/cuda/include \
      -L/usr/local/cuda/lib64 \
      -Wl,-rpath,\$ORIGIN \
      -lnvjpeg -lcudart \
      -o /out/libxgc_gazebo_snapshot_jpeg_hardware.so
    cp -L /usr/local/cuda/lib64/libnvjpeg.so.11 /out/libnvjpeg.so.11
    cp -L /usr/local/cuda/lib64/libcudart.so.11.0 /out/libcudart.so.11.0
    cp /NGC-DL-CONTAINER-LICENSE /out/NVIDIA-CUDA-CONTAINER-LICENSE
    strip --strip-unneeded /out/libxgc_gazebo_snapshot_jpeg_hardware.so
    ldd /out/libxgc_gazebo_snapshot_jpeg_hardware.so \
      | grep -Fq "libnvjpeg.so.11 => /out/libnvjpeg.so.11"
    ldd /out/libxgc_gazebo_snapshot_jpeg_hardware.so \
      | grep -Fq "libcudart.so.11.0 => /out/libcudart.so.11.0"
    ! ldd /out/libxgc_gazebo_snapshot_jpeg_hardware.so | grep -F "not found"
  '

python3 - "$OUTPUT_DIR" "$CUDA_IMAGE" <<'PY'
import hashlib
import json
from pathlib import Path
import platform
import sys

root = Path(sys.argv[1])
files = [
    "libxgc_gazebo_snapshot_jpeg_hardware.so",
    "libnvjpeg.so.11",
    "libcudart.so.11.0",
    "NVIDIA-CUDA-CONTAINER-LICENSE",
]
manifest = {
    "schema": "xgc2.gazebo-camera.nvjpeg-bundle.v1",
    "backend": "nvjpeg-cuda",
    "cudaBuildImage": sys.argv[2],
    "architecture": platform.machine(),
    "files": [
        {
            "name": name,
            "bytes": (root / name).stat().st_size,
            "sha256": "sha256:" + hashlib.sha256((root / name).read_bytes()).hexdigest(),
        }
        for name in files
    ],
}
(root / "bundle-manifest.json").write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY

printf 'nvJPEG backend bundle: %s\n' "$OUTPUT_DIR"
