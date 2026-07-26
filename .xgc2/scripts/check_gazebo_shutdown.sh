#!/usr/bin/env bash
set -euo pipefail

readonly RUNS="${1:-2}"
if ! [[ "${RUNS}" =~ ^[0-9]+$ ]] || (( RUNS < 2 )); then
  echo "usage: $0 [runs>=2]" >&2
  exit 2
fi

command -v rostest >/dev/null
rospack find gazebo_sim_camera >/dev/null

readonly REPORT_DIR="$(mktemp -d /tmp/xgc2-camera-shutdown.XXXXXX)"
trap 'rm -rf -- "${REPORT_DIR}"' EXIT

for ((run = 1; run <= RUNS; ++run)); do
  run_log="${REPORT_DIR}/run-${run}.log"
  if ! rostest gazebo_sim_camera static_camera_contract.test >"${run_log}" 2>&1; then
    cat "${run_log}" >&2
    echo "Gazebo camera lifecycle run ${run} failed" >&2
    exit 1
  fi
  if grep -Eiq 'segmentation fault|sigsegv|exit code -11|return value -11|exit code 139' "${run_log}"; then
    cat "${run_log}" >&2
    echo "Gazebo camera lifecycle run ${run} crashed during shutdown" >&2
    exit 1
  fi
  echo "Gazebo camera lifecycle run ${run}/${RUNS} passed"
done
