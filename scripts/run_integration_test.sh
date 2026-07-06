#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

DURATION_S="${INTEGRATION_DURATION_S:-60}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-71}"

if [[ ! -f "${REPO_ROOT}/external/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]]; then
  echo "[integration] PX4 binary missing; running bootstrap first"
  bash "${REPO_ROOT}/scripts/bootstrap_px4.sh"
fi

# shellcheck disable=SC1091
source "${REPO_ROOT}/.venv/bin/activate"
source /opt/ros/humble/setup.bash
python3 -m pip install -r requirements.txt
python3 -m colcon build --packages-select drone_system
source install/setup.bash

export ROS_DOMAIN_ID
export HEADLESS=1

echo "[integration] launching headless full stack for ${DURATION_S}s (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
ros2 launch drone_system full_stack.launch.py headless:=true &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" || true
    wait "${LAUNCH_PID}" || true
  fi
  pkill -x px4 || true
  pkill -x mavsdk_server || true
  pkill -x gzserver || true
  pkill -x gzclient || true
}
trap cleanup EXIT

sleep "${DURATION_S}"

LATEST_RUN="$(ls -td artifacts/run_* 2>/dev/null | head -n 1 || true)"
if [[ -z "${LATEST_RUN}" ]]; then
  echo "[integration] no artifacts/run_* directory found" >&2
  exit 1
fi

echo "[integration] validating ${LATEST_RUN}"
python3 scripts/check_integration_run.py "${LATEST_RUN}"
python3 tools/log_summary.py "${LATEST_RUN}/events.log" | tee "${LATEST_RUN}/log_summary.txt"

if [[ -f "${LATEST_RUN}/telemetry.csv" ]]; then
  python3 tools/plot_run.py "${LATEST_RUN}/telemetry.csv" "${LATEST_RUN}/plots"
fi

echo "[integration] passed"
