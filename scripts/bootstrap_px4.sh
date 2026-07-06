#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_ROOT="${REPO_ROOT}/external/PX4-Autopilot"
VENV="${REPO_ROOT}/.venv"

echo "[bootstrap_px4] repo root: ${REPO_ROOT}"

if [[ ! -d "${VENV}" ]]; then
  python3 -m venv --system-site-packages "${VENV}"
fi
# shellcheck disable=SC1091
source "${VENV}/bin/activate"

python3 -m pip install --upgrade pip
python3 -m pip install \
  kconfiglib pyros-genmsg empy jinja2 numpy packaging pyserial pyyaml jsonschema mavsdk

if [[ ! -d "${PX4_ROOT}/.git" ]]; then
  echo "[bootstrap_px4] cloning PX4-Autopilot (shallow)..."
  mkdir -p "${REPO_ROOT}/external"
  git clone --depth 1 --branch main https://github.com/PX4/PX4-Autopilot.git "${PX4_ROOT}"
else
  echo "[bootstrap_px4] PX4-Autopilot already present, skipping clone"
fi

cd "${PX4_ROOT}"

apply_patch() {
  local patch_file="$1"
  if [[ -f "${patch_file}" ]]; then
    echo "[bootstrap_px4] applying $(basename "${patch_file}")"
    patch -p1 --forward --batch < "${patch_file}" || true
  fi
}

apply_patch "${REPO_ROOT}/patches/px4-gazebo-classic-build.patch"

echo "[bootstrap_px4] building PX4 SITL + Gazebo Classic (no launch)..."
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
export ROS_VERSION=2
make px4_sitl gazebo-classic_iris DONT_RUN=1

echo "[bootstrap_px4] done"
