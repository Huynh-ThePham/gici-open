#!/usr/bin/env bash
# Build GICI ROS 1 wrapper (catkin) for UrbanNav RRR.
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ros1] ERROR: ROS Noetic not found at ${ROS_SETUP}" >&2
  echo "[ros1] Install: http://wiki.ros.org/noetic/Installation/Ubuntu" >&2
  echo "[ros1] On Ubuntu 22.04 you may need a Noetic Docker or dual ROS setup." >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${ROS_SETUP}"

if ! ./scripts/verify_upstream_fidelity.sh; then
  echo "[ros1] ERROR: GICI core must match upstream f2b8579 before building wrapper." >&2
  exit 1
fi

cd "${REPO}/ros_wrapper"
catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"
echo "[ros1] Build OK. Source: source ${REPO}/ros_wrapper/devel/setup.bash"
