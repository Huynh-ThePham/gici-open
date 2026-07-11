#!/usr/bin/env bash
# Run the ported GICI ROS 2 wrapper on UrbanNav (GNSS-only RTK by default).
#
# Steps performed:
#   1. Convert the UrbanNav GICI GNSS ROS1 bags -> one ROS 2 bag (if not done yet).
#   2. Resolve the OUTPUT_DIR placeholder in the config into output/ros2_urbannav/<ds>.
#   3. Launch gici_ros2_main and play the ROS 2 GNSS bag.
#
# Usage:
#   scripts/ros2/run_urbannav_ros2.sh medium [rate]
#   scripts/ros2/run_urbannav_ros2.sh deep   [rate]
#
# Notes:
#   - GICI processes GNSS from the message week/tow, not the bag clock, so playback
#     rate only affects pacing. Keep rate near 1 (default): the upstream f2b8579 core
#     is multi-threaded and does not like GNSS arriving much faster than ~1 Hz.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"
DS="${1:-medium}"
RATE="${2:-1}"

# Dataset bag locations (override with env vars if your dataset lives elsewhere)
DATA_ROOT="${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}"
case "${DS}" in
  medium)
    GNSS_DIR="${URBANNAV_MEDIUM_GNSS_DIR:-${DATA_ROOT}/OneDrive_1_7-11-2026/urbannav/medium}"
    ;;
  deep)
    GNSS_DIR="${URBANNAV_DEEP_GNSS_DIR:-${DATA_ROOT}/UrbanNav-HK-Deep-Urban-1/deep}"
    ;;
  *)
    echo "Unknown dataset '${DS}'. Use 'medium' or 'deep'." >&2
    exit 1
    ;;
esac

OUT_DIR="${REPO}/output/ros2_urbannav/${DS}"
BAG_OUT="${OUT_DIR}/gnss_ros2"
mkdir -p "${OUT_DIR}/log"

# ROS environment
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

# 1. Convert GNSS bags -> ROS 2 (once)
if [[ ! -d "${BAG_OUT}" ]]; then
  echo "[run] Converting GNSS ROS1 bags -> ROS 2 bag at ${BAG_OUT}"
  python3 "${REPO}/scripts/ros2/urbannav_gnss_bag_to_ros2.py" \
    --in "${GNSS_DIR}/gnss_rover.bag" \
         "${GNSS_DIR}/gnss_reference.bag" \
         "${GNSS_DIR}/gnss_ephemeris_G.bag" \
         "${GNSS_DIR}/gnss_ephemeris_R.bag" \
         "${GNSS_DIR}/gnss_ephemeris_E.bag" \
         "${GNSS_DIR}/gnss_ephemeris_C.bag" \
    --out "${BAG_OUT}"
else
  echo "[run] Reusing existing ROS 2 bag at ${BAG_OUT}"
fi

# 2. Resolve config placeholder
CFG_SRC="${WS}/install/gici_ros2/share/gici_ros2/config/ros_urbannav.yaml"
CFG="${OUT_DIR}/ros_urbannav.yaml"
sed "s#OUTPUT_DIR#${OUT_DIR}#g" "${CFG_SRC}" > "${CFG}"
echo "[run] Config: ${CFG}"
echo "[run] Solutions will be written to ${OUT_DIR}/solution.txt and solution_2.txt"

# 3. Launch node + play bag
# NOTE: run the executable DIRECTLY, not via `ros2 run`. When launched in the
# background/non-interactively, `ros2 run` does not forward SIGINT to the child
# gici_ros2_main, so the node never shuts down (it hangs spinning). Invoking the
# binary directly lets rclcpp's own SIGINT handler stop it cleanly.
NODE_EXE="${WS}/install/gici_ros2/lib/gici_ros2/gici_ros2_main"
echo "[run] Starting gici_ros2_main ..."
"${NODE_EXE}" "${CFG}" > "${OUT_DIR}/node.log" 2>&1 &
NODE_PID=$!
sleep 3

echo "[run] Playing ROS 2 GNSS bag at rate ${RATE} ..."
# The node subscribes with reliable + keep-all QoS, so no messages are dropped even
# at high rate; a large read-ahead queue lets the player buffer ahead.
ros2 bag play "${BAG_OUT}" --rate "${RATE}" --read-ahead-queue-size 5000 || true

# Let the estimator drain, then stop the node cleanly (flushes the solution file).
sleep 5
kill -INT "${NODE_PID}" 2>/dev/null || true
# Give the node time to join its estimator/stream threads and flush output.
for _ in $(seq 1 30); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.5; done
kill -9 "${NODE_PID}" 2>/dev/null || true

echo "[run] Done. Solution epochs (GPGGA):"
grep -c GPGGA "${OUT_DIR}/solution_2.txt" 2>/dev/null || echo 0
echo "[run] Node log: ${OUT_DIR}/node.log"
