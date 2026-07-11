#!/usr/bin/env bash
# Run the ported GICI ROS 2 wrapper on UrbanNav in FULL RRR mode
# (RTK + IMU + camera, tightly coupled: rtk_imu_camera_rrr).
#
# Steps:
#   1. Build ONE merged ROS 2 bag (GNSS + IMU + left camera) with sensor stamps
#      shifted onto the GNSS GPST timeline and ephemeris bursted up-front.
#   2. Resolve OUTPUT_DIR in the RRR config.
#   3. Launch gici_ros2_main and play the merged bag.
#
# Usage:
#   scripts/ros2/run_urbannav_rrr_ros2.sh [medium] [rate]
#
# Notes:
#   - RRR is heavier than GNSS-only. rate=1 (default) plays at real time (~13 min).
#     Inputs subscribe with reliable + keep-all QoS, so if the estimator lags the
#     player blocks (no dropped IMU/image) instead of racing ahead.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"
DS="${1:-medium}"
RATE="${2:-1}"

DATA_ROOT="${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}"
case "${DS}" in
  medium)
    GNSS_DIR="${URBANNAV_MEDIUM_GNSS_DIR:-${DATA_ROOT}/OneDrive_1_7-11-2026/urbannav/medium}"
    SENSORS="${URBANNAV_MEDIUM_SENSORS:-${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1/ros/UrbanNav-HK_TST-20210517_sensors.bag}"
    ;;
  *)
    echo "Only 'medium' is wired up for RRR (needs the matching sensors.bag)." >&2
    exit 1
    ;;
esac

OUT_DIR="${REPO}/output/ros2_urbannav_rrr/${DS}"
BAG_OUT="${OUT_DIR}/rrr_ros2"
mkdir -p "${OUT_DIR}/log"

set +u  # ROS setup scripts reference unbound vars under `set -u`
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

# 1. Build the merged RRR ROS 2 bag (once)
if [[ ! -d "${BAG_OUT}" ]]; then
  echo "[rrr] Building merged GNSS+IMU+camera ROS 2 bag at ${BAG_OUT}"
  python3 "${REPO}/scripts/ros2/urbannav_rrr_to_ros2.py" \
    --gnss-dir "${GNSS_DIR}" \
    --sensors  "${SENSORS}" \
    --out      "${BAG_OUT}"
else
  echo "[rrr] Reusing existing ROS 2 bag at ${BAG_OUT}"
fi

# 2. Resolve config placeholder (read from source tree -> no rebuild needed)
CFG_SRC="${WS}/src/gici_ros2/config/ros_urbannav_rrr.yaml"
CFG="${OUT_DIR}/ros_urbannav_rrr.yaml"
sed "s#OUTPUT_DIR#${OUT_DIR}#g" "${CFG_SRC}" > "${CFG}"
echo "[rrr] Config: ${CFG}"
echo "[rrr] Trajectory -> ${OUT_DIR}/solution.txt"

# 3. Launch node + play bag (run the binary directly so SIGINT reaches it)
# Defensive: kill any stale node/player from a previous run. Two bag players
# publishing to the same topics at different positions makes the estimator see
# time going backwards ("Image timestamp descending") and drop everything.
pkill -9 -f "bag play .*rrr_ros2" 2>/dev/null || true
pkill -9 -f "gici_ros2_main" 2>/dev/null || true
sleep 1
NODE_EXE="${WS}/install/gici_ros2/lib/gici_ros2/gici_ros2_main"
echo "[rrr] Starting gici_ros2_main ..."
"${NODE_EXE}" "${CFG}" > "${OUT_DIR}/node.log" 2>&1 &
NODE_PID=$!
sleep 3

echo "[rrr] Playing merged bag at rate ${RATE} ..."
ros2 bag play "${BAG_OUT}" --rate "${RATE}" --read-ahead-queue-size 10000 || true

# Let the estimator drain, then stop cleanly (flushes the solution file).
sleep 8
kill -INT "${NODE_PID}" 2>/dev/null || true
for _ in $(seq 1 60); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.5; done
kill -9 "${NODE_PID}" 2>/dev/null || true

echo "[rrr] Done. Solution epochs (GPGGA):"
grep -c GPGGA "${OUT_DIR}/solution.txt" 2>/dev/null || echo 0
echo "[rrr] Node log tail:"
tail -n 20 "${OUT_DIR}/node.log" 2>/dev/null || true
