#!/usr/bin/env bash
# Standard ROS 2 launch entry for GICI board RRR (OpenVINS-style interface).
#
# Usage:
#   ./scripts/ros2/launch_gici_board_std.sh postfile 1.1
#   ./scripts/ros2/launch_gici_board_std.sh bag 1.1 [rate]
#
# Publishes (when estimator runs):
#   /gici/odom   nav_msgs/Odometry  (+ TF World -> base_link)
#   /gici/path   nav_msgs/Path
#   /gici/pose   geometry_msgs/PoseStamped
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"
# shellcheck source=dataset_paths.sh
source "${REPO}/scripts/dataset_paths.sh"
# shellcheck source=ros2_bag_replay_common.sh
source "${REPO}/scripts/ros2/ros2_bag_replay_common.sh"

MODE="${1:?mode: postfile|bag}"
DATASET_ID="${2:?dataset id: 1.1|3.1|4.1}"
RATE="${3:-1}"

RTCM_START="${GICI_RTCM_START_TIME:-$(gici_rtcm_start_for "${DATASET_ID}")}"
DATASET_DIR="${GICI_DATA_ROOT}/${DATASET_ID}"
OUT_DIR="${GICI_ROS2_BOARD_OUT:-${REPO}/output/ros2_gici_board_std/${DATASET_ID}}"
LOG_DIR="${OUT_DIR}/log"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

CFG="${OUT_DIR}/gici_board.yaml"
python3 "${REPO}/scripts/ros2/render_gici_config.py" \
  --mode "${MODE}" \
  --out "${CFG}" \
  --dataset-dir "${DATASET_DIR}" \
  --output-dir "${OUT_DIR}" \
  --log-dir "${LOG_DIR}" \
  --rtcm-start "${RTCM_START}"

echo "[gici-std] Mode=${MODE} dataset=${DATASET_ID} config=${CFG}"

if [[ "${MODE}" == "postfile" ]]; then
  exec ros2 launch gici_ros2 gici_board.launch.py "config:=${CFG}"
fi

# --- bag replay (reuse hardened pipeline, config from std template) ---
BAG_OUT="${OUT_DIR}/rrr_ros2"
"${REPO}/scripts/ros2/build_gici_board_ros1_bags.sh" "${DATASET_ID}" "${RTCM_START}"
if [[ ! -d "${BAG_OUT}" ]]; then
  python3 "${REPO}/scripts/ros2/gici_board_to_ros2.py" \
    --dataset-dir "${DATASET_DIR}" --out "${BAG_OUT}" --force
fi

SOLUTION="${OUT_DIR}/solution.txt"
rm -f "${SOLUTION}" "${OUT_DIR}/node.log"
cleanup_ros2_gici_session "${LOG_DIR}/cleanup.log"

NODE_EXE="${WS}/install/gici_ros2/lib/gici_ros2/gici_ros2_main"
[[ -x "${NODE_EXE}" ]] || { echo "ERROR: build ros2_wrapper first" >&2; exit 1; }

NODE_PID=""
PLAYER_PID=""
on_exit() {
  [[ -n "${PLAYER_PID}" ]] && kill -INT "${PLAYER_PID}" 2>/dev/null || true
  [[ -n "${NODE_PID}" ]] && stop_gici_node "${NODE_PID}"
  cleanup_ros2_gici_session "${LOG_DIR}/cleanup_post.log"
}
trap on_exit EXIT INT TERM

"${NODE_EXE}" "${CFG}" > "${OUT_DIR}/node.log" 2>&1 &
NODE_PID=$!

if ! wait_for_gici_node_ready "${NODE_PID}" "${OUT_DIR}/node.log" "${LOG_DIR}/ready.log" 90; then
  echo "[gici-std] ERROR: node not ready" >&2
  exit 1
fi
if ! health_check_reference_station "${BAG_OUT}" "${NODE_PID}" "${OUT_DIR}/node.log" "${LOG_DIR}/health_check.log"; then
  echo "[gici-std] WARN: reference health-check failed; continuing" >&2
fi

ros2 bag play "${BAG_OUT}" --rate "${RATE}" --read-ahead-queue-size 10000 \
  > "${LOG_DIR}/play.log" 2>&1 &
PLAYER_PID=$!

while process_alive_non_zombie "${PLAYER_PID}"; do
  process_alive_non_zombie "${NODE_PID}" || break
  sleep 1
done
wait "${PLAYER_PID}" 2>/dev/null || true
PLAYER_PID=""

drain_solution_epochs "${SOLUTION}" "${NODE_PID}" 1500
stop_gici_node "${NODE_PID}"
NODE_PID=""

EPOCHS="$(count_gpgga "${SOLUTION}")"
echo "[gici-std] Done. GPGGA=${EPOCHS} solution=${SOLUTION}"
echo "[gici-std] Topics: /gici/odom /gici/path /gici/pose (while node was running)"
[[ "${EPOCHS}" -gt 0 ]] || exit 1
