#!/usr/bin/env bash
# Run GICI as a LIVE ROS 2 fusion node — algorithm driven by topic callbacks.
#
# Unlike postfile mode (batch replay from .bin files), live mode uses:
#   replay.enable: false  +  type: ros streamers  →  MultiSensorEstimating
# Sensor data arrives on ROS topics; rclcpp::spin feeds the estimator in real time.
#
# Usage:
#   ./scripts/ros2/launch_gici_live.sh board 1.1              # node only (connect sensors)
#   ./scripts/ros2/launch_gici_live.sh board 1.1 bag [rate]   # node + bag replay test
#   ./scripts/ros2/launch_gici_live.sh urbannav medium [rate]   # UrbanNav live + bag
#
# With sim time (bag --clock):
#   USE_SIM_TIME=1 ./scripts/ros2/launch_gici_live.sh board 1.1 bag 1.0
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"
# shellcheck source=dataset_paths.sh
source "${REPO}/scripts/dataset_paths.sh"
# shellcheck source=ros2_bag_replay_common.sh
source "${REPO}/scripts/ros2/ros2_bag_replay_common.sh"

PROFILE="${1:?profile: board|urbannav}"
ARG2="${2:?dataset id (board: 1.1|3.1|4.1 ; urbannav: medium)}"
ARG3="${3:-}"
ARG4="${4:-1}"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

USE_SIM_TIME="${USE_SIM_TIME:-0}"
SIM_ARG=""
[[ "${USE_SIM_TIME}" == "1" ]] && SIM_ARG="use_sim_time:=true"

run_board() {
  local DATASET_ID="${ARG2}"
  local SUBMODE="${ARG3:-node}"
  local RATE="${ARG4:-1}"
  local RTCM_START="${GICI_RTCM_START_TIME:-$(gici_rtcm_start_for "${DATASET_ID}")}"
  local DATASET_DIR="${GICI_DATA_ROOT}/${DATASET_ID}"
  local OUT_DIR="${GICI_ROS2_LIVE_OUT:-${REPO}/output/ros2_gici_live/${DATASET_ID}}"
  local LOG_DIR="${OUT_DIR}/log"
  mkdir -p "${LOG_DIR}"

  local CFG="${OUT_DIR}/gici_live.yaml"
  python3 "${REPO}/scripts/ros2/render_gici_config.py" \
    --mode live \
    --out "${CFG}" \
    --dataset-dir "${DATASET_DIR}" \
    --output-dir "${OUT_DIR}" \
    --log-dir "${LOG_DIR}" \
    --rtcm-start "${RTCM_START}"

  echo "[gici-live] Board profile dataset=${DATASET_ID} config=${CFG}"

  if [[ "${SUBMODE}" == "node" ]]; then
    echo "[gici-live] Waiting for sensor topics (/gici/gnss_*, /gici/imu_raw, /gici/image_raw)"
    echo "[gici-live] Publish sensors or run: ros2 bag play <bag> --rate ${RATE} ${USE_SIM_TIME:+--clock}"
    exec ros2 launch gici_ros2 gici_live.launch.py "config:=${CFG}" ${SIM_ARG}
  fi

  if [[ "${SUBMODE}" != "bag" ]]; then
    echo "Unknown board submode: ${SUBMODE} (use 'node' or 'bag')" >&2
    exit 2
  fi

  local BAG_OUT="${OUT_DIR}/rrr_ros2"
  "${REPO}/scripts/ros2/build_gici_board_ros1_bags.sh" "${DATASET_ID}" "${RTCM_START}"
  if [[ ! -d "${BAG_OUT}" ]]; then
    python3 "${REPO}/scripts/ros2/gici_board_to_ros2.py" \
      --dataset-dir "${DATASET_DIR}" --out "${BAG_OUT}" --force
  fi

  local SOLUTION="${OUT_DIR}/solution.txt"
  rm -f "${SOLUTION}" "${OUT_DIR}/node.log"
  cleanup_ros2_gici_session "${LOG_DIR}/cleanup.log"

  local NODE_EXE="${WS}/install/gici_ros2/lib/gici_ros2/gici_ros2_main"
  [[ -x "${NODE_EXE}" ]] || { echo "ERROR: build ros2_wrapper first" >&2; exit 1; }

  local NODE_PID="" PLAYER_PID=""
  on_exit() {
    [[ -n "${PLAYER_PID}" ]] && kill -INT "${PLAYER_PID}" 2>/dev/null || true
    [[ -n "${NODE_PID}" ]] && stop_gici_node "${NODE_PID}"
    cleanup_ros2_gici_session "${LOG_DIR}/cleanup_post.log"
  }
  trap on_exit EXIT INT TERM

  "${NODE_EXE}" "${CFG}" > "${OUT_DIR}/node.log" 2>&1 &
  NODE_PID=$!

  if ! wait_for_gici_node_ready "${NODE_PID}" "${OUT_DIR}/node.log" "${LOG_DIR}/ready.log" 90; then
    echo "[gici-live] ERROR: node not ready" >&2
    exit 1
  fi
  if ! health_check_reference_station "${BAG_OUT}" "${NODE_PID}" "${OUT_DIR}/node.log" "${LOG_DIR}/health_check.log"; then
    echo "[gici-live] WARN: reference health-check failed; continuing" >&2
  fi

  local PLAY_CLOCK=""
  [[ "${USE_SIM_TIME}" == "1" ]] && PLAY_CLOCK="--clock"
  ros2 bag play "${BAG_OUT}" --rate "${RATE}" --read-ahead-queue-size 10000 ${PLAY_CLOCK} \
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

  local EPOCHS
  EPOCHS="$(count_gpgga "${SOLUTION}")"
  echo "[gici-live] Done. GPGGA=${EPOCHS} solution=${SOLUTION}"
  echo "[gici-live] Live topics were: /gici/odom /gici/path /gici/pose"
  [[ "${EPOCHS}" -gt 0 ]] || exit 1
}

run_urbannav() {
  local DS="${ARG2}"
  local RATE="${ARG3:-1}"
  [[ "${DS}" == "medium" ]] || { echo "Only medium wired for urbannav live" >&2; exit 1; }

  local DATA_ROOT="${URBANNAV_DATA_ROOT:-/media/theph/Data1/Research/dataset/UrbanNavDataset}"
  local GNSS_DIR="${URBANNAV_MEDIUM_GNSS_DIR:-${DATA_ROOT}/OneDrive_1_7-11-2026/urbannav/medium}"
  local SENSORS="${URBANNAV_MEDIUM_SENSORS:-${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1/ros/UrbanNav-HK_TST-20210517_sensors.bag}"
  local OUT_DIR="${GICI_URBANNAV_LIVE_OUT:-${REPO}/output/ros2_urbannav_live/${DS}}"
  local LOG_DIR="${OUT_DIR}/log"
  local BAG="${OUT_DIR}/rrr_ros2"
  mkdir -p "${LOG_DIR}"

  local CFG="${OUT_DIR}/urbannav_live.yaml"
  python3 "${REPO}/scripts/ros2/render_gici_config.py" \
    --mode urbannav-live \
    --out "${CFG}" \
    --output-dir "${OUT_DIR}" \
    --log-dir "${LOG_DIR}"

  if [[ ! -d "${BAG}" ]]; then
    [[ -e "${SENSORS}" ]] || { echo "ERROR: missing ${SENSORS}" >&2; exit 1; }
    python3 "${REPO}/scripts/ros2/urbannav_rrr_to_ros2.py" \
      --gnss-dir "${GNSS_DIR}" --sensors "${SENSORS}" --out "${BAG}"
  fi

  echo "[gici-live] UrbanNav profile dataset=${DS} config=${CFG}"
  ros2 launch gici_ros2 gici_live.launch.py "config:=${CFG}" ${SIM_ARG} &
  local LPID=$!
  sleep 5
  local PLAY_CLOCK=""
  [[ "${USE_SIM_TIME}" == "1" ]] && PLAY_CLOCK="--clock"
  ros2 bag play "${BAG}" --rate "${RATE}" ${PLAY_CLOCK}
  kill -INT "${LPID}" 2>/dev/null || true
  wait "${LPID}" 2>/dev/null || true
}

case "${PROFILE}" in
  board) run_board ;;
  urbannav) run_urbannav ;;
  *) echo "Unknown profile: ${PROFILE}" >&2; exit 2 ;;
esac
