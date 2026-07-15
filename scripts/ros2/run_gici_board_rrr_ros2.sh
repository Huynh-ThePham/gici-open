#!/usr/bin/env bash
# Run GICI board RTK/IMU/Camera RRR on ROS 2.
#
# Modes:
#   --postfile  (default) Direct port of option/post_estimation_RTK_RRR.yaml
#   --bag       ROS topic replay via merged ROS2 bag (real-time path)
#   --force-rebuild-bags  re-convert *.bin -> ROS1 bags even if present
#   --force-reconvert     re-build merged ROS2 bag even if present
#
# Usage:
#   scripts/ros2/run_gici_board_rrr_ros2.sh [--postfile|--bag] <1.1|3.1|4.1> [rate]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"
# shellcheck source=dataset_paths.sh
source "${REPO}/scripts/dataset_paths.sh"
# shellcheck source=ros2_bag_replay_common.sh
source "${REPO}/scripts/ros2/ros2_bag_replay_common.sh"

MODE="postfile"
FORCE_BAG_REBUILD=0
FORCE_RECONVERT=0
POS=()
for a in "$@"; do
  case "${a}" in
    --postfile) MODE="postfile" ;;
    --bag)      MODE="bag" ;;
    --force-rebuild-bags) FORCE_BAG_REBUILD=1 ;;
    --force-reconvert)    FORCE_RECONVERT=1 ;;
    *) POS+=("${a}") ;;
  esac
done
DATASET_ID="${POS[0]:-}"
RATE="${POS[1]:-1}"
if [[ -z "${DATASET_ID}" ]]; then
  echo "Usage: $0 [--postfile|--bag] <1.1|3.1|4.1> [rate]" >&2
  exit 1
fi

# RTCM start date resolved from the single source of truth in dataset_paths.sh.
RTCM_START="${GICI_RTCM_START_TIME:-$(gici_rtcm_start_for "${DATASET_ID}")}"

DATASET_DIR="${GICI_DATA_ROOT}/${DATASET_ID}"
OUT_DIR="${GICI_ROS2_BOARD_OUT:-${REPO}/output/ros2_gici_board/${DATASET_ID}}"
LOG_DIR="${REPO}/logs/baseline/gici_board_ros2/${DATASET_ID}"
mkdir -p "${OUT_DIR}/log" "${LOG_DIR}"
SOLUTION="${OUT_DIR}/solution.txt"
rm -f "${SOLUTION}"

for f in ground_truth.txt gnss_rover.bin gnss_reference.bin gnss_ephemeris.bin imu.bin camera.bin; do
  [[ -f "${DATASET_DIR}/${f}" ]] || {
    echo "ERROR: missing ${DATASET_DIR}/${f}" >&2
    echo "Hint: ./scripts/setup_gici_datasets.sh ${DATASET_ID}" >&2
    exit 1
  }
done

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

NODE_EXE="${WS}/install/gici_ros2/lib/gici_ros2/gici_ros2_main"
[[ -x "${NODE_EXE}" ]] || { echo "ERROR: build ros2_wrapper first" >&2; exit 1; }

stop_node() {
  local pid="$1"
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 15); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 2
  done
  kill -TERM "${pid}" 2>/dev/null || true
  sleep 2
  kill -KILL "${pid}" 2>/dev/null || true
}

if [[ "${MODE}" == "postfile" ]]; then
  CFG_SRC="${WS}/src/gici_ros2/config/ros_gici_board_postfile_rrr.yaml"
  CFG="${OUT_DIR}/ros_gici_board_postfile_rrr.yaml"
  sed -e "s#DATASET_DIR#${DATASET_DIR}#g" \
      -e "s#GICI_ROOT#${REPO}#g" \
      -e "s#OUTPUT_DIR#${OUT_DIR}#g" \
      -e "s#LOG_DIR#${LOG_DIR}#g" \
      -e "s#RTCM_START#${RTCM_START}#g" \
      "${CFG_SRC}" > "${CFG}"
  echo "[ros2-board] Mode: postfile (post_estimation_RTK_RRR.yaml port)"
  echo "[ros2-board] Dataset: ${DATASET_ID}"
  echo "[ros2-board] Config:  ${CFG}"
  echo "[ros2-board] Output:  ${SOLUTION}"
  "${NODE_EXE}" "${CFG}" > "${OUT_DIR}/node.log" 2>&1 &
  NODE_PID=$!
  MIN_LINES="${GICI_MIN_SOLUTION_LINES:-5000}"
  while kill -0 "${NODE_PID}" 2>/dev/null; do
    if [[ -s "${SOLUTION}" ]]; then
      lines_now="$(wc -l < "${SOLUTION}")"
      if (( lines_now >= MIN_LINES )); then
        sleep 3
        [[ "$(wc -l < "${SOLUTION}")" -eq "${lines_now}" ]] && break
      fi
    fi
    sleep 2
  done
  stop_node "${NODE_PID}"
else
  BAG_OUT="${OUT_DIR}/rrr_ros2"
  CFG_SRC="${WS}/src/gici_ros2/config/ros_gici_board_bag_hybrid_rrr.yaml"
  CFG="${OUT_DIR}/ros_gici_board_bag_hybrid_rrr.yaml"

  if (( FORCE_BAG_REBUILD )); then
    GICI_FORCE_BAG_REBUILD=1 "${REPO}/scripts/ros2/build_gici_board_ros1_bags.sh" "${DATASET_ID}" "${RTCM_START}"
  else
    "${REPO}/scripts/ros2/build_gici_board_ros1_bags.sh" "${DATASET_ID}" "${RTCM_START}"
  fi

  if (( FORCE_RECONVERT )) && [[ -d "${BAG_OUT}" ]]; then
    echo "[ros2-board] Removing existing ROS2 bag ${BAG_OUT}"
    rm -rf "${BAG_OUT}"
  fi
  if [[ ! -d "${BAG_OUT}" ]]; then
    python3 "${REPO}/scripts/ros2/gici_board_to_ros2.py" \
      --dataset-dir "${DATASET_DIR}" --out "${BAG_OUT}" --force
  fi

  sed -e "s#DATASET_DIR#${DATASET_DIR}#g" \
      -e "s#OUTPUT_DIR#${OUT_DIR}#g" \
      -e "s#GICI_ROOT#${REPO}#g" \
      -e "s#RTCM_START#${RTCM_START}#g" \
      "${CFG_SRC}" > "${CFG}"
  echo "[ros2-board] Mode: bag replay (hybrid: rover/ref/imu/cam from bag, eph+DCB from bin)"
  echo "[ros2-board] Dataset: ${DATASET_ID}  rate=${RATE}"
  echo "[ros2-board] ROS2 bag: ${BAG_OUT}"
  echo "[ros2-board] Output:   ${SOLUTION}"

  REPLAY_LOG="${OUT_DIR}/log/replay.log"
  HC_LOG="${OUT_DIR}/log/health_check.log"
  CLEANUP_LOG="${OUT_DIR}/log/cleanup.log"
  READY_LOG="${OUT_DIR}/log/ready.log"
  MAX_ATTEMPTS="${GICI_BAG_REPLAY_ATTEMPTS:-2}"
  MIN_EPOCHS="${GICI_MIN_SOLUTION_EPOCHS:-1500}"
  ATTEMPT_OK=0

  cleanup_ros2_gici_session "${CLEANUP_LOG}"

  for ((attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)); do
    rm -f "${SOLUTION}"
    : > "${OUT_DIR}/node.log"
    {
      echo "=== attempt ${attempt}/${MAX_ATTEMPTS} $(date -Is) ==="
      echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-default}"
    } >>"${REPLAY_LOG}"

    if (( attempt > 1 )); then
      echo "[ros2-board] RETRY ${attempt}/${MAX_ATTEMPTS} after health-check failure" | tee -a "${REPLAY_LOG}"
      cleanup_ros2_gici_session "${CLEANUP_LOG}"
      sleep 2
    fi

    "${NODE_EXE}" "${CFG}" >> "${OUT_DIR}/node.log" 2>&1 &
    NODE_PID=$!

    if ! wait_for_gici_node_ready "${NODE_PID}" "${OUT_DIR}/node.log" "${READY_LOG}" "${GICI_NODE_READY_TIMEOUT_S:-90}"; then
      echo "[ros2-board] node not ready on attempt ${attempt}" | tee -a "${REPLAY_LOG}"
      stop_gici_node "${NODE_PID}"
      continue
    fi

    if ! health_check_reference_station "${BAG_OUT}" "${NODE_PID}" "${OUT_DIR}/node.log" "${HC_LOG}"; then
      echo "[ros2-board] reference health-check failed on attempt ${attempt}" | tee -a "${REPLAY_LOG}"
      stop_gici_node "${NODE_PID}"
      cleanup_ros2_gici_session "${CLEANUP_LOG}"
      continue
    fi

    echo "[ros2-board] Starting full bag play (rate=${RATE})" >>"${REPLAY_LOG}"
    ros2 bag play "${BAG_OUT}" --rate "${RATE}" --read-ahead-queue-size 10000 \
      > "${OUT_DIR}/log/play.log" 2>&1 &
    PLAYER_PID=$!

    while process_alive_non_zombie "${PLAYER_PID}"; do
      process_alive_non_zombie "${NODE_PID}" || break
      sleep 1
    done
    wait "${PLAYER_PID}" 2>/dev/null || true

    drain_solution_epochs "${SOLUTION}" "${NODE_PID}" "${MIN_EPOCHS}"
    stop_gici_node "${NODE_PID}"

    EPOCHS="$(count_gpgga "${SOLUTION}")"
    echo "[ros2-board] attempt ${attempt}: GPGGA epochs=${EPOCHS}" >>"${REPLAY_LOG}"
    if (( EPOCHS > 0 )); then
      ATTEMPT_OK=1
      break
    fi
    echo "[ros2-board] empty solution on attempt ${attempt}" | tee -a "${REPLAY_LOG}"
    cleanup_ros2_gici_session "${CLEANUP_LOG}"
  done

  if (( ATTEMPT_OK == 0 )); then
    echo "[ros2-board] ERROR: all ${MAX_ATTEMPTS} attempt(s) failed; see ${REPLAY_LOG} ${HC_LOG}" >&2
    exit 1
  fi
fi

EPOCHS="$(count_gpgga "${SOLUTION}")"
echo "[ros2-board] Done. GPGGA epochs: ${EPOCHS}"
if [[ "${EPOCHS}" -le 0 ]]; then
  echo "[ros2-board] ERROR: no solution; see ${OUT_DIR}/node.log" >&2
  tail -30 "${OUT_DIR}/node.log" >&2 || true
  exit 1
fi
