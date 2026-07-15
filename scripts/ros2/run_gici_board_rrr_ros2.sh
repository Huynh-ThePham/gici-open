#!/usr/bin/env bash
# Run GICI board RTK/IMU/Camera RRR on ROS 2.
#
# Modes:
#   --postfile  (default) Direct port of option/post_estimation_RTK_RRR.yaml
#   --bag       ROS topic replay via merged ROS2 bag (experimental)
#
# Usage:
#   scripts/ros2/run_gici_board_rrr_ros2.sh [--postfile|--bag] <1.1|3.1|4.1> [rate]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"
# shellcheck source=dataset_paths.sh
source "${REPO}/scripts/dataset_paths.sh"

MODE="postfile"
POS=()
for a in "$@"; do
  case "${a}" in
    --postfile) MODE="postfile" ;;
    --bag)      MODE="bag" ;;
    *) POS+=("${a}") ;;
  esac
done
DATASET_ID="${POS[0]:-}"
RATE="${POS[1]:-1}"
if [[ -z "${DATASET_ID}" ]]; then
  echo "Usage: $0 [--postfile|--bag] <1.1|3.1|4.1> [rate]" >&2
  exit 1
fi

case "${DATASET_ID}" in
  1.1|3.1) RTCM_START="${GICI_RTCM_START_TIME:-2023.03.20}" ;;
  4.1)     RTCM_START="${GICI_RTCM_START_TIME:-2023.03.21}" ;;
  *)
    echo "Unknown dataset '${DATASET_ID}'." >&2
    exit 1
    ;;
esac

DATASET_DIR="${GICI_DATA_ROOT}/${DATASET_ID}"
OUT_DIR="${REPO}/output/ros2_gici_board/${DATASET_ID}"
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
  kill -INT "${NODE_PID}" 2>/dev/null || true
  for _ in $(seq 1 60); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.5; done
else
  BAG_OUT="${OUT_DIR}/rrr_ros2"
  CFG_SRC="${WS}/src/gici_ros2/config/ros_gici_board_rrr.yaml"
  CFG="${OUT_DIR}/ros_gici_board_rrr.yaml"
  need_ros1_bags=0
  for b in gnss_rover.bag gnss_reference.bag gnss_ephemeris.bag imu.bag image.bag; do
    [[ -f "${DATASET_DIR}/${b}" ]] || need_ros1_bags=1
  done
  [[ "${need_ros1_bags}" -eq 1 ]] && "${REPO}/scripts/ros2/build_gici_board_ros1_bags.sh" "${DATASET_ID}" "${RTCM_START}"
  [[ ! -d "${BAG_OUT}" ]] && python3 "${REPO}/scripts/ros2/gici_board_to_ros2.py" --dataset-dir "${DATASET_DIR}" --out "${BAG_OUT}"
  sed -e "s#OUTPUT_DIR#${OUT_DIR}#g" -e "s#GICI_ROOT#${REPO}#g" "${CFG_SRC}" > "${CFG}"
  echo "[ros2-board] Mode: bag replay"
  "${NODE_EXE}" "${CFG}" > "${OUT_DIR}/node.log" 2>&1 &
  NODE_PID=$!
  sleep 3
  ros2 bag play "${BAG_OUT}" --rate "${RATE}" --read-ahead-queue-size 10000 > "${OUT_DIR}/log/play.log" 2>&1 &
  PLAYER_PID=$!
  while kill -0 "${PLAYER_PID}" 2>/dev/null; do
    kill -0 "${NODE_PID}" 2>/dev/null || break
    sleep 1
  done
  wait "${PLAYER_PID}" 2>/dev/null || true
  prev=-1; same=0
  for _ in $(seq 1 180); do
    sleep 1
    c="$(grep -c GPGGA "${SOLUTION}" 2>/dev/null || echo 0)"
    [[ "${c}" -eq "${prev}" ]] && same=$((same + 1)) || same=0
    prev="${c}"
    [[ "${same}" -ge 5 ]] && break
    kill -0 "${NODE_PID}" 2>/dev/null || break
  done
  kill -INT "${NODE_PID}" 2>/dev/null || true
  for _ in $(seq 1 60); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.5; done
fi

EPOCHS="$(grep -c GPGGA "${SOLUTION}" 2>/dev/null || echo 0)"
echo "[ros2-board] Done. GPGGA epochs: ${EPOCHS}"
if [[ "${EPOCHS}" -le 0 ]]; then
  echo "[ros2-board] ERROR: no solution; see ${OUT_DIR}/node.log" >&2
  tail -30 "${OUT_DIR}/node.log" >&2 || true
  exit 1
fi
