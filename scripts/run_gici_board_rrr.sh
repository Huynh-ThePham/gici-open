#!/usr/bin/env bash
# Run upstream GICI-open post-file RTK/IMU/Camera RRR on a GICI board dataset.
# Uses author template: option/post_estimation_RTK_RRR.yaml @ f2b8579
#
# Usage:
#   ./scripts/run_gici_board_rrr.sh 1.1   # RTCM start auto-resolved per dataset
#   ./scripts/run_gici_board_rrr.sh 3.1   # (see gici_rtcm_start_for in dataset_paths.sh)
#   ./scripts/run_gici_board_rrr.sh 4.1
# Override with GICI_RTCM_START_TIME=YYYY.MM.DD if needed.
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=dataset_paths.sh
source "${ROOT_DIR}/scripts/dataset_paths.sh"

DATASET_ID="${1:-}"
if [[ -z "$DATASET_ID" ]]; then
  printf 'Usage: %s <dataset-id>   e.g. 1.1, 3.1, 4.1\n' "$0" >&2
  exit 1
fi

DATASET_DIR="${GICI_DATA_ROOT}/${DATASET_ID}"
OUT_DIR="${GICI_BASELINE_OUT:-${ROOT_DIR}/results/baseline/gici_board/${DATASET_ID}}"
LOG_DIR="${GICI_BASELINE_LOG:-${ROOT_DIR}/logs/baseline/gici_board/${DATASET_ID}}"
TEMPLATE="${ROOT_DIR}/option/post_estimation_RTK_RRR.yaml"
CONFIG="${OUT_DIR}/run_config.yaml"
GICI_MAIN="${GICI_MAIN:-${ROOT_DIR}/build/gici_main}"
RTCM_START="${GICI_RTCM_START_TIME:-$(gici_rtcm_start_for "$DATASET_ID")}"
MIN_SOLUTION_LINES="${GICI_MIN_SOLUTION_LINES:-5000}"

mkdir -p "${OUT_DIR}/output" "$LOG_DIR"
OUT_SOLUTION="${OUT_DIR}/output/solution.txt"
rm -f "$OUT_SOLUTION"

if [[ ! -x "$GICI_MAIN" ]]; then
  printf 'ERROR: %s not found — run ./scripts/build_research.sh\n' "$GICI_MAIN" >&2
  exit 1
fi
if [[ ! -f "$TEMPLATE" ]]; then
  printf 'ERROR: upstream template missing: %s\n' "$TEMPLATE" >&2
  exit 1
fi
for f in ground_truth.txt gnss_rover.bin gnss_reference.bin gnss_ephemeris.bin imu.bin camera.bin; do
  if [[ ! -f "${DATASET_DIR}/${f}" ]]; then
    printf 'ERROR: missing %s/%s\n' "$DATASET_DIR" "$f" >&2
    printf 'Hint: ./scripts/setup_gici_datasets.sh %s\n' "$DATASET_ID" >&2
    exit 1
  fi
done

sed \
  -e "s|<data-directory>|${DATASET_DIR}|g" \
  -e "s|<gici-root-directory>|${ROOT_DIR}|g" \
  -e "s|<output-directory>|${OUT_DIR}/output|g" \
  -e "s|<log-directory>|${LOG_DIR}|g" \
  -e "s|start_time: 2023.03.20|start_time: ${RTCM_START}|g" \
  "$TEMPLATE" > "$CONFIG"

printf 'GICI board RRR — upstream post_estimation_RTK_RRR.yaml\n'
printf '  Dataset : %s\n' "$DATASET_DIR"
printf '  Config  : %s\n' "$CONFIG"
printf '  Output  : %s\n\n' "$OUT_SOLUTION"

"$GICI_MAIN" "$CONFIG" > "${LOG_DIR}/run.stdout" 2> "${LOG_DIR}/run.stderr" &
GICI_PID=$!
while kill -0 "$GICI_PID" 2>/dev/null; do
  if [[ -s "$OUT_SOLUTION" ]]; then
    lines_now="$(wc -l < "$OUT_SOLUTION")"
    if (( lines_now >= MIN_SOLUTION_LINES )); then
      sleep 3
      if [[ "$(wc -l < "$OUT_SOLUTION")" -eq "$lines_now" ]]; then
        kill -INT "$GICI_PID" 2>/dev/null || true
        for _ in $(seq 1 15); do
          kill -0 "$GICI_PID" 2>/dev/null || break
          sleep 2
        done
        kill -TERM "$GICI_PID" 2>/dev/null || true
        sleep 2
        kill -KILL "$GICI_PID" 2>/dev/null || true
        break
      fi
    fi
  fi
  sleep 2
done
wait "$GICI_PID" 2>/dev/null || true

if [[ ! -s "$OUT_SOLUTION" ]]; then
  printf 'ERROR: no solution; see %s\n' "${LOG_DIR}/run.stderr" >&2
  exit 1
fi

lines="$(wc -l < "$OUT_SOLUTION")"
gpgga="$(grep -c GPGGA "$OUT_SOLUTION" || true)"
printf 'Done: %s (%s lines, %s GPGGA)\n' "$OUT_SOLUTION" "$lines" "$gpgga"
