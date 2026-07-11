#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET_DIR="${GICI_DATASET_1_1:-/home/theph/ws_ncs/1.1}"
OUT_DIR="${GICI_BASELINE_OUT:-${ROOT_DIR}/results/baseline/1_1}"
LOG_DIR="${GICI_BASELINE_LOG:-${ROOT_DIR}/logs/baseline/1_1}"
TEMPLATE="${ROOT_DIR}/research/config/rtk_imu_camera_rrr_1_1.yaml"
CONFIG="${OUT_DIR}/run_config.yaml"
GICI_MAIN="${GICI_MAIN:-${ROOT_DIR}/build/gici_main}"

mkdir -p "${OUT_DIR}/output" "$LOG_DIR"
OUT_SOLUTION="${OUT_DIR}/output/solution.txt"
rm -f "$OUT_SOLUTION"

if [[ ! -x "$GICI_MAIN" ]]; then
  printf 'ERROR: %s not found. Run ./scripts/build_research.sh first.\n' "$GICI_MAIN" >&2
  exit 1
fi
if [[ ! -f "$TEMPLATE" ]]; then
  printf 'ERROR: baseline template missing: %s\n' "$TEMPLATE" >&2
  exit 1
fi
if [[ ! -f "${DATASET_DIR}/ground_truth.txt" ]]; then
  printf 'ERROR: dataset 1.1 not found at %s\n' "$DATASET_DIR" >&2
  exit 1
fi

sed \
  -e "s|<GICI_ROOT>|${ROOT_DIR}|g" \
  -e "s|<DATASET_1_1>|${DATASET_DIR}|g" \
  -e "s|<OUTPUT_DIR>|${OUT_DIR}/output|g" \
  -e "s|<LOG_DIR>|${LOG_DIR}|g" \
  "$TEMPLATE" > "$CONFIG"

printf 'Running locked baseline RTK/IMU/Camera RRR on dataset 1.1\n'
printf 'Dataset: %s\n' "$DATASET_DIR"
printf 'Config:  %s\n' "$CONFIG"
printf 'Output:  %s\n\n' "$OUT_SOLUTION"

/usr/bin/time -f 'ELAPSED:%e' "$GICI_MAIN" "$CONFIG" \
  > "${LOG_DIR}/run.stdout" 2> "${LOG_DIR}/run.stderr" || {
    if [[ -s "$OUT_SOLUTION" ]]; then
      printf 'WARN: gici_main exited non-zero but solution exists (%s lines)\n' "$(wc -l < "$OUT_SOLUTION")"
    else
      exit 1
    fi
  }

lines="$(wc -l < "$OUT_SOLUTION")"
printf 'Baseline run complete: %s (%s lines)\n' "$OUT_SOLUTION" "$lines"
