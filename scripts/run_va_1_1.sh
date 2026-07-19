#!/usr/bin/env bash
# Research RTK/IMU/Camera RRR-VA on GICI dataset 1.1 (open-sky) with the fast marginal
# ambiguity covariance + per-AR-epoch equivalence/cost logging vs ceres::Covariance.
#
# Intentionally separate from run_baseline_1_1.sh: writes under results/research/ so it can
# never overwrite or masquerade as the locked baseline. Uses the byte-identical-except-AR
# config research/config/rtk_imu_camera_rrr_va_1_1.yaml.
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=dataset_paths.sh
source "${ROOT_DIR}/scripts/dataset_paths.sh"
DATASET_DIR="${GICI_DATASET_1_1}"
OUT_DIR="${GICI_VA_OUT:-${ROOT_DIR}/results/research/gici_board_va/1_1}"
LOG_DIR="${GICI_VA_LOG:-${ROOT_DIR}/logs/research/gici_board_va/1_1}"
TEMPLATE="${GICI_VA_TEMPLATE:-${ROOT_DIR}/research/config/rtk_imu_camera_rrr_va_1_1.yaml}"
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
  printf 'ERROR: VA template missing: %s\n' "$TEMPLATE" >&2
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

printf 'Running research RTK/IMU/Camera RRR-VA on dataset 1.1\n'
printf 'Dataset: %s\n' "$DATASET_DIR"
printf 'Config:  %s\n' "$CONFIG"
printf 'Output:  %s\n\n' "$OUT_SOLUTION"

"$GICI_MAIN" "$CONFIG" > "${LOG_DIR}/run.stdout" 2> "${LOG_DIR}/run.stderr" &
GICI_PID=$!
while kill -0 "$GICI_PID" 2>/dev/null; do
  if [[ -s "$OUT_SOLUTION" ]]; then
    lines_now="$(wc -l < "$OUT_SOLUTION")"
    if (( lines_now >= 7000 )); then
      sleep 3
      if [[ "$(wc -l < "$OUT_SOLUTION")" -eq "$lines_now" ]]; then
        kill -INT "$GICI_PID" 2>/dev/null || true
        break
      fi
    fi
  fi
  sleep 2
done
# Upstream gici_main may ignore SIGINT after completion (known hang); the solution
# is already complete and stable at this point, so escalate after a grace period.
for _ in $(seq 1 15); do
  kill -0 "$GICI_PID" 2>/dev/null || break
  sleep 2
done
kill -9 "$GICI_PID" 2>/dev/null || true
wait "$GICI_PID" 2>/dev/null || true

if [[ ! -s "$OUT_SOLUTION" ]]; then
  printf 'ERROR: no solution output; see %s\n' "${LOG_DIR}/run.stderr" >&2
  exit 1
fi

lines="$(wc -l < "$OUT_SOLUTION")"
printf 'VA run complete: %s (%s lines)\n' "$OUT_SOLUTION" "$lines"
printf 'Equivalence/cost log lines: %s (grep "\\[vaar-fast\\]" %s/run.stderr)\n' \
  "$(grep -c '\[vaar-fast\]' "${LOG_DIR}/run.stderr" || true)" "$LOG_DIR"
