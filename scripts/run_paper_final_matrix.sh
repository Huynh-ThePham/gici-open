#!/usr/bin/env bash
# Paper final evaluation matrix (SOLO protocol, n=3 independent repeats).
#
# For each of 3 repeats × 3 datasets {medium,deep,harsh} × 2 methods:
#   baseline  = run_urbannav_rrr_baseline.py  (author RRR, rtk_imu_camera_rrr)
#   proposed  = run_urbannav_rrr_va4.py        (vision-aided AR, rtk_imu_camera_rrr_va, va4)
# = 18 runs total, run ONE AT A TIME (post-file replay is throughput/wall-clock
# paced, so a fair paired comparison requires the machine otherwise idle).
#
# Resumable: a run whose metrics.json already exists is skipped, so re-launching
# after an interruption continues where it stopped. Each run's own evaluation
# (raw-ENU RMSE vs GT; Harsh auto-gated to Q<=2 and t_rel[453,2764]s) is written
# by the runner into <out>/<ds>/metrics.json.
#
# Usage:  scripts/run_paper_final_matrix.sh [OUT_ROOT]
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
OUT="${1:-$REPO/results/research/paper_final_20260720}"
LOG="$OUT/matrix.log"
mkdir -p "$OUT"

mark(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

run_one(){  # method_tag runner_py dataset rep
  local tag="$1" runner="$2" ds="$3" rep="$4"
  local out="$OUT/rep${rep}/${tag}"
  local metrics="$out/${ds}/metrics.json"
  if [[ -s "$metrics" ]]; then
    mark "SKIP  rep${rep} ${tag} ${ds} (metrics.json exists)"
    return 0
  fi
  mark "RUN   rep${rep} ${tag} ${ds}"
  mkdir -p "$out"
  python3 "scripts/${runner}" "$ds" --out-root "$out" >> "$out/${ds}.stdout" 2>&1
  if [[ -s "$metrics" ]]; then
    local rmse; rmse=$(python3 -c "import json;print(round(json.load(open('$metrics'))['metrics']['rmse_h_m'],3))" 2>/dev/null)
    mark "DONE  rep${rep} ${tag} ${ds}  rmse_h=${rmse} m"
  else
    mark "FAIL  rep${rep} ${tag} ${ds}  (see $out/${ds}.stdout)"
  fi
}

mark "PAPER_FINAL_MATRIX start out=$OUT (18 runs, solo)"
for rep in 1 2 3; do
  for ds in medium deep harsh; do
    run_one bl  run_urbannav_rrr_baseline.py "$ds" "$rep"
    run_one va4 run_urbannav_rrr_va4.py      "$ds" "$rep"
  done
done
mark "PAPER_FINAL_MATRIX all runs done -> aggregating"
python3 "scripts/eval_paper_final_matrix.py" "$OUT" | tee -a "$LOG"
mark "PAPER_FINAL_MATRIX complete"
