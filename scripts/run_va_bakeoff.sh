#!/usr/bin/env bash
# VA variant bake-off (SOLO): baseline + all VA-family variants, ONCE each, on the
# corrected 5 s-base data, across the given datasets. Rank per dataset to compare.
#
# Methods:
#   bl    run_urbannav_rrr_baseline.py  (author RRR, no VA)
#   va    run_urbannav_rrr_va.py        (VA-v2: P_s>=0.999 Teunissen gate)
#   va3   run_urbannav_rrr_va3.py       (VA-v3: soft/revocable fix)
#   va4   run_urbannav_rrr_va4.py       (VA-v4: success-rate dose)
#   rfva  run_urbannav_rrr_rfva.py      (VA-v4 + Tukey bounded-influence GNSS loss)
#   rfcva run_urbannav_rrr_rfcva.py     (VA-v4 + Cauchy GNSS loss)
#
# Resumable (skips a run whose metrics.json exists). One gici_main at a time.
# Order: cheap datasets first (medium < deep < harsh) for early signal.
# Usage:  scripts/run_va_bakeoff.sh [OUT_ROOT] [DATASETS...]
set -uo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"; cd "$REPO"
OUT="${1:-$REPO/results/research/va_bakeoff_20260720}"; shift || true
DATASETS=("$@"); [[ ${#DATASETS[@]} -eq 0 ]] && DATASETS=(medium deep harsh)
LOG="$OUT/bakeoff.log"; mkdir -p "$OUT"
mark(){ echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

# ISO (deterministic) runners: num_threads=1, max_solver_time=1e9 -> reproducible
# n=1, load- and thread-scheduling-independent (the 0.04/4-thread real-time configs
# vary run-to-run 2.4-3.8 m on Deep, which would swamp method differences).
declare -a M=(
  "bl:run_urbannav_rrr_bl_iso.py"
  "va:run_urbannav_rrr_va_iso.py"
  "va3:run_urbannav_rrr_va3_iso.py"
  "va4:run_urbannav_rrr_va4_iso.py"
  "rfva:run_urbannav_rrr_rfva_iso.py"
  "rfcva:run_urbannav_rrr_rfcva_iso.py"
)

mark "VA_BAKEOFF start datasets=[${DATASETS[*]}] out=$OUT (${#M[@]} methods, solo)"
for ds in "${DATASETS[@]}"; do
  for entry in "${M[@]}"; do
    tag="${entry%%:*}"; runner="${entry##*:}"
    out="$OUT/$tag"; metrics="$out/$ds/metrics.json"
    if [[ -s "$metrics" ]]; then mark "SKIP  $tag/$ds (exists)"; continue; fi
    mark "RUN   $tag/$ds ($runner)"
    mkdir -p "$out"
    python3 "scripts/$runner" "$ds" --out-root "$out" >> "$out/$ds.stdout" 2>&1 || true
    if [[ -s "$metrics" ]]; then
      r=$(python3 -c "import json;print(round(json.load(open('$metrics'))['metrics']['rmse_h_m'],3))" 2>/dev/null)
      mark "DONE  $tag/$ds  rmse_h=${r} m"
    else
      mark "FAIL  $tag/$ds (see $out/$ds.stdout)"
    fi
  done
  mark "---- ranking $ds ----"
  python3 "scripts/eval_va_bakeoff.py" "$OUT" "$ds" | tee -a "$LOG"
done
mark "VA_BAKEOFF complete"
