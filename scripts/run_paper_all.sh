#!/usr/bin/env bash
# ============================================================================
# Master reproduction launcher (file-mode, solo protocol).
# Regenerates every number the paper reports, one gici_main at a time on an
# otherwise-idle machine. RESUMABLE: any run whose solution.txt is already
# complete is skipped, so a killed launcher can be relaunched, and today's
# validated runs need not be redone if pointed at their root.
#
# Datasets required (all present as of 2026-07-19; see
# research/paper/DATASETS_AND_RUN_MATRIX.md): GICI-board 1.1, UrbanNav Deep,
# UrbanNav Medium. Harsh (Mongkok) is NOT part of this launcher (add later).
#
# Usage:  scripts/run_paper_all.sh [OUT_ROOT]
#   OUT_ROOT default: results/research/paper_repro
# Stages: A build+guard  B VA-1.1 equivalence  C Deep solo  D Medium  E tables
# ~3.5-4 h unattended. Progress markers in $LOG.
# ============================================================================
set -u
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_ROOT="${1:-$REPO/results/research/paper_repro}"
LOG="$OUT_ROOT/run_paper_all.log"
mkdir -p "$OUT_ROOT/deep" "$OUT_ROOT/medium" "$OUT_ROOT/board_1_1"
cd "$REPO"
mark() { echo "[$(date -Is)] $*" | tee -a "$LOG"; }

# solution completeness (lines): 1.1=7100, medium=7379ep*4, deep=15119ep*4
complete() { [[ -s "$1" ]] && [[ "$(wc -l < "$1")" -ge "$2" ]]; }
wait_idle() { until ! pgrep -x gici_main >/dev/null 2>&1; do sleep 10; done; }

# Run one UrbanNav arm/run if not already complete. $1 runner.py $2 dataset $3 arm $4 k $5 min-lines
urbannav_run() {
  local runner="$1" ds="$2" arm="$3" k="$4" minl="$5"
  local out="$OUT_ROOT/$ds/$arm/run$k"
  local sol="$out/$ds/output/solution.txt"
  if complete "$sol" "$minl"; then mark "SKIP $arm $ds run$k (complete)"; return 0; fi
  wait_idle
  mark "RUN  $arm $ds run$k"
  python3 "scripts/$runner" "$ds" --out-root "$out" > "$out.stdout" 2>&1 || true
  if complete "$sol" "$minl"; then mark "DONE $arm $ds run$k"; else mark "WARN $arm $ds run$k INCOMPLETE ($([[ -s $sol ]] && wc -l < "$sol" || echo 0) lines)"; fi
}

mark "PAPER_ALL_START out_root=$OUT_ROOT"

# ---- Stage A: build + frozen-baseline guard (gate) --------------------------
mark "STAGE_A build"
./scripts/build_research.sh > "$OUT_ROOT/build.log" 2>&1 || { mark "ABORT build failed"; exit 1; }
GUARD_OUT="$OUT_ROOT/board_1_1/bl_guard"
if complete "$GUARD_OUT/output/solution.txt" 7100; then
  mark "SKIP guard run (complete)"
else
  wait_idle; mark "RUN  frozen guard (baseline 1.1, flags off)"
  GICI_BASELINE_OUT="$GUARD_OUT" GICI_BASELINE_LOG="$GUARD_OUT/log" \
    bash scripts/run_author_eval_1_1.sh > "$OUT_ROOT/board_1_1/guard.stdout" 2>&1 || true
fi
GPASS=$(python3 -c "import json,sys;
try: print('PASS' if json.load(open('$GUARD_OUT/evaluation/ape_metrics.json'))['pass'] else 'FAIL')
except Exception as e: print('FAIL')" 2>/dev/null)
mark "GUARD $GPASS"
if [[ "$GPASS" != "PASS" ]]; then mark "ABORT frozen guard failed -- build perturbs baseline"; exit 1; fi

# ---- Stage B: VA-1.1 covariance equivalence + cost (Table 1) -----------------
mark "STAGE_B VA-1.1 equivalence"
VA_OUT="$OUT_ROOT/board_1_1/va"
if complete "$VA_OUT/output/solution.txt" 7100; then
  mark "SKIP VA-1.1 (complete)"
else
  wait_idle; mark "RUN  VA-1.1 (benchmark log)"
  GICI_VA_OUT="$VA_OUT" GICI_VA_LOG="$VA_OUT" \
    bash scripts/run_va_1_1.sh > "$OUT_ROOT/board_1_1/va.stdout" 2>&1 || true
  mark "DONE VA-1.1 ($([[ -s $VA_OUT/output/solution.txt ]] && wc -l < "$VA_OUT/output/solution.txt" || echo 0) lines)"
fi

# ---- Stage C: UrbanNav Deep, solo -------------------------------------------
mark "STAGE_C Deep solo"
for k in 1 2 3; do urbannav_run run_urbannav_rrr_baseline.py deep bl       "$k" 60000; done
for k in 1 2 3; do urbannav_run run_urbannav_rrr_va4.py      deep va4      "$k" 60000; done
for k in 1 2 3; do urbannav_run run_urbannav_rrr_rfcva.py    deep rfcauchy "$k" 60000; done
urbannav_run run_urbannav_rrr_va.py   deep va2     1 60000   # context
urbannav_run run_urbannav_rrr_va3.py  deep va3     1 60000   # context
urbannav_run run_urbannav_rrr_rfva.py deep rftukey 1 60000   # context (expected crash)

# ---- Stage D: UrbanNav Medium (noise floor) ---------------------------------
mark "STAGE_D Medium"
urbannav_run run_urbannav_rrr_baseline.py medium bl  1 29000
urbannav_run run_urbannav_rrr_va4.py      medium va4 1 29000

# ---- Stage E: tables --------------------------------------------------------
mark "STAGE_E tables"
wait_idle
python3 scripts/eval_paper_tables.py "$OUT_ROOT" --tier3 \
  --va-log "$VA_OUT/run.stderr" > "$OUT_ROOT/tables.stdout" 2>&1 || \
  python3 scripts/eval_paper_tables.py "$OUT_ROOT" --tier3 --va-log "$VA_OUT/run.stderr" 2>&1 | tail -40
mark "PAPER_ALL_DONE tables -> $OUT_ROOT/paper_tables.md"
