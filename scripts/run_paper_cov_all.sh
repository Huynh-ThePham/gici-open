#!/usr/bin/env bash
# ============================================================================
# Paper-1 COVARIANCE re-run (clean tree). DO NOT use scripts/run_paper_all.sh
# (that launcher regenerates the old VA-v4 / RF chain).
#
# Fresh OUT_ROOT every time by default. Never reuse:
#   results/research/paper_repro/
#   logs/research/gici_board_va/1_1/   (old log lacks matrix fields)
#
# Stages (solo, one gici_main at a time — keep machine idle):
#   A  build + frozen baseline guard
#   B  Board 1.1 fast vs Ceres matrix validation (NEW log required)
#   C  Medium float-path isolation (GATE — abort if FAIL_TRAJ_IDENTITY)
#   D  Deep confirmatory: shadow | proposed | ceres-oracle | local (n=1 each)
#   E  Summaries
#
# Usage:
#   scripts/run_paper_cov_all.sh                          # dated OUT_ROOT
#   scripts/run_paper_cov_all.sh results/research/paper_cov_manual
#   FORCE_RERUN=1 scripts/run_paper_cov_all.sh ...       # ignore SKIP
# ============================================================================
set -u
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_ROOT="${1:-$REPO/results/research/paper_cov_$STAMP}"
LOG="$OUT_ROOT/run_paper_cov_all.log"
FORCE_RERUN="${FORCE_RERUN:-0}"
mkdir -p "$OUT_ROOT"
cd "$REPO"
mark() { echo "[$(date -Is)] $*" | tee -a "$LOG"; }

# Stale shells sometimes export URBANNAV_DATA_ROOT=.../UrbanNavDataset (missing).
# Prefer the layout that actually contains Medium/Deep; runners also self-heal.
if [[ -d /media/theph/Data1/Research/dataset/UrbanNav-HK-Medium-Urban-1 ]]; then
  export URBANNAV_DATA_ROOT=/media/theph/Data1/Research/dataset
fi
mark "URBANNAV_DATA_ROOT=${URBANNAV_DATA_ROOT:-}"

if [[ "$OUT_ROOT" == *"paper_repro"* ]]; then
  mark "ABORT: refusing OUT_ROOT under paper_repro (old VA-v4 tree). Pick a new paper_cov_* path."
  exit 1
fi

complete() { [[ -s "$1" ]] && [[ "$(wc -l < "$1")" -ge "$2" ]]; }
wait_idle() { until ! pgrep -x gici_main >/dev/null 2>&1; do sleep 10; done; }

has_matrix_log() {
  local f="$1"
  [[ -s "$f" ]] && grep -q 'max_diag_rel=' "$f"
}

mark "PAPER_COV_START out_root=$OUT_ROOT force=$FORCE_RERUN"
mark "binary=$(ls -la build/gici_main 2>/dev/null || echo MISSING)"

# ---- A: build + guard -------------------------------------------------------
mark "STAGE_A build"
./scripts/build_research.sh > "$OUT_ROOT/build.log" 2>&1 || { mark "ABORT build failed"; exit 1; }
GUARD_OUT="$OUT_ROOT/board_1_1/bl_guard"
if [[ "$FORCE_RERUN" != "1" ]] && complete "$GUARD_OUT/output/solution.txt" 7100; then
  mark "SKIP guard"
else
  wait_idle; mark "RUN  frozen guard (baseline 1.1)"
  rm -rf "$GUARD_OUT"
  mkdir -p "$GUARD_OUT"
  GICI_BASELINE_OUT="$GUARD_OUT" GICI_BASELINE_LOG="$GUARD_OUT/log" \
    bash scripts/run_author_eval_1_1.sh > "$OUT_ROOT/board_1_1/guard.stdout" 2>&1 || true
fi
GPASS=$(python3 -c "import json
try: print('PASS' if json.load(open('$GUARD_OUT/evaluation/ape_metrics.json'))['pass'] else 'FAIL')
except Exception: print('FAIL')" 2>/dev/null)
mark "GUARD $GPASS"
[[ "$GPASS" == "PASS" ]] || { mark "ABORT guard failed"; exit 1; }

# ---- B: board matrix validation --------------------------------------------
mark "STAGE_B board VA-1.1 matrix validation"
VA_OUT="$OUT_ROOT/board_1_1/va"
VA_LOG="$VA_OUT/run.stderr"
NEED_B=1
if [[ "$FORCE_RERUN" != "1" ]] && complete "$VA_OUT/output/solution.txt" 7100 && has_matrix_log "$VA_LOG"; then
  NEED_B=0
  mark "SKIP VA-1.1 (complete + matrix fields present)"
fi
if [[ "$NEED_B" == "1" ]]; then
  wait_idle; mark "RUN  VA-1.1 (must emit max_diag_rel)"
  rm -rf "$VA_OUT"
  mkdir -p "$VA_OUT/output"
  GICI_VA_OUT="$VA_OUT" GICI_VA_LOG="$VA_OUT" \
    bash scripts/run_va_1_1.sh > "$OUT_ROOT/board_1_1/va.stdout" 2>&1 || true
  if ! has_matrix_log "$VA_LOG"; then
    mark "ABORT: new board log missing max_diag_rel — binary/config mismatch"
    exit 1
  fi
  mark "DONE VA-1.1 lines=$(wc -l < "$VA_OUT/output/solution.txt" 2>/dev/null || echo 0)"
fi
python3 scripts/eval_cov_matrix_metrics.py "$VA_LOG" | tee "$OUT_ROOT/board_1_1/matrix_metrics.txt" | tee -a "$LOG"

# ---- C: Medium float-path isolation (GATE) ---------------------------------
mark "STAGE_C Medium iso (GATE)"
BL_ISO="$OUT_ROOT/iso/bl"
VA_ISO="$OUT_ROOT/iso/va"
run_iso() {
  local runner="$1" out="$2"
  local sol="$out/medium/output/solution.txt"
  if [[ "$FORCE_RERUN" != "1" ]] && complete "$sol" 25000; then
    mark "SKIP iso $(basename "$out")"
    return 0
  fi
  wait_idle; mark "RUN  iso $(basename "$out")"
  rm -rf "$out"; mkdir -p "$out"
  python3 "scripts/$runner" medium --out-root "$out" > "$out/stdout.txt" 2>&1 || true
  complete "$sol" 25000 || mark "WARN iso incomplete $(basename "$out")"
}
run_iso run_urbannav_rrr_bl_iso.py "$BL_ISO"
run_iso run_urbannav_rrr_va_iso.py "$VA_ISO"

mark "CHECK float-path identity"
python3 scripts/check_float_path_identity.py \
  --a "$BL_ISO/medium/output/solution.txt" \
  --b "$VA_ISO/medium/output/solution.txt" \
  --dataset medium | tee "$OUT_ROOT/iso/identity.txt" | tee -a "$LOG"
if ! grep -q 'PASS_TRAJ_IDENTITY' "$OUT_ROOT/iso/identity.txt"; then
  mark "ABORT: float-path isolation FAILED — do not trust Deep e2e until fixed"
  mark "See research/paper/SUBMISSION_GAPS.md (WlFix / solver / Evaluate)"
  exit 2
fi
mark "GATE PASS float-path identity"

# ---- D: Deep confirmatory (n=1 each, deterministic iso solver settings) ----
mark "STAGE_D Deep confirmatory n=1"
deep_run() {
  local runner="$1" arm="$2" extra=("${@:3}")
  local out="$OUT_ROOT/deep/$arm"
  local sol="$out/deep/output/solution.txt"
  if [[ "$FORCE_RERUN" != "1" ]] && complete "$sol" 60000; then
    mark "SKIP deep $arm"; return 0
  fi
  wait_idle; mark "RUN  deep $arm"
  rm -rf "$out"; mkdir -p "$out"
  python3 "scripts/$runner" deep --out-root "$out" "${extra[@]}" > "$out/stdout.txt" 2>&1 || true
  if complete "$sol" 60000; then mark "DONE deep $arm"; else mark "WARN deep $arm incomplete"; fi
}
deep_run run_urbannav_rrr_bl_iso.py      shadow
deep_run run_urbannav_rrr_va_iso.py      proposed
deep_run run_urbannav_rrr_va_local.py    local
# Ceres oracle is slow — allow 6h
deep_run run_urbannav_rrr_ceres_oracle.py ceres --timeout-s 21600

# ---- E: summaries -----------------------------------------------------------
mark "STAGE_E summaries"
python3 - <<PY | tee "$OUT_ROOT/deep_summary.txt" | tee -a "$LOG"
import json
from pathlib import Path
root = Path("$OUT_ROOT") / "deep"
for arm in ["shadow", "proposed", "local", "ceres"]:
    p = root / arm / "deep" / "metrics.json"
    if not p.is_file():
        print(f"{arm}: MISSING")
        continue
    m = json.loads(p.read_text())["metrics"]
    print(f"{arm}: h={m['rmse_h_m']:.3f} u={m['rmse_u_m']:.3f} yaw={m['yaw_rmse_deg']:.3f} "
          f"fix={100*m['fixed_rate']:.1f}% matched={m['n_matched']}")
PY

mark "PAPER_COV_DONE out_root=$OUT_ROOT"
mark "Next: paste board_1_1/matrix_metrics.txt + deep_summary.txt into LaTeX; do NOT cite paper_repro."
