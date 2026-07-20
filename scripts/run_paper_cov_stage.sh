#!/usr/bin/env bash
# ============================================================================
# Run ONE paper-cov stage, verify, write status, exit non-zero on failure.
# Does not auto-chain — call again for the next stage after review.
#
# Usage:
#   export URBANNAV_DATA_ROOT=/media/theph/Data1/Research/dataset
#   OUT=$(cat results/research/paper_cov_LATEST)
#   scripts/run_paper_cov_stage.sh A|$OUT     # build+guard
#   scripts/run_paper_cov_stage.sh B          # board matrix
#   scripts/run_paper_cov_stage.sh C          # medium iso GATE
#   scripts/run_paper_cov_stage.sh D1         # deep shadow
#   scripts/run_paper_cov_stage.sh D2         # deep proposed
#   scripts/run_paper_cov_stage.sh D3         # deep local
#   scripts/run_paper_cov_stage.sh D4         # deep ceres oracle
#   scripts/run_paper_cov_stage.sh E          # summaries
#   scripts/run_paper_cov_stage.sh status     # print status board
# ============================================================================
set -euo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

STAGE="${1:-}"
OUT_ROOT="${2:-}"
if [[ -z "$OUT_ROOT" ]]; then
  OUT_ROOT="$(cat "$REPO/results/research/paper_cov_LATEST")"
fi
if [[ -z "$STAGE" ]]; then
  echo "usage: $0 A|B|C|D1|D2|D3|D4|E|status [OUT_ROOT]" >&2
  exit 2
fi

export URBANNAV_DATA_ROOT="${URBANNAV_DATA_ROOT:-/media/theph/Data1/Research/dataset}"
export GICI_DATA_ROOT="${GICI_DATA_ROOT:-/media/theph/Data1/Research/dataset}"
export GICI_MAIN="${GICI_MAIN:-$REPO/build/gici_main}"

mkdir -p "$OUT_ROOT/status" "$OUT_ROOT/board_1_1" "$OUT_ROOT/iso" "$OUT_ROOT/deep"
LOG="$OUT_ROOT/run_stages.log"
mark() { echo "[$(date -Is)] [$STAGE] $*" | tee -a "$LOG"; }
pass() { echo PASS > "$OUT_ROOT/status/${STAGE}.txt"; mark "RESULT PASS $*"; }
fail() { echo "FAIL: $*" > "$OUT_ROOT/status/${STAGE}.txt"; mark "RESULT FAIL $*"; exit 1; }

wait_idle() {
  if pgrep -x gici_main >/dev/null 2>&1; then
    mark "waiting for existing gici_main to finish..."
    until ! pgrep -x gici_main >/dev/null 2>&1; do sleep 10; done
  fi
}

complete_lines() { [[ -s "$1" ]] && [[ "$(wc -l < "$1")" -ge "$2" ]]; }

case "$STAGE" in
  status)
    echo "OUT_ROOT=$OUT_ROOT"
    echo "URBANNAV_DATA_ROOT=$URBANNAV_DATA_ROOT"
    ls -la "$OUT_ROOT/status" 2>/dev/null || true
    for s in A B C D1 D2 D3 D4 E; do
      f="$OUT_ROOT/status/${s}.txt"
      if [[ -f "$f" ]]; then printf '  %-3s %s\n' "$s" "$(cat "$f")"; else printf '  %-3s (pending)\n' "$s"; fi
    done
    exit 0
    ;;

  A)
    mark "build + frozen baseline guard"
    wait_idle
    ./scripts/build_research.sh > "$OUT_ROOT/build.log" 2>&1 || fail "build"
    # String lives in libgici.so (shared), not the thin gici_main wrapper.
    LIBGICI="$(dirname "$GICI_MAIN")/libgici.so"
    mark "check strings in $LIBGICI"
    if [[ ! -f "$LIBGICI" ]]; then fail "missing $LIBGICI"; fi
    if ! strings "$LIBGICI" | grep -F 'max_diag_rel=' >/dev/null; then
      fail "libgici missing max_diag_rel — rebuild VA estimator"
    fi
    mark "binary ok md5=$(md5sum "$GICI_MAIN" | awk '{print $1}') lib=$(md5sum "$LIBGICI" | awk '{print $1}')"
    GUARD_OUT="$OUT_ROOT/board_1_1/bl_guard"
    rm -rf "$GUARD_OUT"; mkdir -p "$GUARD_OUT"
    GICI_BASELINE_OUT="$GUARD_OUT" GICI_BASELINE_LOG="$GUARD_OUT/log" \
      bash scripts/run_author_eval_1_1.sh > "$OUT_ROOT/board_1_1/guard.stdout" 2>&1 || true
    GPASS=$(python3 -c "import json
try: print('PASS' if json.load(open('$GUARD_OUT/evaluation/ape_metrics.json'))['pass'] else 'FAIL')
except Exception as e: print('FAIL:'+str(e))" 2>/dev/null || echo FAIL)
    echo "$GPASS" > "$OUT_ROOT/board_1_1/guard_verdict.txt"
    [[ "$GPASS" == "PASS" ]] || fail "frozen baseline guard $GPASS"
    pass "guard APE in band"
    ;;

  B)
    [[ -f "$OUT_ROOT/status/A.txt" && "$(cat "$OUT_ROOT/status/A.txt")" == PASS ]] || fail "Stage A not PASS"
    mark "board 1.1 VA matrix validation"
    wait_idle
    VA_OUT="$OUT_ROOT/board_1_1/va"
    rm -rf "$VA_OUT"; mkdir -p "$VA_OUT/output"
    GICI_VA_OUT="$VA_OUT" GICI_VA_LOG="$VA_OUT" \
      bash scripts/run_va_1_1.sh > "$OUT_ROOT/board_1_1/va.stdout" 2>&1 || true
    VA_LOG="$VA_OUT/run.stderr"
    [[ -s "$VA_LOG" ]] || fail "missing run.stderr"
    grep -q 'max_diag_rel=' "$VA_LOG" || fail "log lacks max_diag_rel (old binary?)"
    NFAST=$(grep -c '\[vaar-fast\]' "$VA_LOG" || true)
    [[ "$NFAST" -ge 1000 ]] || fail "too few [vaar-fast] lines: $NFAST"
    python3 scripts/eval_cov_matrix_metrics.py "$VA_LOG" | tee "$OUT_ROOT/board_1_1/matrix_metrics.txt"
    complete_lines "$VA_OUT/output/solution.txt" 7100 || fail "board solution incomplete"
    pass "matrix metrics written; n_vaar_fast=$NFAST"
    ;;

  C)
    [[ -f "$OUT_ROOT/status/B.txt" && "$(cat "$OUT_ROOT/status/B.txt")" == PASS ]] || fail "Stage B not PASS"
    mark "Medium float-path isolation GATE"
    wait_idle
    BL="$OUT_ROOT/iso/bl"; VA="$OUT_ROOT/iso/va"
    rm -rf "$BL" "$VA"; mkdir -p "$BL" "$VA"
    mark "RUN medium shadow iso"
    python3 scripts/run_urbannav_rrr_bl_iso.py medium --out-root "$BL" | tee "$BL/stdout.txt"
    mark "RUN medium proposed iso"
    wait_idle
    python3 scripts/run_urbannav_rrr_va_iso.py medium --out-root "$VA" | tee "$VA/stdout.txt"
    SOLA="$BL/medium/output/solution.txt"
    SOLB="$VA/medium/output/solution.txt"
    complete_lines "$SOLA" 25000 || fail "medium bl incomplete"
    complete_lines "$SOLB" 25000 || fail "medium va incomplete"
    python3 scripts/check_float_path_identity.py \
      --a "$SOLA" --b "$SOLB" --dataset medium \
      | tee "$OUT_ROOT/iso/identity.txt"
    grep -q 'PASS_TRAJ_IDENTITY' "$OUT_ROOT/iso/identity.txt" || fail "float-path identity"
    grep -q 'PASS_FIX_ZERO' "$OUT_ROOT/iso/identity.txt" || fail "fix-zero check"
    pass "Medium identity OK"
    ;;

  D1)
    [[ -f "$OUT_ROOT/status/C.txt" && "$(cat "$OUT_ROOT/status/C.txt")" == PASS ]] || fail "Stage C not PASS"
    mark "Deep shadow (iso)"
    wait_idle
    OUTD="$OUT_ROOT/deep/shadow"; rm -rf "$OUTD"; mkdir -p "$OUTD"
    python3 scripts/run_urbannav_rrr_bl_iso.py deep --out-root "$OUTD" | tee "$OUTD/stdout.txt"
    complete_lines "$OUTD/deep/output/solution.txt" 60000 || fail "deep shadow incomplete"
    python3 -c "import json; m=json.load(open('$OUTD/deep/metrics.json'))['metrics'];
print(f\"h={m['rmse_h_m']:.3f} u={m['rmse_u_m']:.3f} yaw={m['yaw_rmse_deg']:.3f} fix={100*m['fixed_rate']:.2f}% n={m['n_matched']}\")" \
      | tee "$OUTD/summary.txt"
    pass "$(cat "$OUTD/summary.txt")"
    ;;

  D2)
    [[ -f "$OUT_ROOT/status/D1.txt" && "$(cat "$OUT_ROOT/status/D1.txt")" == PASS ]] || fail "Stage D1 not PASS"
    mark "Deep proposed (iso fast cov)"
    wait_idle
    OUTD="$OUT_ROOT/deep/proposed"; rm -rf "$OUTD"; mkdir -p "$OUTD"
    python3 scripts/run_urbannav_rrr_va_iso.py deep --out-root "$OUTD" | tee "$OUTD/stdout.txt"
    complete_lines "$OUTD/deep/output/solution.txt" 60000 || fail "deep proposed incomplete"
    python3 -c "import json; m=json.load(open('$OUTD/deep/metrics.json'))['metrics'];
print(f\"h={m['rmse_h_m']:.3f} u={m['rmse_u_m']:.3f} yaw={m['yaw_rmse_deg']:.3f} fix={100*m['fixed_rate']:.2f}% n={m['n_matched']}\")" \
      | tee "$OUTD/summary.txt"
    pass "$(cat "$OUTD/summary.txt")"
    ;;

  D3)
    [[ -f "$OUT_ROOT/status/D2.txt" && "$(cat "$OUT_ROOT/status/D2.txt")" == PASS ]] || fail "Stage D2 not PASS"
    mark "Deep local/delta negative control"
    wait_idle
    OUTD="$OUT_ROOT/deep/local"; rm -rf "$OUTD"; mkdir -p "$OUTD"
    python3 scripts/run_urbannav_rrr_va_local.py deep --out-root "$OUTD" | tee "$OUTD/stdout.txt"
    complete_lines "$OUTD/deep/output/solution.txt" 60000 || fail "deep local incomplete"
    python3 -c "import json; m=json.load(open('$OUTD/deep/metrics.json'))['metrics'];
print(f\"h={m['rmse_h_m']:.3f} u={m['rmse_u_m']:.3f} yaw={m['yaw_rmse_deg']:.3f} fix={100*m['fixed_rate']:.2f}% n={m['n_matched']}\")" \
      | tee "$OUTD/summary.txt"
    pass "$(cat "$OUTD/summary.txt")"
    ;;

  D4)
    # Allow D2→D4 directly (3-turn runbook skips optional local D3).
    if [[ -f "$OUT_ROOT/status/D3.txt" && "$(cat "$OUT_ROOT/status/D3.txt")" == PASS ]]; then
      :
    elif [[ -f "$OUT_ROOT/status/D2.txt" && "$(cat "$OUT_ROOT/status/D2.txt")" == PASS ]]; then
      mark "D3 skipped (optional local); proceeding D2 → D4"
    else
      fail "Stage D2 (or D3) not PASS"
    fi
    mark "Deep Ceres oracle (slow; timeout 6h)"
    wait_idle
    OUTD="$OUT_ROOT/deep/ceres"; rm -rf "$OUTD"; mkdir -p "$OUTD"
    python3 scripts/run_urbannav_rrr_ceres_oracle.py deep --out-root "$OUTD" --timeout-s 21600 \
      | tee "$OUTD/stdout.txt"
    complete_lines "$OUTD/deep/output/solution.txt" 60000 || fail "deep ceres incomplete"
    python3 -c "import json; m=json.load(open('$OUTD/deep/metrics.json'))['metrics'];
print(f\"h={m['rmse_h_m']:.3f} u={m['rmse_u_m']:.3f} yaw={m['yaw_rmse_deg']:.3f} fix={100*m['fixed_rate']:.2f}% n={m['n_matched']}\")" \
      | tee "$OUTD/summary.txt"
    pass "$(cat "$OUTD/summary.txt")"
    ;;

  E)
    for s in A B C D1 D2 D4; do
      [[ -f "$OUT_ROOT/status/${s}.txt" && "$(cat "$OUT_ROOT/status/${s}.txt")" == PASS ]] || fail "Stage $s not PASS"
    done
    mark "summaries (3-turn: local D3 optional)"
    {
      echo "# Board matrix"
      cat "$OUT_ROOT/board_1_1/matrix_metrics.txt"
      echo
      echo "# Medium identity"
      cat "$OUT_ROOT/iso/identity.txt"
      echo
      echo "# Deep arms"
      for arm in shadow proposed ceres; do
        echo -n "$arm: "
        cat "$OUT_ROOT/deep/$arm/summary.txt" 2>/dev/null || echo MISSING
      done
      if [[ -f "$OUT_ROOT/deep/local/summary.txt" ]]; then
        echo -n "local (optional): "
        cat "$OUT_ROOT/deep/local/summary.txt"
      fi
    } | tee "$OUT_ROOT/CAMPAIGN_SUMMARY.md"
    pass "CAMPAIGN_SUMMARY.md written — STOP after 3 turns"
    ;;

  *)
    echo "unknown stage: $STAGE" >&2
    exit 2
    ;;
esac
