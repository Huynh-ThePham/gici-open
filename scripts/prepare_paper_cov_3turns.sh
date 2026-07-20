#!/usr/bin/env bash
# Prepare ONLY — creates a fresh OUT_ROOT and READY checklist. Does NOT run gici_main.
set -euo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"

export URBANNAV_DATA_ROOT=/media/theph/Data1/Research/dataset
export GICI_DATA_ROOT=/media/theph/Data1/Research/dataset
export GICI_MAIN="$REPO/build/gici_main"

if pgrep -x gici_main >/dev/null 2>&1; then
  echo "ERROR: gici_main is running. Stop it before preparing a clean campaign." >&2
  exit 1
fi

STAMP=$(date +%Y%m%d_%H%M%S)
OUT="$REPO/results/research/paper_cov_${STAMP}_3turns"
mkdir -p "$OUT"/{board_1_1,iso,deep,status}
echo "$OUT" > "$REPO/results/research/paper_cov_LATEST"

LIBGICI="$REPO/build/libgici.so"
test -x "$GICI_MAIN"
test -f "$LIBGICI"
strings "$LIBGICI" | grep -F 'max_diag_rel=' >/dev/null

MD5_MAIN=$(md5sum "$GICI_MAIN" | awk '{print $1}')
MD5_LIB=$(md5sum "$LIBGICI" | awk '{print $1}')

# Validate arms (no execution of estimator)
fail=0
for cmd in \
  "python3 scripts/run_urbannav_rrr_bl_iso.py medium --validate-only" \
  "python3 scripts/run_urbannav_rrr_va_iso.py medium --validate-only" \
  "python3 scripts/run_urbannav_rrr_bl_iso.py deep --validate-only" \
  "python3 scripts/run_urbannav_rrr_va_iso.py deep --validate-only" \
  "python3 scripts/run_urbannav_rrr_ceres_oracle.py deep --validate-only"
 do
  ok=$(eval $cmd 2>&1 | python3 -c "import sys,re,json; t=sys.stdin.read(); m=re.search(r'\{[\s\S]*\}\s*$',t); print(json.loads(m.group())['ok'] if m else 'parse_fail')")
  echo "validate $ok :: $cmd"
  [[ "$ok" == "True" ]] || fail=1
done
[[ "$fail" -eq 0 ]]

cat > "$OUT/ENV.md" <<EOF
# Paper-1 cov — 3-turn clean campaign (PREPARED, NOT STARTED)
OUT_ROOT=$OUT
URBANNAV_DATA_ROOT=$URBANNAV_DATA_ROOT
GICI_DATA_ROOT=$GICI_DATA_ROOT
GICI_MAIN=$GICI_MAIN
gici_main_md5=$MD5_MAIN
libgici_md5=$MD5_LIB
prepared=$(date -Is)
hostname=$(hostname)
runbook=research/paper/RUNBOOK_3_TURNS.md

## Turns
1. A + B  (board guard + matrix)
2. C      (Medium iso GATE)
3. D1 + D2 + D4  (Deep shadow / proposed / ceres) then STOP

## Do not use
- results/research/paper_repro
- logs/research/gici_board_va/1_1
- any prior paper_cov_* except this OUT_ROOT
- scripts/run_paper_all.sh
EOF

cat > "$OUT/READY.txt" <<EOF
READY_FOR_TURN_1
OUT_ROOT=$OUT
NEXT= scripts/run_paper_cov_stage.sh A \"\$OUT\"
THEN= scripts/run_paper_cov_stage.sh B \"\$OUT\"
VERIFY= research/paper/RUNBOOK_3_TURNS.md § Lượt 1
DO_NOT_START_TURN_2_UNTIL_TURN_1_PASS
EOF

# status board: all pending
for s in A B C D1 D2 D4; do
  echo "pending" > "$OUT/status/${s}.txt"
done

echo
echo "======== PREPARE COMPLETE (no runs started) ========"
echo "OUT_ROOT=$OUT"
echo "LATEST -> $(cat results/research/paper_cov_LATEST)"
echo "Read: research/paper/RUNBOOK_3_TURNS.md"
echo "Start Turn 1 only when you decide:"
echo "  export URBANNAV_DATA_ROOT=/media/theph/Data1/Research/dataset"
echo "  OUT=\$(cat results/research/paper_cov_LATEST)"
echo "  scripts/run_paper_cov_stage.sh A \"\$OUT\""
cat "$OUT/READY.txt"
