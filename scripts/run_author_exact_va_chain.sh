#!/usr/bin/env bash
# Author-exact chain: propose (VA) on 3 datasets + baseline Harsh (redo interrupted).
# Config: relative_frequency=1.0, num_threads:4, max_solver_time:0.04 (author real-time).
# Launched DETACHED (setsid) so /compact / session teardown cannot kill it.
set -u
cd /home/theph/ws_ncs/gici_vision_aided_ar
REPO=/home/theph/ws_ncs/gici_vision_aided_ar
VA_OUT=$REPO/results/research/author_exact_va_20260720
BL_OUT=$REPO/results/research/author_exact_20260720
LOG=$VA_OUT/chain.log
mkdir -p "$VA_OUT"
say(){ echo "[chain $(date '+%H:%M:%S')] $*" | tee -a "$LOG"; }

say "START author-exact VA chain (relative_frequency=1.0, 4-thread/0.04)"

for ds in medium deep harsh; do
  say "RUN  va/$ds"
  python3 scripts/run_urbannav_rrr_va.py "$ds" --out-root "$VA_OUT" >>"$LOG" 2>&1
  rc=$?
  h=$(python3 -c "import json;print(json.load(open('$VA_OUT/$ds/metrics.json'))['metrics']['rmse_h_m'])" 2>/dev/null || echo NA)
  say "DONE va/$ds rc=$rc rmse_h=$h"
done

# Redo baseline harsh (was killed by /compact mid-stream at ~28883 epochs)
say "RUN  bl/harsh (redo interrupted)"
python3 scripts/run_urbannav_rrr_baseline.py harsh --out-root "$BL_OUT" >>"$LOG" 2>&1
rc=$?
h=$(python3 -c "import json;print(json.load(open('$BL_OUT/harsh/metrics.json'))['metrics']['rmse_h_m'])" 2>/dev/null || echo NA)
say "DONE bl/harsh rc=$rc rmse_h=$h"

say "ALL DONE"
