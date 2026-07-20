#!/usr/bin/env bash
# Author-exact MULTI-SEED campaign (n=5) for the paper comparison table.
# Purpose: the author real-time config (relative_frequency=1.0, num_threads=4,
# max_solver_time=0.04) is NON-DETERMINISTIC (Deep spread ~2.4-3.8 m run-to-run).
# A single run cannot support any BL-vs-VA claim. This runs each arm x scene 5x
# and lets scripts/eval_author_exact_multiseed.py report mean +/- std vs Chi et al.
# Table V. Framing stays: covariance method; accuracy = comparable/honest, NOT a win.
#
# Solo-gici_main policy: waits for the seed-1 chain to finish before starting, then
# runs strictly sequentially (never two gici_main at once) so numbers stay clean.
# Launch DETACHED (setsid) so /compact / session teardown cannot kill it:
#   setsid bash scripts/run_author_exact_multiseed.sh <seed1_chain_pid> &
set -u
cd /home/theph/ws_ncs/gici_vision_aided_ar
REPO=/home/theph/ws_ncs/gici_vision_aided_ar
ROOT=$REPO/results/research/author_exact_multiseed_20260720
LOG=$ROOT/campaign.log
mkdir -p "$ROOT"
say(){ echo "[campaign $(date '+%H:%M:%S')] $*" | tee -a "$LOG"; }

CHAIN_PID=${1:-}

say "START author-exact multiseed campaign (n=5, seeds 2-5 here; seed1 = existing chain)"

# 1. Solo-gici_main: wait for the seed-1 chain (and any live gici_main) to finish.
if [ -n "$CHAIN_PID" ]; then
  say "waiting for seed-1 chain PID $CHAIN_PID ..."
  while kill -0 "$CHAIN_PID" 2>/dev/null; do sleep 30; done
  say "seed-1 chain PID $CHAIN_PID exited."
fi
while pgrep -x gici_main >/dev/null 2>&1; do
  say "a gici_main is still running; waiting ..."
  sleep 30
done
say "no gici_main running; proceeding."

# 2. Import seed-1 metrics into the tree so the aggregator sees a uniform layout:
#    $ROOT/{arm}/seed{N}/{scene}/metrics.json
BL1=$REPO/results/research/author_exact_20260720
VA1=$REPO/results/research/author_exact_va_20260720
for scene in medium deep harsh; do
  for pair in "bl:$BL1" "va:$VA1"; do
    arm=${pair%%:*}; src=${pair#*:}
    dst=$ROOT/$arm/seed1/$scene
    mkdir -p "$dst"
    if [ -f "$src/$scene/metrics.json" ]; then
      cp "$src/$scene/metrics.json" "$dst/metrics.json"
      say "imported seed1 $arm/$scene"
    else
      say "WARN seed1 $arm/$scene metrics.json MISSING ($src)"
    fi
  done
done

# 3. Seeds 2..5, both arms, all scenes, strictly sequential.
for N in 2 3 4 5; do
  for scene in medium deep harsh; do
    say "RUN  bl seed$N $scene"
    python3 scripts/run_urbannav_rrr_baseline.py "$scene" --out-root "$ROOT/bl/seed$N" >>"$LOG" 2>&1
    say "DONE bl seed$N $scene rc=$?"

    say "RUN  va seed$N $scene"
    python3 scripts/run_urbannav_rrr_va.py "$scene" --out-root "$ROOT/va/seed$N" >>"$LOG" 2>&1
    say "DONE va seed$N $scene rc=$?"
  done
done

say "ALL SEEDS DONE"
python3 scripts/eval_author_exact_multiseed.py "$ROOT" >>"$LOG" 2>&1 || say "aggregator failed"
say "aggregate table written to $ROOT/multiseed_table.md"
