#!/usr/bin/env bash
# ============================================================================
# Canonical GICI-board result set (Paper 1).
# Baseline + VA on every GICI board, n runs each, SOLO protocol (one gici_main
# at a time -- GICI configs are wall-clock-bounded via max_solver_time => load-
# sensitive => solo is required for a fair, reproducible set).
#
# Arms:
#   baseline  research config rtk_imu_camera_rrr_1_1.yaml (frozen baseline estimator)
#   va        VA fast marginal covariance, benchmark OFF (the method; fast)
#   va_bench  VA + per-epoch fast-vs-ceres equivalence logging (benchmark ON; slow)
#
# Per-board dataset dir + RTCM start date are substituted from dataset_paths.sh.
# Resumable: a run with a .done marker is skipped.
#
# Usage:
#   scripts/run_gici_canonical.sh [OUT_ROOT] [NRUNS] [ARMS] [BOARDS]
#   ARMS default "baseline va"    (space list from {baseline,va,va_bench})
#   BOARDS default all 12         (space list)
# Env probe:  ARMS=va_bench BOARDS=1.1 NRUNS=1 scripts/run_gici_canonical.sh
# ============================================================================
set -u
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "${REPO}/scripts/dataset_paths.sh"

OUT_ROOT="${1:-${REPO}/results/research/gici_canonical}"
NRUNS="${2:-${NRUNS:-3}}"
ARMS="${3:-${ARMS:-baseline va}}"
BOARDS="${4:-${BOARDS:-1.1 1.2 2.1 2.2 3.1 3.2 3.3 4.1 4.2 4.3 5.1 5.2}}"

GICI_MAIN="${GICI_MAIN:-${REPO}/build/gici_main}"
BL_TMPL="${REPO}/research/config/rtk_imu_camera_rrr_1_1.yaml"
VA_TMPL="${REPO}/research/config/rtk_imu_camera_rrr_va_1_1.yaml"   # benchmark ON as shipped

# Completion is decided by REASON, not by a flat line count (a truncated run must never be
# marked .done). Real completion = the input streams are exhausted and gici_main goes idle
# (its known post-EOF SIGINT-ignore hang): process still ALIVE but no new solution line for
# STALL_S. A crash (process dies) or a MAX_WALL/eph timeout is NOT completion -> retried.
STALL_S="${STALL_S:-150}"     # process-alive + no growth this long => EOF reached (generous;
                              #   a mid-stream gap this long is implausible on backpressure-paced replay)
MIN_FLOOR="${MIN_FLOOR:-500}" # sanity floor: fewer lines than this is a broken run, never .done
NOEPH_S="${NOEPH_S:-600}"     # 0 lines this long => ephemeris stall / slow first-fix -> abort (no .done)
MAX_WALL="${MAX_WALL:-21600}" # 6 h hard ceiling (large boards 5.1/5.2); hitting it => FAILED, never .done
# Reference full lengths (lines): GICI board 1.1 = 7100. Others recorded per-run in status.txt;
# compare siblings across the 3 runs at eval time to catch any truncation.

LOG="${OUT_ROOT}/run_gici_canonical.log"
mkdir -p "$OUT_ROOT"
mark() { echo "[$(date -Is)] $*" | tee -a "$LOG"; }

make_config() {  # board arm out -> writes $out/config.yaml, echoes path
  local board="$1" arm="$2" out="$3"
  local ds="${GICI_DATA_ROOT}/${board}"
  local start; start="$(gici_rtcm_start_for "$board")"
  local tmpl="$BL_TMPL"; [[ "$arm" == va || "$arm" == va_bench ]] && tmpl="$VA_TMPL"
  local cfg="${out}/config.yaml"
  sed \
    -e "s|<DATASET_1_1>|${ds}|g" \
    -e "s|<GICI_ROOT>|${REPO}|g" \
    -e "s|<OUTPUT_DIR>|${out}/output|g" \
    -e "s|<LOG_DIR>|${out}/log|g" \
    -e "s|start_time: 2023.03.20|start_time: ${start}|g" \
    "$tmpl" > "$cfg"
  # va (fast, no benchmark): turn the expensive ceres comparison OFF
  if [[ "$arm" == va ]]; then
    sed -i 's|benchmark_joint_ambiguity_covariance: true|benchmark_joint_ambiguity_covariance: false|g' "$cfg"
  fi
  echo "$cfg"
}

run_one() {  # board arm k
  local board="$1" arm="$2" k="$3"
  local out="${OUT_ROOT}/${board}/${arm}/run${k}"
  local sol="${out}/output/solution.txt"
  if [[ -f "${out}/.done" ]]; then mark "SKIP ${board} ${arm} run${k} (done)"; return 0; fi
  mkdir -p "${out}/output" "${out}/log"
  local cfg; cfg="$(make_config "$board" "$arm" "$out")"
  rm -f "$sol"
  mark "RUN  ${board} ${arm} run${k}"
  "$GICI_MAIN" "$cfg" > "${out}/log/run.stdout" 2> "${out}/log/run.stderr" &
  local pid=$!
  local t0=$SECONDS last=0 last_t=$SECONDS n reason=""
  while kill -0 "$pid" 2>/dev/null; do
    sleep 5
    n=0; [[ -s "$sol" ]] && n="$(wc -l < "$sol")"
    if (( n > last )); then last=$n; last_t=$SECONDS; fi
    # genuine completion: still ALIVE + output quiet for STALL_S (EOF idle) + past sanity floor
    if (( n >= MIN_FLOOR && SECONDS - last_t >= STALL_S )); then reason=STALL_COMPLETE; kill -INT "$pid" 2>/dev/null; break; fi
    if (( n == 0 && SECONDS - t0 >= NOEPH_S )); then reason=NOEPH; mark "  !! ${board} ${arm} run${k} NO-OUTPUT ${NOEPH_S}s (eph stall / slow first-fix) -> abort"; kill -INT "$pid" 2>/dev/null; break; fi
    if (( SECONDS - t0 >= MAX_WALL )); then reason=MAXWALL; mark "  !! ${board} ${arm} run${k} MAX_WALL ${MAX_WALL}s -> stop (NOT complete)"; kill -INT "$pid" 2>/dev/null; break; fi
  done
  # loop exited on its own => process ended without our SIGINT (crash or, on some builds, clean exit)
  [[ -z "$reason" ]] && reason=DIED
  # gici_main can ignore SIGINT after completion; escalate after grace
  local g; for g in $(seq 1 15); do kill -0 "$pid" 2>/dev/null || break; sleep 2; done
  kill -9 "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true
  n=0; [[ -s "$sol" ]] && n="$(wc -l < "$sol")"
  local vaar; vaar=0; grep -q '\[vaar-fast\]' "${out}/log/run.stderr" 2>/dev/null && vaar="$(grep -c '\[vaar-fast\]' "${out}/log/run.stderr")"
  printf 'reason=%s lines=%s vaar=%s wall=%ss\n' "$reason" "$n" "$vaar" "$((SECONDS - t0))" > "${out}/status.txt"
  # Mark .done ONLY on a genuine EOF-idle completion past the sanity floor. MAXWALL / NOEPH /
  # DIED(crash) / sub-floor are left WITHOUT .done so the next invocation retries them, and are
  # never silently accepted as a finished (possibly truncated) run.
  if [[ "$reason" == STALL_COMPLETE ]] && (( n >= MIN_FLOOR )); then
    touch "${out}/.done"
    mark "DONE ${board} ${arm} run${k}: COMPLETE lines=${n} vaar=${vaar} wall=$((SECONDS - t0))s"
  else
    mark "INCOMPLETE ${board} ${arm} run${k}: reason=${reason} lines=${n} wall=$((SECONDS - t0))s (WILL RETRY)"
  fi
}

mark "GICI_CANONICAL_START out=${OUT_ROOT} nruns=${NRUNS} arms='${ARMS}' boards='${BOARDS}'"
for board in $BOARDS; do
  for arm in $ARMS; do
    for k in $(seq 1 "$NRUNS"); do
      run_one "$board" "$arm" "$k"
    done
  done
done
mark "GICI_CANONICAL_DONE"
