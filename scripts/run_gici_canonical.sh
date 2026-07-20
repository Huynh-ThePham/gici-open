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

STALL_S="${STALL_S:-25}"      # solution stable this long => done
MIN_FLOOR="${MIN_FLOOR:-100}" # need at least this many solution lines to count as progress
NOEPH_S="${NOEPH_S:-360}"     # if 0 lines this long => ephemeris stall, give up fast
MAX_WALL="${MAX_WALL:-3000}"  # hard per-run ceiling

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
  local t0=$SECONDS last=0 last_t=$SECONDS n
  while kill -0 "$pid" 2>/dev/null; do
    sleep 5
    n=0; [[ -s "$sol" ]] && n="$(wc -l < "$sol")"
    if (( n > last )); then last=$n; last_t=$SECONDS; fi
    if (( n >= MIN_FLOOR && SECONDS - last_t >= STALL_S )); then kill -INT "$pid" 2>/dev/null; break; fi
    if (( n == 0 && SECONDS - t0 >= NOEPH_S )); then mark "  !! ${board} ${arm} run${k} NO-OUTPUT ${NOEPH_S}s (eph stall?) -> abort"; kill -INT "$pid" 2>/dev/null; break; fi
    if (( SECONDS - t0 >= MAX_WALL )); then mark "  !! ${board} ${arm} run${k} MAX_WALL -> stop"; kill -INT "$pid" 2>/dev/null; break; fi
  done
  # gici_main can ignore SIGINT after completion; escalate after grace
  local g; for g in $(seq 1 15); do kill -0 "$pid" 2>/dev/null || break; sleep 2; done
  kill -9 "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true
  n=0; [[ -s "$sol" ]] && n="$(wc -l < "$sol")"
  local vaar; vaar="$(grep -c '\[vaar-fast\]' "${out}/log/run.stderr" 2>/dev/null || echo 0)"
  mark "DONE ${board} ${arm} run${k}: lines=${n} vaar=${vaar} wall=$((SECONDS - t0))s"
  if (( n >= MIN_FLOOR )); then touch "${out}/.done"; else mark "  WARN ${board} ${arm} run${k} INCOMPLETE (${n} lines)"; fi
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
