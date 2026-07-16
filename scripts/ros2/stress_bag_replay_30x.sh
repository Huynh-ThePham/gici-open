#!/usr/bin/env bash
# Stress test: N consecutive real-time bag replays @ rate 1.0 (patched branch).
#
# Usage:
#   ./scripts/ros2/stress_bag_replay_30x.sh [1.1] [runs] [rate]
#   STRESS_RUNS=30 STRESS_RATE=1 ./scripts/ros2/stress_bag_replay_30x.sh 1.1
#
# Output:
#   results/stress/bag_replay_<id>/summary.csv
#   results/stress/bag_replay_<id>/summary.json
#   results/stress/bag_replay_<id>/run_XXX/
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=dataset_paths.sh
source "${REPO}/scripts/dataset_paths.sh"
# shellcheck source=ros2_bag_replay_common.sh
source "${REPO}/scripts/ros2/ros2_bag_replay_common.sh"

DATASET_ID="${1:-1.1}"
RUNS="${2:-${STRESS_RUNS:-30}}"
RATE="${3:-${STRESS_RATE:-1}}"
STRESS_SUFFIX="${STRESS_SUFFIX:-}"
TIMEOUT_S="${STRESS_TIMEOUT_S:-600}"   # bag ~200s + init; 10 min ceiling
MIN_GPGGA="${STRESS_MIN_GPGGA:-1700}"  # full trajectory (~1785 typical)
# Strict realtime defaults (override with STRESS_PROFILE=legacy for pos<=0.25 rot unchecked)
if [[ "${STRESS_PROFILE:-strict}" == "strict" ]]; then
  APE_POS_MAX="${STRESS_APE_POS_MAX:-0.20}"
  APE_ROT_MAX="${STRESS_APE_ROT_MAX:-1.0}"
else
  APE_POS_MAX="${STRESS_APE_POS_MAX:-0.25}"
  APE_ROT_MAX="${STRESS_APE_ROT_MAX:-999}"
fi
# STRESS_VARIANT: default | nosparsify (disable backend sparsify in config)
STRESS_VARIANT="${STRESS_VARIANT:-}"
STRESS_RUNNER="${STRESS_RUNNER:-bag}"

if [[ -n "${STRESS_ROOT:-}" ]]; then
  :
elif [[ -n "${STRESS_SUFFIX}" ]]; then
  STRESS_ROOT="${REPO}/results/stress/bag_replay_${DATASET_ID}_${STRESS_SUFFIX}"
else
  STRESS_ROOT="${REPO}/results/stress/bag_replay_${DATASET_ID}"
fi
mkdir -p "${STRESS_ROOT}"

set +u
source /opt/ros/humble/setup.bash
source "${REPO}/ros2_wrapper/install/setup.bash"
set -u

CSV="${STRESS_ROOT}/summary.csv"
JSON="${STRESS_ROOT}/summary.json"
echo "run,exit_code,gpgga,sparsify_count,runtime_s,ape_pos_m,ape_rot_deg,segfault,deadlock,trajectory_ok,ape_ok,ape_strict_ok,notes" > "${CSV}"

pass_complete=0
pass_no_crash=0
pass_no_deadlock=0
pass_trajectory=0
pass_ape=0
pass_ape_strict=0
failed_runs=()

printf 'Stress bag replay: dataset=%s runs=%s rate=%s timeout=%ss profile=%s runner=%s variant=%s\n' \
  "${DATASET_ID}" "${RUNS}" "${RATE}" "${TIMEOUT_S}" "${STRESS_PROFILE:-strict}" "${STRESS_RUNNER}" "${STRESS_VARIANT:-default}"
printf 'APE limits: pos<=%sm rot<=%sdeg  min_gpgga=%s\n' "${APE_POS_MAX}" "${APE_ROT_MAX}" "${MIN_GPGGA}"
printf 'Output: %s\n\n' "${STRESS_ROOT}"

run_replay() {
  local out_dir="$1"
  if [[ "${STRESS_RUNNER}" == "live" ]]; then
    local render_mode="${GICI_LIVE_RENDER_MODE:-live}"
    [[ "${STRESS_VARIANT}" == "nosparsify" ]] && render_mode="live-nosparsify"
    env GICI_ROS2_LIVE_OUT="${out_dir}" GICI_LIVE_RENDER_MODE="${render_mode}" \
      "${REPO}/scripts/ros2/launch_gici_live.sh" board "${DATASET_ID}" bag "${RATE}"
  else
    local -a bag_args=(--bag)
    [[ "${STRESS_VARIANT}" == "nosparsify" ]] && bag_args+=(--nosparsify)
    env GICI_ROS2_BOARD_OUT="${out_dir}" \
      "${REPO}/scripts/ros2/run_gici_board_rrr_ros2.sh" "${bag_args[@]}" "${DATASET_ID}" "${RATE}"
  fi
}
export REPO DATASET_ID RATE STRESS_RUNNER STRESS_VARIANT
export -f run_replay 2>/dev/null || true

for ((i = 1; i <= RUNS; i++)); do
  run_id="$(printf '%03d' "${i}")"
  RUN_DIR="${STRESS_ROOT}/run_${run_id}"
  OUT="${RUN_DIR}/replay"
  EVAL="${RUN_DIR}/eval"
  mkdir -p "${OUT}/log" "${EVAL}/output"

  cleanup_ros2_gici_session "${RUN_DIR}/cleanup.log"
  export ROS_DOMAIN_ID="${GICI_ROS_DOMAIN_ID:-$((70 + i))}"

  notes=""
  segfault=0
  deadlock=0
  trajectory_ok=0
  ape_ok=0
  ape_strict_ok=0

  t0=$(date +%s.%N)
  set +e
  timeout "${TIMEOUT_S}" bash -c 'run_replay "$1"' bash "${OUT}" \
    > "${RUN_DIR}/run.log" 2>&1
  exit_code=$?
  set -e
  t1=$(date +%s.%N)
  runtime_s=$(python3 -c "print(f'{$t1 - $t0:.1f}')")

  if (( exit_code == 124 )); then
    deadlock=1
    notes="timeout"
  fi

  gpgga="$(count_gpgga "${OUT}/solution.txt")"
  sparsify="$(rg -c 'Sparsifying measurements' "${OUT}/node.log" 2>/dev/null || true)"
  sparsify="${sparsify:-0}"

  if (( gpgga >= MIN_GPGGA )); then
    trajectory_ok=1
  else
    notes="${notes:+$notes; }gpgga=${gpgga}<${MIN_GPGGA}"
  fi

  if rg -q 'Received a segment fault|handleSegv' "${OUT}/node.log" 2>/dev/null; then
    if (( trajectory_ok == 0 )); then
      segfault=1
      notes="${notes:+$notes; }segfault"
    else
      notes="${notes:+$notes; }shutdown_segfault_ignored"
    fi
  fi
  if (( exit_code == 139 )); then
    segfault=1
    notes="${notes:+$notes; }sigsegv"
  fi
  if (( exit_code == 134 )) && (( trajectory_ok == 0 )) \
      && rg -q 'Received a segment fault|handleSegv|CHECK failed' "${OUT}/node.log" 2>/dev/null; then
    segfault=1
    notes="${notes:+$notes; }aborted"
  fi

  ape_pos="nan"
  ape_rot="nan"
  if [[ -s "${OUT}/solution.txt" ]]; then
    cp "${OUT}/solution.txt" "${EVAL}/output/solution.txt"
    if GICI_BASELINE_OUT="${EVAL}" "${REPO}/scripts/run_author_eval_gici_board.sh" \
        "${DATASET_ID}" --skip-run >> "${RUN_DIR}/eval.log" 2>&1; then
      read -r ape_pos ape_rot < <(python3 -c "
import json
m=json.load(open('${EVAL}/evaluation/ape_metrics.json'))
print(m['ape_translation_rmse_m'], m['ape_rotation_rmse_deg'])
")
      if python3 -c "import math; p=float('${ape_pos}'); r=float('${ape_rot}'); exit(0 if p <= ${APE_POS_MAX} else 1)"; then
        ape_ok=1
      else
        notes="${notes:+$notes; }ape_pos=${ape_pos}"
      fi
      if python3 -c "import math; p=float('${ape_pos}'); r=float('${ape_rot}'); exit(0 if p <= ${APE_POS_MAX} and r <= ${APE_ROT_MAX} else 1)"; then
        ape_strict_ok=1
      else
        notes="${notes:+$notes; }ape_rot=${ape_rot}"
      fi
    else
      notes="${notes:+$notes; }eval_failed"
    fi
  fi

  run_pass=1
  (( exit_code == 0 )) || run_pass=0
  (( segfault == 0 )) || run_pass=0
  (( deadlock == 0 )) || run_pass=0
  (( trajectory_ok == 1 )) || run_pass=0
  (( ape_ok == 1 )) || run_pass=0
  if [[ "${STRESS_PROFILE:-strict}" == "strict" ]]; then
    (( ape_strict_ok == 1 )) || run_pass=0
  fi

  if (( run_pass )); then
    pass_complete=$((pass_complete + 1))
  else
    failed_runs+=("${run_id}")
  fi
  (( segfault == 0 )) && pass_no_crash=$((pass_no_crash + 1))
  (( deadlock == 0 )) && pass_no_deadlock=$((pass_no_deadlock + 1))
  (( trajectory_ok == 1 )) && pass_trajectory=$((pass_trajectory + 1))
  (( ape_ok == 1 )) && pass_ape=$((pass_ape + 1))
  (( ape_strict_ok == 1 )) && pass_ape_strict=$((pass_ape_strict + 1))

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${run_id}" "${exit_code}" "${gpgga}" "${sparsify}" "${runtime_s}" \
    "${ape_pos}" "${ape_rot}" "${segfault}" "${deadlock}" \
    "${trajectory_ok}" "${ape_ok}" "${ape_strict_ok}" "${notes}" >> "${CSV}"

  printf '[%s/%s] exit=%s gpgga=%s sparsify=%s runtime=%ss ape=%s/%s seg=%s hang=%s domain=%s %s\n' \
    "${i}" "${RUNS}" "${exit_code}" "${gpgga}" "${sparsify}" "${runtime_s}" \
    "${ape_pos}" "${ape_rot}" "${segfault}" "${deadlock}" "${ROS_DOMAIN_ID}" \
    "$([[ ${run_pass} -eq 1 ]] && echo PASS || echo FAIL) strict=$([[ ${ape_strict_ok} -eq 1 ]] && echo Y || echo N)"

  cleanup_ros2_gici_session "${RUN_DIR}/cleanup_post.log"
done

python3 - <<PY > "${JSON}"
import csv, json, statistics
from pathlib import Path

root = Path("${STRESS_ROOT}")
rows = list(csv.DictReader((root / "summary.csv").read_text().splitlines()))
pos = [float(r["ape_pos_m"]) for r in rows if r["ape_pos_m"] not in ("", "nan")]
rot = [float(r["ape_rot_deg"]) for r in rows if r["ape_rot_deg"] not in ("", "nan")]
failed = [r.strip() for r in """${failed_runs[*]}""".split() if r.strip()]
summary = {
  "dataset": "${DATASET_ID}",
  "runs": ${RUNS},
  "rate": ${RATE},
  "profile": "${STRESS_PROFILE:-strict}",
  "runner": "${STRESS_RUNNER}",
  "criteria": {
    "all_pass": int("${pass_complete}"),
    "no_segfault": int("${pass_no_crash}"),
    "no_deadlock": int("${pass_no_deadlock}"),
    "full_trajectory": int("${pass_trajectory}"),
    "ape_pos_ok": int("${pass_ape}"),
    "ape_strict_ok": int("${pass_ape_strict}"),
    "min_gpgga": ${MIN_GPGGA},
    "max_ape_pos_m": ${APE_POS_MAX},
    "max_ape_rot_deg": ${APE_ROT_MAX},
  },
  "failed_runs": failed,
  "ape_pos_m": {
    "min": min(pos) if pos else None,
    "max": max(pos) if pos else None,
    "mean": statistics.mean(pos) if pos else None,
    "stdev": statistics.pstdev(pos) if len(pos) > 1 else 0.0,
  },
  "ape_rot_deg": {
    "min": min(rot) if rot else None,
    "max": max(rot) if rot else None,
    "mean": statistics.mean(rot) if rot else None,
    "stdev": statistics.pstdev(rot) if len(rot) > 1 else 0.0,
  },
  "pass_all": (
    int("${pass_complete}") == ${RUNS}
    and int("${pass_no_crash}") == ${RUNS}
    and int("${pass_no_deadlock}") == ${RUNS}
    and int("${pass_trajectory}") == ${RUNS}
    and int("${pass_ape_strict}") == ${RUNS}
  ),
}
# Compare to batch30_v2 on main worktree if present
ref = Path("/home/theph/ws_ncs/gici_research_standard/results/stress/bag_replay_1.1_batch30_v2/summary.csv")
if ref.is_file():
  ref_rows = list(csv.DictReader(ref.read_text().splitlines()))
  ref_pos = [float(r["ape_pos_m"]) for r in ref_rows if r["ape_pos_m"] not in ("", "nan")]
  ref_rot = [float(r["ape_rot_deg"]) for r in ref_rows if r["ape_rot_deg"] not in ("", "nan")]
  summary["reference_batch30_v2"] = {
    "worktree": "gici_research_standard",
    "runs": len(ref_rows),
    "ape_pos_m_median": statistics.median(ref_pos) if ref_pos else None,
    "ape_rot_deg_median": statistics.median(ref_rot) if ref_rot else None,
    "legacy_max_ape_pos_m": 0.25,
  }
print(json.dumps(summary, indent=2))
PY

printf '\n========== STRESS SUMMARY ==========\n'
printf 'Complete (all criteria): %s/%s\n' "${pass_complete}" "${RUNS}"
printf 'No segfault:             %s/%s\n' "${pass_no_crash}" "${RUNS}"
printf 'No deadlock/timeout:     %s/%s\n' "${pass_no_deadlock}" "${RUNS}"
printf 'Full trajectory:         %s/%s\n' "${pass_trajectory}" "${RUNS}"
printf 'APE pos OK:              %s/%s\n' "${pass_ape}" "${RUNS}"
printf 'APE strict (pos+rot):    %s/%s\n' "${pass_ape_strict}" "${RUNS}"
if ((${#failed_runs[@]})); then
  printf 'Failed runs: %s\n' "${failed_runs[*]}"
fi
printf 'CSV : %s\n' "${CSV}"
printf 'JSON: %s\n' "${JSON}"

if [[ "$(python3 -c "import json; print(json.load(open('${JSON}'))['pass_all'])")" == "True" ]]; then
  exit 0
fi
exit 1
