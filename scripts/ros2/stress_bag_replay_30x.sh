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
APE_POS_MAX="${STRESS_APE_POS_MAX:-0.25}"  # outlier vs ~0.13m nominal

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
echo "run,exit_code,gpgga,sparsify_count,runtime_s,ape_pos_m,ape_rot_deg,segfault,deadlock,trajectory_ok,ape_ok,notes" > "${CSV}"

pass_complete=0
pass_no_crash=0
pass_no_deadlock=0
pass_trajectory=0
pass_ape=0
failed_runs=()

printf 'Stress bag replay: dataset=%s runs=%s rate=%s timeout=%ss\n' \
  "${DATASET_ID}" "${RUNS}" "${RATE}" "${TIMEOUT_S}"
printf 'Output: %s\n\n' "${STRESS_ROOT}"

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

  t0=$(date +%s.%N)
  set +e
  timeout "${TIMEOUT_S}" env GICI_ROS2_BOARD_OUT="${OUT}" \
    "${REPO}/scripts/ros2/run_gici_board_rrr_ros2.sh" --bag "${DATASET_ID}" "${RATE}" \
    > "${RUN_DIR}/run.log" 2>&1
  exit_code=$?
  set -e
  t1=$(date +%s.%N)
  runtime_s=$(python3 -c "print(f'{$t1 - $t0:.1f}')")

  if (( exit_code == 124 )); then
    deadlock=1
    notes="timeout"
  fi

  if rg -q 'Received a segment fault|handleSegv' "${OUT}/node.log" 2>/dev/null; then
    segfault=1
    notes="${notes:+$notes; }segfault"
  fi
  if (( exit_code == 134 || exit_code == 139 )); then
    segfault=1
    notes="${notes:+$notes; }aborted"
  fi

  gpgga="$(count_gpgga "${OUT}/solution.txt")"
  sparsify="$(rg -c 'Sparsifying measurements' "${OUT}/node.log" 2>/dev/null || true)"
  sparsify="${sparsify:-0}"

  if (( gpgga >= MIN_GPGGA )); then
    trajectory_ok=1
  else
    notes="${notes:+$notes; }gpgga=${gpgga}<${MIN_GPGGA}"
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
      if python3 -c "import math; p=float('${ape_pos}'); exit(0 if p <= ${APE_POS_MAX} else 1)"; then
        ape_ok=1
      else
        notes="${notes:+$notes; }ape_pos=${ape_pos}"
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

  if (( run_pass )); then
    pass_complete=$((pass_complete + 1))
  else
    failed_runs+=("${run_id}")
  fi
  (( segfault == 0 )) && pass_no_crash=$((pass_no_crash + 1))
  (( deadlock == 0 )) && pass_no_deadlock=$((pass_no_deadlock + 1))
  (( trajectory_ok == 1 )) && pass_trajectory=$((pass_trajectory + 1))
  (( ape_ok == 1 )) && pass_ape=$((pass_ape + 1))

  printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
    "${run_id}" "${exit_code}" "${gpgga}" "${sparsify}" "${runtime_s}" \
    "${ape_pos}" "${ape_rot}" "${segfault}" "${deadlock}" \
    "${trajectory_ok}" "${ape_ok}" "${notes}" >> "${CSV}"

  printf '[%s/%s] exit=%s gpgga=%s sparsify=%s runtime=%ss ape=%s/%s seg=%s hang=%s domain=%s %s\n' \
    "${i}" "${RUNS}" "${exit_code}" "${gpgga}" "${sparsify}" "${runtime_s}" \
    "${ape_pos}" "${ape_rot}" "${segfault}" "${deadlock}" "${ROS_DOMAIN_ID}" \
    "$([[ ${run_pass} -eq 1 ]] && echo PASS || echo FAIL)"

  cleanup_ros2_gici_session "${RUN_DIR}/cleanup_post.log"
done

python3 - <<PY > "${JSON}"
import csv, json, statistics
from pathlib import Path

root = Path("${STRESS_ROOT}")
rows = list(csv.DictReader((root / "summary.csv").read_text().splitlines()))
pos = [float(r["ape_pos_m"]) for r in rows if r["ape_pos_m"] not in ("", "nan")]
rot = [float(r["ape_rot_deg"]) for r in rows if r["ape_rot_deg"] not in ("", "nan")]
sparsify = [int(r["sparsify_count"]) for r in rows if r["sparsify_count"] not in ("", "nan")]
failed = [r.strip() for r in """${failed_runs[*]}""".split() if r.strip()]
summary = {
  "dataset": "${DATASET_ID}",
  "runs": ${RUNS},
  "rate": ${RATE},
  "criteria": {
    "all_pass": int("${pass_complete}"),
    "no_segfault": int("${pass_no_crash}"),
    "no_deadlock": int("${pass_no_deadlock}"),
    "full_trajectory": int("${pass_trajectory}"),
    "ape_ok": int("${pass_ape}"),
    "min_gpgga": ${MIN_GPGGA},
    "max_ape_pos_m": ${APE_POS_MAX},
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
  "sparsify_count": {
    "min": min(sparsify) if sparsify else None,
    "max": max(sparsify) if sparsify else None,
    "mean": statistics.mean(sparsify) if sparsify else None,
    "stdev": statistics.pstdev(sparsify) if len(sparsify) > 1 else 0.0,
    # Backend-overload early warning: runs sparsifying far more than the batch's
    # own norm tend to starve visual updates and blow up rotation APE well before
    # position APE moves much (seen empirically: batch30 runs 015/021 sparsified
    # ~2x the batch mean and lost 20-40x nominal rotation APE). Not a pass/fail
    # gate -- just surfaces the leading indicator instead of only the trailing one.
    "outlier_runs": [
      r["run"] for r in rows
      if r["sparsify_count"] not in ("", "nan")
      and len(sparsify) > 1
      and int(r["sparsify_count"]) > statistics.mean(sparsify) + 2 * statistics.pstdev(sparsify)
    ],
  },
  "pass_all": (
    int("${pass_complete}") == ${RUNS}
    and int("${pass_no_crash}") == ${RUNS}
    and int("${pass_no_deadlock}") == ${RUNS}
    and int("${pass_trajectory}") == ${RUNS}
    and int("${pass_ape}") == ${RUNS}
  ),
}
print(json.dumps(summary, indent=2))
PY

printf '\n========== STRESS SUMMARY ==========\n'
printf 'Complete (all criteria): %s/%s\n' "${pass_complete}" "${RUNS}"
printf 'No segfault:             %s/%s\n' "${pass_no_crash}" "${RUNS}"
printf 'No deadlock/timeout:     %s/%s\n' "${pass_no_deadlock}" "${RUNS}"
printf 'Full trajectory:         %s/%s\n' "${pass_trajectory}" "${RUNS}"
printf 'APE within bounds:       %s/%s\n' "${pass_ape}" "${RUNS}"
if ((${#failed_runs[@]})); then
  printf 'Failed runs: %s\n' "${failed_runs[*]}"
fi
sparsify_outliers="$(python3 -c "import json; d=json.load(open('${JSON}'))['sparsify_count']; print(' '.join(d['outlier_runs']))" 2>/dev/null || true)"
if [[ -n "${sparsify_outliers}" ]]; then
  printf 'Sparsify outliers (backend-overload early warning): %s\n' "${sparsify_outliers}"
fi
printf 'CSV : %s\n' "${CSV}"
printf 'JSON: %s\n' "${JSON}"

if [[ "$(python3 -c "import json; print(json.load(open('${JSON}'))['pass_all'])")" == "True" ]]; then
  exit 0
fi
exit 1
