#!/usr/bin/env bash
# Author eval (evo_ape Sim(3) @ 1 Hz) for ROS1 UrbanNav RRR output.
#
# Usage: scripts/ros1/run_eval.sh [table5|native] [medium|deep]
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MODE="${URBANNAV_ROS1_MODE:-table5}"
POS=()
for a in "$@"; do
  case "${a}" in
    table5|native) MODE="${a}" ;;
    *) POS+=("${a}") ;;
  esac
done
DS="${POS[0]:-medium}"

if [[ "${MODE}" == "table5" ]]; then
  SOL="${REPO}/output/ros1_urbannav_rrr_table5/${DS}/run/solution.txt"
  EVAL_OUT="${REPO}/results/baseline/urbannav_ros1_table5/${DS}"
else
  SOL="${REPO}/output/ros1_urbannav_rrr/${DS}/solution.txt"
  EVAL_OUT="${REPO}/results/baseline/urbannav_ros1/${DS}"
fi

if [[ ! -f "${SOL}" ]]; then
  echo "[ros1-eval] ERROR: missing ${SOL} — run docker_run_rrr.sh ${MODE} ${DS} first" >&2
  exit 1
fi

mkdir -p "${EVAL_OUT}/output"
cp "${SOL}" "${EVAL_OUT}/output/solution.txt"
GICI_BASELINE_OUT="${EVAL_OUT}" "${REPO}/scripts/run_author_eval_urbannav.sh" "${DS}"
