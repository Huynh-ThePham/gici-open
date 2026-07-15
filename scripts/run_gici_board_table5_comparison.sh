#!/usr/bin/env bash
# Table V comparison: GICI board 1.1 → 3.1 → 4.1 (file-mode + ROS2).
#
# Usage:
#   scripts/run_gici_board_table5_comparison.sh [--file-only|--ros2-only|--skip-run]
#
# File-mode: option/post_estimation_RTK_RRR.yaml (author open config)
# ROS2:      ros_gici_board_rrr.yaml (ported from ros_real_time_estimation_RTK_RRR.yaml)
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="both"
SKIP_RUN=0
for a in "$@"; do
  case "${a}" in
    --file-only) MODE="file" ;;
    --ros2-only) MODE="ros2" ;;
    --skip-run)  SKIP_RUN=1 ;;
  esac
done

chmod +x "${REPO}/scripts/run_gici_board_rrr.sh" \
         "${REPO}/scripts/run_author_eval_gici_board.sh" \
         "${REPO}/scripts/ros2/run_gici_board_rrr_ros2.sh" \
         "${REPO}/scripts/ros2/build_gici_board_ros1_bags.sh"

echo "[table5] Extracting datasets 1.1 3.1 4.1 (if needed) ..."
"${REPO}/scripts/setup_gici_datasets.sh" 1.1 3.1 4.1

SEQUENCE=(1.1 3.1 4.1)
FAILED=()

for id in "${SEQUENCE[@]}"; do
  printf '\n========== GICI %s — Table V RTK RRR ==========\n' "$id"

  # RTCM start date is auto-resolved per dataset inside each runner
  # (see gici_rtcm_start_for in scripts/dataset_paths.sh).
  if [[ "${MODE}" != "ros2" ]]; then
    printf '\n--- File-mode (post_estimation_RTK_RRR.yaml) ---\n'
    if [[ "${SKIP_RUN}" -eq 0 ]]; then
      "${REPO}/scripts/run_gici_board_rrr.sh" "${id}" || FAILED+=("file-${id}")
    fi
    GICI_BASELINE_OUT="${REPO}/results/baseline/gici_board/${id}" \
      "${REPO}/scripts/run_author_eval_gici_board.sh" "${id}" --skip-run || FAILED+=("file-eval-${id}")
  fi

  if [[ "${MODE}" != "file" ]]; then
    printf '\n--- ROS2 (post-file port) ---\n'
    if [[ "${SKIP_RUN}" -eq 0 ]]; then
      "${REPO}/scripts/ros2/run_gici_board_rrr_ros2.sh" "${id}" 1 || FAILED+=("ros2-${id}")
    fi
    OUT="${REPO}/output/ros2_gici_board/${id}"
    EVAL_OUT="${REPO}/results/baseline/gici_board_ros2/${id}"
    mkdir -p "${EVAL_OUT}/output"
    cp "${OUT}/solution.txt" "${EVAL_OUT}/output/solution.txt"
    GICI_BASELINE_OUT="${EVAL_OUT}" \
      "${REPO}/scripts/run_author_eval_gici_board.sh" "${id}" --skip-run || FAILED+=("ros2-eval-${id}")
  fi
done

printf '\n========== Table V summary (RTK RRR) ==========\n'
printf '%-6s %-10s %-12s %-12s %-12s %-12s\n' "ID" "Pipeline" "Pos RMSE" "Rot RMSE" "Paper pos" "Paper rot"
for id in "${SEQUENCE[@]}"; do
  case "${id}" in
    1.1) pp="0.03"; pr="0.54" ;;
    3.1) pp="0.29"; pr="1.58" ;;
    4.1) pp="0.08"; pr="0.54" ;;
  esac
  for pipe in file ros2; do
    [[ "${MODE}" == "file" && "${pipe}" == "ros2" ]] && continue
    [[ "${MODE}" == "ros2" && "${pipe}" == "file" ]] && continue
    if [[ "${pipe}" == "file" ]]; then
      m="${REPO}/results/baseline/gici_board/${id}/evaluation/ape_metrics.json"
    else
      m="${REPO}/results/baseline/gici_board_ros2/${id}/evaluation/ape_metrics.json"
    fi
    label="file-mode"; [[ "${pipe}" == "ros2" ]] && label="ros2"
    if [[ -f "${m}" ]]; then
      python3 -c "import json,sys;d=json.load(open(sys.argv[1]));print(f\"{d['dataset']:<6} {sys.argv[2]:<10} {d['ape_translation_rmse_m']:<12.4f} {d['ape_rotation_rmse_deg']:<12.3f} {sys.argv[3]:<12} {sys.argv[4]:<12}\")" "${m}" "${label}" "${pp}" "${pr}"
    else
      printf '%-6s %-10s %-12s %-12s %-12s %-12s\n' "${id}" "${label}" "N/A" "N/A" "${pp}" "${pr}"
    fi
  done
done

if ((${#FAILED[@]})); then
  printf '\nFailed steps: %s\n' "${FAILED[*]}"
  exit 1
fi
