#!/usr/bin/env bash
# ROS2 RRR sequence: GICI board 1.1 -> 3.1 -> 4.1 (post-file port), evaluate,
# and print a side-by-side comparison vs file-mode and paper Table V.
#
# Usage:
#   ./scripts/run_gici_board_ros2_sequence.sh            # run 1.1 3.1 4.1
#   ./scripts/run_gici_board_ros2_sequence.sh 3.1 4.1    # subset
#   GICI_ROS2_SKIP_RUN=1 ./scripts/run_gici_board_ros2_sequence.sh   # eval only
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
chmod +x "${ROOT_DIR}/scripts/ros2/run_gici_board_rrr_ros2.sh" \
         "${ROOT_DIR}/scripts/run_author_eval_gici_board.sh" 2>/dev/null || true

SEQUENCE=("$@")
[[ ${#SEQUENCE[@]} -eq 0 ]] && SEQUENCE=(1.1 3.1 4.1)
SKIP_RUN="${GICI_ROS2_SKIP_RUN:-0}"
FAILED=()

for id in "${SEQUENCE[@]}"; do
  printf '\n========== GICI %s (ROS2) ==========\n' "$id"
  OUT="${ROOT_DIR}/output/ros2_gici_board/${id}"
  EVAL_OUT="${ROOT_DIR}/results/baseline/gici_board_ros2/${id}"

  if (( SKIP_RUN == 0 )); then
    if ! "${ROOT_DIR}/scripts/ros2/run_gici_board_rrr_ros2.sh" "$id" 1; then
      FAILED+=("run-${id}")
      continue
    fi
  fi

  if [[ ! -s "${OUT}/solution.txt" ]]; then
    printf 'ERROR: missing %s/solution.txt\n' "$OUT" >&2
    FAILED+=("nosol-${id}")
    continue
  fi
  mkdir -p "${EVAL_OUT}/output"
  cp "${OUT}/solution.txt" "${EVAL_OUT}/output/solution.txt"
  GICI_BASELINE_OUT="${EVAL_OUT}" \
    "${ROOT_DIR}/scripts/run_author_eval_gici_board.sh" "$id" --skip-run || FAILED+=("eval-${id}")
done

printf '\n================= Table V — RTK RRR (file-mode vs ROS2) =================\n'
printf '%-6s %-14s %-12s %-12s %-12s %-12s\n' "ID" "Pipeline" "PosRMSE(m)" "RotRMSE(deg)" "PaperPos" "PaperRot"
for id in "${SEQUENCE[@]}"; do
  case "$id" in
    1.1) pp=0.03; pr=0.54 ;;
    3.1) pp=0.29; pr=1.58 ;;
    4.1) pp=0.08; pr=0.54 ;;
    *)   pp="-";  pr="-" ;;
  esac
  fm="${ROOT_DIR}/results/baseline/gici_board/${id}/evaluation/ape_metrics.json"
  r2="${ROOT_DIR}/results/baseline/gici_board_ros2/${id}/evaluation/ape_metrics.json"
  for label in file-mode ros2; do
    m="$fm"; [[ "$label" == ros2 ]] && m="$r2"
    if [[ -f "$m" ]]; then
      python3 -c "import json;d=json.load(open('$m'));print(f\"{'$id':<6} {'$label':<14} {d['ape_translation_rmse_m']:<12.4f} {d['ape_rotation_rmse_deg']:<12.3f} {'$pp':<12} {'$pr':<12}\")"
    else
      printf '%-6s %-14s %-12s %-12s %-12s %-12s\n' "$id" "$label" "N/A" "N/A" "$pp" "$pr"
    fi
  done
done

if ((${#FAILED[@]})); then
  printf '\nFailed steps: %s\n' "${FAILED[*]}"
  exit 1
fi
