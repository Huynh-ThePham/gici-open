#!/usr/bin/env bash
# Real-time ROS2 bag replay pipeline for GICI board RTK RRR.
#
# Steps: *.bin -> ROS1 bags (author tool) -> merged ROS2 bag -> gici_ros2_main --bag
#        -> author APE eval, compared vs file-mode / postfile baseline.
#
# Usage:
#   ./scripts/run_gici_board_bag_replay.sh 1.1
#   ./scripts/run_gici_board_bag_replay.sh 3.1 4.1
#   GICI_BAG_REPLAY_SKIP_RUN=1 ./scripts/run_gici_board_bag_replay.sh 1.1  # eval only
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
chmod +x "${ROOT_DIR}/scripts/ros2/run_gici_board_rrr_ros2.sh" \
         "${ROOT_DIR}/scripts/ros2/build_gici_board_ros1_bags.sh" \
         "${ROOT_DIR}/scripts/run_author_eval_gici_board.sh" 2>/dev/null || true

SEQUENCE=("$@")
[[ ${#SEQUENCE[@]} -eq 0 ]] && SEQUENCE=(1.1)
SKIP_RUN="${GICI_BAG_REPLAY_SKIP_RUN:-0}"
RATE="${GICI_BAG_REPLAY_RATE:-1}"
FAILED=()

for id in "${SEQUENCE[@]}"; do
  printf '\n========== GICI %s — ROS2 bag replay ==========\n' "$id"
  OUT="${ROOT_DIR}/output/ros2_gici_board_bag/${id}"
  EVAL_OUT="${ROOT_DIR}/results/baseline/gici_board_ros2_bag/${id}"
  POSTFILE_EVAL="${ROOT_DIR}/results/baseline/gici_board_ros2/${id}/evaluation/ape_metrics.json"

  if (( SKIP_RUN == 0 )); then
    mkdir -p "${OUT}/log"
    GICI_ROS2_BOARD_OUT="${OUT}" \
      "${ROOT_DIR}/scripts/ros2/run_gici_board_rrr_ros2.sh" \
        --bag ${GICI_FORCE_BAG_REBUILD:+--force-rebuild-bags} --force-reconvert "${id}" "${RATE}" \
        2>&1 | tee "${OUT}/run.log" || { FAILED+=("run-${id}"); continue; }
  fi

  SOL="${OUT}/solution.txt"
  if [[ ! -s "${SOL}" ]]; then
    printf 'ERROR: no solution for %s\n' "$id" >&2
    FAILED+=("nosol-${id}")
    continue
  fi

  mkdir -p "${EVAL_OUT}/output"
  cp "${SOL}" "${EVAL_OUT}/output/solution.txt"
  GICI_BASELINE_OUT="${EVAL_OUT}" \
    "${ROOT_DIR}/scripts/run_author_eval_gici_board.sh" "$id" --skip-run || FAILED+=("eval-${id}")
done

printf '\n========== Bag replay vs postfile (ROS2) ==========\n'
printf '%-6s %-14s %-12s %-12s %-14s %-12s\n' "ID" "Pipeline" "PosRMSE(m)" "RotRMSE(deg)" "RefPos(m)" "RefRot(deg)"
for id in "${SEQUENCE[@]}"; do
  bag_m="${ROOT_DIR}/results/baseline/gici_board_ros2_bag/${id}/evaluation/ape_metrics.json"
  pf_m="${ROOT_DIR}/results/baseline/gici_board_ros2/${id}/evaluation/ape_metrics.json"
  if [[ -f "${bag_m}" ]]; then
    python3 -c "
import json,sys
bag=json.load(open('${bag_m}'))
pf=json.load(open('${pf_m}')) if __import__('pathlib').Path('${pf_m}').is_file() else {}
rp=pf.get('ape_translation_rmse_m','N/A')
rr=pf.get('ape_rotation_rmse_deg','N/A')
rp_s=f'{rp:.4f}' if isinstance(rp,float) else str(rp)
rr_s=f'{rr:.3f}' if isinstance(rr,float) else str(rr)
print(f\"{bag['dataset']:<6} {'bag-replay':<14} {bag['ape_translation_rmse_m']:<12.4f} {bag['ape_rotation_rmse_deg']:<12.3f} {rp_s:<14} {rr_s:<12}\")
"
  else
    printf '%-6s %-14s %-12s %-12s %-14s %-12s\n' "$id" "bag-replay" "N/A" "N/A" "N/A" "N/A"
  fi
done

if ((${#FAILED[@]})); then
  printf '\nFailed: %s\n' "${FAILED[*]}"
  exit 1
fi
