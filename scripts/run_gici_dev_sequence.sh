#!/usr/bin/env bash
# Re-run author pipeline: GICI 1.1 → 3.1 → 4.1 (upstream post_estimation_RTK_RRR.yaml)
set -Eeuo pipefail
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
chmod +x "${ROOT_DIR}/scripts/run_gici_board_rrr.sh" "${ROOT_DIR}/scripts/run_author_eval_gici_board.sh"

SEQUENCE=(1.1 3.1 4.1)
FAILED=()
for id in "${SEQUENCE[@]}"; do
  printf '\n========== GICI %s ==========\n' "$id"
  # RTCM start date auto-resolved per dataset (see dataset_paths.sh).
  "${ROOT_DIR}/scripts/run_author_eval_gici_board.sh" "$id" || FAILED+=("$id")
done

printf '\n========== Summary ==========\n'
for id in "${SEQUENCE[@]}"; do
  m="${ROOT_DIR}/results/baseline/gici_board/${id}/evaluation/ape_metrics.json"
  if [[ -f "$m" ]]; then
    python3 -c "import json; d=json.load(open('$m')); print(f\"  {d['dataset']}: pos={d['ape_translation_rmse_m']:.4f}m rot={d['ape_rotation_rmse_deg']:.3f}deg\")"
  else
    printf '  %s: NO RESULT\n' "$id"
  fi
done
if ((${#FAILED[@]})); then
  printf 'Failed: %s\n' "${FAILED[*]}"
  exit 1
fi
