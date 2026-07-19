#!/usr/bin/env bash
# Three-tier metric eval for a completed UrbanNav run directory.
#   raw ENU (no alignment)  -- primary/deployment metric; emitted by eval_paper_tables.py
#   ATE SE(3)  = evo_ape -a                 (rotation+translation align, NO scale)
#   APE Sim(3) = evo_ape -a --correct_scale (Chi et al. Table V metric)
# Reuses the author GT-prep (interpolate GT to this solution's timestamps).
#
# Usage: eval_deep_3tier.sh <dataset: medium|deep> <OUT_DIR containing output/solution.txt>
# Prints: "<OUT_DIR>  ATE_SE3=<m>  APE_Sim3=<m> / <deg>"
set -euo pipefail
DATASET="${1:?dataset (medium|deep)}"
OUT_DIR="${2:?run out dir}"
ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# Pull dataset specs (scene dir, GT file, GPS offset) from the single source of truth.
read -r SCENE_DIR GT_RAW GPS_OFF < <(python3 - "$DATASET" <<'PY'
import sys; sys.path.insert(0, "/home/theph/ws_ncs/gici_vision_aided_ar/scripts")
import run_urbannav_rrr_baseline as r
ds = r.DATASETS[sys.argv[1]]
print(ds.root, ds.gt_file, ds.gps_week_day_offset)
PY
)

FC="${ROOT_DIR}/tools/evaluation/format_converters/build"
AL="${ROOT_DIR}/tools/evaluation/alignment/build"
SOLUTION="${OUT_DIR}/output/solution.txt"
[ -s "$SOLUTION" ] || { echo "$OUT_DIR  MISSING_SOLUTION"; exit 0; }
EVAL_DIR="${OUT_DIR}/eval3tier"; GT_DIR="${EVAL_DIR}/gt"
mkdir -p "$EVAL_DIR" "$GT_DIR"

IE_GT="${GT_DIR}/gt.ie"
python3 "${ROOT_DIR}/scripts/urbannav_gt_to_ie.py" "${SCENE_DIR}/${GT_RAW}" "$IE_GT" >/dev/null 2>&1
IE_GT_SOL="${GT_DIR}/gt.sol.ie"
python3 "${ROOT_DIR}/scripts/urbannav_interp_gt_to_solution.py" \
  "$IE_GT" "$SOLUTION" "$IE_GT_SOL" --gps-week-day-offset "$GPS_OFF" >/dev/null 2>&1
"${FC}/ie_to_nmea" "$IE_GT_SOL" >/dev/null 2>&1
GT_NMEA="${GT_DIR}/gt.sol.nmea"; mv "${IE_GT_SOL}.nmea" "$GT_NMEA"
"${AL}/nmea_align_timestamp" "$GT_NMEA" "$SOLUTION" >/dev/null 2>&1
"${FC}/nmea_to_tum" "$SOLUTION" >/dev/null 2>&1
"${FC}/nmea_to_tum" "${GT_NMEA}.aligned" >/dev/null 2>&1
mv "${SOLUTION}.tum" "${EVAL_DIR}/est.tum"
mv "${GT_NMEA}.aligned.tum" "${EVAL_DIR}/gt.tum"

rmse() { grep -E "^\s*rmse" | grep -oE "[0-9.]+$"; }
SE3=$(evo_ape tum "${EVAL_DIR}/gt.tum" "${EVAL_DIR}/est.tum" -a 2>/dev/null | rmse || true)
SIM3=$(evo_ape tum "${EVAL_DIR}/gt.tum" "${EVAL_DIR}/est.tum" -a --correct_scale 2>/dev/null | rmse || true)
SIM3ROT=$(evo_ape tum "${EVAL_DIR}/gt.tum" "${EVAL_DIR}/est.tum" -a --correct_scale --pose_relation angle_deg 2>/dev/null | rmse || true)
printf '%s  ATE_SE3=%s  APE_Sim3=%s / %s\n' "$OUT_DIR" "${SE3:-NA}" "${SIM3:-NA}" "${SIM3ROT:-NA}"
