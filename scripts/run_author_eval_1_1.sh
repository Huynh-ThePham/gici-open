#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET_DIR="${GICI_DATASET_1_1:-/home/theph/ws_ncs/1.1}"
OUT_DIR="${GICI_BASELINE_OUT:-${ROOT_DIR}/results/baseline/1_1}"
EVAL_DIR="${OUT_DIR}/evaluation"
GT_DIR="${OUT_DIR}/gt_pipeline"

FC="${ROOT_DIR}/tools/evaluation/format_converters/build"
AL="${ROOT_DIR}/tools/evaluation/alignment/build"
EXPECTED="${ROOT_DIR}/research/baseline/expected_1_1.json"

require_bin() {
  if [[ ! -x "$1" ]]; then
    printf 'ERROR: missing %s — run ./scripts/build_eval_tools.sh\n' "$1" >&2
    exit 1
  fi
}

"${ROOT_DIR}/scripts/run_baseline_1_1.sh"

require_bin "${FC}/ie_to_nmea"
require_bin "${FC}/nmea_to_tum"
require_bin "${AL}/nmea_pose_to_pose"
require_bin "${AL}/nmea_align_timestamp"
command -v evo_ape >/dev/null || { printf 'ERROR: evo_ape not found (pip install evo)\n' >&2; exit 1; }

SOLUTION="${OUT_DIR}/output/solution.txt"
mkdir -p "$EVAL_DIR" "$GT_DIR"

printf '\nAuthor evaluation pipeline (dataset README §4)\n'

"${FC}/ie_to_nmea" "${DATASET_DIR}/ground_truth.txt"
mv "${DATASET_DIR}/ground_truth.txt.nmea" "${GT_DIR}/ground_truth.txt.nmea"

"${AL}/nmea_pose_to_pose" "${GT_DIR}/ground_truth.txt.nmea"
"${AL}/nmea_align_timestamp" "${GT_DIR}/ground_truth.txt.nmea.transformed" "$SOLUTION"

"${FC}/nmea_to_tum" "$SOLUTION"
"${FC}/nmea_to_tum" "${GT_DIR}/ground_truth.txt.nmea.transformed.aligned"

mv "${SOLUTION}.tum" "${EVAL_DIR}/trajectory_est.tum"
mv "${GT_DIR}/ground_truth.txt.nmea.transformed.aligned.tum" "${EVAL_DIR}/trajectory_gt.tum"

GT_TUM="${EVAL_DIR}/trajectory_gt.tum"
EST_TUM="${EVAL_DIR}/trajectory_est.tum"

evo_ape tum "$GT_TUM" "$EST_TUM" -va --align --correct_scale \
  | tee "${EVAL_DIR}/ape_translation.txt"

evo_ape tum "$GT_TUM" "$EST_TUM" -va --align --correct_scale --pose_relation angle_deg \
  | tee "${EVAL_DIR}/ape_rotation.txt"

python3 - <<'PY' "$EVAL_DIR" "$EXPECTED"
import json, re, sys
from pathlib import Path

eval_dir = Path(sys.argv[1])
expected_path = Path(sys.argv[2])

def parse_rmse(path: Path) -> float:
    text = path.read_text()
    m = re.search(r"^\s*rmse\s+([0-9.]+)\s*$", text, re.MULTILINE)
    if not m:
        raise SystemExit(f"Could not parse RMSE from {path}")
    return float(m.group(1))

pos_rmse = parse_rmse(eval_dir / "ape_translation.txt")
rot_rmse = parse_rmse(eval_dir / "ape_rotation.txt")
expected = json.loads(expected_path.read_text())
locked = expected["locked_reproduce_2026_07_11"]
tol = expected["tolerance"]

metrics = {
    "dataset": "1.1",
    "algorithm": "rtk_imu_camera_rrr",
    "pipeline": "author_official",
    "ape_translation_rmse_m": pos_rmse,
    "ape_rotation_rmse_deg": rot_rmse,
    "paper_reference": expected["paper_reference"],
    "locked_reference": locked,
    "pass": (
        abs(pos_rmse - locked["ape_translation_rmse_m"]) <= tol["ape_translation_rmse_m"]
        and abs(rot_rmse - locked["ape_rotation_rmse_deg"]) <= tol["ape_rotation_rmse_deg"]
    ),
}
out = eval_dir / "ape_metrics.json"
out.write_text(json.dumps(metrics, indent=2) + "\n")

print("=" * 60)
print("Author APE check — dataset 1.1")
print("=" * 60)
print(f"  APE position RMSE : {pos_rmse:.4f} m  (paper {expected['paper_reference']['ape_position_m']:.2f} m)")
print(f"  APE rotation RMSE : {rot_rmse:.3f} deg  (paper {expected['paper_reference']['ape_rotation_deg']:.2f} deg)")
print(f"  Locked reference  : {locked['ape_translation_rmse_m']:.4f} m / {locked['ape_rotation_rmse_deg']:.3f} deg")
print(f"  PASS              : {metrics['pass']}")
print(f"  Metrics JSON      : {out}")
print("=" * 60)
PY
