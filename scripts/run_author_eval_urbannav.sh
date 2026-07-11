#!/usr/bin/env bash
# Author evaluation pipeline (evo_ape Sim(3)) for UrbanNav Medium / Deep.
# UrbanNav GT is TST INS (body=IMU); skip nmea_pose_to_pose used for GICI 1.1 fiber IMU.
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}"
DATASET="${1:-}"

if [[ -z "$DATASET" || ( "$DATASET" != "medium" && "$DATASET" != "deep" ) ]]; then
  printf 'Usage: %s <medium|deep>\n' "$0" >&2
  exit 1
fi

case "$DATASET" in
  medium)
    SCENE_DIR="${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1"
    GT_RAW="UrbanNav_TST_GT_raw.txt"
    OUT_DIR="${GICI_BASELINE_OUT:-${ROOT_DIR}/results/baseline/urbannav/medium}"
    EXPECTED="${ROOT_DIR}/research/baseline/expected_urbannav_medium.json"
    ;;
  deep)
    SCENE_DIR="${DATA_ROOT}/UrbanNav-HK-Deep-Urban-1"
    GT_RAW="UrbanNav_whampoa_raw.txt"
    OUT_DIR="${GICI_BASELINE_OUT:-${ROOT_DIR}/results/baseline/urbannav/deep}"
    EXPECTED="${ROOT_DIR}/research/baseline/expected_urbannav_deep.json"
    ;;
esac

FC="${ROOT_DIR}/tools/evaluation/format_converters/build"
AL="${ROOT_DIR}/tools/evaluation/alignment/build"
EVAL_DIR="${OUT_DIR}/evaluation"
GT_DIR="${OUT_DIR}/gt_pipeline"
SOLUTION="${OUT_DIR}/output/solution.txt"

require_bin() {
  if [[ ! -x "$1" ]]; then
    printf 'ERROR: missing %s — run ./scripts/build_eval_tools.sh\n' "$1" >&2
    exit 1
  fi
}

require_bin "${FC}/ie_to_nmea"
require_bin "${FC}/nmea_to_tum"
require_bin "${AL}/nmea_align_timestamp"
command -v evo_ape >/dev/null || { printf 'ERROR: evo_ape not found (pip install evo)\n' >&2; exit 1; }

if [[ ! -f "$SOLUTION" ]]; then
  printf 'ERROR: missing solution %s — run scripts/run_urbannav_rrr_baseline.py first\n' "$SOLUTION" >&2
  exit 1
fi

mkdir -p "$EVAL_DIR" "$GT_DIR"

printf '\nAuthor evaluation pipeline — UrbanNav %s\n' "$DATASET"

IE_GT="${GT_DIR}/ground_truth.ie"
IE_GT_HR="${GT_DIR}/ground_truth.solution_rate.ie"
python3 "${ROOT_DIR}/scripts/urbannav_gt_to_ie.py" "${SCENE_DIR}/${GT_RAW}" "$IE_GT"

case "$DATASET" in
  medium) GPS_WEEK_DAY_OFFSET="86400.0" ;;
  deep)   GPS_WEEK_DAY_OFFSET="432000.0" ;;
esac
python3 "${ROOT_DIR}/scripts/urbannav_interp_gt_to_solution.py" \
  "$IE_GT" "$SOLUTION" "$IE_GT_HR" --gps-week-day-offset "$GPS_WEEK_DAY_OFFSET"

"${FC}/ie_to_nmea" "$IE_GT_HR"
mv "${IE_GT_HR}.nmea" "${GT_DIR}/ground_truth.solution_rate.nmea"

"${AL}/nmea_align_timestamp" "${GT_DIR}/ground_truth.solution_rate.nmea" "$SOLUTION"

"${FC}/nmea_to_tum" "$SOLUTION"
"${FC}/nmea_to_tum" "${GT_DIR}/ground_truth.solution_rate.nmea.aligned"

mv "${SOLUTION}.tum" "${EVAL_DIR}/trajectory_est.tum"
mv "${GT_DIR}/ground_truth.solution_rate.nmea.aligned.tum" "${EVAL_DIR}/trajectory_gt.tum"

GT_TUM="${EVAL_DIR}/trajectory_gt.tum"
EST_TUM="${EVAL_DIR}/trajectory_est.tum"

evo_ape tum "$GT_TUM" "$EST_TUM" -va --align --correct_scale \
  | tee "${EVAL_DIR}/ape_translation.txt"

evo_ape tum "$GT_TUM" "$EST_TUM" -va --align --correct_scale --pose_relation angle_deg \
  | tee "${EVAL_DIR}/ape_rotation.txt"

if [[ "$DATASET" == "medium" ]]; then
  python3 "${ROOT_DIR}/scripts/diagnose_urbannav_medium_eval.py" --out-dir "$OUT_DIR" || true
fi

python3 - <<'PY' "$EVAL_DIR" "$EXPECTED" "$DATASET"
import json, re, sys
from pathlib import Path

eval_dir = Path(sys.argv[1])
expected_path = Path(sys.argv[2])
dataset = sys.argv[3]

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
paper = expected["paper_reference"]

metrics = {
    "dataset": f"urbannav_{dataset}",
    "algorithm": "rtk_imu_camera_rrr",
    "pipeline": "author_official",
    "ape_translation_rmse_m": pos_rmse,
    "ape_rotation_rmse_deg": rot_rmse,
    "paper_reference": paper,
    "locked_reference": locked,
    "pass": (
        abs(pos_rmse - locked["ape_translation_rmse_m"]) <= tol["ape_translation_rmse_m"]
        and abs(rot_rmse - locked["ape_rotation_rmse_deg"]) <= tol["ape_rotation_rmse_deg"]
    ),
}
out = eval_dir / "ape_metrics.json"
out.write_text(json.dumps(metrics, indent=2) + "\n")

print("=" * 60)
print(f"Author APE check — UrbanNav {dataset}")
print("=" * 60)
print(f"  APE position RMSE : {pos_rmse:.4f} m  (paper {paper['ape_position_m']:.2f} m)")
print(f"  APE rotation RMSE : {rot_rmse:.3f} deg  (paper {paper['ape_rotation_deg']:.2f} deg)")
print(f"  Locked reference  : {locked['ape_translation_rmse_m']:.4f} m / {locked['ape_rotation_rmse_deg']:.3f} deg")
print(f"  PASS (reproduce)  : {metrics['pass']}")
print(f"  Metrics JSON      : {out}")
print("=" * 60)
PY
