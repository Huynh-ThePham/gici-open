#!/usr/bin/env bash
# Author eval (README §4) for any GICI board dataset with ground_truth.txt.
# Usage: ./scripts/run_author_eval_gici_board.sh <1.1|3.1|4.1> [--skip-run]
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=dataset_paths.sh
source "${ROOT_DIR}/scripts/dataset_paths.sh"

DATASET_ID="${1:-}"
SKIP_RUN=0
if [[ "${2:-}" == "--skip-run" ]]; then
  SKIP_RUN=1
fi

if [[ -z "$DATASET_ID" ]]; then
  printf 'Usage: %s <dataset-id> [--skip-run]\n' "$0" >&2
  exit 1
fi

DATASET_DIR="${GICI_DATA_ROOT}/${DATASET_ID}"
OUT_DIR="${GICI_BASELINE_OUT:-${ROOT_DIR}/results/baseline/gici_board/${DATASET_ID}}"
EVAL_DIR="${OUT_DIR}/evaluation"
GT_DIR="${OUT_DIR}/gt_pipeline"
FC="${ROOT_DIR}/tools/evaluation/format_converters/build"
AL="${ROOT_DIR}/tools/evaluation/alignment/build"
case "${DATASET_ID}" in
  1.1) EXPECTED="${ROOT_DIR}/research/baseline/expected_1_1.json" ;;
  3.1) EXPECTED="${ROOT_DIR}/research/baseline/expected_gici_board_3_1.json" ;;
  4.1) EXPECTED="${ROOT_DIR}/research/baseline/expected_gici_board_4_1.json" ;;
  *)   EXPECTED="${ROOT_DIR}/research/baseline/expected_1_1.json" ;;
esac

require_bin() {
  if [[ ! -x "$1" ]]; then
    printf 'ERROR: missing %s — run ./scripts/build_eval_tools.sh\n' "$1" >&2
    exit 1
  fi
}

if (( SKIP_RUN == 0 )); then
  # RTCM start date is auto-resolved per dataset inside run_gici_board_rrr.sh.
  "${ROOT_DIR}/scripts/run_gici_board_rrr.sh" "$DATASET_ID"
fi

require_bin "${FC}/ie_to_nmea"
require_bin "${FC}/nmea_to_tum"
require_bin "${AL}/nmea_pose_to_pose"
require_bin "${AL}/nmea_align_timestamp"
command -v evo_ape >/dev/null || { printf 'ERROR: evo_ape not found\n' >&2; exit 1; }

SOLUTION="${OUT_DIR}/output/solution.txt"
if [[ ! -f "$SOLUTION" ]]; then
  printf 'ERROR: missing %s\n' "$SOLUTION" >&2
  exit 1
fi
if [[ ! -f "${DATASET_DIR}/ground_truth.txt" ]]; then
  printf 'ERROR: missing %s/ground_truth.txt\n' "$DATASET_DIR" >&2
  exit 1
fi

mkdir -p "$EVAL_DIR" "$GT_DIR"

printf '\nAuthor evaluation — GICI board %s (README §4)\n' "$DATASET_ID"

"${FC}/ie_to_nmea" "${DATASET_DIR}/ground_truth.txt"
mv "${DATASET_DIR}/ground_truth.txt.nmea" "${GT_DIR}/ground_truth.txt.nmea"

"${AL}/nmea_pose_to_pose" "${GT_DIR}/ground_truth.txt.nmea"
"${AL}/nmea_align_timestamp" "${GT_DIR}/ground_truth.txt.nmea.transformed" "$SOLUTION"

"${FC}/nmea_to_tum" "$SOLUTION"
"${FC}/nmea_to_tum" "${GT_DIR}/ground_truth.txt.nmea.transformed.aligned"

mv "${SOLUTION}.tum" "${EVAL_DIR}/trajectory_est.tum"
mv "${GT_DIR}/ground_truth.txt.nmea.transformed.aligned.tum" "${EVAL_DIR}/trajectory_gt.tum"

evo_ape tum "${EVAL_DIR}/trajectory_gt.tum" "${EVAL_DIR}/trajectory_est.tum" \
  -va --align --correct_scale | tee "${EVAL_DIR}/ape_translation.txt"

evo_ape tum "${EVAL_DIR}/trajectory_gt.tum" "${EVAL_DIR}/trajectory_est.tum" \
  -va --align --correct_scale --pose_relation angle_deg | tee "${EVAL_DIR}/ape_rotation.txt"

python3 - <<'PY' "$EVAL_DIR" "$EXPECTED" "$DATASET_ID" "$SOLUTION"
import json, re, sys
from pathlib import Path

eval_dir = Path(sys.argv[1])
expected_path = Path(sys.argv[2])
dataset_id = sys.argv[3]
solution = Path(sys.argv[4])

def parse_rmse(path: Path) -> float:
    text = path.read_text()
    m = re.search(r"^\s*rmse\s+([0-9.]+)\s*$", text, re.MULTILINE)
    if not m:
        raise SystemExit(f"Could not parse RMSE from {path}")
    return float(m.group(1))

def count_gga(path: Path) -> int:
    return sum(1 for line in path.read_text(errors="ignore").splitlines() if "GPGGA" in line)

pos_rmse = parse_rmse(eval_dir / "ape_translation.txt")
rot_rmse = parse_rmse(eval_dir / "ape_rotation.txt")
expected = json.loads(expected_path.read_text()) if expected_path.is_file() else {}
paper = expected.get("paper_reference", {})
paper_tol = expected.get("paper_tolerance", {"ape_translation_rmse_m": 0.35, "ape_rotation_rmse_deg": 0.35})
# Locked-reference key is date-stamped (e.g. locked_reproduce_2026_07_11); take
# whichever one is present rather than hardcoding a single dataset's lock date.
locked_key = next((k for k in expected if k.startswith("locked_reproduce_")), None)
locked = expected.get(locked_key) if locked_key else None
tol = expected.get("tolerance", {"ape_translation_rmse_m": 0.05, "ape_rotation_rmse_deg": 0.15})

metrics = {
    "dataset": dataset_id,
    "algorithm": "rtk_imu_camera_rrr",
    "pipeline": "author_post_estimation_RTK_RRR.yaml",
    "solution_gpgga_epochs": count_gga(solution),
    "ape_translation_rmse_m": pos_rmse,
    "ape_rotation_rmse_deg": rot_rmse,
    "paper_reference": paper,
    "locked_reference": locked,
    "pass": False,
}
if locked:
    metrics["pass"] = (
        abs(pos_rmse - locked["ape_translation_rmse_m"]) <= tol["ape_translation_rmse_m"]
        and abs(rot_rmse - locked["ape_rotation_rmse_deg"]) <= tol["ape_rotation_rmse_deg"]
    )
if paper:
    metrics["paper_pass"] = (
        pos_rmse <= paper.get("ape_position_m", 999) + paper_tol.get("ape_translation_rmse_m", 0.35)
        and rot_rmse <= paper.get("ape_rotation_deg", 999) + paper_tol.get("ape_rotation_rmse_deg", 0.35)
    )

out = eval_dir / "ape_metrics.json"
out.write_text(json.dumps(metrics, indent=2) + "\n")

print("=" * 60)
print(f"Author APE — GICI {dataset_id} (upstream RTK_RRR.yaml)")
print("=" * 60)
print(f"  GPGGA epochs      : {metrics['solution_gpgga_epochs']}")
print(f"  APE position RMSE : {pos_rmse:.4f} m")
print(f"  APE rotation RMSE : {rot_rmse:.3f} deg")
if locked:
    print(f"  Locked reference  : {locked['ape_translation_rmse_m']:.4f} m / {locked['ape_rotation_rmse_deg']:.3f} deg")
    print(f"  PASS vs locked    : {metrics['pass']}")
if paper:
    print(f"  Paper Table V     : {paper.get('ape_position_m')} m / {paper.get('ape_rotation_deg')} deg")
    print(f"  PASS vs paper     : {metrics.get('paper_pass')}")
print(f"  Metrics JSON      : {out}")
print("=" * 60)
PY
