#!/usr/bin/env bash
# Author evaluation pipeline (evo_ape Sim(3)) for UrbanNav Medium / Deep.
# Matches dataset README §4 + AUTHOR_METHODOLOGY.md UrbanNav adaptations:
#   urbannav_gt_to_ie → ie_to_nmea → interp GT to solution rate → nmea_align_timestamp
#   → upstream nmea_to_tum → evo_ape (--align --correct_scale)
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=dataset_paths.sh
source "${ROOT_DIR}/scripts/dataset_paths.sh"
DATA_ROOT="${URBANNAV_DATA_ROOT}"
DATASET="${1:-}"

if [[ -z "$DATASET" || ( "$DATASET" != "medium" && "$DATASET" != "deep" ) ]]; then
  printf 'Usage: %s <medium|deep>\n' "$0" >&2
  exit 1
fi

case "$DATASET" in
  medium)
    SCENE_DIR="${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1"
    GT_RAW="UrbanNav_TST_GT_raw.txt"
    GPS_WEEK_DAY_OFFSET="86400.0"
    OUT_DIR="${GICI_BASELINE_OUT:-${ROOT_DIR}/results/baseline/urbannav/medium}"
    EXPECTED="${ROOT_DIR}/research/baseline/expected_urbannav_medium.json"
    ;;
  deep)
    SCENE_DIR="${DATA_ROOT}/UrbanNav-HK-Deep-Urban-1"
    GT_RAW="UrbanNav_whampoa_raw.txt"
    GPS_WEEK_DAY_OFFSET="432000.0"
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

printf '\nAuthor evaluation pipeline — UrbanNav %s (README §4, full trajectory)\n' "$DATASET"

IE_GT="${GT_DIR}/ground_truth.ie"
python3 "${ROOT_DIR}/scripts/urbannav_gt_to_ie.py" "${SCENE_DIR}/${GT_RAW}" "$IE_GT"

IE_GT_SOL="${GT_DIR}/ground_truth.solution_rate.ie"
python3 "${ROOT_DIR}/scripts/urbannav_interp_gt_to_solution.py" \
  "$IE_GT" "$SOLUTION" "$IE_GT_SOL" --gps-week-day-offset "$GPS_WEEK_DAY_OFFSET"

"${FC}/ie_to_nmea" "$IE_GT_SOL"
GT_NMEA="${GT_DIR}/ground_truth.solution_rate.nmea"
mv "${IE_GT_SOL}.nmea" "$GT_NMEA"

# High-rate interpolated GT aligned to solution timestamps (author tool).
"${AL}/nmea_align_timestamp" "$GT_NMEA" "$SOLUTION"
GT_NMEA_ALIGNED="${GT_NMEA}.aligned"

"${FC}/nmea_to_tum" "$SOLUTION"
"${FC}/nmea_to_tum" "$GT_NMEA_ALIGNED"

mv "${SOLUTION}.tum" "${EVAL_DIR}/trajectory_est.tum"
mv "${GT_NMEA_ALIGNED}.tum" "${EVAL_DIR}/trajectory_gt.tum"

GT_TUM="${EVAL_DIR}/trajectory_gt.tum"
EST_TUM="${EVAL_DIR}/trajectory_est.tum"

evo_ape tum "$GT_TUM" "$EST_TUM" -va --align --correct_scale \
  | tee "${EVAL_DIR}/ape_translation.txt"

evo_ape tum "$GT_TUM" "$EST_TUM" -va --align --correct_scale --pose_relation angle_deg \
  | tee "${EVAL_DIR}/ape_rotation.txt"

if [[ "$DATASET" == "medium" ]]; then
  python3 "${ROOT_DIR}/scripts/diagnose_urbannav_medium_eval.py" --out-dir "$OUT_DIR" || true
fi

python3 - <<'PY' "$EVAL_DIR" "$EXPECTED" "$DATASET" "$SOLUTION"
import json, re, sys
from pathlib import Path

eval_dir = Path(sys.argv[1])
expected_path = Path(sys.argv[2])
dataset = sys.argv[3]
solution = Path(sys.argv[4])

def parse_rmse(path: Path) -> float:
    text = path.read_text()
    m = re.search(r"^\s*rmse\s+([0-9.]+)\s*$", text, re.MULTILINE)
    if not m:
        raise SystemExit(f"Could not parse RMSE from {path}")
    return float(m.group(1))

def count_gga(path: Path) -> int:
    n = 0
    for line in path.read_text(errors="ignore").splitlines():
        if "GPGGA" in line:
            n += 1
    return n

def detect_algorithm(solution: Path, default: str = "rtk_imu_camera_rrr") -> str:
    # Read the estimator type from the run_config.yaml that produced this solution rather
    # than hardcoding it, so a VA (or any other) estimator is never mislabeled as the
    # baseline in the emitted metrics. Walk up a few levels to find the config.
    for base in [solution.parent, solution.parent.parent, solution.parent.parent.parent]:
        cfg = base / "run_config.yaml"
        if cfg.exists():
            m = re.search(r"^\s*type:\s*(rtk_imu_camera_rrr\w*)\s*$",
                          cfg.read_text(errors="ignore"), re.MULTILINE)
            if m:
                return m.group(1)
    return default

pos_rmse = parse_rmse(eval_dir / "ape_translation.txt")
rot_rmse = parse_rmse(eval_dir / "ape_rotation.txt")
expected = json.loads(expected_path.read_text())
paper = expected["paper_reference"]
paper_tol = expected.get("paper_tolerance", {"ape_translation_rmse_m": 0.35, "ape_rotation_rmse_deg": 0.35})
# Locked-reference key is date-stamped (e.g. locked_reproduce_2026_07_16); take
# the most recent one present rather than hardcoding specific dates.
locked_key = max(
    (k for k in expected if k.startswith("locked_reproduce_")), default=None
)
locked = expected.get(locked_key) if locked_key else None

metrics = {
    "dataset": f"urbannav_{dataset}",
    "algorithm": detect_algorithm(solution),
    "pipeline": "author_readme4_interp_gt_full",
    "solution_gpgga_epochs": count_gga(solution),
    "ape_translation_rmse_m": pos_rmse,
    "ape_rotation_rmse_deg": rot_rmse,
    "paper_reference": paper,
    "paper_pass": (
        pos_rmse <= paper["ape_position_m"] + paper_tol["ape_translation_rmse_m"]
        and rot_rmse <= paper["ape_rotation_deg"] + paper_tol["ape_rotation_rmse_deg"]
    ),
    "locked_reference": locked,
    "pass": (
        abs(pos_rmse - locked["ape_translation_rmse_m"]) <= expected["tolerance"]["ape_translation_rmse_m"]
        and abs(rot_rmse - locked["ape_rotation_rmse_deg"]) <= expected["tolerance"]["ape_rotation_rmse_deg"]
    ) if locked else False,
}
out = eval_dir / "ape_metrics.json"
out.write_text(json.dumps(metrics, indent=2) + "\n")

print("=" * 60)
print(f"Author APE check — UrbanNav {dataset} (README §4)")
print("=" * 60)
print(f"  Solution GPGGA     : {metrics['solution_gpgga_epochs']}")
print(f"  APE position RMSE  : {pos_rmse:.4f} m  (paper {paper['ape_position_m']:.2f} m)")
print(f"  APE rotation RMSE  : {rot_rmse:.3f} deg  (paper {paper['ape_rotation_deg']:.2f} deg)")
if locked:
    print(f"  Locked reference   : {locked['ape_translation_rmse_m']:.4f} m / {locked['ape_rotation_rmse_deg']:.3f} deg")
print(f"  PASS vs paper      : {metrics['paper_pass']}")
print(f"  PASS vs locked     : {metrics['pass']}")
print(f"  Metrics JSON       : {out}")
print("=" * 60)
PY
