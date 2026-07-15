#!/usr/bin/env bash
# Analyze rotation APE outliers from a stress-test batch.
#
# Usage:
#   ./scripts/ros2/analyze_rotation_outliers.sh results/stress/bag_replay_1.1/batch30_v2
#   ./scripts/ros2/analyze_rotation_outliers.sh results/stress/bag_replay_1.1 2.0
#
# Writes: <batch>/rotation_outlier_analysis.json
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BATCH_ROOT="${1:?batch root dir}"
ROT_THRESHOLD="${2:-2.0}"
DATASET_ID="${3:-1.1}"

CSV="${BATCH_ROOT}/summary.csv"
OUT_JSON="${BATCH_ROOT}/rotation_outlier_analysis.json"
[[ -f "${CSV}" ]] || { echo "ERROR: missing ${CSV}" >&2; exit 1; }

python3 - "${CSV}" "${OUT_JSON}" "${ROT_THRESHOLD}" "${DATASET_ID}" "${REPO}" <<'PY'
import csv, json, statistics, sys
from pathlib import Path

csv_path, out_json, rot_threshold, dataset_id, repo = sys.argv[1:6]
rot_threshold = float(rot_threshold)
repo = Path(repo)

rows = []
with open(csv_path, newline="") as f:
    for raw in f:
        raw = raw.strip()
        if not raw or raw.startswith("run,"):
            continue
        parts = raw.split(",")
        if len(parts) < 12:
            continue
        run_id = parts[0]
        try:
            exit_code = int(parts[1])
            gpgga = int("".join(c for c in parts[2] if c.isdigit()) or "0")
            sparsify = int(parts[3])
            runtime_s = float(parts[4])
            ape_pos = None if parts[5] in ("", "nan") else float(parts[5])
            ape_rot = None if parts[6] in ("", "nan") else float(parts[6])
        except (ValueError, IndexError):
            continue
        if ape_rot is None:
            continue
        rows.append({
            "run": run_id,
            "exit_code": exit_code,
            "gpgga": gpgga,
            "sparsify_count": sparsify,
            "runtime_s": runtime_s,
            "ape_pos_m": ape_pos,
            "ape_rot_deg": ape_rot,
        })

rots = [r["ape_rot_deg"] for r in rows]
median = statistics.median(rots)
mad = statistics.median([abs(x - median) for x in rots]) if rots else 0.0

outliers = []
for r in rows:
    rot = r["ape_rot_deg"]
    if rot >= rot_threshold or (mad > 0 and abs(rot - median) > 3 * 1.4826 * mad):
        run_dir = Path(csv_path).parent / f"run_{r['run']}"
        replay = run_dir / "replay"
        outliers.append({
            "run": r["run"],
            "ape_rot_deg": rot,
            "ape_pos_m": r["ape_pos_m"],
            "gpgga": r["gpgga"],
            "sparsify_count": r["sparsify_count"],
            "median_delta_deg": rot - median,
            "run_dir": str(run_dir),
            "node_log": str(replay / "node.log"),
            "solution": str(replay / "solution.txt"),
        })

# Pull sparsification / init hints from node logs for outlier runs.
for o in outliers:
    logp = Path(o["node_log"])
    hints = {"sparsify_events": 0, "ref_antenna_errors": 0, "segfault": False}
    if logp.is_file():
        text = logp.read_text(errors="ignore")
        hints["sparsify_events"] = text.count("Sparsifying measurements")
        hints["ref_antenna_errors"] = text.count("Unable to get antenna position of reference station")
        hints["segfault"] = "Received a segment fault" in text or "handleSegv" in text
    o["log_hints"] = hints

summary = {
    "dataset": dataset_id,
    "batch_root": str(Path(csv_path).parent),
    "runs_analyzed": len(rows),
    "rotation_threshold_deg": rot_threshold,
    "ape_rot_deg": {
        "min": min(rots) if rots else None,
        "max": max(rots) if rots else None,
        "median": median,
        "mean": statistics.mean(rots) if rots else None,
        "mad": mad,
    },
    "outlier_runs": [o["run"] for o in outliers],
    "outliers": outliers,
    "interpretation": (
        "Rotation APE spikes on otherwise complete trajectories are typically "
        "alignment sensitivity (yaw ambiguity / short baseline segments) under "
        "real-time sparsification load, not position divergence. "
        "Check sparsify_events vs median and compare pos APE (should stay ~0.07-0.15m)."
    ),
}
Path(out_json).write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
PY
