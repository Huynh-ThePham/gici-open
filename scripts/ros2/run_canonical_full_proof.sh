#!/usr/bin/env bash
# End-to-end proof run: converter -> canonical bag -> full RRR @ rate 1 -> guard
# -> author eval vs file-mode baseline. Writes a machine-readable report.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"
DS="${1:-medium}"
RATE="${2:-1}"
OUT_DIR="${REPO}/output/ros2_urbannav_rrr_canonical/${DS}"
PROOF_DIR="${OUT_DIR}/proof_$(date -u +%Y%m%dT%H%M%SZ)"
REPORT="${PROOF_DIR}/canonical_proof.json"
EXPECTED="${REPO}/research/baseline/expected_urbannav_medium.json"
FILEMODE="${REPO}/research/baseline/filemode_urbannav_rrr_v1.json"

mkdir -p "${PROOF_DIR}"

log() { echo "[proof] $*" | tee -a "${PROOF_DIR}/proof.log"; }

log "Proof run started: dataset=${DS} rate=${RATE}"
log "Output: ${PROOF_DIR}"

# --- 1) Patch fingerprint (source must contain critical fixes) ---
python3 - <<'PY' "${REPO}" "${PROOF_DIR}/patch_check.json"
import json, sys
from pathlib import Path
repo = Path(sys.argv[1])
checks = {
    "makeEigenShared": (repo / "include/gici/utility/common.h").read_text(),
    "transformationFromPoseParameters": (repo / "include/gici/utility/transform.h").read_text(),
    "pose_block_normalize": (repo / "src/estimate/pose_parameter_block.cpp").read_text(),
    "fromApproximateRotationMatrix": (repo / "third_party/rpg_vikit/vikit_common/src/vikit_cameras/ncamera_yaml_serialization.cpp").read_text(),
    "gnss_free_nullguard": (repo / "src/stream/formator.cpp").read_text(),
    "rinex_null_terminate": (repo / "src/stream/formator.cpp").read_text(),
}
ok = {
    "makeEigenShared": "makeEigenShared" in checks["makeEigenShared"] and "shared_ptr<T>(new T" in checks["makeEigenShared"],
    "transformationFromPoseParameters": "transformationFromPoseParameters" in checks["transformationFromPoseParameters"],
    "pose_block_normalize": "q.normalize()" in checks["pose_block_normalize"] and "svo::Quaternion" in checks["pose_block_normalize"],
    "fromApproximateRotationMatrix": "fromApproximateRotationMatrix" in checks["fromApproximateRotationMatrix"],
    "gnss_free_nullguard": "if (observation != nullptr)" in checks["gnss_free_nullguard"],
    "rinex_null_terminate": "line_.push_back('\\0')" in checks["rinex_null_terminate"],
}
out = {"all_ok": all(ok.values()), "checks": ok}
Path(sys.argv[2]).write_text(json.dumps(out, indent=2) + "\n")
if not out["all_ok"]:
    print("PATCH CHECK FAILED:", ok, file=sys.stderr)
    sys.exit(2)
print("patch_check: OK")
PY

# --- 2) Rebuild ROS2 wrapper (Release) ---
log "Rebuilding gici_ros2 (Release)..."
set +u
source /opt/ros/humble/setup.bash
cd "${WS}"
colcon build --symlink-install --packages-select gici_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
  >> "${PROOF_DIR}/build.log" 2>&1
source install/setup.bash
set -u

# --- 3) Force fresh pipeline artifacts ---
log "Removing stale canonical bag + solution for reproducible rebuild..."
rm -rf "${OUT_DIR}/canonical_bag" "${OUT_DIR}/solution.txt" "${OUT_DIR}/node.log"
mkdir -p "${OUT_DIR}/log"

# --- 4) Converter: RINEX/BSX -> canonical bag ---
log "Running urbannav_rinex_to_ros2.sh ${DS}..."
"${REPO}/scripts/ros2/urbannav_rinex_to_ros2.sh" "${DS}" \
  2>&1 | tee "${PROOF_DIR}/converter.log"

# --- 5) Full canonical RRR @ rate ---
log "Running full canonical RRR (rate=${RATE}, expect ~13 min playback + drain)..."
START_TS=$(date +%s)
set +e
"${REPO}/scripts/ros2/run_urbannav_rrr_ros2.sh" --canonical "${DS}" "${RATE}" \
  2>&1 | tee "${PROOF_DIR}/rrr.log"
RRR_RC=$?
set -e
END_TS=$(date +%s)
WALL_S=$((END_TS - START_TS))
log "RRR finished rc=${RRR_RC} wall_s=${WALL_S}"

cp -a "${OUT_DIR}/solution.txt" "${PROOF_DIR}/solution.txt" 2>/dev/null || true
cp -a "${OUT_DIR}/node.log" "${PROOF_DIR}/node.log" 2>/dev/null || true
cp -a "${OUT_DIR}/canonical_bag/manifest.json" "${PROOF_DIR}/manifest.json" 2>/dev/null || true

# --- 6) Solution stats + guard ---
python3 - <<'PY' "${OUT_DIR}/solution.txt" "${OUT_DIR}/canonical_bag/manifest.json" "${PROOF_DIR}/solution_stats.json"
import json, math, sys
from pathlib import Path
solution = Path(sys.argv[1])
manifest = Path(sys.argv[2])
out = Path(sys.argv[3])

def parse_solution(path: Path):
    gga = []
    fix_counts = {}
    for line in path.read_text(errors="ignore").splitlines():
        if "GPGGA" not in line:
            continue
        parts = line.split(",")
        if len(parts) < 7 or len(parts[1]) < 6:
            continue
        try:
            hh, mm = int(parts[1][0:2]), int(parts[1][2:4])
            ss = float(parts[1][4:])
            q = int(parts[6])
        except ValueError:
            continue
        gga.append(hh * 3600.0 + mm * 60.0 + ss)
        fix_counts[q] = fix_counts.get(q, 0) + 1
    return gga, fix_counts

gga, fix_counts = parse_solution(solution)
m = json.loads(manifest.read_text())
rover_lo, rover_hi = m["rover_drive_utc"]

def sod(t): return math.fmod(t, 86400.0)
def unwrap(v, a):
    while v - a > 43200: v -= 86400
    while a - v > 43200: v += 86400
    return v

drive_start = sod(rover_lo)
drive_end = unwrap(sod(rover_hi), drive_start)
sol_start = unwrap(gga[0], drive_start) if gga else 0
sol_end = unwrap(gga[-1], sol_start) if gga else 0
if gga and sol_end < sol_start:
    sol_end += 86400
drive_span = max(0.0, drive_end - drive_start)
sol_span = max(0.0, sol_end - sol_start) if gga else 0.0
end_gap = drive_end - sol_end if gga else drive_span
rtk_fixed = fix_counts.get(5, 0)
epochs = len(gga)
guard_pass = (
    epochs > 0 and drive_span > 0
    and sol_span >= drive_span * 0.85
    and end_gap <= 75.0
)
stats = {
    "epochs_gpgga": epochs,
    "solution_span_s": round(sol_span, 2),
    "drive_span_s": round(drive_span, 2),
    "end_gap_s": round(end_gap, 2),
    "fix_quality_counts": fix_counts,
    "rtk_fixed_epochs_q5": rtk_fixed,
    "rtk_fixed_ratio": round(rtk_fixed / epochs, 4) if epochs else 0.0,
    "guard_pass": guard_pass,
}
out.write_text(json.dumps(stats, indent=2) + "\n")
print(json.dumps(stats, indent=2))
if not guard_pass:
    sys.exit(4)
PY

# --- 7) Author eval vs file-mode locked baseline ---
EVAL_OUT="${PROOF_DIR}/author_eval"
mkdir -p "${EVAL_OUT}/output"
cp "${OUT_DIR}/solution.txt" "${EVAL_OUT}/output/solution.txt"
export GICI_BASELINE_OUT="${EVAL_OUT}"
log "Running author eval..."
"${REPO}/scripts/run_author_eval_urbannav.sh" "${DS}" \
  2>&1 | tee "${PROOF_DIR}/author_eval.log"

# --- 8) Aggregate proof report ---
python3 - <<'PY' "${PROOF_DIR}" "${EXPECTED}" "${FILEMODE}" "${RRR_RC}" "${WALL_S}" "${REPORT}"
import json, sys
from pathlib import Path

proof = Path(sys.argv[1])
expected_path = Path(sys.argv[2])
filemode_path = Path(sys.argv[3])
rrr_rc = int(sys.argv[4])
wall_s = float(sys.argv[5])
report_path = Path(sys.argv[6])

patch = json.loads((proof / "patch_check.json").read_text())
stats = json.loads((proof / "solution_stats.json").read_text())
expected = json.loads(expected_path.read_text())
filemode = json.loads(filemode_path.read_text())
locked = expected.get("locked_reproduce_2026_07_14") or expected["locked_reproduce_2026_07_11"]
fm_locked = filemode["datasets"]["medium"]["locked"]
tol = expected["tolerance"]

ape_metrics = {}
ape_file = proof / "author_eval" / "evaluation" / "ape_metrics.json"
if ape_file.exists():
    ape_metrics = json.loads(ape_file.read_text())

def within(a, b, t):
    return abs(a - b) <= t

report = {
    "proof_tag": "ros2-canonical-full-rrr-proof-v1",
    "rrr_exit_code": rrr_rc,
    "wall_s": wall_s,
    "patch_check": patch,
    "solution_stats": stats,
    "filemode_baseline": {
        "tag": filemode["baseline_tag"],
        "solution_gpgga_epochs": fm_locked["solution_gpgga_epochs"],
        "ape_translation_rmse_m": fm_locked["ape_translation_rmse_m"],
        "ape_rotation_rmse_deg": fm_locked["ape_rotation_rmse_deg"],
    },
    "author_eval": ape_metrics,
    "criteria": {
        "patches_present": patch["all_ok"],
        "guard_pass": stats["guard_pass"],
        "rrr_exit_zero": rrr_rc == 0,
        "epochs_within_5pct": (
            stats["epochs_gpgga"] >= int(fm_locked["solution_gpgga_epochs"] * 0.95)
            if stats["epochs_gpgga"] else False
        ),
        "ape_translation_within_tol": (
            within(ape_metrics.get("ape_translation_rmse_m", -1),
                   locked["ape_translation_rmse_m"], tol["ape_translation_rmse_m"])
            if ape_metrics else False
        ),
        "ape_rotation_within_tol": (
            within(ape_metrics.get("ape_rotation_rmse_m", ape_metrics.get("ape_rotation_rmse_deg", -1)),
                   locked["ape_rotation_rmse_deg"], tol["ape_rotation_rmse_deg"])
            if ape_metrics else False
        ),
    },
}
# fix key name for rotation
if ape_metrics:
    report["criteria"]["ape_rotation_within_tol"] = within(
        ape_metrics.get("ape_rotation_rmse_deg", -1),
        locked["ape_rotation_rmse_deg"],
        tol["ape_rotation_rmse_deg"],
    )

report["all_pass"] = all(report["criteria"].values())
report_path.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
sys.exit(0 if report["all_pass"] else 5)
PY

log "Proof report: ${REPORT}"
