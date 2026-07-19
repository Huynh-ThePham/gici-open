#!/usr/bin/env bash
# Run the ported GICI ROS 2 wrapper on UrbanNav in FULL RRR mode
# (RTK + IMU + camera, tightly coupled: rtk_imu_camera_rrr).
#
# Two modes:
#   --canonical  Same-input, file-mode-EQUIVALENT run. Uses the canonical bag
#                (rover .obs + HKKT base .rnx + brdc eph .rnx + DCB .BSX, i.e. the
#                EXACT inputs of research/config/rtk_imu_camera_rrr_urbannav.yaml)
#                built by scripts/ros2/urbannav_rinex_to_ros2.sh, and the
#                config/ros_urbannav_rrr_upstream_equivalent.yaml estimator. This
#                is the config to compare numerically against the file-mode baseline.
#   --adapted    (default) ROS 2-native run. Uses the merged bag from the UrbanNav
#                gici GNSS ROS 1 bags (HKSC base, no DCB) + config/
#                ros_urbannav_rrr_ros2_adapted.yaml.
#
# Usage:
#   scripts/ros2/run_urbannav_rrr_ros2.sh [--canonical|--adapted] [medium] [rate]
#
# Dataset paths come from environment variables (see below); nothing is hardcoded
# to a specific machine beyond overridable defaults.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"

MODE="adapted"
POS=()
for a in "$@"; do
  case "${a}" in
    --canonical) MODE="canonical" ;;
    --adapted)   MODE="adapted" ;;
    --*) echo "Unknown flag: ${a}" >&2; exit 2 ;;
    *) POS+=("${a}") ;;
  esac
done
DS="${POS[0]:-medium}"
RATE="${POS[1]:-1}"

if [[ "${DS}" != "medium" ]]; then
  echo "Only 'medium' is wired up for RRR (needs the matching sensors.bag)." >&2
  exit 1
fi

DATA_ROOT="${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}"

NODE_PID=""
PLAYER_PID=""
STOPPED_PLAYER_AFTER_NODE_DIED=0
cleanup() {
  [[ -n "${PLAYER_PID}" ]] && kill -INT "${PLAYER_PID}" 2>/dev/null || true
  if [[ -n "${NODE_PID}" ]] && kill -0 "${NODE_PID}" 2>/dev/null; then
    kill -INT "${NODE_PID}" 2>/dev/null || true
    for _ in $(seq 1 20); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.5; done
    kill -9 "${NODE_PID}" 2>/dev/null || true
  fi
  [[ -n "${PLAYER_PID}" ]] && kill -9 "${PLAYER_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

process_alive_non_zombie() {
  local pid="$1"
  [[ -n "${pid}" && -r "/proc/${pid}/stat" ]] || return 1
  local state
  state="$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null || true)"
  [[ -n "${state}" && "${state}" != "Z" ]]
}

solution_epochs() {
  grep -c GPGGA "${SOLUTION}" 2>/dev/null || echo 0
}

validate_solution_complete() {
  local epochs
  epochs="$(solution_epochs)"
  if [[ "${epochs}" -le 0 ]]; then
    echo "[rrr] ERROR: no GPGGA epochs were written to ${SOLUTION}" >&2
    return 1
  fi

  if [[ "${MODE}" != "canonical" ]]; then
    return 0
  fi

  python3 - "${SOLUTION}" "${MANIFEST}" <<'PY'
import json
import math
import sys
from pathlib import Path

solution = Path(sys.argv[1])
manifest = Path(sys.argv[2])

try:
    m = json.loads(manifest.read_text())
    rover_lo, rover_hi = m["rover_drive_utc"]
except Exception as exc:
    print(f"[rrr] ERROR: unable to read canonical manifest {manifest}: {exc}", file=sys.stderr)
    sys.exit(2)

gga_times = []
for line in solution.read_text(errors="ignore").splitlines():
    if "GPGGA" not in line:
        continue
    parts = line.split(",")
    if len(parts) < 2 or len(parts[1]) < 6:
        continue
    try:
        hh = int(parts[1][0:2])
        mm = int(parts[1][2:4])
        ss = float(parts[1][4:])
    except ValueError:
        continue
    gga_times.append(hh * 3600.0 + mm * 60.0 + ss)

if not gga_times:
    print(f"[rrr] ERROR: {solution} contains no parseable GPGGA timestamps.", file=sys.stderr)
    sys.exit(3)

def sod(unix_time: float) -> float:
    return math.fmod(unix_time, 86400.0)

def unwrap_near(value: float, anchor: float) -> float:
    while value - anchor > 43200.0:
        value -= 86400.0
    while anchor - value > 43200.0:
        value += 86400.0
    return value

drive_start = sod(rover_lo)
drive_end = unwrap_near(sod(rover_hi), drive_start)
sol_start = unwrap_near(gga_times[0], drive_start)
sol_end = unwrap_near(gga_times[-1], sol_start)
if sol_end < sol_start:
    sol_end += 86400.0

drive_span = max(0.0, drive_end - drive_start)
sol_span = max(0.0, sol_end - sol_start)
end_gap = drive_end - sol_end

# Allow visual/GNSS initialization to delay the first solution, but do not allow
# a short early node failure to masquerade as a completed canonical run.
min_span_ratio = 0.85
max_end_gap_s = 75.0
ok = drive_span > 0.0 and sol_span >= drive_span * min_span_ratio and end_gap <= max_end_gap_s

print(
    f"[rrr] Canonical coverage: solution_span={sol_span:.1f}s, "
    f"drive_span={drive_span:.1f}s, end_gap={end_gap:.1f}s, "
    f"epochs={len(gga_times)}"
)
if not ok:
    print(
        "[rrr] ERROR: canonical solution is incomplete; refusing to treat "
        "node failure as a completed run.",
        file=sys.stderr,
    )
    sys.exit(4)
PY
}

set +u  # ROS setup scripts reference unbound vars under `set -u`
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

# --- Resolve mode-specific config, bag, and output directory ---
if [[ "${MODE}" == "canonical" ]]; then
  CFG_SRC="${WS}/src/gici_ros2/config/ros_urbannav_rrr_upstream_equivalent.yaml"
  OUT_DIR="${REPO}/output/ros2_urbannav_rrr_canonical/${DS}"
  BAG_OUT="${OUT_DIR}/canonical_bag"
  MANIFEST="${BAG_OUT}/manifest.json"

  ROOT="${URBANNAV_MEDIUM_ROOT:-${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1}"
  ROVER="${URBANNAV_MEDIUM_ROVER:-${ROOT}/gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs}"
  BASE="${URBANNAV_MEDIUM_BASE:-${ROOT}/gnss/base/hkkt137g.rnx}"
  EPH="${URBANNAV_MEDIUM_EPH:-${ROOT}/gnss/base/brdc1370.rnx}"
  DCB="${URBANNAV_MEDIUM_DCB:-${REPO}/research/dcb/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX}"
  SENSORS="${URBANNAV_MEDIUM_SENSORS:-${ROOT}/ros/UrbanNav-HK_TST-20210517_sensors.bag}"

  mkdir -p "${OUT_DIR}/log"

  # Rebuild the canonical bag only if the manifest is missing or any recorded
  # input hash no longer matches the current file (not merely if the dir exists).
  need_build=1
  if [[ -f "${MANIFEST}" ]]; then
    if python3 - "${MANIFEST}" "${REPO}/scripts/ros2/finalize_rinex_bag.py" "${ROVER}" "${BASE}" "${EPH}" "${DCB}" "${SENSORS}" <<'PY'
import sys, json, hashlib
manifest, converter, *files = sys.argv[1:]
keys = ["rover", "base", "eph", "dcb", "sensors"]
try:
    m = json.load(open(manifest))
except Exception:
    sys.exit(2)
def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for c in iter(lambda: f.read(1 << 20), b""):
            h.update(c)
    return h.hexdigest()
if m.get("converter_sha256") != sha(converter):
    sys.exit(5)
for k, p in zip(keys, files):
    rec = m.get("inputs", {}).get(k, {}).get("sha256")
    try:
        if rec != sha(p):
            sys.exit(3)
    except FileNotFoundError:
        sys.exit(4)
sys.exit(0)
PY
    then
      need_build=0
      echo "[rrr] Canonical bag manifest matches current inputs; reusing ${BAG_OUT}"
    else
      echo "[rrr] Canonical bag stale or missing; rebuilding."
    fi
  fi
  if [[ "${need_build}" -ne 0 ]]; then
    "${REPO}/scripts/ros2/urbannav_rinex_to_ros2.sh" "${DS}"
  fi
else
  CFG_SRC="${WS}/src/gici_ros2/config/ros_urbannav_rrr_ros2_adapted.yaml"
  OUT_DIR="${REPO}/output/ros2_urbannav_rrr/${DS}"
  BAG_OUT="${OUT_DIR}/rrr_ros2"

  GNSS_DIR="${URBANNAV_MEDIUM_GNSS_DIR:-${DATA_ROOT}/OneDrive_1_7-11-2026/urbannav/medium}"
  SENSORS="${URBANNAV_MEDIUM_SENSORS:-${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1/ros/UrbanNav-HK_TST-20210517_sensors.bag}"

  mkdir -p "${OUT_DIR}/log"
  if [[ ! -d "${BAG_OUT}" ]]; then
    echo "[rrr] Building merged GNSS+IMU+camera ROS 2 bag at ${BAG_OUT}"
    for f in "${SENSORS}"; do
      [[ -e "${f}" ]] || { echo "ERROR: missing ${f}; set URBANNAV_MEDIUM_SENSORS." >&2; exit 1; }
    done
    python3 "${REPO}/scripts/ros2/urbannav_rrr_to_ros2.py" \
      --gnss-dir "${GNSS_DIR}" --sensors "${SENSORS}" --out "${BAG_OUT}"
  else
    echo "[rrr] Reusing existing ROS 2 bag at ${BAG_OUT}"
  fi
fi

export ROS_LOG_DIR="${OUT_DIR}/log/ros2"
mkdir -p "${ROS_LOG_DIR}"

# Resolve config placeholder (read from source tree -> no rebuild needed)
CFG="${OUT_DIR}/$(basename "${CFG_SRC}")"
sed "s#OUTPUT_DIR#${OUT_DIR}#g" "${CFG_SRC}" > "${CFG}"
SOLUTION="${OUT_DIR}/solution.txt"
rm -f "${SOLUTION}"
echo "[rrr] Mode: ${MODE}"
echo "[rrr] Config: ${CFG}"
echo "[rrr] Bag: ${BAG_OUT}"
echo "[rrr] Trajectory -> ${SOLUTION}"

# Launch node (run the binary directly so SIGINT reaches it)
NODE_EXE="${WS}/install/gici_ros2/lib/gici_ros2/gici_ros2_main"
echo "[rrr] Starting gici_ros2_main ..."
"${NODE_EXE}" "${CFG}" > "${OUT_DIR}/node.log" 2>&1 &
NODE_PID=$!
sleep 3
if ! kill -0 "${NODE_PID}" 2>/dev/null; then
  wait "${NODE_PID}" 2>/dev/null || true
  echo "[rrr] gici_ros2_main exited before playback; see ${OUT_DIR}/node.log" >&2
  exit 1
fi

echo "[rrr] Playing bag at rate ${RATE} ..."
ros2 bag play "${BAG_OUT}" --rate "${RATE}" --read-ahead-queue-size 10000 \
  > "${OUT_DIR}/log/play.log" 2>&1 &
PLAYER_PID=$!
PLAYER_STATUS=0
NODE_DIED=0
while process_alive_non_zombie "${PLAYER_PID}"; do
  if ! process_alive_non_zombie "${NODE_PID}"; then
    wait "${NODE_PID}" 2>/dev/null || true
    NODE_DIED=1
    STOPPED_PLAYER_AFTER_NODE_DIED=1
    kill -INT "${PLAYER_PID}" 2>/dev/null || true
    break
  fi
  sleep 1
done
wait "${PLAYER_PID}" || PLAYER_STATUS=$?
PLAYER_PID=""

if [[ "${NODE_DIED}" -eq 0 ]] && ! process_alive_non_zombie "${NODE_PID}"; then
  wait "${NODE_PID}" 2>/dev/null || true
  NODE_DIED=1
fi

# Drain by signal: wait until the estimator stops emitting new epochs (solution
# file stops growing) rather than a fixed sleep, then stop the node cleanly.
if [[ "${NODE_DIED}" -eq 0 ]]; then
  echo "[rrr] Draining estimator (waiting for solution to stabilize) ..."
  prev=-1; same=0
  for _ in $(seq 1 180); do
    sleep 1
    c="$(grep -c GPGGA "${SOLUTION}" 2>/dev/null || echo 0)"
    if [[ "${c}" -eq "${prev}" ]]; then
      same=$((same + 1)); [[ "${same}" -ge 5 ]] && break
    else
      same=0
    fi
    prev="${c}"
    kill -0 "${NODE_PID}" 2>/dev/null || break
  done
  kill -INT "${NODE_PID}" 2>/dev/null || true
  for _ in $(seq 1 60); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.5; done
  if kill -0 "${NODE_PID}" 2>/dev/null; then
    echo "[rrr] gici_ros2_main did not stop after SIGINT; forcing shutdown" >&2
    kill -9 "${NODE_PID}" 2>/dev/null || true
  fi
fi
NODE_PID=""

echo "[rrr] Done. Solution epochs (GPGGA):"
solution_epochs
echo "[rrr] Node log tail:"
tail -n 20 "${OUT_DIR}/node.log" 2>/dev/null || true
if [[ "${PLAYER_STATUS}" -ne 0 && "${STOPPED_PLAYER_AFTER_NODE_DIED}" -eq 0 ]]; then
  echo "[rrr] ros2 bag play failed with status ${PLAYER_STATUS}" >&2
  exit "${PLAYER_STATUS}"
fi
if [[ "${NODE_DIED}" -ne 0 ]]; then
  EPOCHS="$(solution_epochs)"
  if grep -q "Check failed: seq.size() > 1" "${OUT_DIR}/node.log" 2>/dev/null; then
    if validate_solution_complete; then
      echo "[rrr] gici_ros2_main hit upstream teardown CHECK after ${EPOCHS} epochs; treating node failure as invalid baseline run." >&2
    else
      echo "[rrr] gici_ros2_main hit upstream teardown CHECK and the solution is incomplete." >&2
    fi
  else
    echo "[rrr] gici_ros2_main exited during playback; see ${OUT_DIR}/node.log" >&2
  fi
  exit 1
else
  validate_solution_complete
fi
