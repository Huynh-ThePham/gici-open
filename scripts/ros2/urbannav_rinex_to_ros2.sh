#!/usr/bin/env bash
# Build the CANONICAL UrbanNav RRR ROS 2 bag whose GNSS is byte/numerically
# identical to the file-mode RRR baseline (same rover .obs, HKKT base .rnx,
# brdc eph .rnx and DCB .BSX), by REPUBLISHING those RINEX/BSX files through the
# core's own formators and recording the ROS topics.
#
# Pipeline:
#   1. Substitute canonical RINEX/BSX paths into urbannav_rinex_republish.yaml.
#   2. Run gici_ros2_main on that config: type:file -> gnss-rinex/dcb-file
#      formators -> ROS publishers on /gici/*. Outputs use transient_local QoS,
#      so a late-joining recorder still receives the full retained history.
#   3. `ros2 bag record` the /gici/* topics.
#   4. finalize_rinex_bag.py reclocks GNSS to internal UTC, bursts eph/ant/DCB,
#      merges IMU+camera from the sensors bag, and writes manifest.json.
#
# Usage:
#   scripts/ros2/urbannav_rinex_to_ros2.sh [medium]
#
# Dataset paths come from environment variables (no personal-machine hardcoding
# required); the defaults below match the standard UrbanNav download layout.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros2_wrapper"
DS="${1:-medium}"

if [[ "${DS}" != "medium" ]]; then
  echo "Only 'medium' is wired up for the canonical RINEX pipeline." >&2
  exit 1
fi

DATA_ROOT="${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}"
ROOT="${URBANNAV_MEDIUM_ROOT:-${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1}"
ROVER="${URBANNAV_MEDIUM_ROVER:-${ROOT}/gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs}"
BASE="${URBANNAV_MEDIUM_BASE:-${ROOT}/gnss/base/hkkt137g.rnx}"
EPH="${URBANNAV_MEDIUM_EPH:-${ROOT}/gnss/base/brdc1370.rnx}"
DCB="${URBANNAV_MEDIUM_DCB:-${REPO}/research/dcb/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX}"
SENSORS="${URBANNAV_MEDIUM_SENSORS:-${ROOT}/ros/UrbanNav-HK_TST-20210517_sensors.bag}"

for f in "${ROVER}" "${BASE}" "${EPH}" "${DCB}" "${SENSORS}"; do
  if [[ ! -e "${f}" ]]; then
    echo "ERROR: missing canonical input: ${f}" >&2
    echo "Set the matching URBANNAV_MEDIUM_* environment variable." >&2
    exit 1
  fi
done

OUT_DIR="${REPO}/output/ros2_urbannav_rrr_canonical/${DS}"
REPUB_BAG="${OUT_DIR}/republish_bag"
CANON_BAG="${OUT_DIR}/canonical_bag"
mkdir -p "${OUT_DIR}/log"
export ROS_LOG_DIR="${OUT_DIR}/log/ros2"
mkdir -p "${ROS_LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

# Resolve the republish config placeholders (read from source tree; no rebuild).
CFG_SRC="${WS}/src/gici_ros2/config/urbannav_rinex_republish.yaml"
CFG="${OUT_DIR}/urbannav_rinex_republish.yaml"
sed -e "s#<ROVER_OBS>#${ROVER}#g" \
    -e "s#<REF_OBS>#${BASE}#g" \
    -e "s#<EPH_NAV>#${EPH}#g" \
    -e "s#<DCB_FILE>#${DCB}#g" \
    -e "s#<OUTPUT_DIR>#${OUT_DIR}#g" \
    "${CFG_SRC}" > "${CFG}"
echo "[canon] Republish config: ${CFG}"

NODE_EXE="${WS}/install/gici_ros2/lib/gici_ros2/gici_ros2_main"
TOPICS=(
  /gici/gnss_rover/observations
  /gici/gnss_reference/observations
  /gici/gnss_reference/antenna_position
  /gici/gnss_ephemeris/ephemerides
  /gici/gnss_ephemeris/ionosphere_parameter
  /gici/gnss_dcb/code_bias
)

NODE_PID=""
REC_PID=""
cleanup() {
  [[ -n "${REC_PID}" ]] && kill -INT "${REC_PID}" 2>/dev/null || true
  [[ -n "${NODE_PID}" ]] && kill -INT "${NODE_PID}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    { [[ -n "${REC_PID}" ]] && kill -0 "${REC_PID}" 2>/dev/null; } || \
    { [[ -n "${NODE_PID}" ]] && kill -0 "${NODE_PID}" 2>/dev/null; } || break
    sleep 0.3
  done
  [[ -n "${REC_PID}" ]] && kill -9 "${REC_PID}" 2>/dev/null || true
  [[ -n "${NODE_PID}" ]] && kill -9 "${NODE_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rm -rf "${REPUB_BAG}"

# 1-2. Start the republish node; it advertises and retains all output via
#      transient_local, so recording can safely start afterwards.
echo "[canon] Starting republish node ..."
"${NODE_EXE}" "${CFG}" > "${OUT_DIR}/log/republish_node.log" 2>&1 &
NODE_PID=$!

# Wait for all six topics to be advertised.
echo "[canon] Waiting for topics ..."
for _ in $(seq 1 60); do
  ok=1
  avail="$(ros2 topic list 2>/dev/null || true)"
  for t in "${TOPICS[@]}"; do
    grep -qx "${t}" <<<"${avail}" || ok=0
  done
  [[ "${ok}" -eq 1 ]] && break
  kill -0 "${NODE_PID}" 2>/dev/null || { echo "node exited early; see log" >&2; exit 1; }
  sleep 1
done

# Give the file readers time to decode + publish + retain everything.
echo "[canon] Letting readers publish (retained via transient_local) ..."
sleep 15

# 3. Record the retained history (transient_local override so record pulls the
#    full retained history from the already-published node).
QOS_OVERRIDE="${WS}/src/gici_ros2/config/republish_record_qos.yaml"
echo "[canon] Recording -> ${REPUB_BAG}"
ros2 bag record -o "${REPUB_BAG}" \
  --qos-profile-overrides-path "${QOS_OVERRIDE}" \
  "${TOPICS[@]}" \
  > "${OUT_DIR}/log/record.log" 2>&1 &
REC_PID=$!

# Wait until the recorded bag stops growing (all retained history drained).
DB=""
last_size=-1
stable=0
for _ in $(seq 1 120); do
  sleep 2
  DB="$(ls "${REPUB_BAG}"/*.db3 2>/dev/null | head -n1 || true)"
  [[ -z "${DB}" ]] && continue
  size="$(stat -c%s "${DB}" 2>/dev/null || echo 0)"
  if [[ "${size}" -eq "${last_size}" && "${size}" -gt 0 ]]; then
    stable=$((stable + 1))
    [[ "${stable}" -ge 3 ]] && break
  else
    stable=0
  fi
  last_size="${size}"
done

echo "[canon] Stopping recorder and node ..."
kill -INT "${REC_PID}" 2>/dev/null || true
for _ in $(seq 1 40); do kill -0 "${REC_PID}" 2>/dev/null || break; sleep 0.25; done
REC_PID=""
kill -INT "${NODE_PID}" 2>/dev/null || true
for _ in $(seq 1 40); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.25; done
NODE_PID=""

# 4. Finalize: reclock + merge sensors + manifest.
echo "[canon] Finalizing canonical bag -> ${CANON_BAG}"
rm -rf "${CANON_BAG}"
python3 "${REPO}/scripts/ros2/finalize_rinex_bag.py" \
  --republish-bag "${REPUB_BAG}" \
  --sensors "${SENSORS}" \
  --out "${CANON_BAG}" \
  --rover "${ROVER}" --base "${BASE}" --eph "${EPH}" --dcb "${DCB}"

echo "[canon] Done. Canonical bag: ${CANON_BAG}"
echo "[canon] Manifest: ${CANON_BAG}/manifest.json"
