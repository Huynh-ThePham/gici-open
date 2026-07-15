#!/usr/bin/env bash
# Build Table-V-equivalent ROS 1 canonical bag: HKKT+DCB RINEX republish + native sensors.
#
# Pipeline (ROS 1 only):
#   1. gici_ros_main + urbannav_rinex_republish.yaml  (file RINEX -> /gici/* topics)
#   2. rosbag record GNSS topics
#   3. finalize_rinex_bag_ros1.py  (UTC reclock + merge IMU/camera from native sensors.bag)
#
# Usage:
#   scripts/ros1/urbannav_rinex_to_ros1.sh [medium|deep]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS="${REPO}/ros_wrapper"
DS="${1:-medium}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"

if [[ "${DS}" != "medium" && "${DS}" != "deep" ]]; then
  echo "[ros1-rinex] Use 'medium' or 'deep'." >&2
  exit 1
fi

DATA_ROOT="${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}"
REPO_DCB="${REPO}/research/dcb"

case "${DS}" in
  medium)
    ROOT="${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1"
    ROVER="${ROOT}/gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs"
    BASE="${ROOT}/gnss/base/hkkt137g.rnx"
    EPH="${ROOT}/gnss/base/brdc1370.rnx"
    DCB="${REPO_DCB}/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX"
    SENSORS="${ROOT}/ros/UrbanNav-HK_TST-20210517_sensors.bag"
    EPH_DIR="${ROOT}/medium"
    WINDOW="${URBANNAV_TABLE5_WINDOW:-800}"
    ;;
  deep)
    ROOT="${DATA_ROOT}/UrbanNav-HK-Deep-Urban-1"
    ROVER="${ROOT}/gnss/UrbanNav-HK-Deep-Urban-1.ublox.f9p.splitter.obs"
    BASE="${ROOT}/gnss/base/_deprecated/hkkt141g.21o"
    EPH="${ROOT}/gnss/base/_deprecated/brdc_mn.rnx"
    DCB="${REPO_DCB}/CAS0MGXRAP_20211410000_01D_01D_DCB.BSX"
    SENSORS="${ROOT}/ros/UrbanNav-HK_Whampoa-20210521_sensors.bag"
    EPH_DIR="${ROOT}/deep"
    WINDOW="${URBANNAV_TABLE5_WINDOW:-2400}"
    ;;
esac

for f in "${ROVER}" "${BASE}" "${EPH}" "${DCB}" "${SENSORS}"; do
  [[ -f "${f}" ]] || { echo "[ros1-rinex] ERROR: missing ${f}" >&2; exit 1; }
done

OUT_DIR="${REPO}/output/ros1_urbannav_rrr_table5/${DS}"
REPUB_BAG="${OUT_DIR}/republish_gnss.bag"
CANON_BAG="${OUT_DIR}/canonical.bag"
mkdir -p "${OUT_DIR}/log"

if [[ -f "${CANON_BAG}" && "${URBANNAV_FORCE_REBUILD:-0}" != "1" ]]; then
  echo "[ros1-rinex] Reusing ${CANON_BAG}"
  exit 0
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ros1-rinex] ERROR: ROS Noetic required at ${ROS_SETUP}" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${ROS_SETUP}"
source "${WS}/devel/setup.bash"

GICI="${WS}/devel/lib/gici_ros/gici_ros_main"
[[ -x "${GICI}" ]] || { echo "[ros1-rinex] Build wrapper first: ./scripts/ros1/build_ros1_wrapper.sh" >&2; exit 1; }

CFG_SRC="${WS}/src/gici/option/urbannav_rinex_republish.yaml"
CFG="${OUT_DIR}/urbannav_rinex_republish.yaml"
sed -e "s|<ROVER_OBS>|${ROVER}|g" \
    -e "s|<REF_OBS>|${BASE}|g" \
    -e "s|<EPH_NAV>|${EPH}|g" \
    -e "s|<DCB_FILE>|${DCB}|g" \
    -e "s|<OUTPUT_DIR>|${OUT_DIR}|g" \
    "${CFG_SRC}" > "${CFG}"

TOPICS=(
  /gici/gnss_rover/observations
  /gici/gnss_reference/observations
  /gici/gnss_reference/antenna_position
  /gici/gnss_dcb/code_bias
)

ROSCORE_PID=""
NODE_PID=""
REC_PID=""
cleanup() {
  [[ -n "${REC_PID}" ]] && kill -INT "${REC_PID}" 2>/dev/null || true
  [[ -n "${NODE_PID}" ]] && kill -INT "${NODE_PID}" 2>/dev/null || true
  [[ -n "${ROSCORE_PID}" ]] && kill "${ROSCORE_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if ! rostopic list &>/dev/null; then
  roscore > "${OUT_DIR}/log/roscore.log" 2>&1 &
  ROSCORE_PID=$!
  for _ in $(seq 1 30); do rostopic list &>/dev/null && break; sleep 0.5; done
fi

rm -f "${REPUB_BAG}"
echo "[ros1-rinex] Recording GNSS republish -> ${REPUB_BAG}"
rosbag record -O "${REPUB_BAG}" "${TOPICS[@]}" > "${OUT_DIR}/log/record.log" 2>&1 &
REC_PID=$!
sleep 2

echo "[ros1-rinex] Republish config: ${CFG}"
"${GICI}" "${CFG}" > "${OUT_DIR}/log/republish_node.log" 2>&1 &
NODE_PID=$!

# Do not wait for the node to EOF the 24h HKKT ref RINEX. Stop once the recorded
# bag stops growing (same idea as scripts/ros2/urbannav_rinex_to_ros2.sh).
echo "[ros1-rinex] Waiting for republish bag to stabilize ..."
last_size=-1
stable=0
MIN_BAG_BYTES="${URBANNAV_REPUBLISH_MIN_BYTES:-5000000}"
MAX_WAIT="${URBANNAV_REPUBLISH_MAX_WAIT:-180}"
record_start_ts=$(date +%s)
for _ in $(seq 1 "${MAX_WAIT}"); do
  kill -0 "${NODE_PID}" 2>/dev/null || break
  sleep 2
  size=0
  if [[ -f "${REPUB_BAG}.active" ]]; then
    size=$(stat -c%s "${REPUB_BAG}.active" 2>/dev/null || echo 0)
  elif [[ -f "${REPUB_BAG}" ]]; then
    size=$(stat -c%s "${REPUB_BAG}" 2>/dev/null || echo 0)
  fi
  elapsed=$(( $(date +%s) - record_start_ts ))
  if [[ "${size}" -eq "${last_size}" && "${size}" -ge "${MIN_BAG_BYTES}" && "${elapsed}" -ge 30 ]]; then
    stable=$((stable + 1))
    [[ "${stable}" -ge 3 ]] && break
  else
    stable=0
  fi
  last_size="${size}"
done

echo "[ros1-rinex] Stopping recorder and republish node ..."
kill -INT "${REC_PID}" 2>/dev/null || true
for _ in $(seq 1 40); do kill -0 "${REC_PID}" 2>/dev/null || break; sleep 0.25; done
REC_PID=""
kill -INT "${NODE_PID}" 2>/dev/null || true
for _ in $(seq 1 40); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.25; done
NODE_PID=""
sleep 2

[[ -f "${REPUB_BAG}" ]] || { echo "[ros1-rinex] ERROR: no republish bag" >&2; exit 1; }

rm -f "${CANON_BAG}"
python3 "${REPO}/scripts/ros1/finalize_rinex_bag_ros1.py" \
  --republish-bag "${REPUB_BAG}" \
  --eph-dir "${EPH_DIR}" \
  --sensors "${SENSORS}" \
  --out "${CANON_BAG}" \
  --rover "${ROVER}" --base "${BASE}" --eph "${EPH}" --dcb "${DCB}" \
  --window "${WINDOW}"

echo "[ros1-rinex] Canonical Table-V ROS1 bag: ${CANON_BAG}"
