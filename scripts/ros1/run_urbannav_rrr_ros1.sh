#!/usr/bin/env bash
# UrbanNav FULL RRR on ROS 1 — native dataset bags + gici_ros_main + rosbag play.
#
# Usage:
#   scripts/ros1/run_urbannav_rrr_ros1.sh [table5|native] [medium|deep] [rate]
#
# Modes:
#   table5  Paper Table V: HKKT+DCB RINEX -> canonical ROS1 bag (see urbannav_rinex_to_ros1.sh)
#   native  Author HKSC GNSS bags under medium/ or deep/ (default)
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
WS="${REPO}/ros_wrapper"
MODE="${URBANNAV_ROS1_MODE:-table5}"
POS=()
for a in "$@"; do
  case "${a}" in
    table5|native) MODE="${a}" ;;
    *) POS+=("${a}") ;;
  esac
done
DS="${POS[0]:-medium}"
RATE="${POS[1]:-1}"

if [[ "${DS}" != "medium" && "${DS}" != "deep" ]]; then
  echo "[ros1] Unknown dataset '${DS}'. Use 'medium' or 'deep'." >&2
  exit 1
fi

case "${DS}" in
  medium)
    SCENE="UrbanNav-HK-Medium-Urban-1"
    GNSS_DIR="${URBANNAV_MEDIUM_GNSS_DIR:-${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}/${SCENE}/medium}"
    SENSORS="${URBANNAV_MEDIUM_SENSORS:-${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}/${SCENE}/ros/UrbanNav-HK_TST-20210517_sensors.bag}"
    PLAY_DURATION_DEFAULT=800
    RRR_WARMUP_DEFAULT=90
    STALL_WARMUP_DEFAULT=90
    ;;
  deep)
    SCENE="UrbanNav-HK-Deep-Urban-1"
    GNSS_DIR="${URBANNAV_DEEP_GNSS_DIR:-${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}/${SCENE}/deep}"
    SENSORS="${URBANNAV_DEEP_SENSORS:-${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}/${SCENE}/ros/UrbanNav-HK_Whampoa-20210521_sensors.bag}"
    PLAY_DURATION_DEFAULT=2400
    RRR_WARMUP_DEFAULT=300
    STALL_WARMUP_DEFAULT=180
    STALL_SECONDS_DEFAULT=600
    ;;
esac

RRR_WARMUP_DEFAULT="${RRR_WARMUP_DEFAULT:-90}"
STALL_WARMUP_DEFAULT="${STALL_WARMUP_DEFAULT:-90}"
STALL_SECONDS_DEFAULT="${STALL_SECONDS_DEFAULT:-120}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ros1] ERROR: ROS Noetic required. See scripts/ros1/build_ros1_wrapper.sh" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${ROS_SETUP}"
source "${WS}/devel/setup.bash"

GICI_ROS_MAIN="${WS}/devel/lib/gici_ros/gici_ros_main"
if [[ ! -x "${GICI_ROS_MAIN}" ]]; then
  echo "[ros1] ERROR: ${GICI_ROS_MAIN} missing. Run ./scripts/ros1/build_ros1_wrapper.sh" >&2
  exit 1
fi

if [[ "${MODE}" == "table5" ]]; then
  CANON_BAG="${REPO}/output/ros1_urbannav_rrr_table5/${DS}/canonical.bag"
  if [[ ! -f "${CANON_BAG}" || "${URBANNAV_FORCE_REBUILD:-0}" == "1" ]]; then
    echo "[ros1] Building Table-V canonical bag (HKKT+DCB RINEX) ..."
    "${REPO}/scripts/ros1/urbannav_rinex_to_ros1.sh" "${DS}"
  fi
  OUT_DIR="${REPO}/output/ros1_urbannav_rrr_table5/${DS}/run"
  mkdir -p "${OUT_DIR}/log"
  rm -f "${OUT_DIR}/node.log" "${OUT_DIR}/play.log" "${OUT_DIR}/solution.txt"
  CFG_SRC="${WS}/src/gici/option/ros_urbannav_rrr_table5.yaml"
  CFG="${OUT_DIR}/ros_urbannav_rrr_table5.yaml"
  sed "s|OUTPUT_DIR|${OUT_DIR}|g" "${CFG_SRC}" > "${CFG}"
  SOLUTION="${OUT_DIR}/solution.txt"
  : > "${SOLUTION}"
  BAGS=("${CANON_BAG}")
  TOPICS=(
    /gici/gnss_rover/observations
    /gici/gnss_reference/observations
    /gici/gnss_reference/antenna_position
    /gici/gps_ephemeris/ephemerides
    /gici/gps_ephemeris/ionosphere_parameter
    /gici/glonass_ephemeris/ephemerides
    /gici/galileo_ephemeris/ephemerides
    /gici/bds_ephemeris/ephemerides
    /gici/gnss_dcb/code_bias
    /imu/data
    /zed2/camera/left/image_raw
  )
  case "${DS}" in
    medium) PLAY_DURATION_DEFAULT=800; RRR_WARMUP_DEFAULT=300; STALL_WARMUP_DEFAULT=180; STALL_SECONDS_DEFAULT=300 ;;
    deep)   PLAY_DURATION_DEFAULT=2400; RRR_WARMUP_DEFAULT=300; STALL_WARMUP_DEFAULT=180; STALL_SECONDS_DEFAULT=600 ;;
  esac
else
  for f in \
    "${GNSS_DIR}/gnss_rover.bag" \
    "${GNSS_DIR}/gnss_reference.bag" \
    "${GNSS_DIR}/gnss_ephemeris_G.bag" \
    "${GNSS_DIR}/gnss_ephemeris_R.bag" \
    "${GNSS_DIR}/gnss_ephemeris_E.bag" \
    "${GNSS_DIR}/gnss_ephemeris_C.bag" \
    "${SENSORS}"; do
    if [[ ! -f "${f}" ]]; then
      echo "[ros1] ERROR: missing ${f}" >&2
      exit 1
    fi
  done
  OUT_DIR="${REPO}/output/ros1_urbannav_rrr/${DS}"
  mkdir -p "${OUT_DIR}/log"
  CFG_SRC="${WS}/src/gici/option/ros_urbannav_rrr.yaml"
  CFG="${OUT_DIR}/ros_urbannav_rrr.yaml"
  sed "s|OUTPUT_DIR|${OUT_DIR}|g" "${CFG_SRC}" > "${CFG}"
  SOLUTION="${OUT_DIR}/solution.txt"
  : > "${SOLUTION}"
  TOPICS=(
    /gici/gnss_rover/observations
    /gici/gnss_reference/observations
    /gici/gnss_reference/antenna_position
    /gici/gps_ephemeris/ephemerides
    /gici/gps_ephemeris/ionosphere_parameter
    /gici/glonass_ephemeris/ephemerides
    /gici/galileo_ephemeris/ephemerides
    /gici/bds_ephemeris/ephemerides
    /imu/data
    /zed2/camera/left/image_raw
  )
  BAGS=(
    "${GNSS_DIR}/gnss_rover.bag"
    "${GNSS_DIR}/gnss_reference.bag"
    "${GNSS_DIR}/gnss_ephemeris_G.bag"
    "${GNSS_DIR}/gnss_ephemeris_R.bag"
    "${GNSS_DIR}/gnss_ephemeris_E.bag"
    "${GNSS_DIR}/gnss_ephemeris_C.bag"
    "${SENSORS}"
  )
fi

# Kill stale players/nodes on the same topics.
pkill -f "gici_ros_main.*ros_urbannav_rrr" 2>/dev/null || true
pkill -f "rosbag play.*UrbanNav-HK" 2>/dev/null || true
sleep 1

ROSCORE_PID=""
NODE_PID=""
PLAYER_PID=""
STOPPED_PLAYER_AFTER_NODE_DIED=0
NODE_DIED=0
PLAYBACK_START_TS=0

cleanup() {
  [[ -n "${PLAYER_PID}" ]] && kill -INT "${PLAYER_PID}" 2>/dev/null || true
  if [[ -n "${NODE_PID}" ]] && kill -0 "${NODE_PID}" 2>/dev/null; then
    kill -INT "${NODE_PID}" 2>/dev/null || true
    for _ in $(seq 1 30); do kill -0 "${NODE_PID}" 2>/dev/null || break; sleep 0.5; done
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
  local n=0
  if [[ -s "${SOLUTION}" ]]; then
    n="$(grep -c GPGGA "${SOLUTION}" 2>/dev/null)" || n=0
  fi
  echo "${n}"
}

log_line_count() {
  local n=0
  if [[ -f "${OUT_DIR}/node.log" ]]; then
    n="$(wc -l < "${OUT_DIR}/node.log" | tr -d ' ')"
  fi
  echo "${n}"
}

rrr_update_count() {
  local n=0
  if [[ -f "${OUT_DIR}/node.log" ]]; then
    n="$(grep -c 'RTK/IMU/Camera RRR:' "${OUT_DIR}/node.log" 2>/dev/null)" || n=0
  fi
  echo "${n}"
}

report_progress() {
  local phase="$1"
  local elapsed="$2"
  local epochs log_lines rrr
  epochs="$(solution_epochs)"
  log_lines="$(log_line_count)"
  rrr="$(rrr_update_count)"
  echo "[ros1] ${phase}: elapsed=${elapsed}s bag≈$(( elapsed * RATE ))s epochs=${epochs} rrr=${rrr} log_lines=${log_lines}"
}

fail_stalled() {
  echo "[ros1] ERROR: run appears stalled (no solution/log progress). Last node.log:" >&2
  tail -30 "${OUT_DIR}/node.log" >&2 || true
  exit 1
}

fail_node_died() {
  echo "[ros1] ERROR: gici_ros_main exited during playback; see ${OUT_DIR}/node.log" >&2
  tail -30 "${OUT_DIR}/node.log" >&2 || true
  exit 1
}

fail_node_crash() {
  echo "[ros1] ERROR: crash/fatal detected in node.log" >&2
  grep -E 'Check failed:|Segmentation fault|FATAL|core dumped|Aborted at|terminate called' \
    "${OUT_DIR}/node.log" | tail -5 >&2 || true
  tail -20 "${OUT_DIR}/node.log" >&2 || true
  exit 1
}

fail_no_fusion() {
  local elapsed="$1"
  echo "[ros1] ERROR: no RRR fusion after ${elapsed}s (epochs=0, rrr=0). Aborting early." >&2
  echo "[ros1] Hint: check GNSS topics, ephemeris, and node.log tail below." >&2
  tail -30 "${OUT_DIR}/node.log" >&2 || true
  exit 1
}

check_node_crash() {
  [[ -f "${OUT_DIR}/node.log" ]] || return 0
  if grep -qE 'Check failed:|Segmentation fault|FATAL|core dumped|Aborted at|terminate called' \
      "${OUT_DIR}/node.log" 2>/dev/null; then
    fail_node_crash
  fi
}

if ! rostopic list &>/dev/null; then
  echo "[ros1] Starting roscore ..."
  roscore > "${OUT_DIR}/roscore.log" 2>&1 &
  ROSCORE_PID=$!
  for _ in $(seq 1 30); do rostopic list &>/dev/null && break; sleep 0.5; done
fi

echo "[ros1] Mode: ${MODE}"
echo "[ros1] Config: ${CFG}"
echo "[ros1] Output: ${SOLUTION}"
if [[ "${MODE}" == "table5" ]]; then
  echo "[ros1] Canonical bag: ${CANON_BAG} (HKKT+DCB, Table V inputs)"
else
  echo "[ros1] Dataset: ${DS}"
  echo "[ros1] GNSS bags: ${GNSS_DIR} (HKSC — see gnss/info.txt)"
  echo "[ros1] Sensors: ${SENSORS}"
fi

"${GICI_ROS_MAIN}" "${CFG}" > "${OUT_DIR}/node.log" 2>&1 &
NODE_PID=$!
sleep 3

if ! kill -0 "${NODE_PID}" 2>/dev/null; then
  echo "[ros1] ERROR: gici_ros_main exited early; see ${OUT_DIR}/node.log" >&2
  tail -20 "${OUT_DIR}/node.log" >&2 || true
  exit 1
fi

# Ephemeris bags span ~24h; combined rosbag play would otherwise run for a full day.
# Limit to the rover observation window (~784s) plus a small margin.
PLAY_DURATION="${URBANNAV_PLAY_DURATION:-${PLAY_DURATION_DEFAULT}}"

TOPIC_ARGS=()
for t in "${TOPICS[@]}"; do TOPIC_ARGS+=(--topics "${t}"); done

WATCH_INTERVAL="${URBANNAV_WATCH_INTERVAL:-10}"
STALL_WARMUP="${URBANNAV_STALL_WARMUP:-${STALL_WARMUP_DEFAULT:-90}}"
STALL_SECONDS="${URBANNAV_STALL_SECONDS:-${STALL_SECONDS_DEFAULT:-120}}"
PROGRESS_EVERY="${URBANNAV_PROGRESS_EVERY:-30}"
RRR_WARMUP="${URBANNAV_RRR_WARMUP:-${RRR_WARMUP_DEFAULT:-90}}"

echo "[ros1] Playing ${#BAGS[@]} bags at rate ${RATE} (duration ${PLAY_DURATION}s bag time) ..."
echo "[ros1] Watchdog: every ${WATCH_INTERVAL}s | progress ${PROGRESS_EVERY}s | stall ${STALL_SECONDS}s after warmup ${STALL_WARMUP}s | abort if no RRR by ${RRR_WARMUP}s"
rosbag play "${BAGS[@]}" -r "${RATE}" --duration "${PLAY_DURATION}" -q "${TOPIC_ARGS[@]}" \
  > "${OUT_DIR}/play.log" 2>&1 &
PLAYER_PID=$!
PLAYER_STATUS=0
PLAYBACK_START_TS=$(date +%s)
prev_epochs=-1
prev_rrr=-1
prev_log_lines=-1
stall_s=0
next_progress=$((PLAYBACK_START_TS + PROGRESS_EVERY))

while process_alive_non_zombie "${PLAYER_PID}"; do
  if ! process_alive_non_zombie "${NODE_PID}"; then
    wait "${NODE_PID}" 2>/dev/null || true
    NODE_DIED=1
    STOPPED_PLAYER_AFTER_NODE_DIED=1
    kill -INT "${PLAYER_PID}" 2>/dev/null || true
    break
  fi

  check_node_crash

  now=$(date +%s)
  elapsed=$((now - PLAYBACK_START_TS))
  epochs="$(solution_epochs)"
  log_lines="$(log_line_count)"
  rrr="$(rrr_update_count)"

  if [[ "${now}" -ge "${next_progress}" ]]; then
    report_progress "playback" "${elapsed}"
    next_progress=$((now + PROGRESS_EVERY))
  fi

  if [[ "${elapsed}" -ge "${RRR_WARMUP}" && "${epochs}" -eq 0 && "${rrr}" -eq 0 ]]; then
    fail_no_fusion "${elapsed}"
  fi

  if [[ "${elapsed}" -ge "${STALL_WARMUP}" ]]; then
    if [[ "${epochs}" -eq "${prev_epochs}" && "${rrr}" -eq "${prev_rrr}" && "${log_lines}" -eq "${prev_log_lines}" ]]; then
      stall_s=$((stall_s + WATCH_INTERVAL))
      if [[ "${stall_s}" -ge "${STALL_SECONDS}" ]]; then
        fail_stalled
      fi
    else
      stall_s=0
    fi
  fi
  prev_epochs="${epochs}"
  prev_rrr="${rrr}"
  prev_log_lines="${log_lines}"

  sleep "${WATCH_INTERVAL}"
done

wait "${PLAYER_PID}" || PLAYER_STATUS=$?
PLAYER_PID=""

if [[ "${NODE_DIED}" -eq 0 ]] && ! process_alive_non_zombie "${NODE_PID}"; then
  wait "${NODE_PID}" 2>/dev/null || true
  NODE_DIED=1
fi

if [[ "${NODE_DIED}" -ne 0 ]]; then
  fail_node_died
fi

echo "[ros1] Draining estimator (waiting for solution to stabilize) ..."
prev=-1
same=0
for _ in $(seq 1 180); do
  sleep 1
  c="$(solution_epochs)"
  if [[ "${c}" -eq "${prev}" ]]; then
    same=$((same + 1))
    [[ "${same}" -ge 5 ]] && break
  else
    same=0
  fi
  prev="${c}"
  process_alive_non_zombie "${NODE_PID}" || break
done

kill -INT "${NODE_PID}" 2>/dev/null || true
for _ in $(seq 1 60); do process_alive_non_zombie "${NODE_PID}" || break; sleep 0.5; done
if process_alive_non_zombie "${NODE_PID}"; then
  echo "[ros1] gici_ros_main did not stop after SIGINT; forcing shutdown" >&2
  kill -9 "${NODE_PID}" 2>/dev/null || true
fi
NODE_PID=""

EPOCHS="$(solution_epochs)"
WALL_S=$(( $(date +%s) - PLAYBACK_START_TS ))
echo "[ros1] Done. GPGGA epochs: ${EPOCHS} (wall_s=${WALL_S})"
echo "[ros1] Node log: ${OUT_DIR}/node.log"
echo "[ros1] Solution: ${SOLUTION}"

if [[ "${EPOCHS}" -le 0 ]]; then
  echo "[ros1] ERROR: no solution written" >&2
  tail -30 "${OUT_DIR}/node.log" >&2 || true
  exit 1
fi

if [[ "${PLAYER_STATUS}" -ne 0 && "${STOPPED_PLAYER_AFTER_NODE_DIED}" -eq 0 ]]; then
  echo "[ros1] ERROR: rosbag play exit=${PLAYER_STATUS} (see ${OUT_DIR}/play.log)" >&2
  exit "${PLAYER_STATUS}"
fi
