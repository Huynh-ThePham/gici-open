#!/usr/bin/env bash
# Shared helpers for GICI ROS2 real-time bag replay (stress tests + board pipeline).
# shellcheck shell=bash
set -Eeuo pipefail

count_gpgga() {
  local f="$1"
  local n=0
  if [[ -f "${f}" ]]; then
    n="$(grep -c GPGGA "${f}" 2>/dev/null)" || n=0
  fi
  echo "${n}"
}

process_alive_non_zombie() {
  local pid="$1"
  [[ -n "${pid}" && -r "/proc/${pid}/stat" ]] || return 1
  local state
  state="$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null || true)"
  [[ -n "${state}" && "${state}" != "Z" ]]
}

cleanup_ros2_gici_session() {
  local log="${1:-/dev/stderr}"
  {
    echo "[cleanup] $(date -Is) stopping stale gici / bag-play processes"
    pkill -INT -f 'gici_ros2_main' 2>/dev/null || true
    pkill -INT -f 'ros2 bag play' 2>/dev/null || true
    sleep 2
    pkill -TERM -f 'gici_ros2_main' 2>/dev/null || true
    pkill -TERM -f 'ros2 bag play' 2>/dev/null || true
    sleep 1
    pkill -KILL -f 'gici_ros2_main' 2>/dev/null || true
    pkill -KILL -f 'ros2 bag play' 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    echo "[cleanup] done"
  } >>"${log}" 2>&1
}

wait_for_gici_node_ready() {
  local pid="$1"
  local node_log="$2"
  local log="$3"
  local timeout_s="${4:-90}"
  local spinup_s="${GICI_NODE_SPINUP_S:-5}"
  local t0
  t0="$(date +%s)"
  while (( $(date +%s) - t0 < timeout_s )); do
    if ! process_alive_non_zombie "${pid}"; then
      echo "[ready] FAIL: node pid ${pid} not alive" >>"${log}"
      return 1
    fi
    if [[ -f "${node_log}" ]] && grep -qE 'Initialized [0-9]+ streamers.*Running' "${node_log}"; then
      sleep "${spinup_s}"
      echo "[ready] PASS: node initialized (log Running + ${spinup_s}s spinup)" >>"${log}"
      return 0
    fi
    if ros2 node list 2>/dev/null | grep -qE '/gici$'; then
      sleep "${spinup_s}"
      echo "[ready] PASS: /gici visible in ros2 node list (+ ${spinup_s}s spinup)" >>"${log}"
      return 0
    fi
    sleep 0.5
  done
  echo "[ready] FAIL: timeout ${timeout_s}s waiting for node ready" >>"${log}"
  return 1
}

health_check_reference_station() {
  local bag="$1"
  local node_pid="$2"
  local node_log="$3"
  local hc_log="$4"
  local line_before=0
  [[ -f "${node_log}" ]] && line_before="$(wc -l < "${node_log}")"

  {
    echo "=== reference health-check $(date -Is) ==="
    echo "bag=${bag} node_pid=${node_pid} log_lines_before=${line_before}"
  } >>"${hc_log}"

  if ! timeout 30 ros2 bag play "${bag}" \
      --topics /gici/gnss_reference/antenna_position /gici/gnss_reference/observations \
      --rate 10 --read-ahead-queue-size 1000 \
      >>"${hc_log}" 2>&1; then
    echo "[health] WARN: reference probe exited non-zero (continuing check)" >>"${hc_log}"
  fi

  sleep 2
  if ! process_alive_non_zombie "${node_pid}"; then
    echo "[health] FAIL: node died during reference probe" >>"${hc_log}"
    return 1
  fi

  local err_after=0
  if [[ -f "${node_log}" ]]; then
    err_after="$(tail -n +"$((line_before + 1))" "${node_log}" \
      | grep -c 'Unable to get antenna position of reference station' || true)"
  fi
  err_after="${err_after:-0}"
  if (( err_after > 0 )); then
    echo "[health] FAIL: ${err_after} reference-antenna errors after probe" >>"${hc_log}"
    return 1
  fi

  echo "[health] PASS: reference station accepted probe messages" >>"${hc_log}"
  return 0
}

stop_gici_node() {
  local pid="$1"
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 15); do
    process_alive_non_zombie "${pid}" || return 0
    sleep 2
  done
  kill -TERM "${pid}" 2>/dev/null || true
  sleep 2
  kill -KILL "${pid}" 2>/dev/null || true
}

drain_solution_epochs() {
  local solution="$1"
  local node_pid="$2"
  local min_epochs="${3:-1500}"
  local prev=-1
  local same=0
  while process_alive_non_zombie "${node_pid}"; do
    local c
    c="$(count_gpgga "${solution}")"
    if (( c >= min_epochs )); then
      if [[ "${c}" -eq "${prev}" ]]; then
        same=$((same + 1))
        (( same >= 5 )) && break
      else
        same=0
      fi
    fi
    prev="${c}"
    sleep 1
  done
}
