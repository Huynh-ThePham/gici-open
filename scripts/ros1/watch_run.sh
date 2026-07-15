#!/usr/bin/env bash
# Host-side live monitor for ROS1 UrbanNav RRR (native or Docker).
#
# Usage:
#   scripts/ros1/watch_run.sh [out_dir] [docker_container_name]
#
# Watches solution.txt + node.log every N seconds. Exits non-zero on crash or
# prolonged lack of RRR fusion (same thresholds as run_urbannav_rrr_ros1.sh).
set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT_DIR="${1:-${REPO}/output/ros1_urbannav_rrr/medium}"
CONTAINER="${2:-}"
SOLUTION="${OUT_DIR}/solution.txt"
NODE_LOG="${OUT_DIR}/node.log"

INTERVAL="${URBANNAV_WATCH_INTERVAL:-10}"
RRR_WARMUP="${URBANNAV_RRR_WARMUP:-90}"
PROGRESS_EVERY="${URBANNAV_PROGRESS_EVERY:-30}"

solution_epochs() {
  local n=0
  if [[ -s "${SOLUTION}" ]]; then
    n="$(grep -c GPGGA "${SOLUTION}" 2>/dev/null)" || n=0
  fi
  echo "${n}"
}

rrr_count() {
  local n=0
  if [[ -f "${NODE_LOG}" ]]; then
    n="$(grep -c 'RTK/IMU/Camera RRR:' "${NODE_LOG}" 2>/dev/null)" || n=0
  fi
  echo "${n}"
}

log_lines() {
  local n=0
  if [[ -f "${NODE_LOG}" ]]; then
    n="$(wc -l < "${NODE_LOG}" | tr -d ' ')"
  fi
  echo "${n}"
}

container_running() {
  [[ -n "${CONTAINER}" ]] && docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"
}

stop_run() {
  if container_running; then
    echo "[watch] Stopping docker container ${CONTAINER}" >&2
    docker stop "${CONTAINER}" >/dev/null 2>&1 || true
  fi
  pkill -f "gici_ros_main.*ros_urbannav_rrr" 2>/dev/null || true
  pkill -f "rosbag play.*UrbanNav-HK" 2>/dev/null || true
}

fail() {
  local msg="$1"
  echo "[watch] ERROR: ${msg}" >&2
  tail -20 "${NODE_LOG}" >&2 2>/dev/null || true
  stop_run
  exit 1
}

echo "[watch] Monitoring ${OUT_DIR}"
echo "[watch] interval=${INTERVAL}s rrr_warmup=${RRR_WARMUP}s progress=${PROGRESS_EVERY}s"
start_ts=$(date +%s)
fusion_start_ts=""
next_progress=$((start_ts + PROGRESS_EVERY))
node_log_seen_mtime=0

node_log_fresh() {
  [[ -f "${NODE_LOG}" ]] || return 1
  local mtime
  mtime="$(stat -c %Y "${NODE_LOG}" 2>/dev/null || echo 0)"
  [[ "${mtime}" -ge "${start_ts}" ]]
}

if [[ -n "${CONTAINER}" ]]; then
  echo "[watch] Waiting for container ${CONTAINER} ..."
  for _ in $(seq 1 60); do
    container_running && break
    sleep 1
  done
  if ! container_running; then
    echo "[watch] WARN: container ${CONTAINER} not seen; monitoring files only." >&2
    CONTAINER=""
  fi
fi

while true; do
  now=$(date +%s)
  elapsed=$((now - start_ts))

  if node_log_fresh; then
    if grep -qE 'Check failed:|Segmentation fault|FATAL|core dumped|Aborted at|terminate called' \
        "${NODE_LOG}" 2>/dev/null; then
      fail "crash/fatal in node.log"
    fi
  fi

  epochs="$(solution_epochs)"
  rrr="$(rrr_count)"
  lines="$(log_lines)"

  if [[ -z "${fusion_start_ts}" ]] && node_log_fresh \
      && grep -qE 'Running\.\.\.|Initialized .* estimator' "${NODE_LOG}" 2>/dev/null; then
    fusion_start_ts="${now}"
    node_log_seen_mtime="$(stat -c %Y "${NODE_LOG}" 2>/dev/null || echo 0)"
    echo "[watch] Estimator active; fusion watchdog starts now (rrr_warmup=${RRR_WARMUP}s)."
  fi

  if [[ -n "${fusion_start_ts}" ]]; then
    fusion_elapsed=$((now - fusion_start_ts))
  else
    fusion_elapsed=0
  fi

  if [[ "${now}" -ge "${next_progress}" ]]; then
    echo "[watch] elapsed=${elapsed}s fusion=${fusion_elapsed}s epochs=${epochs} rrr=${rrr} log_lines=${lines}"
    next_progress=$((now + PROGRESS_EVERY))
  fi

  if [[ -n "${fusion_start_ts}" && "${fusion_elapsed}" -ge "${RRR_WARMUP}" \
      && "${epochs}" -eq 0 && "${rrr}" -eq 0 ]]; then
    fail "no RRR fusion after ${fusion_elapsed}s (estimator active)"
  fi

  if [[ -n "${CONTAINER}" ]] && ! container_running; then
    # Container ended — keep polling files briefly for final writes.
    if [[ $((now - start_ts)) -gt 30 ]]; then
      echo "[watch] Container ${CONTAINER} stopped."
      break
    fi
  fi

  if [[ -z "${CONTAINER}" ]] && ! pgrep -f "gici_ros_main.*ros_urbannav_rrr" >/dev/null 2>&1 \
      && ! pgrep -f "rosbag play.*UrbanNav-HK" >/dev/null 2>&1; then
    echo "[watch] ROS1 run processes ended."
    break
  fi

  sleep "${INTERVAL}"
done

epochs="$(solution_epochs)"
rrr="$(rrr_count)"
echo "[watch] Final: epochs=${epochs} rrr=${rrr}"
if [[ "${epochs}" -le 0 && "${rrr}" -le 0 ]]; then
  exit 1
fi
