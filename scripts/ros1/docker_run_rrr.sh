#!/usr/bin/env bash
# Run UrbanNav RRR inside ROS 1 Noetic Docker (no local Noetic install needed).
#
# Usage:
#   scripts/ros1/docker_run_rrr.sh [table5|native] [medium|deep] [rate]
#
# First run builds the catkin workspace inside the container (cached in ros_wrapper/build|devel).
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${GICI_ROS1_IMAGE:-gici-ros1-noetic:latest}"
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
DATA_ROOT="${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}"

if ! command -v docker >/dev/null; then
  echo "[docker] ERROR: docker not found" >&2
  exit 1
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[docker] Image ${IMAGE} not found; building ..."
  "${REPO}/scripts/ros1/docker_build.sh"
fi

if [[ ! -d "${DATA_ROOT}" ]]; then
  echo "[docker] ERROR: UrbanNav dataset not found at ${DATA_ROOT}" >&2
  exit 1
fi

if [[ "${DS}" != "medium" && "${DS}" != "deep" ]]; then
  echo "[docker] Unknown dataset '${DS}'. Use 'medium' or 'deep'." >&2
  exit 1
fi

case "${DS}" in
  medium)
    GNSS_MOUNT="/data/urbannav/UrbanNav-HK-Medium-Urban-1/medium"
    SENSORS_MOUNT="/data/urbannav/UrbanNav-HK-Medium-Urban-1/ros/UrbanNav-HK_TST-20210517_sensors.bag"
    ;;
  deep)
    GNSS_MOUNT="/data/urbannav/UrbanNav-HK-Deep-Urban-1/deep"
    SENSORS_MOUNT="/data/urbannav/UrbanNav-HK-Deep-Urban-1/ros/UrbanNav-HK_Whampoa-20210521_sensors.bag"
    ;;
esac

if [[ "${MODE}" == "table5" ]]; then
  OUT_DIR="${REPO}/output/ros1_urbannav_rrr_table5/${DS}/run"
else
  OUT_DIR="${REPO}/output/ros1_urbannav_rrr/${DS}"
fi
CONTAINER="${GICI_ROS1_CONTAINER:-gici-ros1-rrr-run}"
docker rm -f "${CONTAINER}" 2>/dev/null || true

WATCH_PID=""
if [[ "${URBANNAV_WATCH:-1}" != "0" ]]; then
  echo "[docker] Starting host watchdog (URBANNAV_WATCH=0 to disable) ..."
  WATCH_RRR_WARMUP="${URBANNAV_RRR_WARMUP:-}"
  if [[ -z "${WATCH_RRR_WARMUP}" ]]; then
    if [[ "${MODE}" == "table5" ]]; then
      WATCH_RRR_WARMUP=300
    else
      WATCH_RRR_WARMUP=90
    fi
  fi
  (
    sleep 5
    if [[ "${DS}" == "deep" ]]; then
      URBANNAV_RRR_WARMUP="${WATCH_RRR_WARMUP}" \
        "${REPO}/scripts/ros1/watch_run.sh" "${OUT_DIR}" "${CONTAINER}"
    else
      URBANNAV_RRR_WARMUP="${WATCH_RRR_WARMUP}" \
        "${REPO}/scripts/ros1/watch_run.sh" "${OUT_DIR}" "${CONTAINER}"
    fi
  ) &
  WATCH_PID=$!
fi

set +e
docker run --rm \
  --name "${CONTAINER}" \
  --entrypoint bash \
  -v "${REPO}:/workspace/gici:rw" \
  -v "${DATA_ROOT}:/data/urbannav:ro" \
  -e URBANNAV_DATA_ROOT=/data/urbannav \
  -e URBANNAV_MEDIUM_GNSS_DIR="/data/urbannav/UrbanNav-HK-Medium-Urban-1/medium" \
  -e URBANNAV_MEDIUM_SENSORS="/data/urbannav/UrbanNav-HK-Medium-Urban-1/ros/UrbanNav-HK_TST-20210517_sensors.bag" \
  -e URBANNAV_DEEP_GNSS_DIR="/data/urbannav/UrbanNav-HK-Deep-Urban-1/deep" \
  -e URBANNAV_DEEP_SENSORS="/data/urbannav/UrbanNav-HK-Deep-Urban-1/ros/UrbanNav-HK_Whampoa-20210521_sensors.bag" \
  "${IMAGE}" \
  -lc "
    set -eo pipefail
    source /opt/ros/noetic/setup.bash
    cd /workspace/gici
    if [[ "${URBANNAV_SKIP_FIDELITY_CHECK:-0}" != "1" ]]; then
      ./scripts/verify_upstream_fidelity.sh
    else
      echo '[docker] Skipping verify_upstream_fidelity (URBANNAV_SKIP_FIDELITY_CHECK=1)'
    fi
    if [[ ! -x ros_wrapper/devel/lib/gici_ros/gici_ros_main ]] \
        || grep -q '/ws/ros_wrapper' ros_wrapper/build/CMakeCache.txt 2>/dev/null \
        || [[ "${URBANNAV_FORCE_REBUILD:-0}" == "1" ]]; then
      echo '[docker] Building gici_ros_main (first run may take several minutes) ...'
      rm -rf ros_wrapper/build ros_wrapper/devel
      cd ros_wrapper && catkin_make -DCMAKE_BUILD_TYPE=Release -j\$(nproc)
    fi
    cd /workspace/gici
    if ! python3 -c 'import rosbags' 2>/dev/null; then
      if ! command -v pip3 >/dev/null 2>&1; then
        apt-get update -qq
        DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-pip
      fi
      python3 -m pip install -q rosbags
    fi
    export URBANNAV_ROS1_MODE=${MODE}
    scripts/ros1/run_urbannav_rrr_ros1.sh ${MODE} ${DS} ${RATE}
  " | stdbuf -oL -eL tee /tmp/docker_ros1_rrr.log
DOCKER_STATUS=$?
set -e

if [[ -n "${WATCH_PID}" ]]; then
  kill "${WATCH_PID}" 2>/dev/null || true
  wait "${WATCH_PID}" 2>/dev/null || true
fi

exit "${DOCKER_STATUS}"
