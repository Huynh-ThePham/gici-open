#!/usr/bin/env bash
# Convert GICI board *.bin -> ROS 1 bags via author gici_tools.
#
# Hybrid bag replay uses rover + reference + IMU + camera bags. Ephemeris is
# loaded from gnss_ephemeris.bin at runtime (gici_files_to_rosbag segfaults on eph).
#
# Usage: scripts/ros2/build_gici_board_ros1_bags.sh <1.1|3.1|4.1> [rtcm_start]
#   GICI_FORCE_BAG_REBUILD=1  force re-convert even if bags look valid
#
# Requires ROS 1 Noetic (local or GICI_ROS1_IMAGE docker).
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=dataset_paths.sh
source "${REPO}/scripts/dataset_paths.sh"

DATASET_ID="${1:-}"
FORCE="${GICI_FORCE_BAG_REBUILD:-0}"
RTCM_START="${2:-${GICI_RTCM_START_TIME:-$(gici_rtcm_start_for "${DATASET_ID}")}}"
if [[ -z "${DATASET_ID}" ]]; then
  echo "Usage: $0 <1.1|3.1|4.1> [rtcm_start]" >&2
  exit 1
fi

DATASET_DIR="${GICI_DATA_ROOT}/${DATASET_ID}"
OUT_CFG_DIR="${REPO}/output/ros2_gici_board/${DATASET_ID}"
TPL_ROV="${REPO}/tools/ros/src/gici_tools/option/convert_rosbags_rover.yaml"
TPL_REF="${REPO}/tools/ros/src/gici_tools/option/convert_rosbags_ref.yaml"
CFG_ROV="${OUT_CFG_DIR}/convert_rosbags_rover.yaml"
CFG_REF="${OUT_CFG_DIR}/convert_rosbags_ref.yaml"
EXE="${REPO}/tools/ros/devel/lib/gici_tools/gici_files_to_rosbag"
EXE_IMU="${REPO}/tools/ros/devel/lib/gici_tools/imupack_to_rosbag"
EXE_CAM="${REPO}/tools/ros/devel/lib/gici_tools/imagepack_to_rosbag"
IMAGE="${GICI_ROS1_IMAGE:-gici-ros1-noetic:latest}"

declare -A MIN_BAG_BYTES=(
  [gnss_rover.bag]=1000000
  [gnss_reference.bag]=100000
  [imu.bag]=100000
  [image.bag]=10000000
)

bag_valid() {
  local b="$1"
  local min="${MIN_BAG_BYTES[$b]:-1000}"
  [[ -f "${DATASET_DIR}/${b}" ]] || return 1
  local sz
  sz="$(stat -c%s "${DATASET_DIR}/${b}")"
  (( sz >= min ))
}

for f in gnss_rover.bin gnss_reference.bin gnss_ephemeris.bin imu.bin camera.bin; do
  [[ -f "${DATASET_DIR}/${f}" ]] || {
    echo "ERROR: missing ${DATASET_DIR}/${f}" >&2
    echo "Hint: ./scripts/setup_gici_datasets.sh ${DATASET_ID}" >&2
    exit 1
  }
done

need_rebuild=0
if (( FORCE )); then
  need_rebuild=1
else
  for b in gnss_rover.bag gnss_reference.bag imu.bag image.bag; do
    bag_valid "$b" || need_rebuild=1
  done
fi

write_cfgs() {
  mkdir -p "${OUT_CFG_DIR}"
  sed -e "s|<data-directory>|${DATASET_DIR}|g" "${TPL_ROV}" > "${CFG_ROV}"
  sed \
    -e "s|<data-directory>|${DATASET_DIR}|g" \
    -e "s|start_time: 2023.03.20|start_time: ${RTCM_START}|g" \
    "${TPL_REF}" > "${CFG_REF}"
}

run_local() {
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
  cd "${REPO}/tools/ros"
  if [[ ! -x "${EXE}" ]] || [[ ! -x "${EXE_IMU}" ]] || [[ ! -x "${EXE_CAM}" ]]; then
    echo "[gici-board-ros1] Building gici_tools (catkin) ..."
    catkin_make -DCMAKE_BUILD_TYPE=Release
  fi
  echo "[gici-board-ros1] Converting rover GNSS ${DATASET_ID} ..."
  "${EXE}" "${CFG_ROV}"
  echo "[gici-board-ros1] Converting reference GNSS (RTCM ${RTCM_START}) ..."
  "${EXE}" "${CFG_REF}"
  echo "[gici-board-ros1] Converting IMU pack ..."
  "${EXE_IMU}" "${DATASET_DIR}/imu.bin"
  mv -f "${DATASET_DIR}/imu.bin.bag" "${DATASET_DIR}/imu.bag"
  echo "[gici-board-ros1] Converting camera pack (may take several minutes) ..."
  "${EXE_CAM}" "${DATASET_DIR}/camera.bin"
  mv -f "${DATASET_DIR}/camera.bin.bag" "${DATASET_DIR}/image.bag"
}

run_docker() {
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    "${REPO}/scripts/ros1/docker_build.sh"
  fi
  local cfg_rov="/workspace/gici/output/ros2_gici_board/${DATASET_ID}/convert_rosbags_rover.yaml"
  local cfg_ref="/workspace/gici/output/ros2_gici_board/${DATASET_ID}/convert_rosbags_ref.yaml"
  docker run --rm \
    -v "${REPO}:/workspace/gici:rw" \
    -v "${GICI_DATA_ROOT}:${GICI_DATA_ROOT}:rw" \
    -e GICI_DATA_ROOT="${GICI_DATA_ROOT}" \
    --entrypoint bash \
    "${IMAGE}" \
    -lc "
      set -eo pipefail
      source /opt/ros/noetic/setup.bash
      cd /workspace/gici/tools/ros
      if [[ ! -x devel/lib/gici_tools/gici_files_to_rosbag ]]; then
        echo '[docker] Building gici_tools ...'
        catkin_make -DCMAKE_BUILD_TYPE=Release -j\$(nproc)
      fi
      echo '[docker] Rover GNSS ${DATASET_ID} ...'
      ./devel/lib/gici_tools/gici_files_to_rosbag \"${cfg_rov}\"
      echo '[docker] Reference GNSS (RTCM ${RTCM_START}) ...'
      ./devel/lib/gici_tools/gici_files_to_rosbag \"${cfg_ref}\"
      echo '[docker] IMU pack ...'
      ./devel/lib/gici_tools/imupack_to_rosbag \"${DATASET_DIR}/imu.bin\"
      mv -f \"${DATASET_DIR}/imu.bin.bag\" \"${DATASET_DIR}/imu.bag\"
      echo '[docker] Camera pack (long) ...'
      ./devel/lib/gici_tools/imagepack_to_rosbag \"${DATASET_DIR}/camera.bin\"
      mv -f \"${DATASET_DIR}/camera.bin.bag\" \"${DATASET_DIR}/image.bag\"
    "
}

if (( need_rebuild )); then
  echo "[gici-board-ros1] Building ROS1 bags under ${DATASET_DIR} ..."
  rm -f "${DATASET_DIR}/gnss_rover.bag" "${DATASET_DIR}/gnss_reference.bag" \
        "${DATASET_DIR}/imu.bag" "${DATASET_DIR}/image.bag"
  write_cfgs
  if [[ -f /opt/ros/noetic/setup.bash ]] && command -v catkin_make >/dev/null 2>&1; then
    run_local
  else
    echo "[gici-board-ros1] No local Noetic; using Docker ${IMAGE}"
    run_docker
  fi
else
  echo "[gici-board-ros1] SKIP conversion — valid ROS1 bags already present"
fi

for b in gnss_rover.bag gnss_reference.bag imu.bag image.bag; do
  if ! bag_valid "$b"; then
    echo "ERROR: missing or stub ${DATASET_DIR}/${b} (min ${MIN_BAG_BYTES[$b]} bytes)" >&2
    exit 1
  fi
  sz="$(stat -c%s "${DATASET_DIR}/${b}")"
  echo "[gici-board-ros1]   ${b}: $(numfmt --to=iec-i --suffix=B "${sz}" 2>/dev/null || echo "${sz} bytes")"
done
echo "[gici-board-ros1] ROS1 bags ready (eph loaded from *.bin at runtime)"
