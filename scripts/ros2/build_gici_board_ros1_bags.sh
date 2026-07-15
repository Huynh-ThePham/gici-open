#!/usr/bin/env bash
# Convert GICI board *.bin -> ROS 1 bags via author gici_files_to_rosbag.
#
# Usage: scripts/ros2/build_gici_board_ros1_bags.sh <1.1|3.1|4.1> [rtcm_start]
#
# Requires ROS 1 Noetic (local or GICI_ROS1_IMAGE docker).
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
# shellcheck source=dataset_paths.sh
source "${REPO}/scripts/dataset_paths.sh"

DATASET_ID="${1:-}"
RTCM_START="${2:-${GICI_RTCM_START_TIME:-2023.03.20}}"
if [[ -z "${DATASET_ID}" ]]; then
  echo "Usage: $0 <1.1|3.1|4.1> [rtcm_start]" >&2
  exit 1
fi

DATASET_DIR="${GICI_DATA_ROOT}/${DATASET_ID}"
TEMPLATE="${REPO}/tools/ros/src/gici_tools/option/convert_rosbags.yaml"
CFG="${REPO}/output/ros2_gici_board/${DATASET_ID}/convert_rosbags.yaml"
EXE="${REPO}/tools/ros/devel/lib/gici_tools/gici_files_to_rosbag"
IMAGE="${GICI_ROS1_IMAGE:-gici-ros1-noetic:latest}"

for f in gnss_rover.bin gnss_reference.bin gnss_ephemeris.bin imu.bin camera.bin; do
  [[ -f "${DATASET_DIR}/${f}" ]] || {
    echo "ERROR: missing ${DATASET_DIR}/${f}" >&2
    echo "Hint: ./scripts/setup_gici_datasets.sh ${DATASET_ID}" >&2
    exit 1
  }
done

mkdir -p "$(dirname "${CFG}")"
sed \
  -e "s|<data-directory>|${DATASET_DIR}|g" \
  -e "s|start_time: 2023.03.20|start_time: ${RTCM_START}|g" \
  "${TEMPLATE}" > "${CFG}"

need_build=0
if [[ ! -x "${EXE}" ]]; then
  need_build=1
fi

run_local() {
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
  cd "${REPO}/tools/ros"
  if [[ "${need_build}" -eq 1 ]]; then
    echo "[gici-board-ros1] Building gici_tools (catkin) ..."
    catkin_make -DCMAKE_BUILD_TYPE=Release
  fi
  echo "[gici-board-ros1] Converting ${DATASET_ID} bins -> ROS1 bags ..."
  "${EXE}" "${CFG}"
}

run_docker() {
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    "${REPO}/scripts/ros1/docker_build.sh"
  fi
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
      CFG=/workspace/gici/output/ros2_gici_board/${DATASET_ID}/convert_rosbags.yaml
      sed \
        -e 's|<data-directory>|${DATASET_DIR}|g' \
        -e 's|start_time: 2023.03.20|start_time: ${RTCM_START}|g' \
        /workspace/gici/tools/ros/src/gici_tools/option/convert_rosbags.yaml > \"\${CFG}\"
      echo '[docker] gici_files_to_rosbag ${DATASET_ID} ...'
      ./devel/lib/gici_tools/gici_files_to_rosbag \"\${CFG}\"
    "
}

if [[ -f /opt/ros/noetic/setup.bash ]] && command -v catkin_make >/dev/null 2>&1; then
  run_local
else
  echo "[gici-board-ros1] No local Noetic; using Docker ${IMAGE}"
  run_docker
fi

for b in gnss_rover.bag gnss_reference.bag gnss_ephemeris.bag imu.bag image.bag; do
  [[ -f "${DATASET_DIR}/${b}" ]] || { echo "ERROR: missing ${DATASET_DIR}/${b}" >&2; exit 1; }
done
echo "[gici-board-ros1] ROS1 bags ready under ${DATASET_DIR}"
