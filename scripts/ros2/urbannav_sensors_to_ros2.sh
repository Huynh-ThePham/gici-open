#!/usr/bin/env bash
# Convert the UrbanNav sensor ROS1 bag (IMU + ZED2 camera) to a ROS 2 bag.
#
# Only needed for the full RTK/IMU/Camera (RRR) pipeline. The sensor topics use
# standard sensor_msgs, so the generic rosbags-convert tool handles them directly.
#
# Usage:
#   scripts/ros2/urbannav_sensors_to_ros2.sh <input_sensors.bag> <output_ros2_bag_dir>
#
# After conversion, run the RRR pipeline by:
#   - enabling the str_ros_imu + str_ros_camera streamers and the
#     est_gnss_imu_camera_rrr estimator in the config (see config/ros_urbannav.yaml),
#   - playing BOTH the GNSS ROS 2 bag and this sensor ROS 2 bag (two terminals):
#       ros2 bag play <gnss_ros2_bag>
#       ros2 bag play <sensors_ros2_bag> --topics /imu/data /zed2/camera/left/image_raw
set -euo pipefail

IN_BAG="${1:?input ROS1 sensor .bag required}"
OUT_DIR="${2:?output ROS 2 bag directory required}"

if [[ -e "${OUT_DIR}" ]]; then
  echo "ERROR: ${OUT_DIR} already exists; remove it first." >&2
  exit 1
fi

# rosbags-convert (pip package 'rosbags') performs ROS1 -> ROS 2 for standard messages.
rosbags-convert --src "${IN_BAG}" --dst "${OUT_DIR}"
echo "Done. ROS 2 sensor bag at ${OUT_DIR}"
