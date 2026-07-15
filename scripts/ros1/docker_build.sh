#!/usr/bin/env bash
# Build the ROS 1 Noetic Docker image for UrbanNav RRR.
set -Eeuo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${GICI_ROS1_IMAGE:-gici-ros1-noetic:latest}"

docker build -t "${IMAGE}" -f "${REPO}/docker/ros1-noetic/Dockerfile" "${REPO}"
echo "[docker] Built ${IMAGE}"
