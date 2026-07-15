#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
if [[ -f "${CATKIN_WS}/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${CATKIN_WS}/devel/setup.bash"
fi
exec "$@"
