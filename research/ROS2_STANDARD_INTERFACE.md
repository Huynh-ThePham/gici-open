# ROS 2 standard interface research

**Branch:** `research/ros2-standard-interface`  
**Worktree:** `/home/theph/ws_ncs/gici_ros2_standard_interface`  
**Base:** `research/ros2-realtime-fix` @ `realtime-safe-v1` (crash-safe bag replay)

## Goal

Explore converting the GICI ROS 2 **wrapper interface** toward an OpenVINS-style layout
without changing the GICI fusion core (`src/`, `include/`).

## In scope (wrapper / scripts only)

- `ros2 launch` entry points (board RRR, UrbanNav)
- Thin ROS params → runtime YAML (dataset, rate, output dir)
- Standard output topics: `nav_msgs/Odometry`, `nav_msgs/Path`, TF
- Reliable `ros2 launch` / SIGINT shutdown (no direct-binary requirement)
- Topic naming convention document (`/gici/...`)

## Out of scope

- Core algorithm / fusion refactors
- Removing `gici_ros2_msgs` (GNSS remains custom)
- Replacing file-mode baseline (paper Table V stays on YAML postfile)

## Reference branch

Keep `research/ros2-realtime-fix` frozen as the validated runtime baseline.
Merge interface improvements here first; backport only when stable.

## Build (same as main worktree)

```bash
cd /home/theph/ws_ncs/gici_ros2_standard_interface/ros2_wrapper
source /opt/ros/humble/setup.bash
colcon build --packages-select gici_ros2_msgs gici_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
