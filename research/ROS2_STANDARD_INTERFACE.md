# ROS 2 standard interface research

**Branch:** `research/ros2-standard-interface` (GitHub default branch — real-time primary)  
**Worktree:** `/home/theph/ws_ncs/gici_research_standard` (main worktree)  
**Base:** originally `research/ros2-realtime-fix` @ `realtime-safe-v1` (crash-safe bag replay);
that branch was retired 2026-07-17 after merging into this one (see `research/README_ENV.md`).

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

This branch is now the real-time primary (see `research/README_ENV.md`) — the earlier
`research/ros2-realtime-fix` was retired after its correctness fixes were merged here.
Core correctness fixes land here directly and merge forward into
`research/ros2-live-nosparsify-experiment`.

## Build

```bash
cd /home/theph/ws_ncs/gici_research_standard/ros2_wrapper   # main worktree
source /opt/ros/humble/setup.bash
colcon build --packages-select gici_ros2_msgs gici_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The `research/ros2-live-nosparsify-experiment` worktree remains at
`/home/theph/ws_ncs/gici_ros2_standard_interface` for the nosparsify experiment.

## Quick start (this branch)

```bash
# LIVE — GICI chạy trong ROS2 (topic-driven, MultiSensorEstimating)
./scripts/ros2/launch_gici_live.sh board 1.1          # node chờ sensor
./scripts/ros2/launch_gici_live.sh board 1.1 bag 1.0  # test với bag replay

# POSTFILE — batch baseline (không phải live)
./scripts/ros2/launch_gici_board_std.sh postfile 1.1
./scripts/ros2/launch_gici_board_std.sh bag 1.1 1.0   # alias live + bag infra
```

Topic layout: `ros2_wrapper/docs/ROS2_STANDARD_TOPICS.md`  
Live architecture: `ros2_wrapper/docs/ROS2_LIVE_NODE.md`

## Status (board RRR)

| Item | Status |
|------|--------|
| `/gici/odom`, `/gici/path`, `/gici/pose` in postfile + bag YAML | done |
| `gici_ros2_main`: `config_file` param + SIGINT/SIGTERM → `rclcpp::shutdown()` | done |
| `ros2 launch gici_ros2 gici_board.launch.py` | done |
| `scripts/ros2/render_gici_config.py` + `launch_gici_board_std.sh` | done |
| `ros2_wrapper/docs/ROS2_STANDARD_TOPICS.md` | done |
| colcon build + smoke (topics + GPGGA) | verified |
| UrbanNav std launch / topics | done (`ros_urbannav_live_rrr.yaml`, `launch_gici_live.sh urbannav`) |
| Live node doc | `ros2_wrapper/docs/ROS2_LIVE_NODE.md` |
