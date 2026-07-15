# GICI ROS 1 — UrbanNav RRR

Author-native UrbanNav workflow: `gici_ros_main` + `rosbag play`, config
`ros_wrapper/src/gici/option/ros_urbannav_rrr.yaml` (RRR block from
`ros_urbannav.yaml` @ f2b8579).

## Prerequisites

- **ROS Noetic** (`/opt/ros/noetic/setup.bash`) — not ROS 2 Humble  
  **Hoặc Docker** (không cần cài Noetic trên host):

```bash
./scripts/ros1/docker_build.sh          # một lần
./scripts/ros1/docker_run_rrr.sh medium 1
```

- UrbanNav dataset with `medium/*.bag` and `ros/*_sensors.bag`
- GICI core byte-identical to `f2b8579` (`./scripts/verify_upstream_fidelity.sh`)

## Build (native Noetic)

```bash
./scripts/ros1/build_ros1_wrapper.sh
source ros_wrapper/devel/setup.bash
```

## Run (medium, full RRR)

```bash
scripts/ros1/run_urbannav_rrr_ros1.sh medium 1
```

Output: `output/ros1_urbannav_rrr/medium/solution.txt`

Eval (evo_ape vs GT, paper Table V @ 1 Hz):

```bash
scripts/ros1/run_eval.sh medium
# → results/baseline/urbannav_ros1/medium/evaluation/ape_metrics.json
```

Note: paper Table V (3.40 m / 1.30°) uses **file-mode** with **HKKT** base + DCB RINEX.
Native ROS 1 bags under `medium/` use **HKSC** base (`medium/gnss/info.txt`) — same
pipeline, different reference station; use file-mode for Table V numbers.

Environment (optional):

- `URBANNAV_DATA_ROOT`
- `URBANNAV_MEDIUM_GNSS_DIR` — default `.../UrbanNav-HK-Medium-Urban-1/medium`
- `URBANNAV_MEDIUM_SENSORS` — default `.../ros/UrbanNav-HK_TST-20210517_sensors.bag`
- `URBANNAV_PLAY_DURATION` — rosbag play cap in seconds (default `800`)
- `URBANNAV_WATCH_INTERVAL` — watchdog poll interval (default `10`)
- `URBANNAV_STALL_WARMUP` — seconds before stall detection (default `90`)
- `URBANNAV_STALL_SECONDS` — fail if no log/solution progress for this long (default `120`)
- `URBANNAV_PROGRESS_EVERY` — progress line interval during playback (default `30`)
- `URBANNAV_RRR_WARMUP` — abort early if no RRR fusion by this many seconds (default `90`; ROS2 thường có RRR sau ~30s)
- `URBANNAV_WATCH=0` — disable host-side watchdog in `docker_run_rrr.sh`
- `URBANNAV_SKIP_FIDELITY_CHECK=1` — skip byte-identical upstream check (needed when GICI core has local fixes)

Watchdog (built into `run_urbannav_rrr_ros1.sh`):

- Progress every 30s: `epochs`, `rrr`, `log_lines`
- Crash patterns in `node.log` → stop immediately
- No `RTK/IMU/Camera RRR` after 90s → abort (don't wait full 800s bag)
- Stall: no progress on epochs/rrr/log for 120s after 90s warmup

Host monitor (optional, also auto-started by `docker_run_rrr.sh`):

```bash
scripts/ros1/watch_run.sh output/ros1_urbannav_rrr/medium gici-ros1-rrr-run
```

## vs file-mode / ROS 2

| Path | Role |
|------|------|
| **This ROS 1 script** | Author-native ROS baseline (HKSC GNSS bags) |
| File-mode wrapper | Algorithm baseline (HKKT + DCB RINEX) |
| ROS 2 adapted | Port of this ROS 1 bag pipeline |
