# ROS 2 UrbanNav RRR Baseline — V1

Frozen common baseline for the thesis: **GICI-Open ROS 2 full RRR
(RTK + IMU + camera, tightly coupled) on UrbanNav Medium**. All papers branch
from this exact point. Do **not** change algorithm, calibration, timestamps or
YAML in a way that alters this result without cutting a new baseline version.

## Baseline identity

| Item | Value |
| --- | --- |
| Repository name | `gici-open` (fork `Huynh-ThePham/gici-open`, upstream `chichengcn/gici-open`) |
| Local worktree | `gici_research_standard` |
| Branch before freeze | `research/standard-env` |
| Commit before freeze | `01cc563` (`Lock baseline as exact upstream GICI @ f2b8579 with author eval wrappers.`) |
| GICI core reference | upstream `f2b8579`, kept byte-identical (`include/`, `src/`, `third_party/`, `option/`) |
| Freeze date | 2026-07-11 |
| ROS 2 distribution | Humble (`/opt/ros/humble`) |
| Build system | `colcon` / `ament_cmake` |
| Build type | `Release` (`-DCMAKE_BUILD_TYPE=Release`) |
| Estimator | `rtk_imu_camera_rrr` |
| Dataset | UrbanNav HK Medium Urban 1 (`UrbanNav-HK-Medium-Urban-1`) |

## Reproduction

### Build

```bash
cd ros2_wrapper
source /opt/ros/humble/setup.bash
colcon build --packages-select gici_ros2_msgs
source install/setup.bash
colcon build --packages-select gici_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The `gici_ros2` build compiles the GICI core (`gici`, `rtklib`, `svo`,
`vikit_common`, `fast`) out-of-tree and installs those `.so` files next to the
executable with an `$ORIGIN` rpath, so the node runs without `LD_LIBRARY_PATH`.

### ROS 2 bag (created automatically by the run script)

The run script builds ONE merged ROS 2 bag (GNSS + IMU + left camera) on first
use and reuses it afterwards:

```bash
python3 scripts/ros2/urbannav_rrr_to_ros2.py \
  --gnss-dir "$URBANNAV_MEDIUM_GNSS_DIR" \
  --sensors  "$URBANNAV_MEDIUM_SENSORS" \
  --out      output/ros2_urbannav_rrr/medium/rrr_ros2
```

Dataset locations are provided through environment variables (see
`scripts/ros2/run_urbannav_rrr_ros2.sh`):

- `URBANNAV_DATA_ROOT` — root of the UrbanNav dataset download
- `URBANNAV_MEDIUM_GNSS_DIR` — directory with the GNSS ROS 1 bags
- `URBANNAV_MEDIUM_SENSORS` — path to `...-20210517_sensors.bag`

### Run (real time, default)

```bash
scripts/ros2/run_urbannav_rrr_ros2.sh medium 1
```

### Run (slower, if the machine cannot keep up at rate 1)

```bash
scripts/ros2/run_urbannav_rrr_ros2.sh medium 0.5
```

`reliable + keep-all` QoS never drops data; a slower rate just keeps queues small.

### Paths (relative to repo root)

| Item | Path |
| --- | --- |
| RRR config (source of truth) | `ros2_wrapper/src/gici_ros2/config/ros_urbannav_rrr.yaml` |
| RRR config (resolved at run time) | `output/ros2_urbannav_rrr/medium/ros_urbannav_rrr.yaml` (generated, ignored) |
| Merged ROS 2 bag | `output/ros2_urbannav_rrr/medium/rrr_ros2/` (generated, ignored) |
| Trajectory output (NMEA) | `output/ros2_urbannav_rrr/medium/solution.txt` (generated, ignored) |
| Node log | `output/ros2_urbannav_rrr/medium/node.log` (generated, ignored) |

### Evaluation

The trajectory was evaluated with the author evaluation pipeline
(`evo_ape`, Sim(3) alignment) against the UrbanNav TST INS ground truth:

```bash
# Point the author eval script at the ROS 2 RRR output, then run it.
# The script expects the solution at <OUT_DIR>/output/solution.txt.
export GICI_BASELINE_OUT="$PWD/output/ros2_urbannav_rrr/medium"
mkdir -p "$GICI_BASELINE_OUT/output"
cp output/ros2_urbannav_rrr/medium/solution.txt "$GICI_BASELINE_OUT/output/solution.txt"
scripts/run_author_eval_urbannav.sh medium
```

Evaluation tooling is built once via `scripts/build_eval_tools.sh`.

## Sensor pipeline

| Sensor | Topic | Message type |
| --- | --- | --- |
| GNSS rover | rover GNSS observations | `gici_ros2_msgs/msg/GnssObservations` |
| GNSS reference | reference-station observations | `gici_ros2_msgs/msg/GnssObservations` |
| Ephemeris (G/R/E/C) | broadcast ephemeris | `gici_ros2_msgs/msg/GnssEphemerides` |
| IMU | `/imu/data` | `sensor_msgs/msg/Imu` |
| Camera | `/zed2/camera/left/image_raw` | `sensor_msgs/msg/Image` (converted `bgr8 → mono8`) |

Timescale handling (must stay exactly as below):

```text
GNSS: GPST week/TOW → gpst2time → gpst2utc
IMU/camera: UTC header.stamp
Bag record time: internal UTC measurement time
Sensor timestamp shift: 0 seconds
enable_input_align: true
```

Ephemeris / ionosphere / antenna-position messages are bursted at the start of
the bag (their `toe` stamps span ~20 h but the drive is ~13 min), mirroring
file-mode `burst_load`.

## Verified result

```text
GPGGA epochs: 762
RRR updates: 7255
Sensor type: 3
Fix status: 3
Average satellites: approximately 14
APE position: 1.81 m
APE orientation: 0.77 deg
Crash: none
Dropped messages: none observed
Shutdown: clean
```

Comparison to the paper (paper ≈ 3.40 m / 1.30 deg; author file-mode reproduction
≈ 4.55 m / 2.06 deg) is a **numerical comparison only** at this stage. It is
**not** a claim of beating the paper: the ROS 2 output is sampled at the GNSS
epoch rate and the evaluation protocol (alignment, output sampling, GT
interpolation) has **not** yet been confirmed to be identical to the paper's.
Treat the number as an internal, self-consistent baseline, not a published claim.

## Known operational constraints

- Do **not** run two bag players at the same time. A stale player publishing to
  the same topics makes the estimator see time go backwards
  (`Image timestamp descending`) and drop everything. `run_urbannav_rrr_ros2.sh`
  defensively `pkill`s stale `bag play` / `gici_ros2_main` before starting.
- Bag **record time must equal internal measurement time** (UTC). If they differ,
  the high-rate IMU races ahead of the arriving 1 Hz GNSS epoch and the
  input-align buffer discards GNSS (`Throwing data ... latency too large`) → no RTK.
- Do **not** shift IMU/camera `header.stamp` by `+18 s`. UrbanNav sensor stamps
  are already UTC, i.e. already on GICI's internal GNSS timescale.
- RRR is compute-heavy. If the estimator lags at `rate 1`, lower the rate; QoS
  never drops data, so a slower rate only keeps queues small.

## Baseline files (frozen)

```text
scripts/ros2/urbannav_rrr_to_ros2.py         # merged GNSS+IMU+camera ROS1→ROS2 converter
scripts/ros2/run_urbannav_rrr_ros2.sh        # end-to-end RRR run script
ros2_wrapper/src/gici_ros2/                   # rclcpp node + RRR config
ros2_wrapper/src/gici_ros2_msgs/              # 13 GNSS messages ported to rosidl
ros2_wrapper/README.md                        # full pipeline + time-alignment notes
docs/baseline/ROS2_URBANNAV_RRR_BASELINE_V1.md  # this manifest
```

Tag: `baseline-full-rrr-ros2-v1`.
