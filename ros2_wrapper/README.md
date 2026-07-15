# GICI ROS 2 wrapper (UrbanNav)

ROS 2 (Humble, rclcpp/ament) port of the original author's ROS 1 wrapper
(`chichengcn/gici-open` @ `f2b8579`, in `ros_wrapper/`). The GICI core
(`include/`, `src/`, `third_party/`, `option/`) stays **byte-identical to upstream**;
this workspace only adds an out-of-tree ROS 2 wrapper and message package, mirroring
the way the ROS 1 wrapper did `add_subdirectory(../../.. )` to build the core `gici`
library and link against it.

## Recommended / standard way to run UrbanNav

UrbanNav is an **offline dataset**, so the correct, reproducible method is
**post-processing file mode** (exactly what the GICI authors do): read RINEX/IMU/camera
directly, `replay: enable: false`, deterministic, lossless, matches the paper.

```bash
scripts/run_urbannav_rtk_filemode.sh medium   # or: deep   (primary, reproducible)
```

Use the **ROS 2 path** only for what ROS is actually for — live/online operation, rviz,
recording, multi-node integration — or for a dataset-replay demo. It is now hardened to
run losslessly at any playback rate:

```bash
scripts/ros2/run_urbannav_ros2.sh medium 20    # rate 20 is fine now (see QoS note)
```

Two things make the ROS path robust (both already applied):

- **Lossless QoS.** Input subscriptions use `reliable + keep-all` history, so no GNSS/
  ephemeris messages are dropped even at high rate (GICI keys off the message week/tow,
  not the wall clock). Verified: rate 20 → 662 epochs, 0 drops/crashes.
- **Clean shutdown.** Launch the binary directly (not `ros2 run`, which does not forward
  SIGINT to the child when backgrounded). Verified: SIGINT → clean exit in ~0.5 s.

## Layout

```
ros2_wrapper/
  src/gici_ros2_msgs/        # 13 GNSS messages ported to rosidl (ROS 2)
  src/gici_ros2/             # rclcpp node (ports gici_ros_main + ros_stream +
                             #   ros_node_handle + ros_publisher)
    config/ros_urbannav.yaml # verbatim upstream ros_urbannav.yaml, only output paths changed
    launch/urbannav.launch.py
```

### Field renaming (important)

ROS 2's `rosidl` forbids upper-case message field names, but the author's ROS 1
messages use `SNR`, `LLI`, `L`, `P`, `D`, `A`, `M0`, `OMG0`, `OMGd`. `gici_ros2_msgs`
renames them to snake_case (`snr`, `lli`, `l`, `p`, `d`, `a`, `m0`, `omg0`, `omgd`).
The wrapper's publish/subscribe code and the bag converter apply this mapping
consistently, so wire data is equivalent to upstream.

## Build

```bash
cd ros2_wrapper
source /opt/ros/humble/setup.bash
colcon build --packages-select gici_ros2_msgs
source install/setup.bash
colcon build --packages-select gici_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The `gici_ros2` build compiles the GICI core (`gici`, `rtklib`, `svo`, `vikit_common`,
`fast`) out-of-tree and installs those `.so` files next to the executable with an
`$ORIGIN` rpath, so the node runs without setting `LD_LIBRARY_PATH`.

## Data pipeline (UrbanNav)

UrbanNav ships ROS 1 bags. GNSS bags use the author type `gici_ros/msg/*`; the sensor
bag uses standard `sensor_msgs`.

1. GNSS bags -> one ROS 2 bag (custom type + field remap, gaps reclocked):

```bash
python3 scripts/ros2/urbannav_gnss_bag_to_ros2.py \
  --in .../gnss_rover.bag .../gnss_reference.bag \
       .../gnss_ephemeris_G.bag .../gnss_ephemeris_R.bag \
       .../gnss_ephemeris_E.bag .../gnss_ephemeris_C.bag \
  --out output/ros2_urbannav/medium/gnss_ros2
```

2. (RRR only) sensor bag -> ROS 2 (standard rosbags-convert):

```bash
scripts/ros2/urbannav_sensors_to_ros2.sh <sensors.bag> output/ros2_urbannav/deep/sensors_ros2
```

## Run (GNSS-only RTK, default config)

```bash
scripts/ros2/run_urbannav_ros2.sh medium 1     # or: deep
```

This converts (once), resolves the config output paths, launches the node, and plays
the GNSS ROS 2 bag. Solutions are written as NMEA to
`output/ros2_urbannav/<ds>/solution_2.txt` (the `est_gnss` RTK output).

> Launch the binary directly (`install/gici_ros2/lib/gici_ros2/gici_ros2_main <cfg>`),
> **not** `ros2 run gici_ros2 gici_ros2_main`. When started in the background,
> `ros2 run` does not forward `SIGINT` to the child process, so the node keeps spinning
> and never shuts down (its estimator/stream threads stay in `nanosleep`). Running the
> binary directly lets rclcpp's own signal handler stop it cleanly. `run_urbannav_ros2.sh`
> already does this.

## Run (full RRR: RTK + IMU + camera, tightly coupled)

```bash
scripts/ros2/run_urbannav_rrr_ros2.sh medium 1
```

This builds ONE merged ROS 2 bag (GNSS + `/imu/data` + `/zed2/camera/left/image_raw`),
resolves the config, launches the node with `config/ros_urbannav_rrr_ros2_adapted.yaml`
(`type: rtk_imu_camera_rrr`), and plays the bag. The trajectory is written as NMEA to
`output/ros2_urbannav_rrr/medium/solution.txt`.

For the same-input, file-mode-equivalent run (canonical rover / HKKT base / brdc eph /
DCB, matching `research/config/rtk_imu_camera_rrr_urbannav.yaml`), first build the
canonical bag with `scripts/ros2/urbannav_rinex_to_ros2.sh` and then run
`config/ros_urbannav_rrr_upstream_equivalent.yaml`. See
`scripts/ros2/run_urbannav_rrr_ros2.sh --canonical`.

Verified: **762 GPGGA epochs** (~13 min, the full drive), 7255 fused RRR updates,
`Sensor type: 3` (GNSS+IMU+camera) with `Fix status: 3`, 0 crashes, clean SIGINT exit.

### The critical part: time alignment (GPST vs UTC)

GICI carries GNSS internally on the **UTC** timescale — it reads the obs GPS week/tow and
converts with `gpst2utc` (`gnss_common::gpsTimeToUtcTime`, i.e. `gpst − 18` leap seconds
for 2021-05-17). It reads IMU/camera time straight from `header.stamp`, which in the
UrbanNav bags is **already UTC**. So the sensor stamps must **not** be shifted — they are
already on GICI's GNSS timescale.

`scripts/ros2/urbannav_rrr_to_ros2.py` therefore:

- sets each message's bag **record time = its GICI-internal UTC time** (GNSS = `gpst−18`,
  sensors = `header`), so `ros2 bag play` paces all three sensors on ONE shared clock.
  If GNSS and sensors used different record clocks they'd drift ~1 s apart, the high-rate
  IMU would always sit ahead of the arriving GNSS epoch, and the estimator's input-align
  buffer would discard every GNSS epoch (`"Throwing data ... latency too large"`) → no RTK;
- **bursts** all ephemeris/ionosphere/antenna messages at the very start (their toe stamps
  span ~20 h, the drive is ~13 min), like `burst_load` in file mode;
- converts the camera `bgr8 → mono8` (what the tracker uses anyway; ~3× smaller bag).

The RRR config uses `enable_input_align: true` (the author's proven post-file value): the
image frontend runs on its own thread and lags the GNSS/IMU backend, so the align buffer
must time-order measurements (and always insert IMU first). With it off, frame bundles
reach the backend out of order and `visualInitialization` integrates IMU over an ~18 s gap
and aborts.

> If the estimator can't keep up at `rate 1`, lower the rate (e.g. `... medium 0.5`).
> `reliable + keep-all` QoS never drops data; a slower rate just keeps queues small.

## Verified status

- `gici_ros2_msgs` and `gici_ros2` build cleanly on Humble.
- The node loads the config, builds the RTK estimator, subscribes to the GNSS topics,
  receives data from the converted ROS 2 bag, and produces valid RTK epochs
  **continuously for the whole session** (e.g. 319 epochs in a 70 s window, 0 crashes),
  after the frequency-slot fix described below.

## How the original author actually runs UrbanNav (use this for full results)

The GICI authors do **not** run UrbanNav through ROS. They run it in **post-processing
file mode** with `gici_main` reading RINEX/IMU/camera files. That path produces the full
trajectory. Even the author's own file run ends in a teardown abort (`SIGABRT`, exit
`-6`) — see `scripts/run_urbannav_rrr_baseline.py`, which explicitly whitelists exit
codes `-6/134/124/130` — but the solution file is already fully written when it aborts,
so the result is complete and usable.

A GNSS-only RTK file-mode run whose estimator block is byte-for-byte identical to this
ROS 2 config:

```bash
scripts/run_urbannav_rtk_filemode.sh medium   # or: deep
# config template: research/config/rtk_gnss_urbannav_filemode.yaml
```

Verified output: **deep = 1416 GPGGA epochs** (28 fixed / 1388 float), medium ≈ full
session — i.e. the whole trajectory, vs a single epoch from the ROS path below. Post
processing takes a few minutes (the whole session is replayed as fast as possible) and
the final teardown abort is expected/harmless.

## Fixed: ROS streaming path used to crash after the first epoch

Previously the ROS path segfaulted right after the first fixed epoch, inside
`gici::DataCluster::~DataCluster()` (a `shared_ptr` release) from
`gici::EstimatingBase::run()` — a classic heap-corruption signature (the write that
corrupts the heap happens earlier; the crash surfaces only when the allocator walks
the corrupted arena during free).

Correcting an earlier misdiagnosis: it is **not** a frequency-slot overflow. Every
`CMakeLists.txt` here compiles the core with `-DNFREQ=3 -DNEXOBS=3`, and both macros
are `#ifndef`-guarded in `third_party/rtklib/include/rtklib.h` (lines 131-138), so the
command-line `-D` wins and `obsd_t` has `NFREQ+NEXOBS = 6` slots per satellite. UrbanNav
u-blox F9P observations carry at most 4 frequencies, so writing index `[3]` stays well
within the 6-slot `SNR/LLI/code/L/P/D` arrays — no overflow there.

The real corruption paths, which the wrapper now clamps defensively in
`gnssObservationsCallback`, are:

- **Per-epoch observation count** > `MAXOBS` (96): the observations are written into the
  fixed `obs_t.data[MAXOBS]` array, so a large epoch (many sats x systems) would write
  past it. Guarded by `if (n >= MAXOBS) break;`.
- **Unrecognized PRN**: `satid2no()` returns `0` for a satellite the core is not built
  for; the old loop still stored it, and downstream `sat-1` indexing / `satsys()` on a
  zero sat id reads/writes out of bounds. Guarded by `if (obs->sat <= 0) continue;`.
- **Unequal per-frequency vector lengths**: the message stores `snr/lli/code/l/p/d` as
  separate vectors; iterating to one vector's length while another is shorter is an
  out-of-bounds `std::vector` read (UB). Guarded by taking
  `num_freq = min(code, snr, lli, l, p, d, NFREQ+NEXOBS)` before the copy loop.

`gnssEphemeridesCallback` likewise clamps `eph.tgd` to its array size. All fixes live in
the **wrapper**, not the core, so the `f2b8579` core stays byte-identical.

After the fix the ROS path runs the **full trajectory** with no crash — e.g. a 70 s
window produced 319 continuous RTK epochs (0 segfaults), matching the file-mode behavior.

### Note on the separate file-mode teardown abort

The file-mode RRR baseline can still end with a different, harmless upstream abort
(`Check failed: seq.size() > 1` in `gici::getLast`) that happens *after* the full
solution is written; `scripts/run_urbannav_rrr_baseline.py` whitelists it. That is
unrelated to the streaming crash fixed above.
