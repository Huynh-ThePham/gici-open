# UrbanNav File-Mode RRR Baseline — V1

Frozen algorithm baseline for thesis work: **GICI-Open post-file full RRR
(RTK + IMU + camera, tightly coupled) on UrbanNav**. All algorithm improvements
start here. ROS 2 ports mirror this config after file-mode validation.

Do **not** change estimator/calibration in a way that alters locked results
without cutting a new baseline version.

## Baseline identity

| Item | Value |
| --- | --- |
| Baseline tag | `filemode-full-rrr-v1` |
| Upstream repo | `chichengcn/gici-open` |
| Upstream commit (reference) | `f2b8579` (`Sample configurations for post estimation.`) |
| Binary | `build/gici_main` (`CMAKE_BUILD_TYPE=Release`) |
| Estimator | `rtk_imu_camera_rrr` |
| Interface | **post-file** (`type: post-file` streamers, `replay.enable: true`) |
| Freeze date | 2026-07-14 |

## Upstream derivation (author-faithful only)

Config is **not** copied from `UrbanNavDataset/.../gici_rrr/config.yaml` (wrong
GICI-board lever arms; not from upstream).

| Layer | Upstream @ `f2b8579` | Research wrapper |
| --- | --- | --- |
| I/O / stream layout | `option/post_estimation_RTK_RRR_rinex_imutext.yaml` | `research/config/rtk_imu_camera_rrr_urbannav.yaml` (`stream:`) |
| Estimator / calib | `ros_wrapper/src/gici/option/ros_urbannav.yaml` (RRR block, lines 395–518) | same file (`estimate:`) |

Template SHA256 (lock reference):

```text
research/config/rtk_imu_camera_rrr_urbannav.yaml  e7eb80e61767ee2fc5cfc2eb60ccde35961782aca7d62f7ebd09132d11ebc9d4
option/post_estimation_RTK_RRR_rinex_imutext.yaml adcccd0e0a0c48383ad5761158bd7d363d339180e22136910d67e49e3a86a947
ros_wrapper/src/gici/option/ros_urbannav.yaml      c0ac1cb8521497952c142288d9455eaa093351b4997c9d4912d2641d96c2c465
```

Machine-readable copy: `research/baseline/filemode_urbannav_rrr_v1.json`.

## Dataset inputs (session-aligned, author GNSS base)

Environment: `URBANNAV_DATA_ROOT` (default `~/Downloads/UrbanNavDataset-master`).

### Medium — `UrbanNav-HK-Medium-Urban-1`

| Input | Path (under dataset root) |
| --- | --- |
| Rover RINEX | `UrbanNav-HK-Medium-Urban-1/gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs` |
| Base RINEX | `UrbanNav-HK-Medium-Urban-1/gnss/base/hkkt137g.rnx` |
| Ephemeris | `UrbanNav-HK-Medium-Urban-1/gnss/base/brdc1370.rnx` |
| DCB | `research/dcb/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX` |
| IMU | `UrbanNav-HK-Medium-Urban-1/gici_rrr/imu.bin.txt` |
| Camera pack | `UrbanNav-HK-Medium-Urban-1/gici_rrr/camera.bin` (672×376, `buffer_length = 254464`) |

### Deep — `UrbanNav-HK-Deep-Urban-1`

| Input | Path (under dataset root) |
| --- | --- |
| Rover RINEX | `UrbanNav-HK-Deep-Urban-1/gnss/UrbanNav-HK-Deep-Urban-1.ublox.f9p.splitter.obs` |
| Base RINEX | `UrbanNav-HK-Deep-Urban-1/gnss/base/_deprecated/hkkt141g.21o` (**5 s session**, not full-day `hkkt141g.rnx`) |
| Ephemeris | `UrbanNav-HK-Deep-Urban-1/gnss/base/_deprecated/brdc_mn.rnx` |
| DCB | `research/dcb/CAS0MGXRAP_20211410000_01D_01D_DCB.BSX` |
| IMU | `UrbanNav-HK-Deep-Urban-1/gici_rrr/imu.bin.txt` |
| Camera pack | `UrbanNav-HK-Deep-Urban-1/gici_rrr/camera.bin` |

## Reproduction

### Build

```bash
./scripts/check_research_env.sh
./scripts/build_research.sh          # cmake Release → build/gici_main
./scripts/build_eval_tools.sh
./scripts/verify_upstream_fidelity.sh   # must be OK on a clean f2b8579 tree
```

### Run (wrapper config — **canonical**)

```bash
python3 scripts/run_urbannav_rrr_baseline.py medium --config-source wrapper
python3 scripts/run_urbannav_rrr_baseline.py deep   --config-source wrapper
```

`--config-source wrapper` is the default. Do **not** use `--config-source dataset`
for new work (that reads `gici_rrr/config.yaml` from the dataset download).

### Evaluate (author APE, README §4 + UrbanNav adaptations)

```bash
./scripts/run_author_eval_urbannav.sh medium
./scripts/run_author_eval_urbannav.sh deep
```

Pass criteria: `research/baseline/expected_urbannav_{medium,deep}.json`.

### Paths (relative to repo root)

| Item | Path |
| --- | --- |
| Config template (source of truth) | `research/config/rtk_imu_camera_rrr_urbannav.yaml` |
| Resolved config at run time | `results/baseline/urbannav/<ds>/config.yaml` |
| Trajectory (NMEA, 10 Hz internal) | `results/baseline/urbannav/<ds>/output/solution.txt` |
| Run log | `results/baseline/urbannav/<ds>/run.log` |
| Author APE output | `results/baseline/urbannav/<ds>/evaluation/ape_metrics.json` |

## Locked results (author `evo_ape` Sim(3))

| Dataset | APE position | APE rotation | Config | Lock date |
| --- | ---: | ---: | --- | --- |
| Medium | **3.214 m** | **6.325°** | `wrapper` | 2026-07-14 |
| Deep | **0.335 m** | **0.925°** | `wrapper` | 2026-07-14 |

Both re-locked on clean `f2b8579` core + wrapper config. Paper Table V references
remain for comparison only.

## Operational notes

- **Exit codes:** `gici_main` may exit `-6` / `134` (upstream teardown `CHECK`) after
  the solution file is fully written. Whitelisted in `run_urbannav_rrr_baseline.py`.
- **Output rate:** estimator writes ~10 Hz (`output_align_tag: str_imu_file`,
  `output_downsample_rate: 40`). Author eval interpolates GT to solution rate.
- **Replay:** `replay.enable: true`, `speed: 1.0` — pseudo-real-time post-file.
- **Invalid config:** `UrbanNavDataset/.../gici_rrr/config.yaml` — do not use.

## Relationship to ROS 2

| Baseline | Role |
| --- | --- |
| **This file-mode V1** | Algorithm source of truth |
| `ros_urbannav_rrr_upstream_equivalent.yaml` | ROS 2 I/O mirror of this config (same RINEX/DCB inputs via canonical bag) |
| `ROS2_URBANNAV_RRR_BASELINE_V1.md` | ROS 2 adapted path (different GNSS base); **not** file-mode equivalent |

Workflow: improve here → eval pass → sync `estimate:` block to ROS 2 canonical config
→ verify parity separately.

## Baseline files (frozen)

```text
research/config/rtk_imu_camera_rrr_urbannav.yaml   # config template
research/baseline/filemode_urbannav_rrr_v1.json    # manifest (hashes, inputs)
research/baseline/expected_urbannav_medium.json    # pass tolerances
research/baseline/expected_urbannav_deep.json
scripts/run_urbannav_rrr_baseline.py               # run wrapper
scripts/run_author_eval_urbannav.sh                # author APE
docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md # this document
```

Tag: `baseline-full-rrr-filemode-v1`.
