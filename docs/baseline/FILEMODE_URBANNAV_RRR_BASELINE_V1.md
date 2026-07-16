# UrbanNav File-Mode RRR Baseline — V1

Frozen algorithm baseline for thesis work: **GICI-Open post-file full RRR
(RTK + IMU + camera, tightly coupled) on UrbanNav**. All algorithm improvements
start here. ROS 2 ports mirror this config after file-mode validation.

Do **not** change estimator/calibration in a way that alters locked results
without cutting a new baseline version.

> **Deep and Medium are both locked and pass the paper comparison** (see "Locked
> results" below). Medium's paper-accuracy gap (was ~8.0 m vs 3.4 m) was root-caused
> to a too-sparse base RINEX and fixed 2026-07-16 (see notes below).

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

Template SHA256 (lock reference, `research/standard-env` branch tip — updated after
the RRR estimator tuning commit; the old `e7eb80e6...` hash matched only the retired
truncated-run Medium figure, not the current Deep lock or any full-trajectory number):

```text
research/config/rtk_imu_camera_rrr_urbannav.yaml  d8250202c7a878375fd9b33270f1b646fea286905e506f018b86967e9a55d271
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
| Base RINEX | `UrbanNav-HK-Medium-Urban-1/gnss/base/_deprecated/_official_probe/HKKT137_h02_01S_MO.rnx` (**1 s session**, not full-day `hkkt137g.rnx`) |
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

## Locked results (author `evo_ape` Sim(3), full trajectory)

| Dataset | APE position | APE rotation | Epochs | Config | Lock date | Status |
| --- | ---: | ---: | ---: | --- | --- | --- |
| Deep | **2.141 m** | **0.908°** | 15,119 | `wrapper` | 2026-07-15 | **LOCKED — PASS** |
| Medium | **2.717 m** | **1.182°** | 7,617 | `wrapper` | 2026-07-16 | **LOCKED — PASS** |

Deep re-locked on clean `f2b8579` core + wrapper config, full trajectory (the earlier
`0.335 m / 0.925°` figure was from a truncated 2,230-epoch partial run and has been
retired). Paper Table V references remain for comparison only.

**Medium reproducibility was fixed 2026-07-16.** Four independent full-trajectory runs of
the identical `wrapper` config against the identical dataset had produced translation RMSE
of 6.9 m, 7.5 m, 14.1 m, and 16.8 m respectively (~3/5 runs segfaulting) — genuine
run-to-run non-determinism, root-caused to three ASLR-sensitive undefined-behavior bugs
(not config drift, not OpenCV RANSAC, not the `bc3761c` mutex fix, which only affects the
multi-threaded ROS2 path, not this single-threaded post-file path):

- `src/stream/formator.cpp` — `nav.eph[eph.sat-1]`/`nav.geph[prn-1]` indexed without
  checking `satsys()` actually resolved the satellite; an unresolved id underflows the
  heap allocation.
- `src/fusion/gnss_imu_initializer.cpp` — `gnss_solution_measurements_` could be emptied
  by a loop and then read via `front()`/`back()` unconditionally (heap-use-after-free).
- `third_party/rtklib/src/rinex.c` — `init_rnxctr()` zeroed only 6 of `NUMSYS=7` signal-index
  slots, leaving `tobs[6]` as uninitialized memory read by `set_index()`.

All three are documented in `research/UPSTREAM_FIDELITY.md`. Post-fix (still on the old
base RINEX), 3 independent full-trajectory runs clustered within 0.49 m / 0.20° of each
other (7.74/8.23/8.16 m, 2.00/2.21/2.11°) — reproducibility was fixed, but the converged
~8.0 m was still well above paper's 3.40 m.

**Medium's paper-accuracy gap was root-caused and fixed the same day (2026-07-16).** The
diagnostic script (`scripts/diagnose_urbannav_medium_eval.py`) showed the ~8.0 m APE was
driven almost entirely by the vertical channel (`rmse_u ≈ 15 m`, mean bias ≈ 13 m), while
horizontal APE was already close to paper. RTK float/fixed rate was ruled out as the cause
(Deep is also ~99% float yet has `rmse_u ≈ 1.9 m`). Root cause: Medium's base RINEX
(`hkkt137g.rnx`) is a full-day file sampled every 30 s — too sparse for RTK
double-differencing — the same class of issue already fixed for Deep (5 s session-aligned
base instead of its full-day 30 s file). Fix: switched to a session-aligned **1 s** base
(`HKKT137_h02_01S_MO.rnx`, same format/obs-types, 02:00-02:59 covering the rover's
02:33-02:46 window). Post-fix, `rmse_u` drops to ~4.2 m and 3 independent runs give
**3.626 / 2.179 / 2.347 m** translation APE (mean **2.717 m**) and **1.254 / 1.091 / 1.200°**
rotation APE (mean **1.182°**) — all pass vs paper. See
`research/baseline/expected_urbannav_medium.json` for the full sample and
`research/AUTHOR_METHODOLOGY.md` for the diagnostic detail.

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
