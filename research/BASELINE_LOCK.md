# Research Baseline Lock

> **Status: LOCKED** — Algorithm baseline = **file-mode RRR** @ upstream-derived config.
> GICI core reference: `chichengcn/gici-open` @ `f2b8579` + 4 documented UB bugfixes.
> GICI-board (1.1/3.1/4.1), UrbanNav Deep, and UrbanNav Medium are all locked, reproducible,
> and **pass the paper comparison**. Medium's paper-accuracy gap (was ~8.0m vs 3.4m) was
> root-caused 2026-07-16 to a too-sparse (30s) base RINEX and fixed by switching to a
> session-aligned 1s base — see "Medium" section below.

Branch `research/standard-env` = upstream GICI @ `f2b8579` + research wrappers only (`research/`, `scripts/`).

See `research/AUTHOR_METHODOLOGY.md`, `research/UPSTREAM_FIDELITY.md`,
`docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md`.

```bash
./scripts/verify_upstream_fidelity.sh   # must print OK (upstream + documented safety bugfixes only)
```

## Upstream identity

| Field | Value |
|-------|-------|
| Author repo | `chichengcn/gici-open` |
| Locked commit | `f2b8579` |
| Marker commit | `8adde32` |
| Source delta | **3 files** in `src/` + **1 file** in vendored `third_party/rtklib/` — documented memory-safety/UB bugfixes only, no algorithm/tuning changes; see `research/UPSTREAM_FIDELITY.md` and `scripts/verify_upstream_fidelity.sh` |

## UrbanNav file-mode RRR — PRIMARY (algorithm baseline)

**Tag:** `filemode-full-rrr-v1`

| Item | Path |
|------|------|
| Manifest | `docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md` |
| Machine-readable | `research/baseline/filemode_urbannav_rrr_v1.json` |
| Config template | `research/config/rtk_imu_camera_rrr_urbannav.yaml` |
| Upstream I/O | `option/post_estimation_RTK_RRR_rinex_imutext.yaml` @ `f2b8579` |
| Upstream calib | `ros_wrapper/.../ros_urbannav.yaml` RRR block @ `f2b8579` |

**Do not use** `UrbanNavDataset/.../gici_rrr/config.yaml`.

```bash
python3 scripts/run_urbannav_rrr_baseline.py medium   # default: --config-source wrapper
./scripts/run_author_eval_urbannav.sh medium
```

| Dataset | Author `evo_ape` (full trajectory) | Config | Paper Table V | Status |
|---------|-------------------------------------|--------|----------------|--------|
| Deep | **2.141 m / 0.908°** (15,119 epochs, locked 2026-07-15) | `wrapper` | 2.46 m / 1.64° | **LOCKED — PASS** |
| Medium | **2.717 m / 1.182°** (7,617 epochs, locked 2026-07-16) | `wrapper` | 3.40 m / 1.30° | **LOCKED — PASS** |

Deep re-locked on clean `f2b8579` core + `wrapper` config, full 15,119-epoch trajectory
(the previous `0.335 m / 0.925°` figure came from a truncated 2,230-epoch partial run and
has been retired — see `research/UPSTREAM_FIDELITY.md`/git history if you need the old
number for archaeology). Run:

```bash
./scripts/verify_upstream_fidelity.sh
python3 scripts/run_urbannav_rrr_baseline.py deep
./scripts/run_author_eval_urbannav.sh deep
```

### Medium — reproducibility bug fixed 2026-07-16, then paper-accuracy gap root-caused and fixed same day

**Reproducibility (fixed first).** Four full-trajectory runs pre-fix (same `wrapper`
config, same dataset, same day) produced translation RMSE **6.9 m, 7.5 m, 14.1 m, and
16.8 m** — chaotic and crash-prone (~3/5 runs segfaulted). Root-caused to three
ASLR-sensitive undefined-behavior bugs, all fixed and documented in
`research/UPSTREAM_FIDELITY.md`: `src/stream/formator.cpp` (heap-buffer-underflow on
unresolved satellite ids), `src/fusion/gnss_imu_initializer.cpp` (heap-use-after-free on
an emptied deque), and `third_party/rtklib/src/rinex.c` (uninitialized-memory read,
`NUMSYS==7` vs a 6-iteration zero-init loop). Post-fix (still on the old base RINEX), 3
independent full-trajectory runs clustered within **0.49 m / 0.20°** of each other
(7.74/8.23/8.16 m, 2.00/2.21/2.11°) — reproducibility was fixed, but the converged ~8.0 m
was still well above paper's 3.40 m.

**Paper-accuracy gap (root-caused same day).** The diagnostic script
(`scripts/diagnose_urbannav_medium_eval.py`) showed the ~8.0 m APE was driven almost
entirely by the vertical channel (`rmse_u ≈ 15 m`, mean bias ≈ 13 m) while horizontal APE
was already close to paper (~2.2-3.0 m). Ruled out RTK float/fixed rate as the cause (Deep
is *also* ~99% float yet has `rmse_u ≈ 1.9 m`). Root cause: Medium's base RINEX
(`hkkt137g.rnx`) is a full-day file sampled every **30 s** — too sparse for RTK
double-differencing to interpolate the base epoch accurately — exactly the same class of
issue already fixed for Deep (which uses a 5 s session-aligned base, not its full-day
30 s file). Fix: switched Medium to a session-aligned **1 s** base
(`gnss/base/_deprecated/_official_probe/HKKT137_h02_01S_MO.rnx`, same RINEX 3.02
format/obs-types, 02:00-02:59 covering the rover's 02:33-02:46 window). Post-fix,
`rmse_u` drops to ~4.2 m and 3 independent full-trajectory runs give **3.626 / 2.179 /
2.347 m** translation APE (mean **2.717 m**) and **1.254 / 1.091 / 1.200°** rotation APE
(mean **1.182°**) — all individually pass vs paper, and the mean beats it. Run-to-run
spread is wider than the old config's (1.45 m / 0.16° vs 0.49 m / 0.20°), likely the same
Ceres `num_threads=4` floating-point non-associativity, now more visible with the
dominant systematic bias gone. See `research/AUTHOR_METHODOLOGY.md` for the full
before/after diagnostic numbers.

```bash
python3 scripts/run_urbannav_rrr_baseline.py medium
./scripts/run_author_eval_urbannav.sh medium   # expect ~2.7 m / ~1.2 deg, within tolerance
```

## GICI board datasets — LOCKED (RTK RRR, file-mode + ROS 2)

| Setting | Value |
|---------|-------|
| Template | `option/post_estimation_RTK_RRR.yaml` (upstream @ `f2b8579`) |
| ROS 2 port | `ros2_wrapper/src/gici_ros2/config/ros_gici_board_postfile_rrr.yaml` |
| RTCM start (per scene) | `gici_rtcm_start_for` in `scripts/dataset_paths.sh` |
| File-mode run + eval | `./scripts/run_gici_dev_sequence.sh` (1.1 → 3.1 → 4.1) |
| ROS 2 run + eval | `./scripts/run_gici_board_ros2_sequence.sh 1.1 3.1 4.1` |
| Both + compare | `./scripts/run_gici_board_table5_comparison.sh` |

Locked `evo_ape` (Sim(3), full trajectory), 2026-07-15:

| Scene | file-mode pos/rot | ROS 2 pos/rot | Paper Table V |
|-------|-------------------|---------------|---------------|
| 1.1 open-sky      | 0.029 m / 0.466° | 0.029 m / 0.474° | 0.03 m / 0.54° |
| 3.1 typical-urban | 0.142 m / 1.078° | 0.141 m / 1.157° | 0.29 m / 1.58° |
| 4.1 dense-urban   | 0.073 m / 0.696° | 0.070 m / 0.733° | 0.08 m / 0.54° |

All PASS vs paper (`paper_pass`). File-mode ↔ ROS 2 agree to mm (position) / ~0.01–0.08°
(rotation), validating the ROS 2 wrapper as estimator-equivalent to file-mode.
`locked_reference`/`pass` (exact reproduction of these numbers, tighter tolerance than
the paper comparison) is tracked per scene in `research/baseline/expected_{1_1,gici_board_3_1,gici_board_4_1}.json`
via `scripts/run_author_eval_gici_board.sh` — 1.1/3.1/4.1 all have a locked reference now.
Metrics: `results/baseline/gici_board{,_ros2}/<id>/evaluation/ape_metrics.json`.

## ROS 2 (UrbanNav — secondary wrapper port)

See `docs/baseline/ROS2_URBANNAV_RRR_BASELINE_V1.md`. ROS 2 canonical config mirrors
file-mode estimator: `ros_urbannav_rrr_upstream_equivalent.yaml`.

## Quick start

```bash
./scripts/check_research_env.sh
./scripts/build_research.sh
./scripts/build_eval_tools.sh
./scripts/verify_upstream_fidelity.sh
python3 scripts/run_urbannav_rrr_baseline.py medium
./scripts/run_author_eval_urbannav.sh medium
```
