# Research Baseline Lock

> **Status: LOCKED** — Algorithm baseline = **file-mode RRR** @ upstream-derived config.
> GICI core reference: `chichengcn/gici-open` @ `f2b8579`.

Branch `research/standard-env` = upstream GICI @ `f2b8579` + research wrappers only (`research/`, `scripts/`).

**Sibling branch:** `research/ros2-realtime-fix` = runtime bag replay (tag `realtime-safe-v1`); contains one core mutex patch — keep out of `standard-env`.

See `research/AUTHOR_METHODOLOGY.md`, `research/UPSTREAM_FIDELITY.md`,
`docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md`.

```bash
./scripts/verify_upstream_fidelity.sh   # must print OK (zero core delta vs f2b8579)
```

## Upstream identity

| Field | Value |
|-------|-------|
| Author repo | `chichengcn/gici-open` |
| Locked commit | `f2b8579` |
| Marker commit | `8adde32` |
| Source delta | **0 files** in `include/`, `src/`, `tools/evaluation/`, `option/` |

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

| Dataset | Author `evo_ape` (locked 2026-07-14) | Config | Paper Table V |
|---------|--------------------------------------|--------|---------------|
| Medium | **3.214 m / 6.325°** | `wrapper` | 3.40 m / 1.30° |
| Deep | **0.335 m / 0.925°** | `wrapper` | 2.46 m / 1.64° |

Re-locked on clean `f2b8579` core + `wrapper` config. Run:

```bash
./scripts/verify_upstream_fidelity.sh
python3 scripts/run_urbannav_rrr_baseline.py medium
./scripts/run_author_eval_urbannav.sh medium
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
| 1.1 open-sky      | 0.029 m / 0.466° | 0.029 m / 0.479° | 0.03 m / 0.54° |
| 3.1 typical-urban | 0.142 m / 1.078° | 0.141 m / 1.157° | 0.29 m / 1.58° |
| 4.1 dense-urban   | 0.073 m / 0.696° | 0.070 m / 0.733° | 0.08 m / 0.54° |

All PASS vs paper. File-mode ↔ ROS 2 agree to mm (position) / ~0.01–0.08° (rotation),
validating the ROS 2 wrapper as estimator-equivalent to file-mode.
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
