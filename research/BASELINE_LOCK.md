# Research Baseline Lock

> **Status: LOCKED** — GICI core **identical** to `chichengcn/gici-open` @ `f2b8579` (zero source delta).

Branch `research/standard-env` = upstream GICI @ `f2b8579` + research wrappers only (`research/`, `scripts/`).

See `research/AUTHOR_METHODOLOGY.md`, `research/UPSTREAM_FIDELITY.md`.

```bash
./scripts/verify_upstream_fidelity.sh   # must print OK
```

## Upstream identity

| Field | Value |
|-------|-------|
| Author repo | `chichengcn/gici-open` |
| Locked commit | `f2b8579` |
| Marker commit | `8adde32` |
| Source delta | **0 files** in `include/`, `src/`, `tools/evaluation/`, `option/` |

## GICI dataset 1.1 — LOCKED

| Setting | Value |
|---------|-------|
| Template | `option/post_estimation_RTK_RRR.yaml` |
| Wrapper | `research/config/rtk_imu_camera_rrr_1_1.yaml` |
| Run + eval | `./scripts/run_author_eval_1_1.sh` |

| Metric | Locked | Paper Table V |
|--------|--------|---------------|
| APE position | 0.029 m | 0.03 m |
| APE rotation | 0.471° | 0.54° |

## UrbanNav — author-derived wrappers

Templates: `post_estimation_RTK_RRR_rinex_imutext.yaml` + `ros_urbannav.yaml` RRR block.
**Do not use** `UrbanNavDataset/.../gici_rrr/config.yaml`.

```bash
python3 scripts/run_urbannav_rrr_baseline.py deep
./scripts/run_author_eval_urbannav.sh deep
```

| Dataset | Author `evo_ape` (locked) | Paper |
|---------|---------------------------|-------|
| Deep | 2.123 m / 0.946° | 2.46 m / 1.64° |

Medium: re-run batch when RINEX post-file is stable; eval expects `results/baseline/urbannav/medium/output/solution.txt`.

## Quick start

```bash
./scripts/check_research_env.sh
./scripts/build_research.sh
./scripts/build_eval_tools.sh
./scripts/verify_upstream_fidelity.sh
./scripts/run_author_eval_1_1.sh
```
