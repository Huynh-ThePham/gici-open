# Research Baseline Lock

> **Status: PROVISIONAL** — GICI 1.1 PASS; UrbanNav Medium + Deep chưa verify trên
> upstream thuần. Chưa bump tag `research-baseline/v2` cho đến khi đủ 3 smoke tests.

Branch `research/standard-env` is the **candidate upstream baseline** for all
sensor-fusion and adaptive-sensor research in this repository.

## Upstream identity

| Field | Value |
|-------|-------|
| Author repo | `chichengcn/gici-open` |
| Locked commit | `f2b8579` ("Sample configurations for post estimation.") |
| Fork remote | `Huynh-ThePham/gici-open` |
| Marker commit | `8adde32` (`baseline gốc: upstream chichengcn/gici-open @ f2b8579`) |

No NV2 / adaptive / reliability patches on this branch. Research branches must
fork from here and keep baseline behaviour reproducible before adding new logic.

## Locked fusion protocol (dataset 1.1 smoke test)

| Setting | Value |
|---------|-------|
| Estimator | `rtk_imu_camera_rrr` |
| Config template | `research/config/rtk_imu_camera_rrr_1_1.yaml` |
| AR | on |
| Replay | on, speed 1.0 |
| Output downsample | 40 (align IMU) |
| max_keyframes | 3 |
| Lever arm (body) | `[-0.035, 0.354, -0.042]` m |

## Author evaluation pipeline (paper Table V metric)

Do **not** use GPGGA-only horizontal RMSE as the paper APE. Use:

1. `ie_to_nmea` on `ground_truth.txt`
2. `nmea_pose_to_pose` (fiber-IMU → body frame)
3. `nmea_align_timestamp` (GT → solution rate)
4. `nmea_to_tum` on solution and aligned GT
5. `evo_ape` with Sim(3) alignment

```bash
./scripts/run_author_eval_1_1.sh
```

Expected metrics: `research/baseline/expected_1_1.json`

| Metric | Locked value | Paper |
|--------|--------------|-------|
| APE position RMSE | 0.029 m | 0.03 m |
| APE rotation RMSE | 0.471° | 0.54° |

## Pending: UrbanNav upstream smoke tests

Chưa chốt gốc vì thiếu reproduce trên upstream thuần (không patch NV2):

| Dataset | Local path | Paper RTK RRR (pos / rot) | Status |
|---------|------------|----------------------------|--------|
| UrbanNav Medium | `UrbanNavDataset-master/UrbanNav-HK-Medium-Urban-1` | 3.40 m / 1.30° | **TODO** |
| UrbanNav Deep | `UrbanNavDataset-master/UrbanNav-HK-Deep-Urban-1` | 2.46 m / 1.64° | **TODO** |

Cần trước khi chốt `research-baseline/v2`:

1. Config RRR upstream-only cho Medium + Deep (RINEX + `gici_rrr/` converted bins).
2. Calibration / lever-arm từ `extrinsic.yaml` (khác GICI board 1.1).
3. Eval pipeline tương đương (TST GT, body frame, APE evo).
4. Ghi `research/baseline/expected_urbannav_medium.json` và `expected_urbannav_deep.json`.

Fork `gici-open` (paper1) đã có config calibrated — **chưa** chứng minh trên
`gici_research_standard` / upstream thuần @ `f2b8579`.

## Branch policy for fusion / adaptive research

```
research/standard-env          ← locked upstream baseline (this branch)
  ├── paper1/adaptive-measurement-weighting
  ├── paper2/adaptive-ambiguity-modeling
  ├── paper2_vio_adaptive
  ├── paper3/gnss-fault-vi-fallback
  ├── paper4/ai-sensor-reliability
  ├── paper5/reliability-aware-outdoor-slam
  └── paper6/autonomous-navigation-system
```

Rules:

1. New research branches start from `research/standard-env`, not from `master`.
2. Baseline configs live under `research/config/`; experiment configs under branch-specific paths.
3. Every adaptive/fusion change must pass smoke tests: **1.1 + UrbanNav Medium + Deep** (khi đã có expected JSON).
4. Compare against `research/baseline/expected_*.json`, not ad-hoc metrics.

## Quick start

```bash
cd /home/theph/ws_ncs/gici_research_standard
./scripts/check_research_env.sh
./scripts/build_research.sh
./scripts/build_eval_tools.sh
./scripts/run_author_eval_1_1.sh
```

Outputs go to `results/baseline/1_1/` (gitignored).
