# GICI Open — Author methodology audit (@ f2b8579)

Upstream: `chichengcn/gici-open` commit `f2b8579` ("Sample configurations for post estimation.")

This document records what the **original authors ship** versus what this research repo adds.
Use it before changing baseline scripts or pass criteria.

## Official author artifacts @ f2b8579

| Item | Present? | Location |
|------|----------|----------|
| C++ eval tools (`ie_to_nmea`, `nmea_to_tum`, `nmea_align_timestamp`, `nmea_pose_to_pose`, …) | **Yes** | `tools/evaluation/` |
| Dataset README §4 evaluation recipe | **Yes** | `gici-open-dataset` (GICI board datasets with `ground_truth.txt` IE format) |
| `run_evaluation.py` (Python ATE / GPGGA-only) | **No** | Added later in forks; **not** upstream @ f2b8579 |
| `run_baseline_1_1.sh` / UrbanNav baseline scripts | **No** | Research scaffolding only |
| UrbanNav post-file RRR config | **Partial** | `ros_wrapper/.../ros_urbannav.yaml` (ROS template; RRR block is commented) |
| UrbanNav evaluation script | **No** | Must be derived from README §4 + paper Table V |

## Canonical author evaluation (dataset README §4)

Documented and verified on GICI dataset **1.1**:

```text
ie_to_nmea ground_truth.txt
nmea_pose_to_pose ground_truth.txt.nmea          # fiber-IMU → body (GICI board GT only)
nmea_align_timestamp GT.nmea.transformed solution.txt
nmea_to_tum solution.txt
nmea_to_tum GT.nmea.transformed.aligned
evo_ape tum GT.tum EST.tum -va --align --correct_scale
evo_ape tum GT.tum EST.tum -va --align --correct_scale --pose_relation angle_deg
```

**Locked reproduce (2026-07-11):** position RMSE **0.029 m**, rotation RMSE **0.471°** (paper Table V: 0.03 m / 0.54°).

```bash
./scripts/run_author_eval_1_1.sh
```

### What is *not* the paper APE

- `run_evaluation.py` horizontal RMSE (~0.315 m on 1.1): GPGGA-only, no full-pose `evo_ape`.
- Do **not** use it as Table V metric.

## UrbanNav — author-faithful derivation

Authors do **not** publish an UrbanNav eval script @ f2b8579. Paper Table V reports APE (m / deg) on UrbanNav; we apply README §4 with these **documented adaptations**:

| Step | GICI 1.1 (official) | UrbanNav adaptation | Reason |
|------|---------------------|---------------------|--------|
| Ground truth input | `ground_truth.txt` (IE, fiber IMU) | `urbannav_gt_to_ie.py` on TST / Whampoa raw | UrbanNav GT is not IE format |
| Frame transform | `nmea_pose_to_pose` | **Skip** | TST GT is already body (= IMU). Fiber extrinsics break attitude ~180° |
| Timestamp align | `nmea_align_timestamp` high-rate GT → solution | `urbannav_interp_gt_to_solution.py` then `nmea_align_timestamp` | Tool requires 1st file ≥ 2nd rate; GT is 1 Hz, solution ~10 Hz |
| TUM conversion | upstream `nmea_to_tum` | Same (Shanghai ref; `evo_ape --align` handles frame) | README §4 |
| Compare | `evo_ape` Sim(3) | Same | Paper Table V APE |

```bash
./scripts/run_author_eval_urbannav.sh medium
./scripts/run_author_eval_urbannav.sh deep
```

Pass criteria match `run_author_eval_1_1.sh`: reproduce locked `evo_ape` values within tolerance in `research/baseline/expected_urbannav_*.json`. **No** `tst_rmse_h` substitution for medium.

Supplementary smoke metric (not author APE): horizontal `rmse_h` from `scripts/run_urbannav_rrr_baseline.py` — useful for quick checks only.

## UrbanNav run config — author templates only

Use the author's **two** upstream references:

1. **Post-file I/O:** `option/post_estimation_RTK_RRR_rinex_imutext.yaml`
2. **UrbanNav calib / RRR tuning:** `ros_wrapper/src/gici/option/ros_urbannav.yaml` (commented RRR block)

Research wrapper: `research/config/rtk_imu_camera_rrr_urbannav.yaml`.

**Locked manifest:** `docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md` (tag `filemode-full-rrr-v1`).

**Ignore** `UrbanNavDataset/.../gici_rrr/config.yaml` in the dataset download — it is not from `chichengcn/gici-open` and uses wrong GICI-board lever arms.

- `gnss_extrinsics: [0.0, -0.43, -0.16]`
- `body_to_imu_rotation: [0.0, 0.0, 0.0]`
- ZED2 intrinsics / `T_B_C` as in `ros_urbannav.yaml`
- `max_keyframes: 20`, AR on, `output_downsample_rate: 40` (post-file rinex template)

### GNSS base stations (author session alignment)

| Dataset | Rover obs | Base (author) | Ephemeris |
|---------|-----------|---------------|-----------|
| Medium | `UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs` | **`_deprecated/_official_probe/HKKT137_h02_01S_MO.rnx`** (1 s session) | `brdc1370.rnx` |
| Deep | `UrbanNav-HK-Deep-Urban-1.ublox.f9p.splitter.obs` | **`hkkt141g.21o`** (5 s session) | `brdc_mn.rnx` |

Both datasets need a **session-aligned, high-rate** base — the full-day, 30 s files
(`hkkt137g.rnx` for Medium, `hkkt141g.rnx` for Deep) are both too sparse for RTK
double-differencing and were retired: Deep with full-day `hkkt141g.rnx` → ~13 m
horizontal error; Medium with full-day `hkkt137g.rnx` → ~13 m mean **vertical** bias
(see "Known UrbanNav eval gaps" below — root-caused 2026-07-16, four days after the
Deep fix was first documented, since the same fix was never propagated to Medium).

## Known UrbanNav eval gaps vs paper Table V — historical (fixed 2026-07-16)

This section is retained for archaeology; both gaps below are now closed — see
`research/BASELINE_LOCK.md` for current locked numbers.

| Dataset | Author `evo_ape` pos / rot (old, full-day base) | Paper Table V | `rmse_h` smoke |
|---------|---------------------------|---------------|----------------|
| Medium | 8.04 m / 2.11° (also 4.55 m / 2.06° on an earlier partial run) | 3.40 m / 1.30° | **3.02 m** ≈ paper pos |
| Deep | 2.12 m / 0.95° | 2.46 m / 1.64° | **2.47 m** ≈ paper pos |

Medium 3D APE inflation (diagnosed via `scripts/diagnose_urbannav_medium_eval.py`): ~13 m
mean vertical bias between solution GPGGA ellipsoid height and TST `H-Ell`
(`rmse_u ≈ 15 m`). Sim(3) couples vertical error into translation APE. Deep has low
vertical error (`rmse_u ≈ 1.9 m`); author `evo_ape` aligns with paper.

**Root cause and fix (2026-07-16):** RTK float/fixed rate was ruled out first — Deep is
*also* ~99% float (`fixed_rate ≈ 0.009`) yet has 8x lower vertical RMSE than Medium, so
ambiguity resolution status doesn't explain the gap. The actual cause: Medium's base
RINEX (`hkkt137g.rnx`) is sampled every 30 s over the full day — too sparse for RTK
double-differencing to interpolate the base epoch accurately at the rover's update rate.
This is the *same class* of bug already fixed for Deep (full-day 30 s → session-aligned
5 s), just never applied to Medium. Fix: switched Medium to a session-aligned **1 s**
base (`HKKT137_h02_01S_MO.rnx`, identical RINEX 3.02 format/obs-types as the retired
file, covering GPS time 02:00-02:59 which brackets the rover's 02:33-02:46 window).

Post-fix diagnostics (3 independent full-trajectory runs):

| Metric | Before (30 s base) | After (1 s base) |
|--------|--------------------|--------------------|
| `tst_rmse_u_m` | 15.05 | 4.23 |
| `tst_u_bias_mean_m` | 13.02 | 2.12 |
| `evo_ape` translation RMSE | 8.042 m | 2.717 m (mean of 3.626/2.179/2.347) |
| `evo_ape` rotation RMSE | 2.108° | 1.182° (mean of 1.254/1.091/1.200) |
| vs paper (3.40 m / 1.30°) | fail | **pass** |

Run-to-run spread on the new base (1.45 m / 0.16°) is wider than the old config's
(0.49 m / 0.20°) — likely the same Ceres `num_threads=4` floating-point
non-associativity noted elsewhere in this doc, now more visible since the dominant
systematic vertical bias is gone. See `research/baseline/expected_urbannav_medium.json`
for the full sample.

`scripts/diagnose_urbannav_medium_eval.py` documents the vertical-bias diagnosis; it does
**not** define pass/fail — see `research/baseline/expected_urbannav_medium.json` for that.

## Upstream fidelity

Source delta from `f2b8579` is **4 documented memory-safety/UB bugfixes only** (no
algorithm/tuning changes) — see `research/UPSTREAM_FIDELITY.md` and
`./scripts/verify_upstream_fidelity.sh`.
