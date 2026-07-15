# GICI Research Environment

## Branch policy

| Branch | Role | Core vs `f2b8579` |
|--------|------|-------------------|
| **`research/standard-env`** (this branch) | **Upstream-clean reference** — file-mode + ROS2 **postfile**; locked Table V baselines | **0 delta** — run `./scripts/verify_upstream_fidelity.sh` |
| **`research/ros2-realtime-fix`** | **Runtime baseline** — real-time bag replay; tag `realtime-safe-v1` | 1 core mutex patch + wrapper/infra — **do not merge into this branch** |

Real-time bag replay, stress tests, and hybrid bag configs live only on `research/ros2-realtime-fix`.

---

Branch `research/standard-env` = **chichengcn/gici-open @ f2b8579** (identical core) + research wrappers/scripts.

- [`BASELINE_LOCK.md`](BASELINE_LOCK.md) — locked metrics and commands
- [`docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md`](../docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md) — **primary** UrbanNav RRR baseline
- [`AUTHOR_METHODOLOGY.md`](AUTHOR_METHODOLOGY.md) — what is official vs derived
- [`UPSTREAM_FIDELITY.md`](UPSTREAM_FIDELITY.md) — zero source delta policy

## Layout

| Path | Role |
|------|------|
| `option/post_estimation_RTK_RRR.yaml` | Author template — GICI board 1.1 |
| `option/post_estimation_RTK_RRR_rinex_imutext.yaml` | Author template — RINEX + imu-text I/O |
| `research/config/rtk_imu_camera_rrr_urbannav.yaml` | **File-mode RRR V1** (UrbanNav wrapper) |
| `research/baseline/filemode_urbannav_rrr_v1.json` | Machine-readable baseline manifest |
| `research/baseline/expected_*.json` | Reproduce tolerances |
| `scripts/run_author_eval_1_1.sh` | Author APE (README §4) |
| `scripts/run_urbannav_rrr_baseline.py` | UrbanNav file-mode RRR run |
| `scripts/run_author_eval_urbannav.sh` | UrbanNav author APE |

## Dataset layout (`/media/theph/Data1/Research/dataset`)

```bash
./scripts/setup_gici_datasets.sh          # extracts 1.1 + 1.1-rinex-imutext
# other scenes: unzip 1.2.zip, 2.1.zip, ... into the same root
```

| Path | Use |
|------|-----|
| `1.1/` | GICI board open-sky — **algo dev / locked baseline** (`ground_truth.txt`, `*.bin`) |
| `1.1-rinex-imutext/` | Same scene, RINEX + `imu.bin.txt` post-file I/O |
| `UrbanNavDataset/` | UrbanNav (optional; extract separately when needed) |

## Typical workflow — GICI 1.1 (algorithm development)

```bash
cd /home/theph/ws_ncs/gici_research_standard
./scripts/setup_gici_datasets.sh
./scripts/check_research_env.sh
./scripts/build_research.sh
./scripts/build_eval_tools.sh
./scripts/run_author_eval_1_1.sh    # RTK/IMU/Camera RRR + author APE
```

## GICI board baseline — RTK RRR, 1.1 → 3.1 → 4.1 (file-mode + ROS2)

Upstream `option/post_estimation_RTK_RRR.yaml` (file-mode) and its ROS 2 port
give the reproduction of Table V (RTK RRR) across scenes. RTCM stream
`start_time` per scene is auto-resolved by `gici_rtcm_start_for` in
`scripts/dataset_paths.sh` (single source of truth — must equal the dataset
collection date or the RTCM decoder stalls on "Waiting for ephemeris").

```bash
./scripts/setup_gici_datasets.sh 1.1 3.1 4.1   # extract (skips if present)

# File-mode sequence (author post-file pipeline) + author APE:
./scripts/run_gici_dev_sequence.sh              # 1.1 -> 3.1 -> 4.1

# ROS 2 sequence (post-file port of the same estimator) + author APE + compare:
./scripts/run_gici_board_ros2_sequence.sh 1.1 3.1 4.1

# Both pipelines side by side (file-mode + ROS2 + paper):
./scripts/run_gici_board_table5_comparison.sh   # [--file-only|--ros2-only|--skip-run]
```

Locked reproduction (evo_ape, Sim(3), full trajectory):

| Scene | file-mode pos/rot | ROS2 pos/rot | Paper Table V |
|-------|-------------------|--------------|---------------|
| 1.1 open-sky   | 0.029 m / 0.466° | 0.029 m / 0.479° | 0.03 m / 0.54° |
| 3.1 typical-urban | 0.142 m / 1.078° | 0.141 m / 1.157° | 0.29 m / 1.58° |
| 4.1 dense-urban | 0.073 m / 0.696° | 0.070 m / 0.733° | 0.08 m / 0.54° |

Results: `results/baseline/gici_board/<id>/` (file-mode),
`results/baseline/gici_board_ros2/<id>/` (ROS2); metrics in
`evaluation/ape_metrics.json`.

## Typical workflow — UrbanNav RRR (later)

```bash
python3 scripts/run_urbannav_rrr_baseline.py medium
./scripts/run_author_eval_urbannav.sh medium
```

Environment variables (see `scripts/dataset_paths.sh`):

- `GICI_DATA_ROOT` — default `/media/theph/Data1/Research/dataset`
- `GICI_DATASET_1_1` — default `${GICI_DATA_ROOT}/1.1`
- `URBANNAV_DATA_ROOT` — default `${GICI_DATA_ROOT}/UrbanNavDataset`
- `GICI_MAIN` — override path to `gici_main` binary

Keep datasets under `data/`; outputs under `results/` (gitignored).
