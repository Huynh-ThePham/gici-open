# GICI Research Environment

Branch `research/standard-env` = **chichengcn/gici-open @ f2b8579** (identical core) + wrappers.

- [`BASELINE_LOCK.md`](BASELINE_LOCK.md) — locked metrics and commands
- [`AUTHOR_METHODOLOGY.md`](AUTHOR_METHODOLOGY.md) — what is official vs derived
- [`UPSTREAM_FIDELITY.md`](UPSTREAM_FIDELITY.md) — zero source delta policy

## Layout

| Path | Role |
|------|------|
| `option/post_estimation_RTK_RRR.yaml` | Author template — GICI board 1.1 |
| `option/post_estimation_RTK_RRR_rinex_imutext.yaml` | Author template — RINEX + imu-text |
| `research/config/` | Filled templates with local paths |
| `research/baseline/expected_*.json` | Reproduce tolerances |
| `scripts/run_author_eval_1_1.sh` | Author APE (README §4) |
| `scripts/run_urbannav_rrr_baseline.py` | UrbanNav batch (author templates only) |
| `scripts/run_author_eval_urbannav.sh` | UrbanNav author APE |

## Typical workflow

```bash
cd /home/theph/ws_ncs/gici_research_standard
./scripts/check_research_env.sh
./scripts/build_research.sh
./scripts/build_eval_tools.sh
./scripts/run_author_eval_1_1.sh
python3 scripts/run_urbannav_rrr_baseline.py medium
./scripts/run_author_eval_urbannav.sh medium
```

Environment variables:

- `GICI_DATASET_1_1` — default `/home/theph/ws_ncs/1.1`
- `URBANNAV_DATA_ROOT` — default `~/Downloads/UrbanNavDataset-master`

Keep datasets under `data/`; outputs under `results/` (gitignored).
