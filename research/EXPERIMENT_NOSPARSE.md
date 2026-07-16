# Experiment: disable backend sparsify on live/bag ROS2 path

**Branch:** `research/ros2-live-nosparsify-experiment`

## Hypothesis

Realtime bag replay APE ~0.13 m (vs file-mode ~0.03 m) is partly caused by
`enable_backend_data_sparsify: true` dropping measurements when the backend
queue exceeds `pending_num_threshold`.

## Configs

| Template | File |
|----------|------|
| `live-nosparsify` | `ros_gici_board_live_nosparsify_rrr.yaml` |
| `bag-nosparsify` | `ros_gici_board_bag_nosparsify_rrr.yaml` |

Only change vs baseline live/bag: `enable_backend_data_sparsify: false`.

## Run

```bash
# Smoke 10× (bag runner, ~40 min)
./scripts/ros2/run_nosparsify_experiment.sh smoke10 bag

# Smoke 10× (live runner)
./scripts/ros2/run_nosparsify_experiment.sh smoke10 live

# Full 30× after smoke passes
./scripts/ros2/run_nosparsify_experiment.sh batch30 bag
```

Results: `results/stress/bag_replay_1.1_nosparsify_smoke10_bag/`

## Success criteria

Compare against `realtime_strict_v1` (baseline with sparsify):

| Metric | Baseline (sparsify on) | Target (nosparsify) |
|--------|------------------------|---------------------|
| APE pos median | 0.130 m | < 0.08 m (stretch: ~0.03 m) |
| No deadlock | 30/30 | 10/10 smoke minimum |
| sparsify_count | ~1200/run | 0 |

If smoke shows deadlock/timeouts, nosparsify is not viable at rate 1.0 without core changes.
