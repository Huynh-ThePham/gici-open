# GICI Research Environment

Branch `research/standard-env` is the **locked upstream baseline** for sensor-fusion
and adaptive-sensor research. See [`BASELINE_LOCK.md`](BASELINE_LOCK.md) for the
full protocol, expected metrics, and branch policy.

## Layout

- `build/`: local CMake build output, ignored by Git.
- `data/`: local datasets, ignored by Git.
- `results/`: experiment outputs, ignored by Git.
- `logs/`: build and run logs, ignored by Git.
- `research/config/`: locked baseline YAML templates.
- `research/baseline/`: expected reference metrics for smoke tests.
- `scripts/check_research_env.sh`: dependency and toolchain check.
- `scripts/build_research.sh`: repeatable non-ROS Release build.
- `scripts/build_eval_tools.sh`: author evaluation tools (`ie_to_nmea`, etc.).
- `scripts/run_baseline_1_1.sh`: run locked RTK/IMU/Camera RRR on dataset 1.1.
- `scripts/run_author_eval_1_1.sh`: baseline + author APE pipeline (paper metric).

## Baseline Platform

- Ubuntu 22.04 or 20.04.
- GCC/G++ with C++11 support.
- CMake 3.x.
- Eigen3, OpenCV 4, yaml-cpp, glog, gflags, Ceres Solver.
- `evo` Python package for APE (`pip install evo`).
- ROS Noetic is optional and only needed for `ros_wrapper`.

## Typical Workflow

```bash
cd /home/theph/ws_ncs/gici_research_standard
./scripts/check_research_env.sh
./scripts/build_research.sh
./scripts/build_eval_tools.sh
./scripts/run_author_eval_1_1.sh
```

Set `GICI_DATASET_1_1` if dataset 1.1 is not at `/home/theph/ws_ncs/1.1`.

Keep raw datasets under `data/` and generated experiment artifacts under
`results/` or `logs/`.

## Forking for adaptive / fusion research

```bash
git checkout research/standard-env
git checkout -b paperX/your-topic
# implement adaptive logic, then re-run ./scripts/run_author_eval_1_1.sh
```
