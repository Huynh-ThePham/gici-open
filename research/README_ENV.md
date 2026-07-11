# GICI Research Environment

This worktree is intended to be a clean, repeatable research workspace for
`Huynh-ThePham/gici-open.git`.

## Layout

- `build/`: local CMake build output, ignored by Git.
- `data/`: local datasets, ignored by Git.
- `results/`: experiment outputs, ignored by Git.
- `logs/`: build and run logs, ignored by Git.
- `scripts/check_research_env.sh`: dependency and toolchain check.
- `scripts/build_research.sh`: repeatable non-ROS Release build.

## Baseline Platform

- Ubuntu 22.04 or 20.04.
- GCC/G++ with C++11 support.
- CMake 3.x.
- Eigen3.
- OpenCV 4.
- yaml-cpp.
- glog and gflags.
- Ceres Solver.
- ROS Noetic is optional and only needed for `ros_wrapper`.

## Typical Workflow

```bash
cd /home/theph/ws_ncs/gici_research_standard
./scripts/check_research_env.sh
./scripts/build_research.sh
```

Run GICI after a successful build:

```bash
./build/gici_main <gici-config-file>
```

Keep raw datasets under `data/` and generated experiment artifacts under
`results/` or `logs/`.
