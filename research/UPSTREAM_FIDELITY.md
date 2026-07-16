# Upstream fidelity policy

**Policy:** `include/`, `src/`, `tools/evaluation/`, and `option/` must be **byte-identical**
to `chichengcn/gici-open` @ `f2b8579`, **except** a short, explicit allowlist of
memory-safety/UB bugfixes (never algorithm or tuning changes) tracked in
`scripts/verify_upstream_fidelity.sh`. As of this writing that allowlist is:

| File | Fix |
|---|---|
| `src/stream/data_integration.cpp` | free `rs_prc`/`dts_prc`/`var_prc` (leak) |
| `src/stream/formator.cpp` | guard `eph.sat`/GLONASS `prn` bounds before `nav.eph[]`/`nav.geph[]` index (heap-buffer-underflow segfault) |
| `src/fusion/gnss_imu_initializer.cpp` | guard empty deque before `front()`/`back()` (heap-use-after-free) |
| `third_party/rtklib/src/rinex.c` (vendored RTKLIB, not tracked by the script above) | `init_rnxctr()` only zeroed `tobs[0..5]`, but `NUMSYS==7` (adds SYS_IRN) — `tobs[6]` was uninitialized memory that `set_index()` then walks past `MAXOBSTYPE` looking for a null terminator on any RINEX file with no IRNSS header line |

The above are genuine, ASLR-sensitive undefined-behavior bugs in upstream code,
root-caused while debugging non-reproducible UrbanNav Medium results (see
`research/BASELINE_LOCK.md`) — leaving known crash/corruption bugs in place is not a
more faithful reproduction, it just makes results non-deterministic. Any other core
change is out of policy; get it reviewed before landing.

### Real-time (multi-thread) race-condition fix — `research/ros2-realtime-fix` only

The original `imu_state_mutex_` guard (commit `bc3761c`) only covered the `states_`
push/pop shift in `RtkImuCameraRrrEstimator`. A ROS2-standard-practice audit
(2026-07-16) traced the actual concurrent data paths and found it left several other
pieces of state racing between the image-frontend thread and the backend/estimator
thread, unguarded:

- `ImuEstimatorBase`'s `last_timestamp_`/`last_T_WS_`/... cache, read/written outside
  the lock at the start/end of `getPoseEstimateAt`/`getSpeedAndBiasEstimateAt`/
  `getCovarianceAt`.
- `EstimatorBase::covariances_` (a `std::map`), mutated (`insert`/`erase`) in
  `updateCovariance()` with no lock at all — a concurrent `find()` during a map
  rebalance is undefined behavior, not just staleness.
- `graph_->solve()` (inside `optimize()`) overwriting ceres parameter blocks with no
  lock while `getPoseEstimate`/`getSpeedAndBiasEstimate` read them from another thread.
- The `states_` deque shift in every **other** multi-threaded estimator
  (`SppImuCameraRrr`, `GnssImuCameraSrr`, `RtkImuTc`, `SppImuTc`, `GnssImuLc`,
  `PppImuTc`, and the pure-GNSS `Spp`/`Dgnss`/`Sdgnss`) — all had the identical
  unguarded push/pop pattern, just never hit by the specific baselines being tested.

Fix: promoted the mutex to `EstimatorBase` (renamed `estimator_state_mutex_`,
`std::recursive_mutex` since the `*At()` accessors and `optimize()` call into
state-taking helpers that also take the lock) and extended every function above to
hold it for its **entire** body, not just the previously-guarded middle section. See
the file list in `scripts/verify_upstream_fidelity.sh` for the exact diff. Verified:
file-mode (single-threaded path) numbers unchanged (mutex is a no-op there); ROS2
real-time stress-test re-run tracked in `research/BASELINE_LOCK.md`.

Verify:

```bash
./scripts/verify_upstream_fidelity.sh
```

Research additions live only under `research/` and `scripts/` (wrappers, expected metrics, docs).

## UrbanNav RINEX post-file @ f2b8579

The author ships `option/post_estimation_RTK_RRR_rinex_imutext.yaml`, but long RINEX
post-file replay on UrbanNav can **abort** on unmodified `f2b8579` (GNSS nav merge / cold start).

Author-native alternative: `ros_wrapper` + `ros_urbannav.yaml` + rosbag (see upstream comments).

Locked UrbanNav **eval** artifacts (`results/baseline/urbannav/`) may be used when batch replay
is unavailable; re-run when upstream or environment supports it.
