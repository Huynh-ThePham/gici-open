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
| `src/fusion/rtk_imu_camera_rrr_estimator.cpp` | `imu_state_mutex_` guard for the real-time path — **`research/ros2-realtime-fix` only** |

The first three are genuine, ASLR-sensitive undefined-behavior bugs in upstream code,
root-caused while debugging non-reproducible UrbanNav Medium results (see
`research/BASELINE_LOCK.md`) — leaving known crash/corruption bugs in place is not a
more faithful reproduction, it just makes results non-deterministic. Any other core
change is out of policy; get it reviewed before landing.

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
