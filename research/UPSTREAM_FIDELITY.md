# Upstream fidelity policy

**Policy:** `include/`, `src/`, `tools/evaluation/`, and `option/` must be **byte-identical**
to `chichengcn/gici-open` @ `f2b8579`. No local patches on `research/standard-env`.

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
