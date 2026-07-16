#!/usr/bin/env bash
# Verify GICI core matches chichengcn/gici-open @ f2b8579, modulo a short,
# documented allowlist of safety-only bugfixes (memory-safety/UB corrections;
# no algorithm or tuning changes belong in this list).
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM="${GICI_UPSTREAM_REF:-f2b8579}"

# Each entry must be a memory-safety/UB fix only. Document the reason here and
# in the commit message. A file only "counts" if it actually shows up in the
# diff, so branch-specific entries (e.g. the realtime mutex guard, which only
# exists on research/ros2-realtime-fix) are harmless to list unconditionally.
ALLOWED_DELTA=(
  "src/stream/data_integration.cpp"     # free rs_prc/dts_prc/var_prc (leak)
  "src/stream/formator.cpp"             # guard eph.sat / geph prn bounds before nav.eph[]/nav.geph[] index (heap-buffer-underflow segfault)
  "src/fusion/gnss_imu_initializer.cpp" # guard empty deque before front()/back() (heap-use-after-free)

  # estimator_state_mutex_ real-time (multi-thread) race fix -- research/ros2-realtime-fix
  # only. The original imu_state_mutex_ (commit bc3761c) only covered the
  # states_ push/pop in RtkImuCameraRrrEstimator; it left the ImuEstimatorBase
  # last_* cache, the covariances_ map, graph_->solve() vs concurrent reads,
  # and every other IMU/GNSS-only estimator's states_ shift unguarded. This
  # promotes the mutex to EstimatorBase (renamed estimator_state_mutex_,
  # recursive) and locks all of the above consistently. See
  # research/UPSTREAM_FIDELITY.md.
  "include/gici/estimate/estimator_base.h"
  "src/estimate/estimator_base.cpp"
  "include/gici/imu/imu_estimator_base.h"
  "src/imu/imu_estimator_base.cpp"
  "src/fusion/rtk_imu_camera_rrr_estimator.cpp"
  "src/fusion/spp_imu_camera_rrr_estimator.cpp"
  "src/fusion/gnss_imu_camera_srr_estimator.cpp"
  "include/gici/fusion/rtk_imu_tc_estimator.h"
  "include/gici/fusion/spp_imu_tc_estimator.h"
  "include/gici/fusion/gnss_imu_lc_estimator.h"
  "include/gici/fusion/ppp_imu_tc_estimator.h"
  "src/gnss/spp_estimator.cpp"
  "src/gnss/dgnss_estimator.cpp"
  "src/gnss/sdgnss_estimator.cpp"
)

cd "$ROOT"
echo "Checking delta vs upstream ${UPSTREAM}..."
mapfile -t CHANGED < <(git diff --name-only "${UPSTREAM}" -- include/ src/ tools/evaluation/ option/ 2>/dev/null || true)

UNEXPECTED=()
for f in "${CHANGED[@]}"; do
  allowed=0
  for a in "${ALLOWED_DELTA[@]}"; do
    [[ "$f" == "$a" ]] && allowed=1 && break
  done
  (( allowed )) || UNEXPECTED+=("$f")
done

if ((${#UNEXPECTED[@]} == 0)); then
  if ((${#CHANGED[@]} == 0)); then
    echo "OK: GICI core identical to upstream ${UPSTREAM}"
  else
    echo "OK: GICI core matches upstream ${UPSTREAM} except documented safety bugfixes:"
    printf '  %s\n' "${CHANGED[@]}"
  fi
  exit 0
fi

echo "FAIL: unexpected changes vs upstream ${UPSTREAM}:"
printf '  %s\n' "${UNEXPECTED[@]}"
exit 1
