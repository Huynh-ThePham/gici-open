#!/usr/bin/env bash
# Verify GICI core matches chichengcn/gici-open @ f2b8579, modulo a short,
# documented allowlist of safety-only bugfixes (memory-safety/UB corrections;
# no algorithm or tuning changes belong in this list).
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM="${GICI_UPSTREAM_REF:-f2b8579}"

# Each entry must be a memory-safety/UB fix only. Document the reason here and
# in the commit message. A file only "counts" if it actually shows up in the
# diff. Keep this list narrow: file-wide allowlisting can hide algorithm changes.
ALLOWED_DELTA=(
  "src/stream/data_integration.cpp"     # free rs_prc/dts_prc/var_prc (leak)
  "src/stream/formator.cpp"             # guard eph.sat / geph prn bounds before nav.eph[]/nav.geph[] index (heap-buffer-underflow segfault)
  "src/fusion/gnss_imu_initializer.cpp" # guard empty deque before front()/back() (heap-use-after-free)
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
