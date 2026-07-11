#!/usr/bin/env bash
# Verify GICI core matches chichengcn/gici-open @ f2b8579 exactly (zero source delta).
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM="${GICI_UPSTREAM_REF:-f2b8579}"

cd "$ROOT"
echo "Checking delta vs upstream ${UPSTREAM}..."
mapfile -t CHANGED < <(git diff --name-only "${UPSTREAM}" -- include/ src/ tools/evaluation/ option/ 2>/dev/null || true)

if ((${#CHANGED[@]} == 0)); then
  echo "OK: GICI core identical to upstream ${UPSTREAM}"
  exit 0
fi

echo "FAIL: unexpected changes vs upstream ${UPSTREAM}:"
printf '  %s\n' "${CHANGED[@]}"
exit 1
