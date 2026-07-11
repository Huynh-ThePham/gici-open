#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${GICI_BUILD_DIR:-${ROOT_DIR}/build}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${GICI_BUILD_JOBS:-$(nproc)}"

mkdir -p "$BUILD_DIR" "${ROOT_DIR}/logs" "${ROOT_DIR}/results" "${ROOT_DIR}/data"

printf 'Configuring GICI\n'
printf 'Root:       %s\n' "$ROOT_DIR"
printf 'Build dir:  %s\n' "$BUILD_DIR"
printf 'Build type: %s\n' "$BUILD_TYPE"
printf 'Jobs:       %s\n\n' "$JOBS"

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$BUILD_DIR" --parallel "$JOBS"

printf '\nBuild complete: %s/gici_main\n' "$BUILD_DIR"
