#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"
JOBS="${GICI_BUILD_JOBS:-$(nproc)}"

FC_BUILD="${ROOT_DIR}/tools/evaluation/format_converters/build"
AL_BUILD="${ROOT_DIR}/tools/evaluation/alignment/build"

mkdir -p "$FC_BUILD" "$AL_BUILD"

printf 'Building author evaluation tools\n'
printf 'Format converters: %s\n' "$FC_BUILD"
cmake -S "${ROOT_DIR}/tools/evaluation/format_converters" -B "$FC_BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$FC_BUILD" --parallel "$JOBS"

printf 'Alignment tools: %s\n' "$AL_BUILD"
cmake -S "${ROOT_DIR}/tools/evaluation/alignment" -B "$AL_BUILD" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$AL_BUILD" --parallel "$JOBS"

printf '\nEval tools ready:\n'
printf '  %s/ie_to_nmea\n' "$FC_BUILD"
printf '  %s/nmea_to_tum\n' "$FC_BUILD"
printf '  %s/nmea_pose_to_pose\n' "$AL_BUILD"
printf '  %s/nmea_align_timestamp\n' "$AL_BUILD"
