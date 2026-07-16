#!/usr/bin/env bash
# No-sparsify realtime accuracy experiment (branch research/ros2-live-nosparsify-experiment).
#
# Tests whether disable_backend_data_sparsify closes the gap to file-mode (~0.03 m)
# without deadlock/timeout on bag replay @ rate 1.0.
#
# Usage:
#   ./scripts/ros2/run_nosparsify_experiment.sh [smoke10|batch30] [bag|live]
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PHASE="${1:-smoke10}"
RUNNER="${2:-bag}"

case "${PHASE}" in
  smoke10) RUNS=10; SUFFIX=nosparsify_smoke10 ;;
  batch30) RUNS=30; SUFFIX=nosparsify_batch30 ;;
  *) echo "phase: smoke10|batch30" >&2; exit 2 ;;
esac

export STRESS_SUFFIX="${SUFFIX}_${RUNNER}"
export STRESS_VARIANT=nosparsify
export STRESS_RUNNER="${RUNNER}"
export STRESS_PROFILE=strict

echo "[nosparsify-exp] phase=${PHASE} runner=${RUNNER} runs=${RUNS}"
exec "${REPO}/scripts/ros2/stress_bag_replay_30x.sh" 1.1 "${RUNS}" 1
