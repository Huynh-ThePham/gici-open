#!/usr/bin/env bash
# UrbanNav file-mode RRR baseline V1 (wrapper config, author-faithful).
# See docs/baseline/FILEMODE_URBANNAV_RRR_BASELINE_V1.md
set -Eeuo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "${REPO}/scripts/run_urbannav_rrr_baseline.py" --config-source wrapper "$@"
