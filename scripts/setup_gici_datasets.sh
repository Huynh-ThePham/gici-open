#!/usr/bin/env bash
# Extract GICI board datasets from zip archives under GICI_DATA_ROOT.
# Usage: ./scripts/setup_gici_datasets.sh [1.1] [1.1-rinex] ...
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=dataset_paths.sh
source "${ROOT_DIR}/scripts/dataset_paths.sh"

need_zip() {
  local name="$1"
  local dest="${GICI_DATA_ROOT}/${name}"
  if [[ -d "$dest" ]]; then
    printf 'SKIP %s (already extracted at %s)\n' "$name" "$dest"
    return 0
  fi
  local zip="${GICI_DATA_ROOT}/${name}.zip"
  if [[ ! -f "$zip" ]]; then
    printf 'ERROR: missing %s\n' "$zip" >&2
    printf 'Download from: https://github.com/chichengcn/gici-open-dataset (OneDrive/BaiduCloud)\n' >&2
    return 1
  fi
  if ! unzip -tqq "$zip" 2>/dev/null; then
    printf 'ERROR: corrupt/incomplete zip: %s\n' "$zip" >&2
    printf 'Re-download from OneDrive: https://1drv.ms/f/s!Aq2sJkkB0M10jXecBfNuHSuEnrzM?e=Jz91kp\n' >&2
    printf 'Or BaiduCloud (pwd 6ncd): https://pan.baidu.com/s/1xZS-C_42LrGtUB0x6Bw_0A\n' >&2
    return 1
  fi
  printf 'Extracting %s -> %s ...\n' "$zip" "$dest"
  unzip -q "$zip" -d "$GICI_DATA_ROOT"
  printf 'OK %s\n' "$dest"
}

mkdir -p "$GICI_DATA_ROOT"

TARGETS=("$@")
if [[ ${#TARGETS[@]} -eq 0 ]]; then
  TARGETS=(1.1 1.1-rinex-imutext)
fi

for t in "${TARGETS[@]}"; do
  case "$t" in
    1.1-rinex|1.1-rinex-imutext) need_zip "1.1-rinex-imutext" ;;
    [1-5].[1-3]) need_zip "$t" ;;  # GICI board scenes 1.1 .. 5.2
    *)
      printf 'ERROR: unknown dataset %q (expected N.N e.g. 1.1, 3.1, 4.1)\n' "$t" >&2
      exit 1
      ;;
  esac
done

printf '\nDataset paths:\n'
printf '  GICI_DATA_ROOT=%s\n' "$GICI_DATA_ROOT"
printf '  GICI_DATASET_1_1=%s\n' "$GICI_DATASET_1_1"
printf '  GICI_DATASET_1_1_RINEX=%s\n' "$GICI_DATASET_1_1_RINEX"

if [[ -f "${GICI_DATASET_1_1}/ground_truth.txt" ]]; then
  printf '  ground_truth.txt: OK\n'
else
  printf '  WARN: %s/ground_truth.txt not found — run with 1.1\n' "$GICI_DATASET_1_1"
fi
