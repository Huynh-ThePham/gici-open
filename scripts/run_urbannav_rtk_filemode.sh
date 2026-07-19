#!/usr/bin/env bash
# UrbanNav GNSS-only RTK in post-processing FILE mode (author's actual run method).
#
# Produces the FULL trajectory. Crash and timeout exits are treated as failures; do
# not use partial solution files as locked results.
#
# Usage: scripts/run_urbannav_rtk_filemode.sh [medium|deep]
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DS="${1:-medium}"
GICI_MAIN="${GICI_MAIN:-${REPO}/build/gici_main}"
DATA_ROOT="${URBANNAV_DATA_ROOT:-/home/theph/Downloads/UrbanNavDataset-master}"
TEMPLATE="${REPO}/research/config/rtk_gnss_urbannav_filemode.yaml"

case "${DS}" in
  medium)
    ROOT="${DATA_ROOT}/UrbanNav-HK-Medium-Urban-1"
    ROVER="${ROOT}/gnss/UrbanNav-HK-Medium-Urban-1.ublox.f9p.splitter.obs"
    REF="${ROOT}/gnss/base/hkkt137g.rnx"
    EPH="${ROOT}/gnss/base/brdc1370.rnx"
    DCB="${REPO}/research/dcb/CAS0MGXRAP_20211370000_01D_01D_DCB.BSX"
    ;;
  deep)
    ROOT="${DATA_ROOT}/UrbanNav-HK-Deep-Urban-1"
    ROVER="${ROOT}/gnss/UrbanNav-HK-Deep-Urban-1.ublox.f9p.splitter.obs"
    REF="${ROOT}/gnss/base/_deprecated/hkkt141g.21o"
    EPH="${ROOT}/gnss/base/_deprecated/brdc_mn.rnx"
    DCB="${REPO}/research/dcb/CAS0MGXRAP_20211410000_01D_01D_DCB.BSX"
    ;;
  *)
    echo "Unknown dataset '${DS}'. Use 'medium' or 'deep'." >&2
    exit 1
    ;;
esac

OUT_DIR="${REPO}/output/filemode_urbannav/${DS}"
mkdir -p "${OUT_DIR}/log"

CFG="${OUT_DIR}/config.yaml"
sed -e "s#<ROVER_OBS>#${ROVER}#g" \
    -e "s#<REF_OBS>#${REF}#g" \
    -e "s#<EPH_NAV>#${EPH}#g" \
    -e "s#<DCB_FILE>#${DCB}#g" \
    -e "s#<OUTPUT_DIR>#${OUT_DIR}#g" \
    "${TEMPLATE}" > "${CFG}"

echo "[filemode] config: ${CFG}"
echo "[filemode] running ${GICI_MAIN} ..."
rm -f "${OUT_DIR}/solution.txt"
"${GICI_MAIN}" "${CFG}" > "${OUT_DIR}/run.log" 2>&1
code=$?
echo "[filemode] gici_main exit=${code}"
if [[ "${code}" -ne 0 ]]; then
  echo "[filemode] ERROR: gici_main failed; see ${OUT_DIR}/run.log" >&2
  exit "${code}"
fi

if [[ -s "${OUT_DIR}/solution.txt" ]]; then
  echo "[filemode] solution: ${OUT_DIR}/solution.txt"
  echo "[filemode] GPGGA epochs: $(grep -c GPGGA "${OUT_DIR}/solution.txt")"
else
  echo "[filemode] ERROR: no solution written; see ${OUT_DIR}/run.log" >&2
  exit 1
fi
