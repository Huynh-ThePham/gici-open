# GICI / UrbanNav dataset roots for research scripts.
# Source from bash:  source "$(dirname "$0")/dataset_paths.sh"
# Or:              . scripts/dataset_paths.sh

# GICI board data relocated under a dedicated Gici/ subdir 2026-07 (was directly under
# dataset/). Board dirs now at ${GICI_DATA_ROOT}/{1.1,3.1,...}.
export GICI_DATA_ROOT="${GICI_DATA_ROOT:-/media/theph/Data1/Research/dataset/Gici}"

# GICI board datasets (gici-open-dataset numbering)
export GICI_DATASET_1_1="${GICI_DATASET_1_1:-${GICI_DATA_ROOT}/1.1}"
export GICI_DATASET_1_1_RINEX="${GICI_DATASET_1_1_RINEX:-${GICI_DATA_ROOT}/1.1-rinex-imutext}"

# UrbanNav lives beside Gici/ (absolute, decoupled from GICI_DATA_ROOT).
export URBANNAV_DATA_ROOT="${URBANNAV_DATA_ROOT:-/media/theph/Data1/Research/dataset/UrbanNav}"

# RTCM reference/ephemeris stream start date per GICI board dataset.
# MUST match the dataset collection date (gici-open-dataset README table),
# otherwise the RTCM decoder cannot resolve week-number and stalls on
# "Waiting for ephemeris". Single source of truth for all runners.
gici_rtcm_start_for() {
  case "$1" in
    1.1|2.1)                     echo "2023.03.20" ;;
    1.2|2.2|3.1|3.2|3.3|4.2|4.3) echo "2023.03.27" ;;
    4.1|5.1|5.2)                 echo "2023.05.21" ;;
    *)                           echo "2023.03.20" ;;
  esac
}
