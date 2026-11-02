#!/usr/bin/env bash
# systemd entrypoint: source the ROS 2 + workspace overlays, then exec the HMI.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/boonraygrn/glim_ws/install/setup.bash}"
CONFIG="${GNSS_HMI_CONFIG:-$HERE/../config/gnss_hmi.yaml}"
PYTHON="${GNSS_HMI_PYTHON:-python3}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# shellcheck disable=SC1090
source "$ROS_SETUP"
if [[ -f "$WS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$WS_SETUP"
fi

cd "$HERE/.."
exec "$PYTHON" -m gnss_hmi.main "$CONFIG"
