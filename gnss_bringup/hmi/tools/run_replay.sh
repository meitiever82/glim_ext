#!/usr/bin/env bash
# One-command local RTCM replay: start the time-synced feeder + the app, so the
# rtkrcv solving chain / skyplot / diagnosis light up without a vehicle.
# See docs/replay-demo.md.
#
# Usage: tools/run_replay.sh <data_dir> [speed] [config]
#   <data_dir>  directory containing base.rtcm3 and rover.rtcm3 (produced by
#               rnx2rtcm; see docs/replay-demo.md)
#   [speed]     playback speed-up (default 25 -> ~2.5 min per hour of data)
#   [config]    app config (default config.replay.example.yaml, web :8083)
#
# The feeder serves base->6201 / rover->6202, matching config.replay.example.yaml's
# corrections/raw_obs ports. Ctrl+C stops the app; re-running stops any leftover.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA="${1:?usage: tools/run_replay.sh <data_dir with base.rtcm3/rover.rtcm3> [speed] [config]}"
SPEED="${2:-25}"
CONFIG="${3:-$REPO/config.replay.example.yaml}"
PY="$REPO/.venv/bin/python"

for f in "$DATA/base.rtcm3" "$DATA/rover.rtcm3" "$PY" "$CONFIG"; do
  [ -e "$f" ] || { echo "missing: $f" >&2; exit 1; }
done

# Stop any running app / rtkrcv / feeder. Scan /proc rather than pgrep so
# orphaned rtkrcv children (dialed into the reserve ports, not the web port)
# are caught too. Running from a script file, this script's own command line is
# just its path, so the patterns below never match it (no self-kill).
for pid in /proc/[0-9]*; do
  cl=$(tr '\0' ' ' < "$pid/cmdline" 2>/dev/null) || continue
  case "$cl" in
    *gnss_hmi.main*|*tools/bin/rtkrcv*|*rtkfeed_timed*) kill "${pid#/proc/}" 2>/dev/null;;
  esac
done
sleep 2

"$PY" "$REPO/tools/rtkfeed_timed.py" "$DATA/base.rtcm3" 6201 "$DATA/rover.rtcm3" 6202 "$SPEED" \
    >/tmp/rtk_replay_feed.log 2>&1 &
echo "feeder pid $! (speed ${SPEED}x); log: /tmp/rtk_replay_feed.log"
sleep 1
echo "app -> http://127.0.0.1:8083   (Ctrl+C to stop)"
exec "$PY" -m gnss_hmi.main "$CONFIG"
