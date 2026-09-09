#!/usr/bin/env bash
# Task 9 Step 6: assert that librtk_global.so loaded, subscribed and produced
# factors during a glim_rosbag run.
#
#   ros2 run glim_ros glim_rosbag <bag> 2>&1 | tee /tmp/rtk_global.log
#   tools/check_rtk_global_log.sh /tmp/rtk_global.log
set -uo pipefail
log="${1:?usage: $0 <glim_rosbag log>}"
fail=0
check() {  # <description> <grep pattern>
  if grep -qE "$2" "$log"; then echo "ok    $1"; else echo "FAIL  $1  (pattern: $2)"; fail=1; fi
}
absent() {
  if grep -qE "$2" "$log"; then echo "FAIL  $1  (found: $2)"; fail=1; else echo "ok    $1"; fi
}
check  "module initialised"            "initializing rtk_global"
check  "backend thread started"        "starting rtk_global backend thread"
check  "T_world_enu bootstrapped"      "T_world_enu="
check  "fixes received (>0)"           "fix_received=[1-9][0-9]*"
check  "factors added (>0)"            "factors_added=[1-9][0-9]*"
absent "no msg type mismatch"          "msg type mismatch"
absent "no deserialisation failure"    "failed to deserialize"
absent "no config load failure"        "failed to open .*config_rtk_global"
grep -E "rtk_global stats:" "$log" || true
exit $fail
