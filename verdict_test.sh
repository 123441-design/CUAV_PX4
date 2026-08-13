#!/bin/bash
# PX4 HITL Final Verdict Script
# Run in WSL: bash ~/PX4-test/verdict_test.sh

echo "=== PX4 SIH + height_commander Verdict Test ==="

pkill -9 px4 2>/dev/null
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0
sleep 1

cd ~/PX4-test/build/px4_sitl_default
{
  echo "commander arm -f"
  sleep 5
  echo "commander takeoff"
  sleep 8
  echo ""
  echo "echo ====== HEIGHT_COMMANDER VERDICT ======"
  echo "listener vehicle_local_position"
  sleep 3
  echo "height_commander status"
  sleep 3
  echo ""
} | PX4_SYS_AUTOSTART=10040 timeout 45 ./bin/px4 2>&1 | strings | grep -E "z: .*-|vz:|alt=|Armed|Takeoff|DELTA|State:.*[1-9]|ASCEND|DESCEND|HOLD|======|height_commander.*State|vehicle_local_position" | grep -v pxh
