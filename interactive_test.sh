#!/bin/bash
# PX4 interactive shell test
# Run in WSL: bash ~/PX4-test/interactive_test.sh

pkill -9 px4 2>/dev/null
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0
sleep 1

cd ~/PX4-test/build/px4_sitl_default

# Feed commands through a pipe using sleep
{
  echo "commander check"
  sleep 5
  echo "commander arm -f"
  sleep 10
  echo "listener vehicle_local_position"
  sleep 5
  echo "height_commander status"
  sleep 3
  echo "shutdown"
} | PX4_SYS_AUTOSTART=10040 timeout 45 ./bin/px4 2>&1 | tee /tmp/px4_interactive.log
