#!/bin/bash
# px4_run_cmd.sh — run a command via px4 in-band, then exit
# Usage: sh ~/PX4-test/px4_run_cmd.sh
# This sends a command to px4 by connecting to the daemon socket.

# The socket may be refused from outside, try px4-commander with correct instance
cd ~/PX4-test/build/px4_sitl_default
echo "Status:"
echo "commander status" | timeout 5 ./bin/px4-listener vehicle_status 2>/dev/null || true
echo ""
echo "Arm attempt:"
echo "commander arm" | timeout 5 ./bin/px4-commander 2>/dev/null || true
sleep 3
echo ""
echo "Check:"
echo "listener vehicle_status" | timeout 5 nc -U /tmp/px4-sock-0 2>/dev/null || echo "nc failed, socket not accepting connections"
