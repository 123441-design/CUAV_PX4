#!/bin/bash
# Run PX4 SITL test in background and capture output
set -e

BUILD_DIR="$HOME/PX4-test/build/px4_sitl_default"
LOG="/tmp/px4_test_output.log"
CMDS="/tmp/px4_commands.txt"

# Kill old instances
pkill -9 px4 2>/dev/null || true
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0 /tmp/px4_*_log
sleep 2

# Write commands to file
cat > "$CMDS" << 'CMDS'
echo "=== PX4 SHELL STARTED ==="
sleep 8
echo "=== CHECK STATUS ==="
echo "listener vehicle_status"
sleep 3
echo "=== ARM ==="
echo "commander arm"
sleep 3
echo "=== TAKEOFF ==="
echo "commander takeoff"
sleep 5
echo "=== HEIGHT DATA ==="
echo "listener vehicle_local_position"
sleep 3
echo "=== HEIGHT COMMANDER STATUS ==="
echo "height_commander status"
sleep 2
echo "=== DONE ==="
sleep 2
CMDS

# Run PX4 with a pty using script
cd "$BUILD_DIR"
PX4_SYS_AUTOSTART=10040 bash -c './bin/px4' < "$CMDS" > "$LOG" 2>&1 &

PX4_PID=$!
echo "PX4 PID: $PX4_PID"
echo "Waiting 35 seconds..."

sleep 35

# Check if still running
if kill -0 $PX4_PID 2>/dev/null; then
    echo "PX4 is still running, killing..."
    kill $PX4_PID 2>/dev/null
fi

wait $PX4_PID 2>/dev/null || true

echo "=== OUTPUT ==="
cat "$LOG"
