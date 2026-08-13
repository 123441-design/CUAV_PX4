#!/bin/bash
# PX4 SIH Final Test Script for height_commander
# Run this in your WSL terminal:  bash ~/PX4-test/run_final_test.sh

echo "=== PX4 Height Commander SIH Test ==="
echo ""

# Clean up old instances
pkill -9 px4 2>/dev/null
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0
sleep 1

# Start PX4 in daemon mode
cd ~/PX4-test/build/px4_sitl_default
PX4_SYS_AUTOSTART=10040 ./bin/px4 -d </dev/null &
PX4_PID=$!
echo "PX4 started (PID=$PX4_PID)"

# Wait for init
echo "Waiting for PX4 to initialize..."
sleep 25

# Check if alive
if kill -0 $PX4_PID 2>/dev/null; then
    echo "PX4 is alive!"
else
    echo "PX4 died! Check log."
    exit 1
fi

# Force arm (skip preflight checks)
echo ""
echo "=== Force Arm ==="
echo "commander arm -f" | nc -U /tmp/px4-sock-0 2>/dev/null && echo "Command sent" || echo "Socket not available"
sleep 5

# Takeoff
echo ""
echo "=== Takeoff ==="
echo "commander takeoff" | nc -U /tmp/px4-sock-0 2>/dev/null && echo "Command sent" || echo "Socket not available"
sleep 8

# Check altitude
echo ""
echo "=== Altitude Data ==="
echo "listener vehicle_local_position" | nc -U /tmp/px4-sock-0 2>/dev/null || echo "Could not read altitude"

# Check height_commander
echo ""
echo "=== Height Commander ==="
echo "height_commander status" | nc -U /tmp/px4-sock-0 2>/dev/null || echo "Could not check height_commander"

# Clean up
echo ""
echo "=== Test Complete ==="
kill $PX4_PID 2>/dev/null
wait $PX4_PID 2>/dev/null
echo "Done."
