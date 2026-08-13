#!/bin/bash
# PX4 SIH HITL Test — run this from your WSL terminal
# Usage:  bash ~/PX4-test/hitl_test.sh

echo "=== PX4 SIH + QGC HITL Test ==="
echo ""

# 1. Kill old instances
pkill -9 px4 2>/dev/null
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0
sleep 1

# 2. Start PX4 in daemon mode
cd ~/PX4-test/build/px4_sitl_default
PX4_SYS_AUTOSTART=10040 ./bin/px4 -d </dev/null &
PX4_PID=$!
echo "PX4 started (PID=$PX4_PID)"

# 3. Wait for full init (EKF takes time to converge)
echo "Waiting 30s for EKF and sensors to initialize..."
sleep 30

# 4. Check if alive
if ! kill -0 $PX4_PID 2>/dev/null; then
    echo "ERROR: PX4 died"
    exit 1
fi
echo "PX4 is alive!"

# 5. Check socket
if [ -S /tmp/px4-sock-0 ]; then
    echo "Shell socket: OK"
else
    echo "Shell socket: MISSING"
fi

# 6. Send commands via px4 shell socket
echo ""
echo "=== Sending commands ==="

# Force arm (skip preflight)
echo "Arming with -f..."
echo "commander arm -f" | nc -U /tmp/px4-sock-0 -w 3 2>/dev/null
sleep 5

# Takeoff
echo "Takeoff..."
echo "commander takeoff" | nc -U /tmp/px4-sock-0 -w 3 2>/dev/null
sleep 8

# Get altitude data
echo ""
echo "=== Altitude ==="
echo "listener vehicle_local_position" | nc -U /tmp/px4-sock-0 -w 5 2>/dev/null

# Get height commander status
echo ""
echo "=== Height Commander ==="
echo "height_commander status" | nc -U /tmp/px4-sock-0 -w 3 2>/dev/null

echo ""
echo "=== Test Complete ==="
echo "PX4 log: /tmp/px4_final.log"
echo "To stop: kill $PX4_PID"
