#!/bin/bash
# PX4 SIH auto-test script
# Run this ON YOUR WSL TERMINAL:
#   source ~/PX4-test/sitl_test.sh

echo "=== PX4 SIH Height Commander Test ==="
echo "1. Killing old PX4 instances..."
pkill -9 px4 2>/dev/null
rm -f /tmp/px4-sock-0 /tmp/px4_lock-0
sleep 1

echo "2. Starting PX4 in background (daemon mode)..."
cd ~/PX4-test/build/px4_sitl_default
PX4_SYS_AUTOSTART=10040 ./bin/px4 -d </dev/null &
PX4_PID=$!
echo "   PX4 PID: $PX4_PID"

echo "3. Waiting for PX4 to initialize (15 seconds)..."
sleep 15

echo "4. Connecting via MAVLink..."
python3 -c "
from pymavlink import mavutil
import time

# Restart connection fresh
conn = mavutil.mavlink_connection('udp:127.0.0.1:14550')
conn.wait_heartbeat(timeout=10)
print(f'Heartbeat: mode={conn.flightmode}')

time.sleep(3)

# Check health
print('Checking for preflight issues...')
for i in range(5):
    msg = conn.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
    if msg:
        print(f'  [{msg.severity}] {msg.text}')

# Arm
print('Arming...')
conn.mav.command_long_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, 0, 0, 0, 0, 0, 0)
time.sleep(4)
armed = conn.motors_armed()
print(f'Armed: {armed}')

if armed:
    print('Takeoff...')
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, float('nan'), 0, 0, 2.5)
    time.sleep(6)

    print('Altitude data:')
    for i in range(5):
        msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            print(f'  alt={msg.relative_alt/1000:.1f}m')
    print('SUCCESS!')
else:
    print('Arming failed.')
    for i in range(10):
        msg = conn.recv_match(type='STATUSTEXT', blocking=True, timeout=2)
        if msg:
            print(f'  [{msg.severity}] {msg.text}')
"

echo ""
echo "=== Test complete, cleaning up ==="
kill $PX4_PID 2>/dev/null
echo "Done. Run 'pxh-log' to see full PX4 log in /tmp/px4_test2.log"
