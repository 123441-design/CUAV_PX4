import time
from pymavlink import mavutil

# Listen on 14550 for heartbeats
conn = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
conn.wait_heartbeat(timeout=15)
print("Heartbeat. sys=" + str(conn.target_system) + " comp=" + str(conn.target_component))

# Connect send to 18570
sender = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255, source_component=1)
time.sleep(0.5)

# Set COM_ARM_WO_GPS
print("Setting COM_ARM_WO_GPS=1...")
sender.mav.param_set_send(
    conn.target_system, conn.target_component,
    b"COM_ARM_WO_GPS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
time.sleep(3)

# Arm
print("Arming...")
sender.mav.command_long_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(5)

# Listen for response
for i in range(15):
    msg = conn.recv_msg()
    if msg:
        t = msg.get_type()
        if t in ("COMMAND_ACK", "STATUSTEXT", "HEARTBEAT"):
            print(t + ": " + str(msg)[:250])
    else:
        time.sleep(0.3)

# Check if armed
msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
if msg:
    armed = (msg.base_mode & 128) != 0
    print("Armed: " + str(armed))
else:
    print("No updated heartbeat")
