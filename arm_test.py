import time
from pymavlink import mavutil

conn = mavutil.mavlink_connection("udp:127.0.0.1:14550", source_system=255)
conn.wait_heartbeat(timeout=10)
print("Connected")

# Arm immediately
conn.mav.command_long_send(
    conn.target_system, conn.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# Wait and listen for response
for i in range(20):
    msg = conn.recv_msg()
    if msg:
        t = msg.get_type()
        if t in ("COMMAND_ACK", "STATUSTEXT", "HEARTBEAT", "SYS_STATUS", "EKF_STATUS_REPORT"):
            print("{}: {}".format(t, str(msg)[:200]))
    else:
        time.sleep(0.5)

print("\nMotors armed: " + str(conn.motors_armed()))
