#!/usr/bin/env python3
"""SITL-only MyLink Offboard stream generator."""

import sys
import time

import serial
from pymavlink.dialects.v20 import common as mavlink2


port_name = sys.argv[1]
duration = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
motor_test = "--motor-test" in sys.argv[3:]
post_ack_seconds = 25.0 if motor_test else 0.0
mav = mavlink2.MAVLink(None, srcSystem=42, srcComponent=191)
mav.robust_parsing = True

mask = (
    mavlink2.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)

with serial.Serial(port_name, 115200, timeout=0.02, write_timeout=1.0) as port:
    start = time.monotonic()
    next_send = start
    mode_sent = False
    preactive_motor_test_sent = False
    motor_test_sent = False
    motor_test_ack = None
    motor_test_ack_time = None
    count = 0

    while time.monotonic() - start < duration:
        now = time.monotonic()

        if now < next_send:
            time.sleep(next_send - now)
            continue

        setpoint = mav.set_position_target_local_ned_encode(
            int(now * 1000) & 0xFFFFFFFF, 1, 1, mavlink2.MAV_FRAME_LOCAL_NED,
            mask, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
        )
        port.write(setpoint.pack(mav, force_mavlink1=False))
        count += 1
        next_send += 0.2

        if motor_test and not preactive_motor_test_sent and now - start >= 0.3:
            command = mav.command_long_encode(
                1, 1, mavlink2.MAV_CMD_DO_MOTOR_TEST, 0,
                1.0, float(mavlink2.MOTOR_TEST_THROTTLE_PERCENT), 60.0,
                1.0, 4.0, 0.0, 0.0,
            )
            port.write(command.pack(mav, force_mavlink1=False))
            port.flush()
            preactive_motor_test_sent = True
            print("PREACTIVE_MOTOR_TEST_SENT_EXPECT_DROP")

        if not mode_sent and now - start >= 0.7:
            command = mav.command_long_encode(
                1, 1, mavlink2.MAV_CMD_DO_SET_MODE, 0,
                float(mavlink2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), 6.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
            )
            port.write(command.pack(mav, force_mavlink1=False))
            port.flush()
            mode_sent = True
            print("MODE_REQUEST_SENT")

        if motor_test and mode_sent and not motor_test_sent and now - start >= 1.8:
            command = mav.command_long_encode(
                1, 1, mavlink2.MAV_CMD_DO_MOTOR_TEST, 0,
                1.0, float(mavlink2.MOTOR_TEST_THROTTLE_PERCENT), 25.0,
                3.0, 4.0, 0.0, 0.0,
            )
            port.write(command.pack(mav, force_mavlink1=False))
            port.flush()
            motor_test_sent = True
            print("MOTOR_TEST_SENT motor=1 throttle=25% timeout=3s")

        for byte in port.read(port.in_waiting or 1):
            response = mav.parse_char(bytes((byte,)))

            if response and response.get_type() == "COMMAND_ACK" \
                    and response.command == mavlink2.MAV_CMD_DO_MOTOR_TEST:
                motor_test_ack = response.result
                motor_test_ack_time = time.monotonic()

        if motor_test_ack_time is not None and time.monotonic() - motor_test_ack_time >= post_ack_seconds:
            break

    port.flush()
    print(f"SETPOINTS_SENT={count}")

    if motor_test:
        print(f"MOTOR_TEST_ACK={motor_test_ack}")

        if motor_test_ack != mavlink2.MAV_RESULT_ACCEPTED:
            raise SystemExit(4)
