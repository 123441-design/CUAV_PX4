#!/usr/bin/env python3
"""Assert the MyLink direct-motor control path from a SITL ULog."""

import math
import sys

from pyulog import ULog


def dataset(ulog, name):
    return next(data for data in ulog.data_list if data.name == name and data.multi_id == 0)


ulog = ULog(sys.argv[1])
motors = dataset(ulog, "actuator_motors").data
outputs = dataset(ulog, "actuator_outputs").data
control_mode = dataset(ulog, "vehicle_control_mode").data
offboard_mode = dataset(ulog, "offboard_control_mode").data

direct_samples = [
    i for i, direct in enumerate(offboard_mode["direct_actuator"])
    if bool(direct)
]
allocation_disabled = any(
    not bool(enabled) for enabled in control_mode["flag_control_allocation_enabled"]
)
motor_25_samples = [
    i for i in range(len(motors["timestamp"]))
    if all(math.isclose(float(motors[f"control[{motor}]"][i]), 0.25, abs_tol=1e-4)
           for motor in range(4))
]
motor_zero_after_25 = any(
    motors["timestamp"][i] > motors["timestamp"][motor_25_samples[-1]]
    and all(math.isclose(float(motors[f"control[{motor}]"][i]), 0.0, abs_tol=1e-4)
            for motor in range(4))
    for i in range(len(motors["timestamp"]))
) if motor_25_samples else False
output_1250_samples = [
    i for i in range(len(outputs["timestamp"]))
    if all(math.isclose(float(outputs[f"output[{channel}]"][i]), 1250.0, abs_tol=1.0)
           for channel in range(4))
]
output_min_after_1250 = any(
    outputs["timestamp"][i] > outputs["timestamp"][output_1250_samples[-1]]
    and all(float(outputs[f"output[{channel}]"][i]) <= 1002.0 for channel in range(4))
    for i in range(len(outputs["timestamp"]))
) if output_1250_samples else False

checks = {
    "direct_actuator_samples": len(direct_samples) >= 2,
    "allocation_disabled": allocation_disabled,
    "four_motor_25_percent_samples": len(motor_25_samples) >= 2,
    "four_output_1250us_samples": len(output_1250_samples) >= 2,
    "motor_zero_after_timeout": motor_zero_after_25,
    "output_minimum_after_timeout": output_min_after_1250,
}

for name, passed in checks.items():
    print(f"{name}={'PASS' if passed else 'FAIL'}")

if not all(checks.values()):
    raise SystemExit(1)

print("RESULT PASS_MYLINK_MOTOR_ULOG")
