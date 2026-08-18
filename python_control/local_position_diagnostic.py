#!/usr/bin/env python3
"""Ground-only PX4 optical-flow/local-position diagnostic. Never arms."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

from pymavlink import mavutil

import generated_mavlink as px4_mavlink


# Use the MAVLink development dialect shipped by this PX4 tree. The system
# pymavlink package predates ESTIMATOR_SENSOR_FUSION_STATUS (message 514).
mavutil.mavlink = px4_mavlink


PARAMETERS = (
    "SYS_HAS_GPS",
    "SIM_GPS_USED",
    "EKF2_GPS_CTRL",
    "EKF2_OF_CTRL",
    "EKF2_RNG_CTRL",
    "SIM_GZ_EN_FLOW",
    "SIM_GZ_EN_GPS",
    "SIM_GZ_EN_LIDAR",
)

FUSION_INDEX_GPS = 0
FUSION_INDEX_OPTICAL_FLOW = 1
FUSION_INDEX_BAROMETER = 4
FUSION_INDEX_RANGE_FINDER = 5


class Diagnostic:
    def __init__(self, connection_string: str, duration_s: float, log_path: Path) -> None:
        self.connection_string = connection_string
        self.duration_s = duration_s
        self.log_path = log_path
        self.connection = mavutil.mavlink_connection(
            connection_string,
            source_system=255,
            source_component=px4_mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )
        self.target_system = 0
        self.target_component = 0
        self.parameters: dict[str, float] = {}
        self.message_counts: dict[str, int] = {}
        self.local_samples: list[tuple[float, float, float, float, float, float]] = []
        self.flow_qualities: list[int] = []
        self.flow_integrals: list[tuple[float, float]] = []
        self.range_samples_m: list[float] = []
        self.estimator_flags: list[int] = []
        self.fusion_intended: list[list[int]] = []
        self.fusion_active: list[list[int]] = []
        self.gps_fix_types: list[int] = []
        self.gps_satellites_visible: list[int] = []
        self.armed_seen = False
        self.last_mode = "UNKNOWN"
        self.gcs_heartbeats_sent = 0

    def run(self) -> dict:
        self._log(f"[INFO] Connecting through {self.connection_string}")
        connect_deadline = time.monotonic() + 15.0
        first_heartbeat = None
        while time.monotonic() < connect_deadline:
            candidate = self._receive_one(timeout_s=1.0)
            if (
                candidate is not None
                and candidate.get_type() == "HEARTBEAT"
                and candidate.get_srcSystem() != 255
            ):
                first_heartbeat = candidate
                break
        if first_heartbeat is None:
            raise TimeoutError("15秒内未收到PX4 HEARTBEAT")
        self.target_system = first_heartbeat.get_srcSystem()
        self.target_component = first_heartbeat.get_srcComponent()
        self._handle(first_heartbeat)
        self._log(
            f"[OK] PX4 connected system={self.target_system} "
            f"component={self.target_component}"
        )

        self._request_parameters()
        self._request_streams()

        deadline = time.monotonic() + self.duration_s
        next_heartbeat = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_heartbeat:
                self._send_gcs_heartbeat()
                next_heartbeat = now + 1.0
            message = self._receive_one(timeout_s=0.2)
            if message is not None and message.get_type() != "BAD_DATA":
                self._handle(message)

        result = self._build_result()
        self._write_result(result)
        self.connection.close()
        return result

    def _receive_one(self, timeout_s: float):
        # The diagnostic program's only MAVLink receive entry point.
        return self.connection.recv_match(blocking=True, timeout=timeout_s)

    def _handle(self, message) -> None:
        message_type = message.get_type()
        self.message_counts[message_type] = self.message_counts.get(message_type, 0) + 1

        if message_type == "HEARTBEAT" and message.get_srcSystem() != 255:
            self.armed_seen |= bool(
                message.base_mode & px4_mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.last_mode = mavutil.mode_string_v10(message)

        elif message_type == "PARAM_VALUE":
            raw_name = message.param_id
            name = raw_name.decode(errors="ignore") if isinstance(raw_name, bytes) else raw_name
            name = name.rstrip("\x00")
            if name in PARAMETERS:
                self.parameters[name] = self._decode_parameter_value(message)

        elif message_type == "LOCAL_POSITION_NED":
            values = (
                float(message.x),
                float(message.y),
                float(message.z),
                float(message.vx),
                float(message.vy),
                float(message.vz),
            )
            if all(math.isfinite(value) for value in values):
                self.local_samples.append(values)

        elif message_type == "OPTICAL_FLOW_RAD":
            self.flow_qualities.append(int(message.quality))
            self.flow_integrals.append(
                (float(message.integrated_x), float(message.integrated_y))
            )

        elif message_type == "DISTANCE_SENSOR":
            if int(message.current_distance) != 65535:
                self.range_samples_m.append(float(message.current_distance) / 100.0)

        elif message_type == "ESTIMATOR_STATUS":
            self.estimator_flags.append(int(message.flags))

        elif message_type == "ESTIMATOR_SENSOR_FUSION_STATUS":
            self.fusion_intended.append([int(value) for value in message.intended])
            self.fusion_active.append([int(value) for value in message.active])

        elif message_type == "GPS_RAW_INT":
            self.gps_fix_types.append(int(message.fix_type))
            self.gps_satellites_visible.append(int(message.satellites_visible))

    def _request_parameters(self) -> None:
        for name in PARAMETERS:
            self.connection.mav.param_request_read_send(
                self.target_system,
                self.target_component,
                name.encode("ascii"),
                -1,
            )

    def _request_streams(self) -> None:
        requests = (
            (px4_mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0),
            (px4_mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, 10.0),
            (px4_mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10.0),
            (px4_mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS, 2.0),
            (px4_mavlink.MAVLINK_MSG_ID_ESTIMATOR_SENSOR_FUSION_STATUS, 2.0),
            (px4_mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2.0),
        )
        for message_id, rate_hz in requests:
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                px4_mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(message_id),
                1_000_000.0 / rate_hz,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

    def _send_gcs_heartbeat(self) -> None:
        self.connection.mav.heartbeat_send(
            px4_mavlink.MAV_TYPE_GCS,
            px4_mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            px4_mavlink.MAV_STATE_ACTIVE,
        )
        self.gcs_heartbeats_sent += 1

    def _build_result(self) -> dict:
        last_intended = self.fusion_intended[-1] if self.fusion_intended else None
        last_active = self.fusion_active[-1] if self.fusion_active else None
        last_flags = self.estimator_flags[-1] if self.estimator_flags else 0

        horizontal_velocity_valid = bool(
            last_flags & px4_mavlink.ESTIMATOR_VELOCITY_HORIZ
        )
        vertical_velocity_valid = bool(
            last_flags & px4_mavlink.ESTIMATOR_VELOCITY_VERT
        )
        horizontal_position_valid = bool(
            last_flags
            & (
                px4_mavlink.ESTIMATOR_POS_HORIZ_REL
                | px4_mavlink.ESTIMATOR_POS_HORIZ_ABS
            )
        )
        vertical_position_valid = bool(
            last_flags
            & (
                px4_mavlink.ESTIMATOR_POS_VERT_ABS
                | px4_mavlink.ESTIMATOR_POS_VERT_AGL
            )
        )

        gps_intended = self._fusion_enabled(last_intended, FUSION_INDEX_GPS)
        gps_active = self._fusion_enabled(last_active, FUSION_INDEX_GPS)
        flow_intended = self._fusion_enabled(
            last_intended, FUSION_INDEX_OPTICAL_FLOW
        )
        flow_active = self._fusion_enabled(last_active, FUSION_INDEX_OPTICAL_FLOW)
        baro_active = self._fusion_enabled(last_active, FUSION_INDEX_BAROMETER)
        range_active = self._fusion_enabled(last_active, FUSION_INDEX_RANGE_FINDER)

        local_stats = self._local_statistics()
        flow_data_present = len(self.flow_qualities) >= 5 and max(self.flow_qualities, default=0) > 0
        parameter_gate = (
            self.parameters.get("SYS_HAS_GPS") == 0.0
            and self.parameters.get("SIM_GPS_USED") == 0.0
            and self.parameters.get("EKF2_GPS_CTRL") == 0.0
            and self.parameters.get("EKF2_OF_CTRL") == 1.0
        )

        gates = {
            "gate_1_optical_flow_data": flow_data_present,
            "gate_2_ekf_uses_flow_not_gps": (
                parameter_gate
                and not gps_intended
                and not gps_active
                and flow_intended
                and flow_active
            ),
            "gate_3_local_position_valid": (
                len(self.local_samples) >= 10
                and horizontal_position_valid
                and horizontal_velocity_valid
                and vertical_position_valid
                and vertical_velocity_valid
            ),
            "gate_4_ground_safe_for_offboard_prewarm": (
                not self.armed_seen
                and horizontal_position_valid
                and horizontal_velocity_valid
                and vertical_position_valid
                and vertical_velocity_valid
            ),
        }

        return {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "connection": self.connection_string,
            "duration_s": self.duration_s,
            "parameters": self.parameters,
            "message_counts": self.message_counts,
            "gcs_heartbeats_sent": self.gcs_heartbeats_sent,
            "armed_seen": self.armed_seen,
            "last_mode": self.last_mode,
            "optical_flow": {
                "sample_count": len(self.flow_qualities),
                "quality_min": min(self.flow_qualities, default=None),
                "quality_max": max(self.flow_qualities, default=None),
                "quality_mean": self._mean(self.flow_qualities),
            },
            "range_finder": {
                "sample_count": len(self.range_samples_m),
                "distance_min_m": min(self.range_samples_m, default=None),
                "distance_max_m": max(self.range_samples_m, default=None),
                "distance_mean_m": self._mean(self.range_samples_m),
            },
            "gps_raw": {
                "sample_count": len(self.gps_fix_types),
                "max_fix_type": max(self.gps_fix_types, default=None),
                "max_satellites_visible": max(
                    self.gps_satellites_visible, default=None
                ),
                "usable_fix_seen": any(value >= 3 for value in self.gps_fix_types),
            },
            "estimator": {
                "status_flags": last_flags,
                "horizontal_position_valid": horizontal_position_valid,
                "horizontal_velocity_valid": horizontal_velocity_valid,
                "vertical_position_valid": vertical_position_valid,
                "vertical_velocity_valid": vertical_velocity_valid,
                "gps_intended": gps_intended,
                "gps_active": gps_active,
                "optical_flow_intended": flow_intended,
                "optical_flow_active": flow_active,
                "barometer_active": baro_active,
                "range_finder_active": range_active,
                "fusion_status_samples": len(self.fusion_active),
            },
            "local_position": local_stats,
            "gates": gates,
            "passive_ground_diagnostic_pass": all(gates.values()),
        }

    def _local_statistics(self) -> dict:
        if not self.local_samples:
            return {"sample_count": 0}
        x0, y0, z0, *_ = self.local_samples[0]
        horizontal_drifts = [
            math.hypot(sample[0] - x0, sample[1] - y0)
            for sample in self.local_samples
        ]
        horizontal_speeds = [
            math.hypot(sample[3], sample[4]) for sample in self.local_samples
        ]
        return {
            "sample_count": len(self.local_samples),
            "first": {
                "x": x0,
                "y": y0,
                "z": z0,
                "vx": self.local_samples[0][3],
                "vy": self.local_samples[0][4],
                "vz": self.local_samples[0][5],
            },
            "last": {
                "x": self.local_samples[-1][0],
                "y": self.local_samples[-1][1],
                "z": self.local_samples[-1][2],
                "vx": self.local_samples[-1][3],
                "vy": self.local_samples[-1][4],
                "vz": self.local_samples[-1][5],
            },
            "max_horizontal_drift_m": max(horizontal_drifts),
            "mean_horizontal_speed_m_s": self._mean(horizontal_speeds),
            "max_horizontal_speed_m_s": max(horizontal_speeds),
            "max_abs_vertical_speed_m_s": max(
                abs(sample[5]) for sample in self.local_samples
            ),
        }

    @staticmethod
    def _fusion_enabled(values: list[int] | None, index: int) -> bool | None:
        if values is None or len(values) <= index:
            return None
        return bool(values[index] & 1)

    @staticmethod
    def _decode_parameter_value(message) -> float | int:
        value = float(message.param_value)
        parameter_type = int(message.param_type)
        raw = struct.pack("<f", value)
        if parameter_type == px4_mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack("<B", raw[:1])[0]
        if parameter_type == px4_mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack("<b", raw[:1])[0]
        if parameter_type == px4_mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack("<H", raw[:2])[0]
        if parameter_type == px4_mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack("<h", raw[:2])[0]
        if parameter_type == px4_mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack("<I", raw)[0]
        if parameter_type == px4_mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack("<i", raw)[0]
        return value

    @staticmethod
    def _mean(values) -> float | None:
        return statistics.fmean(values) if values else None

    def _write_result(self, result: dict) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self._log(json.dumps(result, ensure_ascii=False, indent=2))
        self._log(f"[INFO] Diagnostic saved to {self.log_path}")

    @staticmethod
    def _log(message: str) -> None:
        print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {message}", flush=True)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--log-file", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if not 10.0 <= args.duration <= 60.0:
        raise SystemExit("--duration必须在10到60秒之间")
    default_log = (
        Path(__file__).resolve().parent
        / "logs"
        / f"local_position_diagnostic_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    )
    diagnostic = Diagnostic(args.connection, args.duration, args.log_file or default_log)
    try:
        result = diagnostic.run()
    except Exception as error:
        print(f"[FAILED] {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0 if result["passive_ground_diagnostic_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
