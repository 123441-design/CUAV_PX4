#!/usr/bin/env python3
"""Minimal Tkinter UDP control panel for the verified MyLink Bridge V3.1.

The GUI is deliberately packet-oriented.  It does not duplicate PX4 arming,
estimator, safety-state, flight-phase, or mode-selection logic.  Every command is
sent as MAVLink 2 over UDP and PX4 decides whether it is accepted. The WiFi
module and its UART connection to TELEM2 are transparent to this application.
"""

from __future__ import annotations

import argparse
import math
import queue
import socket
import struct
import threading
import time
from dataclasses import dataclass, replace
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Callable

import tkinter as tk
from tkinter import ttk
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


DEFAULT_CONNECTION = "udp:127.0.0.1:14540"
SITL_MYLINK_UDP_PORT = 14541
CONNECTION_SETTINGS_PATH = Path.home() / ".offboard_control_gui_v3_1_udp"
SOURCE_SYSTEM = 42
SOURCE_COMPONENT = 191
TARGET_SYSTEM = 1
TARGET_COMPONENT = 1
CUSTOM_COMPONENT = 25
MAV_CMD_CUSTOM_ACTION = mavlink2.MAV_CMD_USER_1
CUSTOM_ACTION_SEARCH_TOP = 1
CUSTOM_ACTION_DIRECTION_INTENT = 2
CUSTOM_ACTION_REBASE_COMPLETE = 3
CUSTOM_RESULT_STARTED = 1
CUSTOM_RESULT_BUTTON_CONSUMED = 2
CUSTOM_RESULT_LEGACY_ALLOWED = 3
CUSTOM_RESULT_REBASE_ACCEPTED = 4
CUSTOM_RESULT_HANDOVER_PENDING = 5
CUSTOM_RESULT_CONTACT_PRESS = 6
SETPOINT_RATE_HZ = 10.0
TOP_DISTANCE_UDP_PORT = 14600
TOP_DISTANCE_MAGIC = b"V3LD"
TOP_DISTANCE_VERSION = 1
TOP_DISTANCE_PACKET = struct.Struct("!4sBBHQ4H")
TOP_DISTANCE_ARRAY_MARKER_MASK = 0xF0000000
TOP_DISTANCE_ARRAY_MARKER = 0xA0000000
TOP_DISTANCE_ARRAY_VERSION_MASK = 0x0F000000
TOP_DISTANCE_ARRAY_VERSION = 0x01000000
TOP_DISTANCE_ARRAY_VALID_MASK = 0x00F00000
TOP_DISTANCE_ARRAY_VALID_SHIFT = 20
TOP_DISTANCE_ARRAY_RESERVED_MASK = 0x000F0000
TOP_DISTANCE_ARRAY_SEQUENCE_MASK = 0x0000FFFF
MOTOR_OUTPUT_ARRAY_MARKER_MASK = 0xF0000000
MOTOR_OUTPUT_ARRAY_MARKER = 0xB0000000
MOTOR_OUTPUT_ARRAY_VERSION_MASK = 0x0F000000
MOTOR_OUTPUT_ARRAY_VERSION = 0x01000000
MOTOR_OUTPUT_ARRAY_VALID_MASK = 0x00F00000
MOTOR_OUTPUT_ARRAY_VALID_SHIFT = 20
MOTOR_OUTPUT_ARRAY_ARMED_FLAG = 0x00010000
MOTOR_OUTPUT_ARRAY_RESERVED_MASK = 0x000E0000
MOTOR_OUTPUT_ARRAY_SEQUENCE_MASK = 0x0000FFFF
MOTOR_OUTPUT_ARRAY_SCALE = 1000
MOTOR_OUTPUT_STALE_S = 0.5
XY_MAX_SPEED_M_S = 0.10
XY_MAX_ACCEL_M_S2 = 0.30
Z_MAX_SPEED_UP_M_S = 0.30
Z_MAX_SPEED_DOWN_M_S = 0.25
Z_MAX_ACCEL_M_S2 = 0.30
TAKEOFF_LIFTOFF_HEIGHT_M = 0.35
TAKEOFF_LIFTOFF_SPEED_M_S = 0.50
TAKEOFF_LIFTOFF_ACCEL_M_S2 = 0.75
TAKEOFF_CLIMB_SPEED_M_S = 0.30
TAKEOFF_CLIMB_ACCEL_M_S2 = 0.30
ARRIVAL_POSITION_TOL_M = 0.04
ARRIVAL_VELOCITY_TOL_M_S = 0.04
MIN_ALTITUDE_M = 0.3
MAX_ALTITUDE_M = 3.0
MIN_MOVE_M = 0.1
MAX_MOVE_M = 5.0
HEARTBEAT_TIMEOUT_S = 3.0


def load_saved_connection() -> str:
    try:
        saved = CONNECTION_SETTINGS_PATH.read_text(encoding="utf-8").strip()
    except OSError:
        return DEFAULT_CONNECTION
    return saved if saved.lower().startswith("udp:") else DEFAULT_CONNECTION


def save_successful_connection(connection_string: str) -> None:
    connection = connection_string.strip()
    if not connection.lower().startswith("udp:"):
        raise ValueError("仅保存udp:开头的MAVLink地址")
    temporary = CONNECTION_SETTINGS_PATH.with_suffix(".tmp")
    temporary.write_text(connection + "\n", encoding="utf-8")
    temporary.replace(CONNECTION_SETTINGS_PATH)


def sitl_mylink_connection(connection_string: str) -> str | None:
    """Return the SITL-only MyLink side channel for the standard PX4 endpoint."""
    if not connection_string.lower().startswith("udp:"):
        return None
    try:
        host, port_text = connection_string[4:].rsplit(":", 1)
        port = int(port_text)
    except (ValueError, TypeError):
        return None
    if host.lower() not in {"127.0.0.1", "localhost", "0.0.0.0"} or port != 14540:
        return None
    destination = "127.0.0.1" if host == "0.0.0.0" else host
    return f"udpout:{destination}:{SITL_MYLINK_UDP_PORT}"

POSITION_VELOCITY_MASK = (
    mavlink2.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)
TAKEOFF_POSITION_VELOCITY_MASK = (
    POSITION_VELOCITY_MASK
    & ~mavlink2.POSITION_TARGET_TYPEMASK_YAW_IGNORE
)
HORIZONTAL_VELOCITY_MASK = (
    POSITION_VELOCITY_MASK
    | mavlink2.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavlink2.POSITION_TARGET_TYPEMASK_Y_IGNORE
)
HORIZONTAL_VELOCITY_ACCEL_MASK = (
    HORIZONTAL_VELOCITY_MASK
    & ~mavlink2.POSITION_TARGET_TYPEMASK_AX_IGNORE
    & ~mavlink2.POSITION_TARGET_TYPEMASK_AY_IGNORE
)


def px4_mode_name(custom_mode: int) -> str:
    """Return a compact PX4 mode name from HEARTBEAT.custom_mode."""
    main = (int(custom_mode) >> 16) & 0xFF
    sub = (int(custom_mode) >> 24) & 0xFF
    main_names = {
        1: "MANUAL",
        2: "ALTCTL",
        3: "POSCTL",
        4: "AUTO",
        5: "ACRO",
        6: "OFFBOARD",
        7: "STABILIZED",
        8: "RATTITUDE",
    }
    auto_names = {
        1: "AUTO.READY",
        2: "AUTO.TAKEOFF",
        3: "AUTO.LOITER",
        4: "AUTO.MISSION",
        5: "AUTO.RTL",
        6: "AUTO.LAND",
        8: "AUTO.FOLLOW_TARGET",
        9: "AUTO.PRECLAND",
    }
    return auto_names.get(sub, "AUTO") if main == 4 else main_names.get(main, f"MODE({main},{sub})")


@dataclass(frozen=True)
class MyLinkState:
    connected: bool = False
    system_id: int = 0
    component_id: int = 0
    armed: bool = False
    mode: str = "DISCONNECTED"
    x: float | None = None
    y: float | None = None
    z: float | None = None
    vx: float | None = None
    vy: float | None = None
    vz: float | None = None
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    battery_percent: int | None = None
    battery_voltage_v: float | None = None
    heartbeat_age_s: float | None = None
    local_position_age_s: float | None = None
    # Wire/uORB order: front-right, front-left, rear-right, rear-left.
    top_distances_mm: tuple[int | None, int | None, int | None, int | None] = (None, None, None, None)
    top_distance_age_s: float | None = None
    motor_outputs_normalized: tuple[float | None, float | None, float | None, float | None] = (None, None, None, None)
    motor_output_age_s: float | None = None
    rx_counts: tuple[tuple[str, int], ...] = ()
    tx_setpoints: int = 0
    error: str = ""


@dataclass(frozen=True)
class TopDistanceFrame:
    sequence: int
    timestamp_us: int
    valid_mask: int
    distances_mm: tuple[int, int, int, int]
    received_at: float


@dataclass(frozen=True)
class MotorOutputFrame:
    sequence: int
    valid_mask: int
    armed: bool
    outputs_scaled: tuple[int, int, int, int]
    received_at: float


def decode_top_distance_packet(payload: bytes, received_at: float | None = None) -> TopDistanceFrame:
    if len(payload) != TOP_DISTANCE_PACKET.size:
        raise ValueError(f"unexpected lidar packet size {len(payload)}")
    magic, version, valid_mask, sequence, timestamp_us, *distances_mm = TOP_DISTANCE_PACKET.unpack(payload)
    if magic != TOP_DISTANCE_MAGIC or version != TOP_DISTANCE_VERSION:
        raise ValueError("unknown lidar packet protocol")
    if valid_mask & ~0x0F:
        raise ValueError("invalid lidar packet fields")
    return TopDistanceFrame(
        sequence=sequence,
        timestamp_us=timestamp_us,
        valid_mask=valid_mask,
        distances_mm=tuple(distances_mm),
        received_at=time.monotonic() if received_at is None else received_at,
    )


def decode_top_distance_mavlink(message, received_at: float | None = None) -> TopDistanceFrame | None:
    """Decode the project one-frame/four-distance MAVLink PING transport."""
    if message.get_type() != "PING":
        return None
    metadata = int(message.seq)
    if (
        metadata & TOP_DISTANCE_ARRAY_MARKER_MASK != TOP_DISTANCE_ARRAY_MARKER
        or metadata & TOP_DISTANCE_ARRAY_VERSION_MASK != TOP_DISTANCE_ARRAY_VERSION
        or metadata & TOP_DISTANCE_ARRAY_RESERVED_MASK
    ):
        return None
    valid_mask = (metadata & TOP_DISTANCE_ARRAY_VALID_MASK) >> TOP_DISTANCE_ARRAY_VALID_SHIFT
    packed_distances = int(message.time_usec)
    distances_mm = tuple((packed_distances >> (sensor_id * 16)) & 0xFFFF for sensor_id in range(4))
    return TopDistanceFrame(
        sequence=metadata & TOP_DISTANCE_ARRAY_SEQUENCE_MASK,
        timestamp_us=0,
        valid_mask=valid_mask,
        distances_mm=distances_mm,
        received_at=time.monotonic() if received_at is None else received_at,
    )


def decode_motor_output_mavlink(message, received_at: float | None = None) -> MotorOutputFrame | None:
    """Decode one MyLink MAVLink packet containing four normalized motor outputs."""
    if message.get_type() != "PING":
        return None
    metadata = int(message.seq)
    if (
        metadata & MOTOR_OUTPUT_ARRAY_MARKER_MASK != MOTOR_OUTPUT_ARRAY_MARKER
        or metadata & MOTOR_OUTPUT_ARRAY_VERSION_MASK != MOTOR_OUTPUT_ARRAY_VERSION
        or metadata & MOTOR_OUTPUT_ARRAY_RESERVED_MASK
    ):
        return None
    valid_mask = (metadata & MOTOR_OUTPUT_ARRAY_VALID_MASK) >> MOTOR_OUTPUT_ARRAY_VALID_SHIFT
    packed_outputs = int(message.time_usec)
    outputs_scaled = tuple((packed_outputs >> (motor_index * 16)) & 0xFFFF for motor_index in range(4))
    for motor_index, output in enumerate(outputs_scaled):
        if output > MOTOR_OUTPUT_ARRAY_SCALE:
            valid_mask &= ~(1 << motor_index)
    return MotorOutputFrame(
        sequence=metadata & MOTOR_OUTPUT_ARRAY_SEQUENCE_MASK,
        valid_mask=valid_mask,
        armed=bool(metadata & MOTOR_OUTPUT_ARRAY_ARMED_FLAG),
        outputs_scaled=outputs_scaled,
        received_at=time.monotonic() if received_at is None else received_at,
    )


@dataclass(frozen=True)
class CommandAck:
    command: int
    result: int
    result_param2: int
    source_system: int
    source_component: int

    @property
    def project_request_id(self) -> int:
        return (self.result_param2 >> 8) & 0xFFFF

    @property
    def project_result(self) -> int:
        return self.result_param2 & 0xFF


@dataclass(frozen=True)
class TrajectoryState:
    final_target: tuple[float, float, float]
    command_position: tuple[float, float, float]
    command_velocity: tuple[float, float, float]
    command_acceleration: tuple[float, float, float]
    arrived: bool
    takeoff_state: "TakeoffState"
    horizontal_position_hold: bool
    takeoff_yaw: float | None


class TakeoffState(str, Enum):
    IDLE = "TAKEOFF_IDLE"
    LIFTOFF = "TAKEOFF_LIFTOFF"
    CLIMB = "TAKEOFF_CLIMB"
    BRAKE = "TAKEOFF_BRAKE"
    HOLD = "TAKEOFF_HOLD"


class SetpointTrajectoryGenerator:
    """Generate a bounded-velocity, bounded-acceleration Local-NED setpoint."""

    def __init__(
        self,
        *,
        xy_max_speed: float = XY_MAX_SPEED_M_S,
        xy_max_accel: float = XY_MAX_ACCEL_M_S2,
        z_max_speed_up: float = Z_MAX_SPEED_UP_M_S,
        z_max_speed_down: float = Z_MAX_SPEED_DOWN_M_S,
        z_max_accel: float = Z_MAX_ACCEL_M_S2,
        arrival_position_tol: float = ARRIVAL_POSITION_TOL_M,
        arrival_velocity_tol: float = ARRIVAL_VELOCITY_TOL_M_S,
    ) -> None:
        limits = (
            xy_max_speed,
            xy_max_accel,
            z_max_speed_up,
            z_max_speed_down,
            z_max_accel,
            arrival_position_tol,
            arrival_velocity_tol,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError("trajectory limits must be finite and positive")
        self.xy_max_speed = float(xy_max_speed)
        self.xy_max_accel = float(xy_max_accel)
        self.z_max_speed_up = float(z_max_speed_up)
        self.z_max_speed_down = float(z_max_speed_down)
        self.z_max_accel = float(z_max_accel)
        self.arrival_position_tol = float(arrival_position_tol)
        self.arrival_velocity_tol = float(arrival_velocity_tol)
        self._initialized = False
        self._final_target = (0.0, 0.0, 0.0)
        self._command_position = (0.0, 0.0, 0.0)
        self._command_velocity = (0.0, 0.0, 0.0)
        self._command_acceleration = (0.0, 0.0, 0.0)
        self._takeoff_state = TakeoffState.IDLE
        self._takeoff_start_z = 0.0
        self._takeoff_liftoff_z = 0.0
        self._xy_position_hold = True
        self._takeoff_yaw: float | None = None

    @staticmethod
    def _finite_position(position: tuple[float, float, float]) -> tuple[float, float, float]:
        values = tuple(float(value) for value in position)
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError("Local NED position must contain three finite values")
        return values

    @staticmethod
    def _approach_vector(
        current: tuple[float, float],
        target: tuple[float, float],
        max_delta: float,
    ) -> tuple[float, float]:
        dx = target[0] - current[0]
        dy = target[1] - current[1]
        magnitude = math.hypot(dx, dy)
        if magnitude <= max_delta or magnitude <= 1e-9:
            return target
        scale = max_delta / magnitude
        return current[0] + dx * scale, current[1] + dy * scale

    @staticmethod
    def _approach_scalar(current: float, target: float, max_delta: float) -> float:
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)

    def reset(self, actual_position: tuple[float, float, float]) -> TrajectoryState:
        position = self._finite_position(actual_position)
        self._initialized = True
        self._final_target = position
        self._command_position = position
        self._command_velocity = (0.0, 0.0, 0.0)
        self._command_acceleration = (0.0, 0.0, 0.0)
        self._takeoff_state = TakeoffState.IDLE
        self._xy_position_hold = True
        self._takeoff_yaw = None
        return self.snapshot()

    def set_final_target(self, target: tuple[float, float, float]) -> TrajectoryState:
        target = self._finite_position(target)
        if not self._initialized:
            self.reset(target)
        self._final_target = target
        self._takeoff_state = TakeoffState.IDLE
        self._xy_position_hold = True
        self._takeoff_yaw = None
        return self.snapshot()

    def start_takeoff(
        self,
        actual_position: tuple[float, float, float],
        altitude: float,
        yaw: float,
    ) -> TrajectoryState:
        actual = self._finite_position(actual_position)
        altitude = float(altitude)
        if not math.isfinite(altitude) or altitude <= 0.0:
            raise ValueError("takeoff altitude must be finite and positive")
        yaw = float(yaw)
        if not math.isfinite(yaw):
            raise ValueError("takeoff yaw must be finite")

        self._initialized = True
        self._takeoff_start_z = actual[2]
        final_z = actual[2] - altitude
        self._takeoff_liftoff_z = max(
            final_z,
            actual[2] - TAKEOFF_LIFTOFF_HEIGHT_M,
        )
        self._final_target = (actual[0], actual[1], final_z)
        self._command_position = actual
        self._command_velocity = (0.0, 0.0, 0.0)
        self._command_acceleration = (0.0, 0.0, 0.0)
        self._takeoff_state = TakeoffState.LIFTOFF
        self._xy_position_hold = True
        self._takeoff_yaw = yaw
        return self.snapshot()

    def offset_final_target(
        self,
        delta: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        delta = self._finite_position(delta)
        old = self._final_target
        new = tuple(value + change for value, change in zip(old, delta))
        self._final_target = new
        self._takeoff_state = TakeoffState.IDLE
        self._takeoff_yaw = None
        if abs(delta[0]) > 1e-9 or abs(delta[1]) > 1e-9:
            self._xy_position_hold = False
        return old, new

    def snapshot(self) -> TrajectoryState:
        position_error = math.sqrt(sum(
            (target - command) ** 2
            for target, command in zip(self._final_target, self._command_position)
        ))
        velocity = math.sqrt(sum(value * value for value in self._command_velocity))
        return TrajectoryState(
            final_target=self._final_target,
            command_position=self._command_position,
            command_velocity=self._command_velocity,
            command_acceleration=self._command_acceleration,
            arrived=(
                position_error <= self.arrival_position_tol
                and velocity <= self.arrival_velocity_tol
                and self._xy_position_hold
            ),
            takeoff_state=self._takeoff_state,
            horizontal_position_hold=self._xy_position_hold,
            takeoff_yaw=self._takeoff_yaw,
        )

    def takeoff_active(self) -> bool:
        return self._takeoff_state in {
            TakeoffState.LIFTOFF,
            TakeoffState.CLIMB,
            TakeoffState.BRAKE,
        }

    def _update_takeoff_z(self, pz: float, vz: float, dt: float) -> tuple[float, float]:
        final_z = self._final_target[2]
        remaining = max(0.0, pz - final_z)

        if (
            remaining <= self.arrival_position_tol
            and abs(vz) <= self.arrival_velocity_tol
        ):
            self._takeoff_state = TakeoffState.HOLD
            return final_z, 0.0

        if self._takeoff_state is TakeoffState.LIFTOFF:
            climb_brake_distance = (vz * vz) / (2.0 * TAKEOFF_CLIMB_ACCEL_M_S2)
            if remaining <= climb_brake_distance + abs(vz) * dt:
                self._takeoff_state = TakeoffState.BRAKE
            elif pz <= self._takeoff_liftoff_z:
                self._takeoff_state = TakeoffState.CLIMB

        if self._takeoff_state is TakeoffState.CLIMB:
            brake_speed = math.sqrt(2.0 * TAKEOFF_CLIMB_ACCEL_M_S2 * remaining)
            if brake_speed <= TAKEOFF_CLIMB_SPEED_M_S:
                self._takeoff_state = TakeoffState.BRAKE

        if self._takeoff_state is TakeoffState.LIFTOFF:
            max_speed = TAKEOFF_LIFTOFF_SPEED_M_S
            max_accel = TAKEOFF_LIFTOFF_ACCEL_M_S2
            brake_speed = max(
                0.0,
                math.sqrt(2.0 * max_accel * remaining) - 2.0 * max_accel * dt,
            )
        else:
            max_speed = TAKEOFF_CLIMB_SPEED_M_S
            max_accel = TAKEOFF_CLIMB_ACCEL_M_S2
            brake_speed = max(
                0.0,
                math.sqrt(2.0 * max_accel * remaining) - 2.0 * max_accel * dt,
            )

        desired_vz = -min(max_speed, brake_speed)
        vz = self._approach_scalar(vz, desired_vz, max_accel * dt)
        next_pz = pz + vz * dt
        if next_pz <= final_z:
            self._takeoff_state = TakeoffState.HOLD
            return final_z, 0.0
        return next_pz, vz

    def update(
        self,
        actual_position: tuple[float, float, float],
        dt: float,
        actual_velocity: tuple[float, float, float] | None = None,
    ) -> TrajectoryState:
        actual_position = self._finite_position(actual_position)
        if actual_velocity is None:
            actual_velocity = self._command_velocity
        else:
            actual_velocity = self._finite_position(actual_velocity)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("trajectory dt must be finite and positive")
        if not self._initialized:
            return self.reset(actual_position)

        px, py, pz = self._command_position
        vx, vy, vz = self._command_velocity
        previous_velocity = self._command_velocity
        tx, ty, tz = self._final_target

        reference_x, reference_y = (
            (px, py) if self._xy_position_hold else actual_position[:2]
        )
        ex = tx - reference_x
        ey = ty - reference_y
        distance_xy = math.hypot(ex, ey)
        actual_speed_xy = math.hypot(actual_velocity[0], actual_velocity[1])
        speed_xy = math.hypot(vx, vy)
        arrival_speed = speed_xy if self._xy_position_hold else actual_speed_xy
        if distance_xy <= self.arrival_position_tol and arrival_speed <= self.arrival_velocity_tol:
            px, py = tx, ty
            vx, vy = 0.0, 0.0
            self._xy_position_hold = True
        else:
            brake_speed = max(
                0.0,
                math.sqrt(2.0 * self.xy_max_accel * distance_xy)
                - 2.0 * self.xy_max_accel * dt,
            )
            desired_speed = min(self.xy_max_speed, brake_speed)
            if distance_xy > 1e-9:
                desired_velocity_xy = (
                    ex / distance_xy * desired_speed,
                    ey / distance_xy * desired_speed,
                )
            else:
                desired_velocity_xy = (0.0, 0.0)
            vx, vy = self._approach_vector(
                (vx, vy),
                desired_velocity_xy,
                self.xy_max_accel * dt,
            )
            if self._xy_position_hold:
                next_px = px + vx * dt
                next_py = py + vy * dt
                remaining_x = tx - next_px
                remaining_y = ty - next_py
                if distance_xy <= math.hypot(vx * dt, vy * dt) or ex * remaining_x + ey * remaining_y <= 0.0:
                    px, py = tx, ty
                    vx, vy = 0.0, 0.0
                else:
                    px, py = next_px, next_py
            else:
                # Values are carried for diagnostics; XY position fields are masked while moving.
                px, py = tx, ty
            if (
                distance_xy <= self.arrival_position_tol
                and arrival_speed <= self.arrival_velocity_tol
            ):
                px, py = tx, ty
                vx, vy = 0.0, 0.0
                self._xy_position_hold = True

        error_z = tz - pz
        if self.takeoff_active():
            pz, vz = self._update_takeoff_z(pz, vz, dt)
        elif self._takeoff_state is TakeoffState.HOLD:
            pz, vz = tz, 0.0
        elif abs(error_z) <= self.arrival_position_tol and abs(vz) <= self.arrival_velocity_tol:
            pz = tz
            vz = 0.0
        else:
            max_speed_z = self.z_max_speed_up if error_z < 0.0 else self.z_max_speed_down
            brake_speed_z = max(
                0.0,
                math.sqrt(2.0 * self.z_max_accel * abs(error_z))
                - 2.0 * self.z_max_accel * dt,
            )
            desired_vz = math.copysign(min(max_speed_z, brake_speed_z), error_z)
            vz = self._approach_scalar(vz, desired_vz, self.z_max_accel * dt)
            next_pz = pz + vz * dt
            if abs(error_z) <= abs(vz * dt) or error_z * (tz - next_pz) <= 0.0:
                pz = tz
                vz = 0.0
            else:
                pz = next_pz
            if abs(tz - pz) <= self.arrival_position_tol and abs(vz) <= self.arrival_velocity_tol:
                pz = tz
                vz = 0.0

        self._command_position = (px, py, pz)
        self._command_velocity = (vx, vy, vz)
        self._command_acceleration = tuple(
            (current - previous) / dt
            for current, previous in zip(self._command_velocity, previous_velocity)
        )
        return self.snapshot()


class MyLinkMavlinkClient:
    """MAVLink 2 UDP client with exactly one receive loop."""

    def __init__(
        self,
        connection_string: str,
        *,
        source_system: int = SOURCE_SYSTEM,
        source_component: int = SOURCE_COMPONENT,
        target_system: int = TARGET_SYSTEM,
        target_component: int = TARGET_COMPONENT,
        setpoint_rate_hz: float = SETPOINT_RATE_HZ,
        lidar_udp_port: int = TOP_DISTANCE_UDP_PORT,
        logger: Callable[[str], None] | None = None,
        trace_logger: Callable[[str], None] | None = None,
    ) -> None:
        self.connection_string = connection_string.strip() or DEFAULT_CONNECTION
        self.source_system = int(source_system)
        self.source_component = int(source_component)
        self.target_system = int(target_system)
        self.target_component = int(target_component)
        self.setpoint_rate_hz = float(setpoint_rate_hz)
        self.lidar_udp_port = int(lidar_udp_port)
        self.log = logger or (lambda _message: None)
        self.trace_log = trace_logger or (lambda _message: None)

        self._connection = None
        self._custom_connection = None
        self._mav = mavlink2.MAVLink(
            None,
            srcSystem=self.source_system,
            srcComponent=self.source_component,
        )
        self._stop = threading.Event()
        self._tx_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._state_changed = threading.Condition(self._state_lock)
        self._ack_changed = threading.Condition()
        self._acks: list[CommandAck] = []
        self._rx_counts: dict[str, int] = {}
        self._state = MyLinkState()
        self._last_heartbeat: float | None = None
        self._last_local_position: float | None = None
        self._trajectory_lock = threading.Lock()
        self._trajectory = SetpointTrajectoryGenerator()
        self._last_setpoint_time: float | None = None
        self._last_stream_mode: str | None = None
        self._offboard_yaw_ref: float | None = None
        self._setpoints_enabled = False
        self._setpoint_count = 0
        self._command_tx_count = 0
        self._heartbeat_tx_count = 0
        self._legacy_output_allowed = True
        self._project_request_id = 0
        self._project_lock = threading.Lock()
        self._outstanding_project_requests: set[int] = set()
        self._rebase_lock = threading.Lock()
        self._threads: list[threading.Thread] = []
        self._search_top_active = False
        self._lidar_socket: socket.socket | None = None
        self._lidar_lock = threading.Lock()
        self._top_distance_frame: TopDistanceFrame | None = None
        self._top_distance_rx_count = 0
        self._top_distance_logged = False
        self._motor_lock = threading.Lock()
        self._motor_output_frame: MotorOutputFrame | None = None
        self._motor_output_rx_count = 0
        self._motor_output_logged = False

    def start(self, timeout_s: float = 12.0) -> None:
        if self._connection is not None:
            return
        self._connection = mavutil.mavlink_connection(
            self.connection_string,
            source_system=self.source_system,
            source_component=self.source_component,
            dialect="common",
            autoreconnect=True,
            force_connected=True,
        )
        custom_endpoint = sitl_mylink_connection(self.connection_string)
        if custom_endpoint is not None:
            self._custom_connection = mavutil.mavlink_connection(
                custom_endpoint,
                source_system=self.source_system,
                source_component=self.source_component,
                dialect="common",
                force_connected=True,
            )
        lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        lidar_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lidar_socket.bind(("0.0.0.0", self.lidar_udp_port))
        lidar_socket.settimeout(0.2)
        self._lidar_socket = lidar_socket
        self._stop.clear()
        self._legacy_output_allowed = True
        self._search_top_active = False
        with self._lidar_lock:
            self._top_distance_frame = None
        with self._motor_lock:
            self._motor_output_frame = None
        self._threads = [
            threading.Thread(target=self._receive_loop, name="mylink-rx", daemon=True),
            threading.Thread(target=self._transmit_loop, name="mylink-tx", daemon=True),
            threading.Thread(target=self._lidar_receive_loop, name="cart-lidar-rx", daemon=True),
        ]
        if self._custom_connection is not None:
            self._threads.append(
                threading.Thread(target=self._custom_receive_loop, name="mylink-custom-rx", daemon=True)
            )
        for thread in self._threads:
            thread.start()
        self.wait_until(lambda state: state.connected, timeout_s, "MyLink HEARTBEAT")
        deadline = time.monotonic() + timeout_s
        while self._heartbeat_tx_count < 3:
            if time.monotonic() >= deadline:
                raise TimeoutError("等待 GCS HEARTBEAT 预热超时")
            time.sleep(0.02)
        self.log(
            f"[OK] PX4 connected {self.connection_string} "
            f"SYS={self.target_system} COMP={self.target_component}"
        )

    def close(self) -> None:
        self._stop.set()
        self._legacy_output_allowed = False
        self.clear_search_top_active()
        lidar_socket = self._lidar_socket
        self._lidar_socket = None
        if lidar_socket is not None:
            lidar_socket.close()
        for thread in self._threads:
            thread.join(timeout=1.0)
        self._threads.clear()
        connection = self._connection
        self._connection = None
        if connection is not None:
            connection.close()
        custom_connection = self._custom_connection
        self._custom_connection = None
        if custom_connection is not None:
            custom_connection.close()
        with self._state_changed:
            self._state = replace(self._state, connected=False, mode="DISCONNECTED")
            self._state_changed.notify_all()

    def snapshot(self) -> MyLinkState:
        with self._state_lock:
            now = time.monotonic()
            heartbeat_age = None if self._last_heartbeat is None else now - self._last_heartbeat
            local_age = None if self._last_local_position is None else now - self._last_local_position
            distances, top_age = self._top_distance_snapshot(now)
            motor_outputs, motor_age = self._motor_output_snapshot(now)
            return replace(
                self._state,
                connected=self._state.connected and heartbeat_age is not None and heartbeat_age <= HEARTBEAT_TIMEOUT_S,
                heartbeat_age_s=heartbeat_age,
                local_position_age_s=local_age,
                top_distances_mm=distances,
                top_distance_age_s=top_age,
                motor_outputs_normalized=motor_outputs,
                motor_output_age_s=motor_age,
                rx_counts=tuple(sorted(self._rx_counts.items())),
                tx_setpoints=self._setpoint_count,
            )

    def wait_until(
        self,
        predicate: Callable[[MyLinkState], bool],
        timeout_s: float,
        description: str,
    ) -> MyLinkState:
        deadline = time.monotonic() + timeout_s
        with self._state_changed:
            while True:
                state = self.snapshot_unlocked()
                if predicate(state):
                    return state
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"等待 {description} 超时；当前状态={state}")
                self._state_changed.wait(min(remaining, 0.25))

    def snapshot_unlocked(self) -> MyLinkState:
        now = time.monotonic()
        heartbeat_age = None if self._last_heartbeat is None else now - self._last_heartbeat
        local_age = None if self._last_local_position is None else now - self._last_local_position
        distances, top_age = self._top_distance_snapshot(now)
        motor_outputs, motor_age = self._motor_output_snapshot(now)
        return replace(
            self._state,
            connected=self._state.connected and heartbeat_age is not None and heartbeat_age <= HEARTBEAT_TIMEOUT_S,
            heartbeat_age_s=heartbeat_age,
            local_position_age_s=local_age,
            top_distances_mm=distances,
            top_distance_age_s=top_age,
            motor_outputs_normalized=motor_outputs,
            motor_output_age_s=motor_age,
            rx_counts=tuple(sorted(self._rx_counts.items())),
            tx_setpoints=self._setpoint_count,
        )

    def reset_trajectory(
        self,
        position: tuple[float, float, float],
        *,
        announce: bool = False,
        stream_mode: str | None = None,
    ) -> TrajectoryState:
        with self._trajectory_lock:
            state = self._trajectory.reset(position)
            self._last_setpoint_time = None
            if stream_mode is not None:
                self._last_stream_mode = stream_mode.upper()
                if self._last_stream_mode != "OFFBOARD":
                    self._offboard_yaw_ref = None
        if announce:
            self.log(
                f"[TARGET] reset N={position[0]:.3f} E={position[1]:.3f} D={position[2]:.3f}"
            )
        return state

    def prepare_offboard_target(self, position: tuple[float, float, float]) -> None:
        with self._trajectory_lock:
            if self._last_stream_mode != "OFFBOARD":
                self._trajectory.reset(position)
                self._last_setpoint_time = None
                self._last_stream_mode = "OFFBOARD"

    def set_final_target(
        self,
        target: tuple[float, float, float],
    ) -> TrajectoryState:
        with self._trajectory_lock:
            return self._trajectory.set_final_target(target)

    def offset_final_target(
        self,
        delta: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        with self._trajectory_lock:
            return self._trajectory.offset_final_target(delta)

    def start_takeoff(
        self,
        actual_position: tuple[float, float, float],
        altitude: float,
    ) -> TrajectoryState:
        with self._trajectory_lock:
            if self._offboard_yaw_ref is None:
                raise ValueError("起飞前没有有效航向，拒绝发送起飞设定值")
            state = self._trajectory.start_takeoff(
                actual_position,
                altitude,
                self._offboard_yaw_ref,
            )
            self._last_setpoint_time = None
            self._last_stream_mode = "OFFBOARD"
            return state

    def takeoff_active(self) -> bool:
        with self._trajectory_lock:
            return self._trajectory.takeoff_active()

    def trajectory(self) -> TrajectoryState:
        with self._trajectory_lock:
            return self._trajectory.snapshot()

    def start_setpoints(self) -> None:
        with self._trajectory_lock:
            self._last_setpoint_time = None
        self._setpoints_enabled = True

    def stop_setpoints(self) -> None:
        self._setpoints_enabled = False

    def wait_setpoints(self, fresh_count: int, timeout_s: float = 5.0) -> None:
        start = self._setpoint_count
        deadline = time.monotonic() + timeout_s
        while self._setpoint_count - start < fresh_count:
            if time.monotonic() >= deadline:
                raise TimeoutError("等待 Offboard 预热设定值超时")
            time.sleep(0.02)

    def send_command_long(
        self,
        command: int,
        params: list[float],
        timeout_s: float = 6.0,
        *,
        target_component: int | None = None,
        expected_project_request_id: int | None = None,
    ) -> CommandAck:
        if len(params) != 7:
            raise ValueError("COMMAND_LONG 必须提供7个参数")
        with self._ack_changed:
            start = len(self._acks)
        message = self._mav.command_long_encode(
            self.target_system,
            self.target_component if target_component is None else int(target_component),
            int(command),
            0,
            *[float(value) for value in params],
        )
        self._command_tx_count += 1
        if target_component == CUSTOM_COMPONENT:
            self._write_custom(message)
        else:
            self._write(message)
        deadline = time.monotonic() + timeout_s
        with self._ack_changed:
            while True:
                for ack in self._acks[start:]:
                    request_matches = (
                        expected_project_request_id is None
                        or ack.project_request_id == expected_project_request_id
                    )
                    if (
                        ack.command == command
                        and request_matches
                        and ack.result != mavlink2.MAV_RESULT_IN_PROGRESS
                    ):
                        return ack
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"COMMAND_ACK timeout command={command}")
                self._ack_changed.wait(min(remaining, 0.25))

    def command_tx_count(self) -> int:
        return self._command_tx_count

    def next_project_request_id(self) -> int:
        with self._project_lock:
            self._project_request_id = (self._project_request_id % 0xFFFF) + 1
            return self._project_request_id

    def send_project_command(self, action: int, value: float = 0.0) -> CommandAck:
        request_id = self.next_project_request_id()
        with self._project_lock:
            self._outstanding_project_requests.add(request_id)
        try:
            return self.send_command_long(
                MAV_CMD_CUSTOM_ACTION,
                [float(action), float(value), float(request_id), 0, 0, 0, 0],
                target_component=CUSTOM_COMPONENT,
                expected_project_request_id=request_id,
            )
        finally:
            with self._project_lock:
                self._outstanding_project_requests.discard(request_id)

    def block_legacy_output(self) -> None:
        self._legacy_output_allowed = False

    def allow_legacy_output(self) -> None:
        self._legacy_output_allowed = True

    def direction_intent(self, direction_id: int) -> CommandAck:
        return self.send_project_command(CUSTOM_ACTION_DIRECTION_INTENT, float(direction_id))

    def start_search_top(self) -> CommandAck:
        return self.send_project_command(CUSTOM_ACTION_SEARCH_TOP)

    def search_top_active(self) -> bool:
        return self._search_top_active

    def clear_search_top_active(self) -> None:
        self._search_top_active = False

    def land(self) -> int:
        self.block_legacy_output()
        return self.send_command_long(
            mavlink2.MAV_CMD_NAV_LAND,
            [0, 0, 0, math.nan, math.nan, math.nan, 0],
        ).result

    def _write(self, message) -> None:
        connection = self._connection
        if connection is None:
            raise RuntimeError("MAVLink UDP 未连接")
        with self._tx_lock:
            connection.mav.send(message, force_mavlink1=False)

    def _write_custom(self, message) -> None:
        connection = self._custom_connection or self._connection
        if connection is None:
            raise RuntimeError("MAVLink UDP 未连接")
        with self._tx_lock:
            connection.mav.send(message, force_mavlink1=False)

    def _send_heartbeat(self) -> None:
        heartbeat = self._mav.heartbeat_encode(
            mavlink2.MAV_TYPE_GCS,
            mavlink2.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavlink2.MAV_STATE_ACTIVE,
        )
        self._write(heartbeat)
        # In SITL, MyLink is a dedicated UDP side channel. A heartbeat makes
        # its return peer known immediately so monitoring data arrives before
        # the operator presses SEARCH TOP. Real hardware uses the primary
        # TELEM2/WiFi connection and therefore sends only once.
        if self._custom_connection is not None:
            self._write_custom(heartbeat)
        self._heartbeat_tx_count += 1

    def _top_distance_snapshot(
        self, now: float
    ) -> tuple[tuple[int | None, int | None, int | None, int | None], float | None]:
        with self._lidar_lock:
            frame = self._top_distance_frame
        if frame is None:
            return (None, None, None, None), None
        distances = tuple(
            millimetres if frame.valid_mask & (1 << sensor_id) else None
            for sensor_id, millimetres in enumerate(frame.distances_mm)
        )
        return distances, max(0.0, now - frame.received_at)

    def _accept_top_distance_frame(self, frame: TopDistanceFrame, source: str) -> None:
        with self._lidar_lock:
            previous = self._top_distance_frame
            if previous is not None and previous.sequence == frame.sequence:
                return
            self._top_distance_frame = frame
            self._top_distance_rx_count += 1
        if not self._top_distance_logged:
            self._top_distance_logged = True
            self.log(f"[LIDAR] four-distance stream received via {source}")

    def _motor_output_snapshot(
        self, now: float
    ) -> tuple[tuple[float | None, float | None, float | None, float | None], float | None]:
        with self._motor_lock:
            frame = self._motor_output_frame
        if frame is None:
            return (None, None, None, None), None
        age = max(0.0, now - frame.received_at)
        if age > MOTOR_OUTPUT_STALE_S:
            return (None, None, None, None), age
        outputs = tuple(
            scaled / MOTOR_OUTPUT_ARRAY_SCALE if frame.valid_mask & (1 << motor_index) else None
            for motor_index, scaled in enumerate(frame.outputs_scaled)
        )
        return outputs, age

    def _accept_motor_output_frame(self, frame: MotorOutputFrame, source: str) -> None:
        with self._motor_lock:
            previous = self._motor_output_frame
            if previous is not None and previous.sequence == frame.sequence:
                return
            self._motor_output_frame = frame
            self._motor_output_rx_count += 1
        if not self._motor_output_logged:
            self._motor_output_logged = True
            self.log(f"[MOTOR] normalized four-motor stream received via {source}")

    def _lidar_receive_loop(self) -> None:
        while not self._stop.is_set():
            lidar_socket = self._lidar_socket
            if lidar_socket is None:
                return
            try:
                payload, _source = lidar_socket.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                frame = decode_top_distance_packet(payload)
            except ValueError as exc:
                self.log(f"[LIDAR] ignored packet: {exc}")
                continue
            self._accept_top_distance_frame(frame, f"legacy UDP {self.lidar_udp_port}")

    def _send_setpoint(self) -> None:
        if not self._legacy_output_allowed:
            return

        now = time.monotonic()
        state = self.snapshot()
        actual = (state.x, state.y, state.z)
        if any(value is None for value in actual):
            return
        actual_position = tuple(float(value) for value in actual)
        mode = state.mode.upper()
        with self._trajectory_lock:
            entering_offboard = self._last_stream_mode != "OFFBOARD" and mode == "OFFBOARD"
            if mode != "OFFBOARD":
                self._offboard_yaw_ref = None
            elif self._trajectory.snapshot().takeoff_yaw is None:
                yaw = state.yaw
                if yaw is not None and math.isfinite(float(yaw)):
                    self._offboard_yaw_ref = float(yaw)
            if mode != "OFFBOARD" or entering_offboard:
                self._trajectory.reset(actual_position)
            nominal_dt = 1.0 / self.setpoint_rate_hz
            if self._last_setpoint_time is None:
                dt = nominal_dt
            else:
                dt = min(max(now - self._last_setpoint_time, nominal_dt * 0.5), nominal_dt * 2.0)
            measured_velocity = (state.vx, state.vy, state.vz)
            actual_velocity = (
                tuple(float(value) for value in measured_velocity)
                if all(value is not None and math.isfinite(value) for value in measured_velocity)
                else None
            )
            generated = self._trajectory.update(actual_position, dt, actual_velocity)
            self._last_setpoint_time = now
            self._last_stream_mode = mode

        x, y, z = generated.command_position
        vx, vy, vz = generated.command_velocity
        ax, ay, az = generated.command_acceleration
        takeoff_yaw = generated.takeoff_yaw
        if takeoff_yaw is not None:
            type_mask = TAKEOFF_POSITION_VELOCITY_MASK
            yaw = takeoff_yaw
        else:
            type_mask = (
                POSITION_VELOCITY_MASK
                if generated.horizontal_position_hold
                else HORIZONTAL_VELOCITY_ACCEL_MASK
            )
            yaw = 0.0
        seq = self._setpoint_count + 1
        self._write(self._mav.set_position_target_local_ned_encode(
            int(now * 1000) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            mavlink2.MAV_FRAME_LOCAL_NED,
            type_mask,
            x,
            y,
            z,
            vx,
            vy,
            vz,
            ax,
            ay,
            0,
            yaw,
            0.0,
        ))
        self._setpoint_count = seq
        actual_velocity = (state.vx, state.vy, state.vz)
        self.trace_log(
            "[TRACE] "
            f"seq={seq} monotonic_timestamp={now:.6f} timestamp={now:.3f} mode={mode} "
            f"takeoff_state={generated.takeoff_state.value} "
            f"horizontal_mode={'POSITION_HOLD' if generated.horizontal_position_hold else 'VELOCITY_MOVE'} "
            f"type_mask={type_mask} "
            f"actual=({actual_position[0]:.3f},{actual_position[1]:.3f},{actual_position[2]:.3f}) "
            f"actual_velocity=({self._trace_value(actual_velocity[0])},"
            f"{self._trace_value(actual_velocity[1])},{self._trace_value(actual_velocity[2])}) "
            f"final_target=({generated.final_target[0]:.3f},{generated.final_target[1]:.3f},"
            f"{generated.final_target[2]:.3f}) "
            f"command_position=({x:.3f},{y:.3f},{z:.3f}) "
            f"command_velocity=({vx:.3f},{vy:.3f},{vz:.3f}) "
            f"command_acceleration=({ax:.3f},{ay:.3f},{az:.3f}) "
            f"roll_deg={self._trace_angle(state.roll)} pitch_deg={self._trace_angle(state.pitch)}"
        )

    @staticmethod
    def _trace_value(value: float | None) -> str:
        return "nan" if value is None or not math.isfinite(value) else f"{value:.3f}"

    @staticmethod
    def _trace_angle(value: float | None) -> str:
        return "nan" if value is None or not math.isfinite(value) else f"{math.degrees(value):.2f}"

    def _transmit_loop(self) -> None:
        next_heartbeat = 0.0
        next_setpoint = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            try:
                if now >= next_heartbeat:
                    self._send_heartbeat()
                    next_heartbeat = now + 1.0
                if self._setpoints_enabled and now >= next_setpoint:
                    self._send_setpoint()
                    next_setpoint = now + 1.0 / self.setpoint_rate_hz
            except OSError as exc:
                self._record_error(str(exc))
                return
            time.sleep(0.005)

    def _receive_loop(self) -> None:
        connection = self._connection
        if connection is None:
            return
        while not self._stop.is_set():
            try:
                message = connection.recv_match(blocking=True, timeout=0.1)
            except OSError as exc:
                self._record_error(str(exc))
                return
            if message is not None and message.get_type() != "BAD_DATA":
                self._handle_message(message)

    def _custom_receive_loop(self) -> None:
        connection = self._custom_connection
        if connection is None:
            return
        while not self._stop.is_set():
            try:
                message = connection.recv_match(blocking=True, timeout=0.1)
            except OSError as exc:
                self._record_error(f"MyLink SITL UDP: {exc}")
                return
            if message is not None and message.get_type() != "BAD_DATA":
                self._handle_message(message)

    def _record_error(self, error: str) -> None:
        with self._state_changed:
            self._state = replace(self._state, error=error)
            self._state_changed.notify_all()
        self.log(f"[ERROR] MAVLink UDP: {error}")

    def _handle_message(self, message) -> None:
        name = message.get_type()
        now = time.monotonic()
        if name == "PING":
            top_distance_frame = decode_top_distance_mavlink(message, received_at=now)
            if top_distance_frame is not None:
                self._accept_top_distance_frame(top_distance_frame, "MyLink MAVLink")
            motor_output_frame = decode_motor_output_mavlink(message, received_at=now)
            if motor_output_frame is not None:
                self._accept_motor_output_frame(motor_output_frame, "MyLink MAVLink")
        if name == "HEARTBEAT" and (
            int(message.type) == mavlink2.MAV_TYPE_GCS
            or int(message.autopilot) == mavlink2.MAV_AUTOPILOT_INVALID
        ):
            return
        with self._state_changed:
            self._rx_counts[name] = self._rx_counts.get(name, 0) + 1
            if name == "HEARTBEAT":
                self.target_system = message.get_srcSystem() or self.target_system
                self.target_component = message.get_srcComponent() or self.target_component
                self._last_heartbeat = now
                self._state = replace(
                    self._state,
                    connected=True,
                    system_id=self.target_system,
                    component_id=self.target_component,
                    armed=bool(message.base_mode & mavlink2.MAV_MODE_FLAG_SAFETY_ARMED),
                    mode=px4_mode_name(message.custom_mode),
                )
            elif name == "LOCAL_POSITION_NED":
                self._last_local_position = now
                self._state = replace(
                    self._state,
                    x=float(message.x),
                    y=float(message.y),
                    z=float(message.z),
                    vx=float(message.vx),
                    vy=float(message.vy),
                    vz=float(message.vz),
                )
            elif name == "ATTITUDE":
                self._state = replace(
                    self._state,
                    roll=float(message.roll),
                    pitch=float(message.pitch),
                    yaw=float(message.yaw),
                )
            elif name == "BATTERY_STATUS":
                voltages = [value for value in message.voltages if value not in (0, 0xFFFF)]
                voltage = None if not voltages else sum(voltages) / 1000.0
                remaining = None if int(message.battery_remaining) < 0 else int(message.battery_remaining)
                self._state = replace(
                    self._state,
                    battery_percent=remaining,
                    battery_voltage_v=voltage,
                )
            self._state_changed.notify_all()
        if name == "COMMAND_ACK":
            if (
                int(message.command) == MAV_CMD_CUSTOM_ACTION
                and self._custom_connection is not None
                and int(message.get_srcComponent()) != CUSTOM_COMPONENT
            ):
                return
            ack = CommandAck(
                command=int(message.command),
                result=int(message.result),
                result_param2=int(getattr(message, "result_param2", 0)),
                source_system=int(message.get_srcSystem()),
                source_component=int(message.get_srcComponent()),
            )
            with self._ack_changed:
                self._acks.append(ack)
                self._ack_changed.notify_all()
            self._handle_project_ack(ack)

    def _handle_project_ack(self, ack: CommandAck) -> None:
        if ack.command != MAV_CMD_CUSTOM_ACTION:
            return

        result = ack.project_result
        if result in {
            CUSTOM_RESULT_STARTED,
            CUSTOM_RESULT_BUTTON_CONSUMED,
            CUSTOM_RESULT_HANDOVER_PENDING,
            CUSTOM_RESULT_CONTACT_PRESS,
        }:
            self.block_legacy_output()

        if result == CUSTOM_RESULT_STARTED:
            self._search_top_active = True
            self.log(f"[CUSTOM1] accepted request={ack.project_request_id}; PX4 owns control")
        elif result == CUSTOM_RESULT_BUTTON_CONSUMED:
            self.clear_search_top_active()
        elif result == CUSTOM_RESULT_CONTACT_PRESS:
            self.clear_search_top_active()
            self.log("[CONTACT_PRESS] PX4 confirmed stable contact; pressure ramp active")
        elif result == CUSTOM_RESULT_HANDOVER_PENDING:
            self.clear_search_top_active()
            with self._project_lock:
                response_to_request = ack.project_request_id in self._outstanding_project_requests
            if response_to_request:
                self.log("[HANDOVER] PX4 is already waiting for the current rebase")
                return
            self.log(f"[HANDOVER] PX4 requests V3 rebase id={ack.project_request_id}")
            threading.Thread(
                target=self._complete_handover,
                args=(ack.project_request_id,),
                name="gui-v3-rebase",
                daemon=True,
            ).start()
        elif result == CUSTOM_RESULT_REBASE_ACCEPTED:
            self.allow_legacy_output()

    def _complete_handover(self, handover_id: int) -> bool:
        if not self._rebase_lock.acquire(blocking=False):
            return False

        try:
            state = self.wait_until(
                lambda item: (
                    None not in (item.x, item.y, item.z)
                    and item.local_position_age_s is not None
                    and item.local_position_age_s <= 0.5
                ),
                3.0,
                "fresh LOCAL_POSITION_NED for CUSTOM handover",
            )
            actual = (float(state.x), float(state.y), float(state.z))
            self.reset_trajectory(actual, announce=True, stream_mode=state.mode)
            self.block_legacy_output()
            ack = self.send_project_command(CUSTOM_ACTION_REBASE_COMPLETE, float(handover_id))
            accepted = (
                ack.result == mavlink2.MAV_RESULT_ACCEPTED
                and ack.project_result == CUSTOM_RESULT_REBASE_ACCEPTED
            )
            if not accepted:
                raise RuntimeError(
                    f"rebase rejected result={ack.result} project={ack.project_result}"
                )
            self.allow_legacy_output()
            self.log(
                f"[HANDOVER] rebase complete id={handover_id}; "
                f"legacy target=N{actual[0]:.3f} E{actual[1]:.3f} D{actual[2]:.3f}"
            )
            return True
        except Exception as exc:
            self.log(f"[ERROR] CUSTOM handover remains blocked: {exc}")
            return False
        finally:
            self._rebase_lock.release()


class EventLog:
    def __init__(self, callback: Callable[[str], None] | None = None) -> None:
        log_dir = Path(__file__).resolve().parent / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.path = log_dir / f"gui_v3_mylink_{datetime.now():%Y%m%d_%H%M%S}.log"
        self._file = self.path.open("a", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()
        self.callback = callback

    def write(self, message: str) -> None:
        line = f"{datetime.now():%H:%M:%S} {message}"
        with self._lock:
            self._file.write(line + "\n")
        if self.callback:
            self.callback(line)

    def write_trace(self, message: str) -> None:
        line = f"{datetime.now():%H:%M:%S.%f} {message}"
        with self._lock:
            self._file.write(line + "\n")

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.close()


class MyLinkController:
    """Packet wrapper with target safety tied to PX4's observed mode."""

    def __init__(self, log: EventLog, lidar_udp_port: int = TOP_DISTANCE_UDP_PORT) -> None:
        self.log = log
        self.lidar_udp_port = int(lidar_udp_port)
        self.client: MyLinkMavlinkClient | None = None
        self.origin: tuple[float, float, float] | None = None
        self._last_mode: str | None = None
        self._landing_requested = False
        self.busy = False
        self._lock = threading.RLock()

    def submit(self, label: str, action: Callable[[], object]) -> None:
        with self._lock:
            if self.busy:
                self.log.write(f"[REJECT] {label}: another command is running")
                return
            self.busy = True

        def worker() -> None:
            try:
                action()
            except Exception as exc:
                self.log.write(f"[ERROR] {label}: {exc}")
            finally:
                with self._lock:
                    self.busy = False

        threading.Thread(target=worker, name=f"gui-v3-{label.lower()}", daemon=True).start()

    def connect(self, connection_string: str) -> None:
        if self.client is not None:
            raise RuntimeError("MyLink 已连接")
        client = MyLinkMavlinkClient(
            connection_string,
            lidar_udp_port=self.lidar_udp_port,
            logger=self.log.write,
            trace_logger=getattr(self.log, "write_trace", None),
        )
        self.client = client
        try:
            client.start()
            try:
                save_successful_connection(client.connection_string)
                self.log.write(f"[OK] 已保存UDP地址：{client.connection_string}")
            except (OSError, ValueError) as exc:
                self.log.write(f"[WARN] UDP地址保存失败：{exc}")
            state = client.wait_until(
                lambda item: None not in (item.x, item.y, item.z),
                8.0,
                "LOCAL_POSITION_NED",
            )
            self.origin = (float(state.x), float(state.y), float(state.z))
            self._last_mode = state.mode.upper()
            self._landing_requested = False
            client.reset_trajectory(self.origin, stream_mode=state.mode)
            client.allow_legacy_output()
            client.start_setpoints()
            self.log.write(f"[OK] Origin NED={self.origin}; Local-NED stream 10 Hz started")
        except Exception:
            self.client = None
            client.close()
            raise

    def disconnect(self) -> None:
        client = self.client
        self.client = None
        if client:
            client.close()
        self.origin = None
        self._last_mode = None
        self._landing_requested = False
        self.log.write("[INFO] PX4 UDP disconnected")

    def require_client(self) -> MyLinkMavlinkClient:
        if self.client is None:
            raise RuntimeError("MyLink 尚未连接")
        return self.client

    def command_result(self, label: str, result: int) -> int:
        name = mavlink2.enums["MAV_RESULT"].get(result)
        self.log.write(f"[ACK] {label}: {name.name if name else result}")
        return result

    @property
    def target(self) -> tuple[float, float, float] | None:
        if self.client is None:
            return None
        return self.client.trajectory().final_target

    @property
    def command_setpoint(self) -> TrajectoryState | None:
        if self.client is None:
            return None
        return self.client.trajectory()

    def sync_target_to_mode(self, state: MyLinkState) -> None:
        """Follow actual position outside Offboard and lock it on mode entry."""
        if not state.connected or None in (state.x, state.y, state.z):
            return
        mode = state.mode.upper()
        previous = self._last_mode
        entering = previous is not None and previous != "OFFBOARD" and mode == "OFFBOARD"
        leaving = previous == "OFFBOARD" and mode != "OFFBOARD"
        if mode != "OFFBOARD" or entering:
            if mode != "OFFBOARD":
                self.require_client().clear_search_top_active()
            actual = (float(state.x), float(state.y), float(state.z))
            self.origin = self.origin or actual
            client = self.require_client()
            client.reset_trajectory(actual, stream_mode=mode)
            client.allow_legacy_output()
            if entering:
                self._landing_requested = False
                self.log.write(
                    "[MODE] OFFBOARD entered; final target and command setpoint locked to current Local-NED"
                )
            elif leaving:
                self.log.write("[MODE] OFFBOARD exited; old target discarded, following actual position")
        self._last_mode = mode

    def _require_offboard(self, action: str) -> MyLinkMavlinkClient | None:
        client = self.require_client()
        state = client.snapshot()
        if not state.connected or state.mode.upper() != "OFFBOARD":
            self.log.write(f"[IGNORED] {action}: vehicle is not in OFFBOARD")
            return None
        if self._landing_requested:
            self.log.write(f"[IGNORED] {action}: LAND already requested")
            return None
        if None in (state.x, state.y, state.z) or self.target is None:
            self.log.write(f"[IGNORED] {action}: Local-NED target is not initialized")
            return None
        client.prepare_offboard_target((float(state.x), float(state.y), float(state.z)))
        return client

    @staticmethod
    def number(value: str | float, low: float, high: float, label: str) -> float:
        number = float(value)
        if not math.isfinite(number) or not low <= number <= high:
            raise ValueError(f"{label} must be in {low:g}..{high:g}")
        return number

    def takeoff(self, altitude: str | float) -> None:
        value = self.number(altitude, MIN_ALTITUDE_M, MAX_ALTITUDE_M, "altitude")
        client = self._require_offboard("TAKEOFF")
        if client is None:
            return
        if client.takeoff_active():
            self.log.write("[IGNORED] TAKEOFF: takeoff trajectory is already active")
            return
        state = client.snapshot()
        actual = (float(state.x), float(state.y), float(state.z))
        old = client.trajectory().final_target
        trajectory = client.start_takeoff(actual, value)
        self._log_target_change("TAKEOFF", old, trajectory.final_target)
        self.log.write(
            f"[TAKEOFF] start_z={actual[2]:.3f} "
            f"liftoff_z={max(trajectory.final_target[2], actual[2] - TAKEOFF_LIFTOFF_HEIGHT_M):.3f} "
            f"final_z={trajectory.final_target[2]:.3f}; XY locked to actual position"
        )

    def descend(self, distance: str | float) -> None:
        value = self.number(distance, MIN_MOVE_M, MAX_MOVE_M, "distance")
        client = self._require_offboard("DESCEND")
        if client is None:
            return
        if client.takeoff_active():
            self.log.write("[IGNORED] DESCEND: TAKEOFF trajectory is active")
            return
        if not self._legacy_direction_allowed(client, "DOWN"):
            return
        old, new = client.offset_final_target((0.0, 0.0, value))
        self._log_target_change("DESCEND", old, new)

    def move(self, direction: str, distance: str | float) -> None:
        value = self.number(distance, MIN_MOVE_M, MAX_MOVE_M, "distance")
        client = self._require_offboard(direction)
        if client is None:
            return
        if client.takeoff_active():
            self.log.write(f"[IGNORED] {direction}: TAKEOFF trajectory is active")
            return
        delta = {
            "FORWARD": (value, 0.0, 0.0),
            "BACK": (-value, 0.0, 0.0),
            "RIGHT": (0.0, value, 0.0),
            "LEFT": (0.0, -value, 0.0),
            "UP": (0.0, 0.0, -value),
            "DOWN": (0.0, 0.0, value),
        }
        if direction not in delta:
            raise ValueError(f"unknown direction {direction}")
        if not self._legacy_direction_allowed(client, direction):
            return
        old, new = client.offset_final_target(delta[direction])
        self._log_target_change(direction, old, new)

    def _log_target_change(
        self,
        action: str,
        old: tuple[float, float, float],
        new: tuple[float, float, float],
    ) -> None:
        self.log.write(f"[INPUT] {action} pressed once")
        self.log.write(f"[TARGET] old final target: N={old[0]:.3f} E={old[1]:.3f} D={old[2]:.3f}")
        self.log.write(f"[TARGET] new final target: N={new[0]:.3f} E={new[1]:.3f} D={new[2]:.3f}")

    def _legacy_direction_allowed(self, client: MyLinkMavlinkClient, direction: str) -> bool:
        direction_ids = {
            "FORWARD": 1,
            "BACK": 2,
            "LEFT": 3,
            "RIGHT": 4,
            "UP": 5,
            "DOWN": 6,
        }
        ack = client.direction_intent(direction_ids[direction])
        self.command_result(f"{direction} INTENT", ack.result)

        if (
            ack.result == mavlink2.MAV_RESULT_ACCEPTED
            and ack.project_result == CUSTOM_RESULT_LEGACY_ALLOWED
        ):
            return True

        if (
            ack.result == mavlink2.MAV_RESULT_ACCEPTED
            and ack.project_result == CUSTOM_RESULT_BUTTON_CONSUMED
        ):
            self.log.write(
                f"[CUSTOM] first {direction} only cancels CUSTOM; movement not executed"
            )
            client._complete_handover(ack.project_request_id)
            return False

        self.log.write(
            f"[IGNORED] {direction}: PX4 project_result={ack.project_result}; no target change"
        )
        return False

    def custom(self) -> None:
        client = self._require_offboard("CUSTOM1 SEARCH_TOP")
        if client is None:
            return
        if client.takeoff_active():
            self.log.write("[IGNORED] CUSTOM1: TAKEOFF trajectory is active")
            return
        self.log.write("[CUSTOM1] command sent once")
        ack = client.start_search_top()
        self.command_result("CUSTOM1 SEARCH_TOP", ack.result)
        if not (
            ack.result == mavlink2.MAV_RESULT_ACCEPTED
            and ack.project_result == CUSTOM_RESULT_STARTED
        ):
            client.clear_search_top_active()
            self.log.write(
                f"[CUSTOM1] rejected project_result={ack.project_result}; legacy control unchanged"
            )

    def land(self) -> int:
        self._landing_requested = True
        client = self.require_client()
        client.clear_search_top_active()
        self.log.write("[INPUT] LAND pressed once; further movement targets disabled")
        return self.command_result("LAND", client.land())

    def state(self) -> MyLinkState:
        return MyLinkState() if self.client is None else self.client.snapshot()

    def search_top_active(self) -> bool:
        return self.client is not None and self.client.search_top_active()

    def close(self) -> None:
        if self.client is not None:
            self.client.close()
            self.client = None
        self.origin = None
        self._last_mode = None
        self._landing_requested = False


class OffboardControlGuiV3:
    MODE_ZH = {
        "DISCONNECTED": "未连接",
        "MANUAL": "手动模式",
        "ALTCTL": "高度模式",
        "POSCTL": "位置模式",
        "AUTO": "自动模式",
        "AUTO.READY": "自动准备",
        "AUTO.TAKEOFF": "自动起飞",
        "AUTO.LOITER": "自动悬停",
        "AUTO.MISSION": "自动任务",
        "AUTO.RTL": "自动返航",
        "AUTO.LAND": "自动降落",
        "AUTO.FOLLOW": "自动跟随",
        "AUTO.PRECLAND": "精确降落",
        "ACRO": "特技模式",
        "OFFBOARD": "外部控制模式",
        "STABILIZED": "稳定模式",
        "RATTITUDE": "半自稳模式",
        "TERMINATION": "飞行终止",
    }

    TEXT = {
        "zh": {
            "title": "PX4 MyLink Offboard 控制台 V3.1", "connection": "MAVLink UDP 连接",
            "connect": "连接", "disconnect": "断开", "language": "语言", "state": "PX4 状态",
            "control": "控制", "events": "事件日志", "address": "MAVLink UDP 地址",
            "takeoff": "起飞 TAKEOFF", "descend": "下降 DESCEND", "land": "降落 LAND", "custom": "寻顶模式",
            "forward": "前进", "back": "后退", "left": "左移", "right": "右移",
            "up": "上升", "down": "下降", "altitude": "起飞增量（m）", "distance": "移动步长（m）",
            "status_row": "状态", "position_row": "当前位置", "velocity_row": "当前速度",
            "battery_row": "电池", "top_distance_row": "顶部距离", "motor_output_row": "电机归一化输出",
            "target_row": "最终目标", "command_row": "当前指令",
            "connected": "已连接", "disconnected": "未连接", "system": "系统ID", "component": "组件ID",
            "mode": "模式", "armed": "解锁状态", "armed_yes": "已解锁", "armed_no": "未解锁",
            "heartbeat": "心跳", "north": "北", "east": "东", "down_axis": "下", "speed": "速度",
            "front_left": "左上", "front_right": "右上", "rear_left": "左下", "rear_right": "右下",
        },
        "en": {
            "title": "PX4 MyLink Offboard Console V3.1", "connection": "MAVLink UDP connection",
            "connect": "CONNECT", "disconnect": "DISCONNECT", "language": "Language", "state": "PX4 STATUS",
            "control": "CONTROL", "events": "EVENT LOG", "address": "MAVLink UDP address",
            "takeoff": "TAKEOFF", "descend": "DESCEND", "land": "LAND", "custom": "SEARCH TOP", "forward": "FORWARD",
            "back": "BACK", "left": "LEFT", "right": "RIGHT", "up": "UP", "down": "DOWN",
            "altitude": "Takeoff increment (m)", "distance": "Movement step (m)",
            "status_row": "State", "position_row": "Position", "velocity_row": "Velocity",
            "battery_row": "Battery", "top_distance_row": "Top distance", "motor_output_row": "Normalized motors",
            "target_row": "Final target", "command_row": "Command",
            "connected": "CONNECTED", "disconnected": "DISCONNECTED", "system": "SYS", "component": "COMP",
            "mode": "mode", "armed": "armed", "armed_yes": "True", "armed_no": "False",
            "heartbeat": "heartbeat", "north": "N", "east": "E", "down_axis": "D", "speed": "v",
            "front_left": "Front-left", "front_right": "Front-right",
            "rear_left": "Rear-left", "rear_right": "Rear-right",
        },
    }

    def __init__(self, root: tk.Tk, connection: str, lidar_udp_port: int = TOP_DISTANCE_UDP_PORT) -> None:
        self.root = root
        self.language = "zh"
        self.events: queue.Queue[str] = queue.Queue()
        self.event_history: list[str] = []
        self.log = EventLog(self.events.put)
        self.controller = MyLinkController(self.log, lidar_udp_port)
        self.connection_var = tk.StringVar(value=connection)
        self.language_var = tk.StringVar(value="中文")
        self.altitude_var = tk.StringVar(value="1.0")
        self.distance_var = tk.StringVar(value="0.5")
        self.status_var = tk.StringVar(value=self.tr("disconnected"))
        self.position_var = tk.StringVar(value="N —  E —  D —")
        self.velocity_var = tk.StringVar(value="vx —  vy —  vz —")
        self.battery_var = tk.StringVar(value="—")
        self.top_distance_var = tk.StringVar(value="左上 —  右上 —  左下 —  右下 — mm")
        self.motor_output_vars = tuple(tk.DoubleVar(value=0.0) for _ in range(4))
        self.motor_output_text_vars = tuple(tk.StringVar(value=f"M{index + 1} —") for index in range(4))
        self.motor_output_age_var = tk.StringVar(value="age=—s")
        self.target_var = tk.StringVar(value="N —  E —  D —")
        self.command_var = tk.StringVar(value="N —  E —  D — | v —")
        self._closing = False
        self._build()
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self._refresh()
        self._drain_events()

    def tr(self, key: str) -> str:
        return self.TEXT[self.language][key]

    def display_mode(self, mode: str) -> str:
        return self.MODE_ZH.get(mode, mode) if self.language == "zh" else mode

    def _switch_language(self, _event=None) -> None:
        self.language = "en" if self.language_var.get() == "English" else "zh"
        for child in self.root.winfo_children():
            child.destroy()
        self._build()
        self._restore_events()

    def _build(self) -> None:
        root = self.root
        root.title(self.tr("title"))
        root.geometry("1100x770")
        root.minsize(920, 670)
        style = ttk.Style(root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TButton", padding=(9, 6), font=("Segoe UI", 10))
        style.configure("TLabel", font=("Segoe UI", 10))
        style.configure("TLabelframe.Label", font=("Segoe UI", 10, "bold"))
        style.configure("Motor.Valid.Horizontal.TProgressbar", troughcolor="#d9d9d9", background="#2eaf5d")
        style.configure("Motor.Stale.Horizontal.TProgressbar", troughcolor="#d9d9d9", background="#9e9e9e")
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(2, weight=1)

        connection = ttk.LabelFrame(root, text=self.tr("connection"), padding=10)
        connection.grid(row=0, column=0, columnspan=2, sticky="ew", padx=12, pady=10)
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text=self.tr("address") + ":").grid(row=0, column=0, sticky="w")
        ttk.Entry(connection, textvariable=self.connection_var).grid(row=0, column=1, columnspan=3, sticky="ew", padx=6)
        self.connect_button = ttk.Button(connection, text=self.tr("connect"), command=self._connect)
        self.connect_button.grid(row=0, column=4, padx=3)
        self.disconnect_button = ttk.Button(connection, text=self.tr("disconnect"), command=self._disconnect)
        self.disconnect_button.grid(row=0, column=5, padx=3)
        ttk.Label(connection, text=self.tr("language") + ":").grid(row=0, column=6, padx=(14, 4))
        combo = ttk.Combobox(connection, textvariable=self.language_var, values=("中文", "English"), width=9, state="readonly")
        combo.grid(row=0, column=7)
        combo.bind("<<ComboboxSelected>>", self._switch_language)

        status = ttk.LabelFrame(root, text=self.tr("state"), padding=10)
        status.grid(row=1, column=0, columnspan=2, sticky="ew", padx=12, pady=(0, 10))
        for row, (name, variable) in enumerate((
            (self.tr("status_row"), self.status_var),
            (self.tr("position_row"), self.position_var),
            (self.tr("velocity_row"), self.velocity_var),
            (self.tr("battery_row"), self.battery_var),
            (self.tr("top_distance_row"), self.top_distance_var),
        )):
            ttk.Label(status, text=name + ":", width=10).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Label(status, textvariable=variable, font=("Consolas", 10)).grid(row=row, column=1, sticky="w", pady=2)
        ttk.Label(status, text=self.tr("motor_output_row") + ":", width=16).grid(row=5, column=0, sticky="nw", pady=2)
        motor_frame = ttk.Frame(status)
        motor_frame.grid(row=5, column=1, sticky="ew", pady=2)
        status.columnconfigure(1, weight=1)
        self.motor_progressbars: list[ttk.Progressbar] = []
        for motor_index in range(4):
            motor_frame.columnconfigure(motor_index, weight=1)
            cell = ttk.Frame(motor_frame)
            cell.grid(row=0, column=motor_index, sticky="ew", padx=(0, 8))
            ttk.Label(cell, textvariable=self.motor_output_text_vars[motor_index], font=("Consolas", 9)).grid(row=0, column=0, sticky="w")
            progress = ttk.Progressbar(
                cell,
                maximum=MOTOR_OUTPUT_ARRAY_SCALE,
                variable=self.motor_output_vars[motor_index],
                style="Motor.Stale.Horizontal.TProgressbar",
                length=135,
            )
            progress.grid(row=1, column=0, sticky="ew")
            cell.columnconfigure(0, weight=1)
            self.motor_progressbars.append(progress)
        ttk.Label(motor_frame, textvariable=self.motor_output_age_var, font=("Consolas", 9)).grid(row=0, column=4, rowspan=2, sticky="w")
        for row, (name, variable) in enumerate((
            (self.tr("target_row"), self.target_var),
            (self.tr("command_row"), self.command_var),
        ), start=6):
            ttk.Label(status, text=name + ":", width=10).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Label(status, textvariable=variable, font=("Consolas", 10)).grid(row=row, column=1, sticky="w", pady=2)

        control = ttk.LabelFrame(root, text=self.tr("control"), padding=10)
        control.grid(row=2, column=0, sticky="nsew", padx=(12, 6), pady=(0, 12))
        for column in range(3):
            control.columnconfigure(column, weight=1)
        self.action_buttons: list[ttk.Button] = []
        top_actions = (
            ("takeoff", lambda: self.controller.takeoff(self.altitude_var.get())),
            ("land", self.controller.land),
            ("custom", self.controller.custom),
        )
        for column, (key, action) in enumerate(top_actions):
            button = ttk.Button(control, text=self.tr(key), command=lambda k=key, a=action: self.controller.submit(k.upper(), a))
            button.grid(row=0, column=column, sticky="ew", padx=3, pady=3)
            self.action_buttons.append(button)
        ttk.Label(control, text=self.tr("altitude")).grid(row=1, column=0, sticky="w", pady=(10, 3))
        ttk.Entry(control, textvariable=self.altitude_var, width=8).grid(row=1, column=1, sticky="w")
        ttk.Label(control, text=self.tr("distance")).grid(row=2, column=0, sticky="w", pady=(10, 3))
        ttk.Entry(control, textvariable=self.distance_var, width=8).grid(row=2, column=1, sticky="w")
        directions = (
            ("up", "UP", 3, 0), ("forward", "FORWARD", 3, 1), ("down", "DOWN", 3, 2),
            ("left", "LEFT", 4, 0), ("right", "RIGHT", 4, 2),
            ("back", "BACK", 5, 1),
        )
        for key, direction, row, column in directions:
            action = lambda value=direction: self.controller.move(value, self.distance_var.get())
            button = ttk.Button(control, text=self.tr(key), command=lambda d=direction, a=action: self.controller.submit(d, a))
            button.grid(row=row, column=column, sticky="ew", padx=3, pady=3)
            self.action_buttons.append(button)

        events = ttk.LabelFrame(root, text=self.tr("events"), padding=8)
        events.grid(row=2, column=1, sticky="nsew", padx=(6, 12), pady=(0, 12))
        events.rowconfigure(0, weight=1)
        events.columnconfigure(0, weight=1)
        self.event_text = tk.Text(events, state="disabled", wrap="none", background="#101820", foreground="#d8e8f0", font=("Consolas", 9), relief="flat")
        self.event_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(events, orient="vertical", command=self.event_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.event_text.configure(yscrollcommand=scroll.set)
        ttk.Label(events, text=str(self.log.path)).grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))

    def _connect(self) -> None:
        self.controller.submit("CONNECT", lambda: self.controller.connect(self.connection_var.get().strip()))

    def _disconnect(self) -> None:
        self.controller.submit("DISCONNECT", self.controller.disconnect)

    @staticmethod
    def _fmt(value: float | None) -> str:
        return "—" if value is None else f"{value:.2f}"

    @staticmethod
    def _fmt_mm(value: int | None) -> str:
        return "—" if value is None else str(int(value))

    def _refresh(self) -> None:
        if self._closing:
            return
        state = self.controller.state()
        self.controller.sync_target_to_mode(state)
        connection_text = self.tr("connected") if state.connected else self.tr("disconnected")
        armed_text = self.tr("armed_yes") if state.armed else self.tr("armed_no")
        self.status_var.set(
            f"{connection_text} | {self.tr('system')}={state.system_id} {self.tr('component')}={state.component_id} | "
            f"{self.tr('mode')}={self.display_mode(state.mode)} | {self.tr('armed')}={armed_text} | "
            f"{self.tr('heartbeat')}={self._fmt(state.heartbeat_age_s)}s"
        )
        self.position_var.set(
            f"{self.tr('north')} {self._fmt(state.x)}  {self.tr('east')} {self._fmt(state.y)}  "
            f"{self.tr('down_axis')} {self._fmt(state.z)}"
        )
        self.velocity_var.set(
            f"{self.tr('north')} {self._fmt(state.vx)}  {self.tr('east')} {self._fmt(state.vy)}  "
            f"{self.tr('down_axis')} {self._fmt(state.vz)} m/s"
        )
        self.battery_var.set(f"{state.battery_percent if state.battery_percent is not None else '—'}%  {self._fmt(state.battery_voltage_v)}V")
        # Sensor/uORB order is FR, FL, RR, RL. Present it spatially in the
        # requested GUI order: left-up, right-up, left-down, right-down.
        front_right, front_left, rear_right, rear_left = state.top_distances_mm
        display_distances = (
            (self.tr("front_left"), front_left),
            (self.tr("front_right"), front_right),
            (self.tr("rear_left"), rear_left),
            (self.tr("rear_right"), rear_right),
        )
        top_values = "  ".join(
            f"{label} {self._fmt_mm(value)}" for label, value in display_distances
        )
        self.top_distance_var.set(f"{top_values} mm | age={self._fmt(state.top_distance_age_s)}s")
        motor_stale = state.motor_output_age_s is None or state.motor_output_age_s > MOTOR_OUTPUT_STALE_S
        for motor_index, output in enumerate(state.motor_outputs_normalized):
            progress = self.motor_progressbars[motor_index]
            if motor_stale or output is None:
                self.motor_output_vars[motor_index].set(0.0)
                self.motor_output_text_vars[motor_index].set(f"M{motor_index + 1} —")
                progress.configure(style="Motor.Stale.Horizontal.TProgressbar")
            else:
                normalized = min(max(float(output), 0.0), 1.0)
                self.motor_output_vars[motor_index].set(normalized * MOTOR_OUTPUT_ARRAY_SCALE)
                self.motor_output_text_vars[motor_index].set(
                    f"M{motor_index + 1} {normalized:.3f} ({normalized * 100:.1f}%)"
                )
                progress.configure(style="Motor.Valid.Horizontal.TProgressbar")
        self.motor_output_age_var.set(f"age={self._fmt(state.motor_output_age_s)}s")
        target = self.controller.target
        axes = (self.tr("north"), self.tr("east"), self.tr("down_axis"))
        self.target_var.set(
            f"{axes[0]} —  {axes[1]} —  {axes[2]} —" if target is None
            else f"{axes[0]} {target[0]:.2f}  {axes[1]} {target[1]:.2f}  {axes[2]} {target[2]:.2f}"
        )
        command = self.controller.command_setpoint
        if command is None:
            self.command_var.set(f"{axes[0]} —  {axes[1]} —  {axes[2]} — | {self.tr('speed')} —")
        else:
            px, py, pz = command.command_position
            vx, vy, vz = command.command_velocity
            self.command_var.set(
                f"{axes[0]} {px:.2f}  {axes[1]} {py:.2f}  {axes[2]} {pz:.2f} | "
                f"{self.tr('speed')} {math.sqrt(vx * vx + vy * vy + vz * vz):.2f} m/s"
            )
        connected = state.connected
        busy = self.controller.busy
        self.connect_button.configure(state="disabled" if connected or busy else "normal")
        self.disconnect_button.configure(state="normal" if connected and not busy else "disabled")
        for button in self.action_buttons:
            button.configure(state="normal" if connected and not busy else "disabled")
        self.root.after(200, self._refresh)

    def _restore_events(self) -> None:
        self.event_text.configure(state="normal")
        self.event_text.delete("1.0", "end")
        if self.event_history:
            self.event_text.insert("end", "\n".join(self.event_history) + "\n")
        self.event_text.configure(state="disabled")

    def _drain_events(self) -> None:
        try:
            while True:
                line = self.events.get_nowait()
                self.event_history.append(line)
                self.event_text.configure(state="normal")
                self.event_text.insert("end", line + "\n")
                self.event_text.see("end")
                self.event_text.configure(state="disabled")
        except queue.Empty:
            pass
        if not self._closing:
            self.root.after(100, self._drain_events)

    def _close(self) -> None:
        self._closing = True
        self.controller.close()
        self.log.write("[INFO] GUI V3 closed")
        self.log.close()
        self.root.destroy()


def self_test() -> None:
    assert POSITION_VELOCITY_MASK == 3520
    assert POSITION_VELOCITY_MASK & mavlink2.POSITION_TARGET_TYPEMASK_AX_IGNORE
    assert not POSITION_VELOCITY_MASK & mavlink2.POSITION_TARGET_TYPEMASK_VX_IGNORE
    assert px4_mode_name(6 << 16) == "OFFBOARD"
    generator = SetpointTrajectoryGenerator()
    generator.reset((0.0, 0.0, 0.0))
    generator.offset_final_target((0.5, 0.0, 0.0))
    first = generator.update((0.0, 0.0, 0.0), 0.1)
    # XY position is masked during velocity movement, so the diagnostic
    # position carries final_target while velocity is acceleration-limited.
    assert math.isclose(first.command_position[0], 0.5, abs_tol=1e-9)
    assert math.isclose(first.command_velocity[0], 0.03, abs_tol=1e-9)
    ack = CommandAck(MAV_CMD_CUSTOM_ACTION, 0, (123 << 8) | CUSTOM_RESULT_STARTED, 1, 25)
    assert ack.project_request_id == 123
    assert ack.project_result == CUSTOM_RESULT_STARTED
    packet = TOP_DISTANCE_PACKET.pack(TOP_DISTANCE_MAGIC, TOP_DISTANCE_VERSION, 0x0F, 7, 1234, 10, 20, 30, 40)
    frame = decode_top_distance_packet(packet, received_at=1.0)
    assert frame.sequence == 7 and frame.distances_mm == (10, 20, 30, 40)
    metadata = TOP_DISTANCE_ARRAY_MARKER | TOP_DISTANCE_ARRAY_VERSION | (0x0F << TOP_DISTANCE_ARRAY_VALID_SHIFT) | 8
    packed_distances = 10 | (20 << 16) | (30 << 32) | (40 << 48)
    mavlink_distance = mavlink2.MAVLink(None).ping_encode(packed_distances, metadata, SOURCE_SYSTEM, SOURCE_COMPONENT)
    mavlink_frame = decode_top_distance_mavlink(mavlink_distance, received_at=2.0)
    assert mavlink_frame is not None
    assert mavlink_frame.sequence == 8 and mavlink_frame.distances_mm == (10, 20, 30, 40)
    motor_metadata = (
        MOTOR_OUTPUT_ARRAY_MARKER
        | MOTOR_OUTPUT_ARRAY_VERSION
        | (0x0F << MOTOR_OUTPUT_ARRAY_VALID_SHIFT)
        | MOTOR_OUTPUT_ARRAY_ARMED_FLAG
        | 9
    )
    packed_outputs = 0 | (250 << 16) | (742 << 32) | (1000 << 48)
    mavlink_motor = mavlink2.MAVLink(None).ping_encode(packed_outputs, motor_metadata, SOURCE_SYSTEM, SOURCE_COMPONENT)
    motor_frame = decode_motor_output_mavlink(mavlink_motor, received_at=3.0)
    assert motor_frame is not None and motor_frame.armed
    assert motor_frame.sequence == 9 and motor_frame.outputs_scaled == (0, 250, 742, 1000)
    monitor = MyLinkMavlinkClient("udp:127.0.0.1:14540")
    monitor._accept_motor_output_frame(motor_frame, "self-test")
    fresh_outputs, fresh_age = monitor._motor_output_snapshot(3.2)
    assert fresh_outputs == (0.0, 0.25, 0.742, 1.0) and math.isclose(fresh_age, 0.2)
    stale_outputs, stale_age = monitor._motor_output_snapshot(3.6)
    assert stale_outputs == (None, None, None, None) and math.isclose(stale_age, 0.6)
    controller_names = set(dir(MyLinkController))
    assert {"takeoff", "descend", "move", "land", "custom", "sync_target_to_mode"} <= controller_names
    assert "simulate_laser_signal" not in controller_names
    source = Path(__file__).read_text(encoding="utf-8").split("def self_test()", 1)[0]
    assert "class FlightPhase" not in source
    assert "position_valid" not in source
    assert "failsafe" not in source.lower()
    assert "import serial" not in source
    assert "DEFAULT_BAUD" not in source
    assert "MAV_CMD_COMPONENT_ARM_DISARM" not in source
    assert "MAV_CMD_DO_SET_MODE" not in source
    assert "DEFAULT_CONNECTION" in source
    assert "CUSTOM_ACTION_REBASE_COMPLETE" in source
    assert "TOP_DISTANCE_PACKET" in source
    assert "_send_pending_top_distances" not in source
    assert sitl_mylink_connection("udp:127.0.0.1:14540") == "udpout:127.0.0.1:14541"
    assert sitl_mylink_connection("udp:192.168.1.10:14540") is None
    print("GUI V3 self-test: UDP API, distance/motor monitoring, CUSTOM1 and rebase helpers OK; no connection opened")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default=None)
    parser.add_argument("--lidar-port", type=int, default=TOP_DISTANCE_UDP_PORT)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    root = tk.Tk()
    initial_connection = args.connection or load_saved_connection()
    OffboardControlGuiV3(root, initial_connection, args.lidar_port)
    root.mainloop()


if __name__ == "__main__":
    main()
