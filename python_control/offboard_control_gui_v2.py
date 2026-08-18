#!/usr/bin/env python3
"""PX4 Offboard MAVLink packet sender and telemetry panel (v2).

The GUI sends MAVLink commands without duplicating PX4 GPS, optical-flow,
Local Position, arming, mode, or failsafe gates. PX4 remains responsible for
accepting or rejecting each command. Telemetry is displayed for observation.
"""

from __future__ import annotations

import argparse
import inspect
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Callable

import tkinter as tk
from tkinter import messagebox, ttk

from pymavlink import mavutil

# The desktop copy is intentionally self-contained as a launcher, while the
# validated v4/v6 implementation remains in the PX4 project.  Add that
# project directory to Python's import path when the script is launched from
# Windows Desktop.
PROJECT_CODE_DIR = Path(__file__).resolve().parent
_required_project_files = (
    "mission_controller_v6_optical_flow.py",
    "mission_controller_v4.py",
    "generated_mavlink.py",
)
if not all((PROJECT_CODE_DIR / _name).exists() for _name in _required_project_files):
    for _candidate in (
        Path(r"\\wsl.localhost\Ubuntu-24.04\home\yyy\PX4-test\python_control"),
        Path("/home/yyy/PX4-test/python_control"),
    ):
        if (_candidate / "mission_controller_v6_optical_flow.py").exists():
            PROJECT_CODE_DIR = _candidate
            sys.path.insert(0, str(_candidate))
            break

try:
    from mission_controller_v6_optical_flow import (
        OffboardMavlinkClient,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,
    )
except ImportError as exc:  # pragma: no cover - gives a useful launch error
    raise SystemExit(
        "请从 python_control 目录运行，或将该目录加入 PYTHONPATH。"
    ) from exc


DEFAULT_CONNECTION = "udp:127.0.0.1:14540"
MIN_ALTITUDE_M = 0.3
MAX_ALTITUDE_M = 3.0
MIN_MOVE_M = 0.1
MAX_MOVE_M = 5.0
POSITION_ONLY_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


@dataclass(frozen=True)
class CachedState:
    connected: bool = False
    armed: bool = False
    landed: bool | None = None
    mode: str = "DISCONNECTED"
    failsafe: bool = False
    gcs_connected: bool = False
    local_x: float | None = None
    local_y: float | None = None
    local_z: float | None = None
    vx: float | None = None
    vy: float | None = None
    vz: float | None = None
    roll: float | None = None
    pitch: float | None = None
    yaw: float | None = None
    position_valid: bool | None = None
    gps_active: bool | None = None
    flow_active: bool | None = None
    flow_quality: int | None = None
    system_id: int = 0
    component_id: int = 0
    heartbeat_age: float | None = None
    local_age: float | None = None
    busy: bool = False
    error: str = ""


class FlightPhase(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    READY = "READY"
    ARMED = "ARMED"
    TAKING_OFF = "TAKING_OFF"
    FLYING = "FLYING"
    LANDING = "LANDING"
    LANDED = "LANDED"


class ControlAuthority(str, Enum):
    """Who is currently allowed to issue flight-control actions."""

    RC_CONTROL = "RC_CONTROL"
    OFFBOARD_CONTROL = "OFFBOARD_CONTROL"
    RC_OVERRIDE = "RC_OVERRIDE"


CONTROL_ACTIONS = (
    "ARM", "TAKEOFF", "FORWARD", "BACK", "LEFT", "RIGHT", "UP", "DOWN",
    "HOVER", "LAND", "SAFE LAND", "DISARM", "REENTER OFFBOARD",
)
MOVEMENT_ACTIONS = ("FORWARD", "BACK", "LEFT", "RIGHT", "UP", "DOWN", "HOVER")
RC_MANUAL_MODES = {
    "MANUAL", "STABILIZED", "ACRO", "RATTITUDE", "ALTCTL", "POSCTL",
    "ALTITUDE", "POSITION",
}


def _state_has_local_position(state: CachedState) -> bool:
    values = (state.local_x, state.local_y, state.local_z)
    return state.position_valid is True and all(
        value is not None and math.isfinite(float(value)) for value in values
    )


def _state_mode_known(state: CachedState) -> bool:
    return state.mode.upper() not in {"", "UNKNOWN", "DISCONNECTED"}


def _takeoff_target_reached(
    state: CachedState,
    origin: tuple[float, float, float] | None,
    target: list[float] | None,
) -> bool:
    if not _state_has_local_position(state) or origin is None or target is None:
        return False
    current_altitude = origin[2] - float(state.local_z)
    target_altitude = origin[2] - float(target[2])
    vertical_speed = float(state.vz) if state.vz is not None else math.inf
    return abs(current_altitude - target_altitude) <= 0.15 and abs(vertical_speed) <= 0.30


def derive_flight_phase(
    state: CachedState,
    intent: str | None,
    origin: tuple[float, float, float] | None = None,
    target: list[float] | None = None,
) -> FlightPhase:
    """Derive the GUI phase from PX4 feedback, using intent only as context."""
    if not state.connected:
        return FlightPhase.DISCONNECTED
    if not state.armed:
        return FlightPhase.READY
    if state.landed is True:
        return FlightPhase.LANDED if intent == "LANDING" else FlightPhase.ARMED
    if state.landed is False:
        if intent == "LANDING":
            return FlightPhase.LANDING
        if intent == "TAKING_OFF" and not _takeoff_target_reached(state, origin, target):
            return FlightPhase.TAKING_OFF
        return FlightPhase.FLYING
    # Unknown landed feedback is not enough to claim a transition. Keep the
    # armed phase visible; the permission matrix remains conservative below.
    return FlightPhase.ARMED


def button_permissions(
    state: CachedState,
    phase: FlightPhase,
    authority: ControlAuthority = ControlAuthority.RC_CONTROL,
) -> dict[str, bool]:
    """Return the safety matrix for control buttons from observed feedback."""
    permissions = {name: False for name in CONTROL_ACTIONS}
    if not state.connected or state.busy:
        return permissions

    # Once the pilot has taken over, do not compete with RC input. The only
    # permitted GUI action is an explicit, safety-gated Offboard re-entry.
    if authority is ControlAuthority.RC_OVERRIDE:
        permissions["REENTER OFFBOARD"] = (
            state.armed
            and state.landed is False
            and _state_has_local_position(state)
            and _state_mode_known(state)
            and not state.failsafe
        )
        return permissions

    position_ready = _state_has_local_position(state)
    mode_known = _state_mode_known(state)
    offboard = (
        authority is ControlAuthority.OFFBOARD_CONTROL
        and mode_known
        and state.mode.upper() == "OFFBOARD"
    )
    landed = state.landed is True
    airborne = state.landed is False

    if phase is FlightPhase.READY:
        permissions["ARM"] = (
            not state.armed and landed and position_ready and mode_known
        )
    elif phase is FlightPhase.ARMED:
        permissions["TAKEOFF"] = (
            state.armed and landed and position_ready and offboard
        )
        permissions["LAND"] = state.armed and landed and mode_known
        permissions["SAFE LAND"] = state.armed and landed and mode_known
    elif phase is FlightPhase.TAKING_OFF:
        permissions["LAND"] = state.armed and airborne and mode_known
        permissions["SAFE LAND"] = state.armed and airborne and mode_known
    elif phase is FlightPhase.FLYING:
        for name in MOVEMENT_ACTIONS:
            permissions[name] = state.armed and airborne and position_ready and offboard
        permissions["LAND"] = state.armed and airborne and mode_known
        permissions["SAFE LAND"] = state.armed and airborne and mode_known
    elif phase is FlightPhase.LANDED:
        permissions["DISARM"] = state.armed and landed and mode_known
    # LANDING intentionally leaves every GUI control disabled until PX4
    # reports landed=true, after which only DISARM is available.
    return permissions


def derive_control_authority(
    previous: ControlAuthority,
    state: CachedState,
    *,
    reentry_pending: bool = False,
    mode_request: str | None = None,
) -> tuple[ControlAuthority, bool]:
    """Derive authority from PX4 feedback and explicit GUI intent.

    The returned boolean is the still-pending re-entry request. Keeping this
    logic pure makes the RC takeover and re-entry rules testable without PX4.
    """
    mode = state.mode.upper()
    offboard = mode == "OFFBOARD"
    if not state.connected or not state.armed or state.landed is True:
        return ControlAuthority.RC_CONTROL, False
    if previous is ControlAuthority.RC_OVERRIDE:
        if reentry_pending and offboard:
            return ControlAuthority.OFFBOARD_CONTROL, False
        return ControlAuthority.RC_OVERRIDE, reentry_pending
    if offboard:
        return ControlAuthority.OFFBOARD_CONTROL, False
    if (
        previous is ControlAuthority.OFFBOARD_CONTROL
        and state.landed is False
        and mode in RC_MANUAL_MODES
        and mode_request not in {"LAND", "OFFBOARD_REENTRY"}
    ):
        return ControlAuthority.RC_OVERRIDE, False
    return ControlAuthority.RC_CONTROL, reentry_pending


class EventLog:
    """Thread-safe file + UI event logger."""

    def __init__(self, callback: Callable[[str], None] | None = None) -> None:
        log_dir = PROJECT_CODE_DIR / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        self.path = log_dir / f"gui_offboard_{datetime.now():%Y%m%d_%H%M%S}.log"
        self._file = self.path.open("a", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()
        self._callback = callback

    def write(self, message: str) -> None:
        line = f"{datetime.now():%H:%M:%S} {message}"
        with self._lock:
            self._file.write(line + "\n")
        if self._callback:
            self._callback(line)

    def close(self) -> None:
        with self._lock:
            self._file.close()


class FlightController:
    """Packet-oriented API over the validated MAVLink client."""

    def __init__(self, event_log: EventLog) -> None:
        self.log = event_log
        self.client: OffboardMavlinkClient | None = None
        self.connection_string = DEFAULT_CONNECTION
        self.origin: tuple[float, float, float] | None = None
        self.target_local_ned: list[float] | None = None
        self._client_lock = threading.RLock()
        self._target_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._busy = False
        self._closed = False
        self._reset_target_before_next_arm = False

    def _set_busy(self, value: bool) -> None:
        with self._client_lock:
            self._busy = value

    def _submit(
        self,
        name: str,
        action: Callable[[], object],
        on_done: Callable[[bool], None] | None = None,
    ) -> None:
        with self._client_lock:
            if self._busy:
                self.log.write(f"[REJECT] {name}: another action is running")
                return
            self._busy = True

        def worker() -> None:
            success = False
            try:
                with self._action_lock:
                    result = action()
                success = result is not False
            except Exception as exc:  # UI must remain alive after a rejected command
                self.log.write(f"[ERROR] {name}: {exc}")
            finally:
                if on_done is not None:
                    on_done(success)
                self._set_busy(False)

        threading.Thread(target=worker, name=f"gui-{name.lower()}", daemon=True).start()

    def connect_async(self, connection_string: str) -> None:
        self._submit("CONNECT", lambda: self._connect(connection_string))

    def _connect(self, connection_string: str) -> None:
        with self._client_lock:
            if self.client is not None:
                self.log.write("[REJECT] already connected")
                return
            self.connection_string = connection_string.strip() or DEFAULT_CONNECTION
            client = OffboardMavlinkClient(self.connection_string, setpoint_rate_hz=10.0)
            self.client = client
        self.log.write(f"[INFO] Connecting PX4: {self.connection_string}")
        try:
            client.start()
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                if client.last_heartbeat_monotonic is not None and client.target_system:
                    break
                client.wait_for_update(client.update_sequence(), timeout_s=0.25)
            else:
                raise TimeoutError("等待 PX4 HEARTBEAT 超时")

            # Telemetry streams are for display only. A missing Local Position
            # must not prevent the client from opening or sending packets.
            client.request_estimator_streams()
            local = client.local_position()
            self.origin = (
                (local.x_m, local.y_m, local.z_m)
                if None not in (local.x_m, local.y_m, local.z_m)
                else None
            )
            self._set_target(
                self.origin if self.origin is not None else (0.0, 0.0, 0.0)
            )
            self.log.write(
                f"[OK] PX4 connected SYS={client.target_system} COMP={client.target_component}"
            )
            self.log.write("[OK] Local Position received")
        except Exception:
            with self._client_lock:
                if self.client is client:
                    self.client = None
            client.close()
            raise

    def disconnect_async(self) -> None:
        self._submit("DISCONNECT", self.disconnect)

    def disconnect(self) -> None:
        with self._client_lock:
            client = self.client
            self.client = None
        if client is not None:
            client.close()
            self.log.write("[INFO] PX4 disconnected")
        self.origin = None
        with self._target_lock:
            self.target_local_ned = None

    def _client_or_raise(self) -> OffboardMavlinkClient:
        with self._client_lock:
            if self.client is None:
                raise RuntimeError("尚未连接 PX4")
            return self.client

    def _set_target(self, target: tuple[float, float, float] | list[float]) -> None:
        values = [float(v) for v in target]
        if not all(math.isfinite(v) for v in values):
            raise ValueError("目标位置必须是有限数值")
        with self._target_lock:
            self.target_local_ned = values
        self._reset_target_before_next_arm = False
        client = self._client_or_raise()
        client.set_position_target(*values)

    def _target(self) -> list[float]:
        with self._target_lock:
            if self.target_local_ned is None:
                raise RuntimeError("尚未建立 Local NED 目标")
            return list(self.target_local_ned)

    def _reset_target_to_current_position(
        self,
        client: OffboardMavlinkClient,
        reason: str,
    ) -> None:
        """Forget a previous takeoff target before the next arm operation."""
        local = client.local_position()
        candidate = (local.x_m, local.y_m, local.z_m)
        if all(value is not None and math.isfinite(float(value)) for value in candidate):
            target = tuple(float(value) for value in candidate)
        elif self.origin is not None:
            target = self.origin
        else:
            target = (0.0, 0.0, 0.0)

        with self._target_lock:
            self.target_local_ned = list(target)
        client.set_position_target(*target)
        self.log.write(
            f"[INFO] {reason}; Offboard target reset to "
            f"current position ({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})"
        )

    def read_state(self) -> CachedState:
        with self._client_lock:
            client = self.client
            busy = self._busy
        if client is None:
            return CachedState(busy=busy)
        try:
            snap = client.snapshot()
            local = client.local_position()
            evidence = client.estimator_evidence()
            position_valid = (
                evidence.horizontal_position_valid
                and evidence.vertical_position_valid
            )
            return CachedState(
                connected=True,
                armed=snap.armed,
                landed=snap.landed,
                mode=snap.mode,
                failsafe=snap.failsafe,
                gcs_connected=snap.gcs_connected,
                local_x=local.x_m,
                local_y=local.y_m,
                local_z=local.z_m,
                vx=local.vx_m_s,
                vy=local.vy_m_s,
                vz=local.vz_m_s,
                roll=client.roll_deg,
                pitch=client.pitch_deg,
                yaw=client.yaw_deg,
                position_valid=position_valid,
                gps_active=evidence.gps_active,
                flow_active=evidence.optical_flow_active,
                flow_quality=evidence.flow_quality,
                system_id=client.target_system,
                component_id=client.target_component,
                heartbeat_age=snap.last_heartbeat_age_s,
                local_age=local.age_s,
                busy=busy,
            )
        except Exception as exc:
            return CachedState(connected=True, busy=busy, error=str(exc))

    def _require_connected(self) -> OffboardMavlinkClient:
        # This is a transport check only. PX4 performs all flight-state gates.
        return self._client_or_raise()

    def _send_command_packet(
        self,
        client: OffboardMavlinkClient,
        label: str,
        command: int,
        params: list[float],
    ) -> int | None:
        """Send one command and log its ACK without gating later packets."""
        try:
            result = client.send_command_long(command, params)
            self.log.write(f"[ACK] {label} command={command} result={result}")
            return result
        except Exception as exc:
            # A rejected or missing ACK is PX4/transport feedback, not a GUI
            # precondition. The caller may continue sending independent packets.
            self.log.write(f"[ACK/ERROR] {label} command={command}: {exc}")
            return None

    def arm_async(self, on_done: Callable[[bool], None] | None = None) -> None:
        self._submit("ARM", self.arm, on_done)

    def arm(self) -> None:
        client = self._require_connected()
        if self._reset_target_before_next_arm:
            self._reset_target_to_current_position(client, "ARM after previous DISARM")
            self._reset_target_before_next_arm = False
        target = self._target()
        client.set_position_target(*target)
        client.start_setpoint_stream()
        required = max(20, int(math.ceil(client.setpoint_rate_hz * 2.0)))
        client.wait_for_setpoint_count(required, timeout_s=5.0)
        self.log.write("[MISSION] OFFBOARD prewarm complete")
        self._send_command_packet(
            client,
            "OFFBOARD",
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float(PX4_CUSTOM_MAIN_MODE_OFFBOARD), 0, 0, 0, 0, 0],
        )
        self._send_command_packet(
            client,
            "ARM",
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0, 0, 0, 0, 0, 0],
        )
        self.log.write("[INFO] ARM/OFFBOARD packets sent; PX4 state decides acceptance")

    def reenter_offboard_async(
        self,
        on_done: Callable[[bool], None] | None = None,
    ) -> None:
        self._submit("REENTER OFFBOARD", self.reenter_offboard, on_done)

    def reenter_offboard(self) -> bool:
        """Explicitly request Offboard after an RC takeover.

        This reuses the existing setpoint stream. It never retries the mode
        change automatically; the GUI must request it again after a rejection.
        """
        client = self._require_connected()
        state = self.read_state()
        if not state.armed or state.landed is not False:
            raise RuntimeError("重新进入 Offboard 要求已解锁且处于飞行中")
        if not _state_has_local_position(state):
            raise RuntimeError("重新进入 Offboard 要求 Local Position 有效")
        if state.failsafe or not _state_mode_known(state):
            raise RuntimeError("重新进入 Offboard 要求 PX4 模式和故障状态有效")

        target = self._target()
        client.set_position_target(*target)
        client.start_setpoint_stream()
        # The stream counter is cumulative across ownership changes. Wait for
        # a fresh two-second stream interval before asking PX4 to re-enter.
        time.sleep(2.0)
        self.log.write("[MISSION] OFFBOARD re-entry prewarm complete")
        result = self._send_command_packet(
            client,
            "OFFBOARD RE-ENTRY",
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [
                float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                float(PX4_CUSTOM_MAIN_MODE_OFFBOARD),
                0,
                0,
                0,
                0,
                0,
            ],
        )
        accepted = result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        self.log.write(
            "[INFO] Offboard re-entry packet sent; waiting for PX4 mode feedback"
            if accepted
            else "[WARN] Offboard re-entry was not accepted; authority remains with RC"
        )
        return accepted

    def suspend_setpoint_stream(self) -> None:
        """Stop GUI Offboard setpoints after RC ownership is detected."""
        with self._client_lock:
            client = self.client
        if client is not None:
            client.stop_setpoint_stream()
            self.log.write("[AUTHORITY] RC override active; Offboard setpoint stream suspended")

    def _validate_number(self, value: str, low: float, high: float, name: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} 必须是数字") from exc
        if not math.isfinite(number) or not low <= number <= high:
            raise ValueError(f"{name} 必须在 {low:g}～{high:g} 范围内")
        return number

    def takeoff_async(
        self,
        altitude_text: str,
        on_done: Callable[[bool], None] | None = None,
    ) -> None:
        self._submit("TAKEOFF", lambda: self.takeoff(altitude_text), on_done)

    def takeoff(self, altitude_text: str) -> None:
        altitude = self._validate_number(altitude_text, MIN_ALTITUDE_M, MAX_ALTITUDE_M, "起飞高度")
        client = self._require_connected()
        target = self._target()
        base_z = self.origin[2] if self.origin is not None else 0.0
        target[2] = base_z - altitude
        self._set_target(tuple(target))
        client.start_setpoint_stream()
        self.log.write(
            f"[PACKET] TAKEOFF target_z={target[2]:.3f}; PX4 decides acceptance"
        )

    def move_async(
        self,
        direction: str,
        distance_text: str,
        on_done: Callable[[bool], None] | None = None,
    ) -> None:
        self._submit(direction, lambda: self.move(direction, distance_text), on_done)

    def move(self, direction: str, distance_text: str) -> None:
        distance = self._validate_number(distance_text, MIN_MOVE_M, MAX_MOVE_M, "移动距离")
        client = self._require_connected()
        target = self._target()
        delta = {"FORWARD": (distance, 0), "BACK": (-distance, 0), "RIGHT": (0, distance), "LEFT": (0, -distance)}
        if direction in delta:
            target[0] += delta[direction][0]
            target[1] += delta[direction][1]
        elif direction == "UP":
            target[2] -= distance
        elif direction == "DOWN":
            target[2] += distance
        else:
            raise ValueError(f"未知方向：{direction}")
        if self.origin is not None:
            target_altitude = -(target[2] - self.origin[2])
            if not MIN_ALTITUDE_M <= target_altitude <= MAX_ALTITUDE_M:
                raise ValueError(f"目标高度必须在 {MIN_ALTITUDE_M:g}～{MAX_ALTITUDE_M:g}m")
        self._set_target(tuple(target))
        client.start_setpoint_stream()
        self.log.write(
            f"[PACKET] MOVE {direction} {distance:.2f}m "
            f"target=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f}); PX4 decides acceptance"
        )

    def hover_async(self, on_done: Callable[[bool], None] | None = None) -> None:
        self._submit("HOVER", self.hover, on_done)

    def hover(self) -> None:
        client = self._require_connected()
        state = self.read_state()
        if None not in (state.local_x, state.local_y, state.local_z):
            self._set_target((state.local_x, state.local_y, state.local_z))
        else:
            self._set_target(self._target())
        client.start_setpoint_stream()
        self.log.write("[PACKET] HOVER target sent; PX4 decides acceptance")

    def land_async(self, on_done: Callable[[bool], None] | None = None) -> None:
        self._submit("LAND", lambda: self.land(safe=False), on_done)

    def land(self, safe: bool = False) -> bool:
        client = self._client_or_raise() if safe else self._require_connected()
        result = self._send_command_packet(
            client,
            "LAND",
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            [0, 0, 0, math.nan, math.nan, math.nan, 0],
        )
        self.log.write("[INFO] LAND packet sent; PX4 decides acceptance and landing state")
        return result is not None

    def disarm_async(self, on_done: Callable[[bool], None] | None = None) -> None:
        self._submit("DISARM", self.disarm, on_done)

    def disarm(self) -> bool:
        client = self._require_connected()
        result = self._send_command_packet(
            client,
            "DISARM",
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0, 0, 0, 0, 0, 0, 0],
        )
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.log.write(
                "[WARN] DISARM was not accepted; keeping Offboard setpoint stream "
                "active to avoid an artificial Offboard-loss failsafe"
            )
            return False

        self._reset_target_to_current_position(client, "DISARM accepted")
        self._reset_target_before_next_arm = True
        client.stop_setpoint_stream()
        self.log.write("[INFO] DISARM accepted; setpoint stream stopped")
        return True

    def safe_land_async(self, on_done: Callable[[bool], None] | None = None) -> None:
        self._submit("SAFE LAND", lambda: self.land(safe=True), on_done)

    def close(self) -> None:
        with self._client_lock:
            client = self.client
            self.client = None
        if client is not None:
            client.close()


class OffboardControlGui:
    TRANSLATIONS = {
        "zh": {
            "window": "PX4 Offboard MAVLink 发送控制台 v2", "connection": "连接", "mavlink": "MAVLink 连接：",
            "connect": "连接", "disconnect": "断开连接", "language": "语言：", "flight_state": "飞行状态",
            "telemetry_target": "遥测 / 控制目标", "control": "控制面板", "event_log": "事件日志",
            "phase": "状态机", "authority": "控制权", "px4": "PX4", "system": "系统", "mode": "模式", "armed": "已解锁", "landed": "已着陆",
            "failsafe": "故障保护", "gcs": "GCS 连接", "offboard": "Offboard", "position_valid": "位置有效",
            "gps": "GPS（仅监视）", "flow": "光流 active", "flow_quality": "光流质量", "actual": "当前位置",
            "velocity": "速度", "attitude": "姿态", "target": "控制目标", "takeoff_altitude": "起飞高度（0.3–3.0 m）",
            "movement_distance": "移动距离（0.1–5.0 m）", "arm": "解锁 ARM", "takeoff": "起飞 TAKEOFF",
            "forward": "前进", "back": "后退", "left": "左移", "right": "右移", "up": "上升",
            "down": "下降", "hover": "悬停", "land": "降落 LAND", "safe_land": "安全降落",
            "reenter_offboard": "重新进入 Offboard", "rc_control": "遥控器控制",
            "offboard_control": "GUI Offboard 控制", "rc_override": "遥控器已接管",
            "disarm": "上锁 DISARM", "log": "日志：", "connected": "已连接", "disconnected": "未连接",
            "heartbeat": "心跳", "safety": "安全提示", "disconnect_warning": "断开只关闭本地MAVLink连接，飞控按自身安全逻辑处理。",
            "busy_warning": "当前仍有控制命令执行中，请等待完成。", "armed_warning": "飞行器仍处于 armed，不能退出。请先 LAND + DISARM。",
        },
        "en": {
            "window": "PX4 Offboard MAVLink Sender v2", "connection": "Connection", "mavlink": "MAVLink connection:",
            "connect": "CONNECT", "disconnect": "DISCONNECT", "language": "Language:", "flight_state": "FLIGHT STATE",
            "telemetry_target": "TELEMETRY / TARGET", "control": "CONTROL", "event_log": "EVENT LOG",
            "phase": "Flight phase", "authority": "Control authority", "px4": "PX4", "system": "System", "mode": "Mode", "armed": "Armed", "landed": "Landed",
            "failsafe": "Failsafe", "gcs": "GCS connected", "offboard": "Offboard", "position_valid": "Position valid",
            "gps": "GPS (monitor only)", "flow": "Optical flow active", "flow_quality": "Flow quality", "actual": "Actual position",
            "velocity": "Velocity", "attitude": "Attitude", "target": "Target", "takeoff_altitude": "Takeoff altitude (0.3–3.0 m)",
            "movement_distance": "Movement distance (0.1–5.0 m)", "arm": "ARM", "takeoff": "TAKEOFF",
            "forward": "FORWARD", "back": "BACK", "left": "LEFT", "right": "RIGHT", "up": "UP",
            "down": "DOWN", "hover": "HOVER", "land": "LAND", "safe_land": "SAFE LAND", "disarm": "DISARM",
            "reenter_offboard": "RE-ENTER OFFBOARD", "rc_control": "RC CONTROL",
            "offboard_control": "GUI OFFBOARD CONTROL", "rc_override": "RC OVERRIDE ACTIVE",
            "log": "Log: ", "connected": "CONNECTED", "disconnected": "DISCONNECTED", "heartbeat": "heartbeat",
            "safety": "Safety notice", "disconnect_warning": "Disconnect only closes the local MAVLink client; PX4 handles flight safety.",
            "busy_warning": "A control command is still running. Please wait for it to finish.", "armed_warning": "The vehicle is still armed. LAND + DISARM before closing.",
        },
    }

    def _tr(self, key: str) -> str:
        return self.TRANSLATIONS[self.language_code].get(key, key)

    def _change_language(self, _event=None) -> None:
        self.language_code = "en" if self.language_var.get() == "English" else "zh"
        for child in self.root.winfo_children():
            child.destroy()
        self._build_widgets()
        self._restore_event_history()

    def _restore_event_history(self) -> None:
        self.event_text.configure(state="normal")
        self.event_text.delete("1.0", "end")
        if self._event_history:
            self.event_text.insert("end", "\n".join(self._event_history) + "\n")
            self.event_text.see("end")
        self.event_text.configure(state="disabled")

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.language_code = "zh"
        self.events: queue.Queue[str] = queue.Queue()
        self._event_history: list[str] = []
        self.log = EventLog(self.events.put)
        self.controller = FlightController(self.log)
        self._closing = False
        self._phase_lock = threading.Lock()
        self._phase_intent: str | None = None
        self._flight_phase = FlightPhase.DISCONNECTED
        self._authority_lock = threading.Lock()
        self._authority = ControlAuthority.RC_CONTROL
        self._mode_request: str | None = None
        self._mode_request_deadline = 0.0
        self._reentry_pending = False

        self.connection_var = tk.StringVar(value=DEFAULT_CONNECTION)
        self.language_var = tk.StringVar(value="中文")
        self.altitude_var = tk.StringVar(value="1.00")
        self.distance_var = tk.StringVar(value="0.50")
        self.status_vars = {name: tk.StringVar(value="—") for name in (
            "connection", "phase", "authority", "px4", "mode", "armed", "landed", "failsafe", "gcs", "offboard",
            "position_valid", "gps", "flow", "flow_quality", "system", "actual", "velocity", "attitude", "target",
        )}
        self._build_widgets()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh()
        self._drain_events()

    def _build_widgets(self) -> None:
        root = self.root
        root.title(self._tr("window"))
        root.geometry("1180x790")
        root.minsize(980, 680)
        root.configure(background="#edf1f5")
        style = ttk.Style(root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TButton", padding=(10, 6), font=("Segoe UI", 10))
        style.configure("TLabel", font=("Segoe UI", 10))
        style.configure("TLabelframe.Label", font=("Segoe UI", 10, "bold"))
        root.columnconfigure(0, weight=1)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(2, weight=1)

        conn = ttk.LabelFrame(root, text=self._tr("connection"), padding=10)
        conn.grid(row=0, column=0, columnspan=2, sticky="ew", padx=12, pady=(12, 8))
        conn.columnconfigure(1, weight=1)
        ttk.Label(conn, text=self._tr("mavlink")).grid(row=0, column=0, sticky="w", padx=(0, 8))
        ttk.Entry(conn, textvariable=self.connection_var, width=34).grid(row=0, column=1, sticky="ew", padx=6)
        self.connect_button = ttk.Button(conn, text=self._tr("connect"), command=self._connect)
        self.connect_button.grid(row=0, column=2, padx=4)
        self.disconnect_button = ttk.Button(conn, text=self._tr("disconnect"), command=self._disconnect)
        self.disconnect_button.grid(row=0, column=3, padx=4)
        ttk.Label(conn, text=self._tr("language")).grid(row=0, column=4, sticky="e", padx=(16, 5))
        language_combo = ttk.Combobox(conn, textvariable=self.language_var, values=("中文", "English"), state="readonly", width=10)
        language_combo.grid(row=0, column=5, sticky="e")
        language_combo.bind("<<ComboboxSelected>>", self._change_language)
        ttk.Label(conn, textvariable=self.status_vars["connection"]).grid(row=1, column=0, columnspan=6, sticky="w", pady=(8, 0))

        state_frame = ttk.LabelFrame(root, text=self._tr("flight_state"), padding=10)
        state_frame.grid(row=1, column=0, sticky="nsew", padx=(12, 6), pady=(0, 10))
        for i, key in enumerate(("phase", "authority", "px4", "system", "mode", "armed", "landed", "failsafe", "gcs", "offboard", "position_valid", "gps", "flow", "flow_quality")):
            row, col = divmod(i, 2)
            ttk.Label(state_frame, text=self._tr(key) + ":").grid(row=row, column=col * 2, sticky="w", padx=(0, 8), pady=2)
            ttk.Label(state_frame, textvariable=self.status_vars[key], width=18).grid(row=row, column=col * 2 + 1, sticky="w", pady=2)

        telemetry = ttk.LabelFrame(root, text=self._tr("telemetry_target"), padding=10)
        telemetry.grid(row=1, column=1, sticky="nsew", padx=(6, 12), pady=(0, 10))
        for r, key in enumerate(("actual", "velocity", "attitude", "target")):
            ttk.Label(telemetry, text=self._tr(key) + ":").grid(row=r, column=0, sticky="nw", padx=(0, 10), pady=3)
            ttk.Label(telemetry, textvariable=self.status_vars[key], justify="left", font=("Consolas", 10)).grid(row=r, column=1, sticky="w", pady=3)

        controls = ttk.LabelFrame(root, text=self._tr("control"), padding=10)
        controls.grid(row=2, column=0, sticky="nsew", padx=(12, 6), pady=(0, 12))
        controls.columnconfigure(0, weight=1)
        controls.columnconfigure(1, weight=1)
        ttk.Label(controls, text=self._tr("takeoff_altitude")).grid(row=0, column=0, sticky="w", pady=(0, 4))
        ttk.Entry(controls, textvariable=self.altitude_var, width=10).grid(row=0, column=1, sticky="w")
        self.action_buttons: dict[str, ttk.Button] = {}
        self.action_buttons["ARM"] = ttk.Button(controls, text=self._tr("arm"), command=self._arm)
        self.action_buttons["ARM"].grid(row=1, column=0, sticky="ew", pady=5)
        self.action_buttons["TAKEOFF"] = ttk.Button(controls, text=self._tr("takeoff"), command=self._takeoff)
        self.action_buttons["TAKEOFF"].grid(row=1, column=1, sticky="ew", padx=5, pady=5)
        ttk.Label(controls, text=self._tr("movement_distance")).grid(row=2, column=0, sticky="w", pady=(5, 4))
        ttk.Entry(controls, textvariable=self.distance_var, width=10).grid(row=2, column=1, sticky="w")
        dirs = (("FORWARD", 3, 1, "forward"), ("LEFT", 4, 0, "left"), ("HOVER", 4, 1, "hover"), ("RIGHT", 4, 2, "right"), ("BACK", 5, 1, "back"), ("UP", 3, 0, "up"), ("DOWN", 3, 2, "down"))
        for name, row, col, label_key in dirs:
            command = self._hover if name == "HOVER" else (lambda n=name: self._move(n))
            self.action_buttons[name] = ttk.Button(controls, text=self._tr(label_key), command=command)
            self.action_buttons[name].grid(row=row, column=col, sticky="ew", padx=2, pady=2)
        self.action_buttons["LAND"] = ttk.Button(controls, text=self._tr("land"), command=self._land)
        self.action_buttons["LAND"].grid(row=6, column=0, sticky="ew", pady=(8, 2))
        self.action_buttons["SAFE LAND"] = ttk.Button(controls, text=self._tr("safe_land"), command=self._safe_land)
        self.action_buttons["SAFE LAND"].grid(row=6, column=1, sticky="ew", padx=4, pady=(8, 2))
        self.action_buttons["DISARM"] = ttk.Button(controls, text=self._tr("disarm"), command=self._disarm)
        self.action_buttons["DISARM"].grid(row=7, column=0, columnspan=2, sticky="ew", pady=2)
        self.action_buttons["REENTER OFFBOARD"] = ttk.Button(
            controls,
            text=self._tr("reenter_offboard"),
            command=self._reenter_offboard,
        )
        self.action_buttons["REENTER OFFBOARD"].grid(row=8, column=0, columnspan=2, sticky="ew", pady=2)

        events = ttk.LabelFrame(root, text=self._tr("event_log"), padding=8)
        events.grid(row=2, column=1, sticky="nsew", padx=(6, 12), pady=(0, 12))
        events.rowconfigure(0, weight=1)
        events.columnconfigure(0, weight=1)
        self.event_text = tk.Text(events, height=12, state="disabled", wrap="none", background="#101820", foreground="#d8e8f0", insertbackground="#ffffff", font=("Consolas", 9), relief="flat", padx=8, pady=6)
        self.event_text.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(events, orient="vertical", command=self.event_text.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.event_text.configure(yscrollcommand=scroll.set)
        ttk.Label(events, text=f"{self._tr('log')}{self.log.path}").grid(row=1, column=0, columnspan=2, sticky="w", pady=(7, 0))

    def _connect(self) -> None:
        self.controller.connect_async(self.connection_var.get())

    def _disconnect(self) -> None:
        self.controller.disconnect_async()

    def _arm(self) -> None:
        self._mark_mode_request("ARM_OFFBOARD", timeout_s=10.0)
        self.controller.arm_async(on_done=self._arm_done)

    def _arm_done(self, success: bool) -> None:
        if not success:
            self._clear_mode_request("ARM_OFFBOARD")

    def _takeoff(self) -> None:
        self.controller.takeoff_async(
            self.altitude_var.get(),
            on_done=lambda success: self._intent_done("TAKING_OFF", success),
        )

    def _move(self, direction: str) -> None:
        self.controller.move_async(direction, self.distance_var.get())

    def _hover(self) -> None:
        self.controller.hover_async()

    def _land(self) -> None:
        self._mark_mode_request("LAND", timeout_s=8.0)
        self.controller.land_async(
            on_done=lambda success: self._intent_done("LANDING", success),
        )

    def _safe_land(self) -> None:
        self._mark_mode_request("LAND", timeout_s=8.0)
        self.controller.safe_land_async(
            on_done=lambda success: self._intent_done("LANDING", success),
        )

    def _disarm(self) -> None:
        self.controller.disarm_async(
            on_done=lambda success: self._disarm_done(success),
        )

    def _intent_done(self, intent: str, success: bool) -> None:
        with self._phase_lock:
            if success:
                self._phase_intent = intent
            elif self._phase_intent == intent:
                self._phase_intent = None

    def _disarm_done(self, success: bool) -> None:
        if success:
            with self._phase_lock:
                self._phase_intent = None

    def _reenter_offboard(self) -> None:
        state = self.controller.read_state()
        if not (
            state.connected
            and state.armed
            and state.landed is False
            and _state_has_local_position(state)
            and _state_mode_known(state)
            and not state.failsafe
        ):
            self.log.write(
                "[REJECT] OFFBOARD re-entry: requires connected + armed + airborne "
                "+ valid Local Position + known mode + no failsafe"
            )
            return
        with self._authority_lock:
            if self._authority is not ControlAuthority.RC_OVERRIDE:
                self.log.write("[REJECT] OFFBOARD re-entry: RC override is not active")
                return
            self._reentry_pending = True
        self._mark_mode_request("OFFBOARD_REENTRY", timeout_s=10.0)
        self.log.write("[AUTHORITY] User explicitly requested Offboard re-entry")
        self.controller.reenter_offboard_async(on_done=self._reentry_done)

    def _reentry_done(self, success: bool) -> None:
        if not success:
            with self._authority_lock:
                self._reentry_pending = False
            self._clear_mode_request("OFFBOARD_REENTRY")

    def _mark_mode_request(self, request: str, timeout_s: float) -> None:
        with self._authority_lock:
            self._mode_request = request
            self._mode_request_deadline = time.monotonic() + timeout_s

    def _clear_mode_request(self, request: str | None = None) -> None:
        with self._authority_lock:
            if request is None or self._mode_request == request:
                self._mode_request = None
                self._mode_request_deadline = 0.0

    def _mode_request_snapshot(self) -> str | None:
        with self._authority_lock:
            if self._mode_request is not None and time.monotonic() > self._mode_request_deadline:
                expired = self._mode_request
                self._mode_request = None
                self._mode_request_deadline = 0.0
                if expired == "OFFBOARD_REENTRY":
                    self._reentry_pending = False
                self.log.write(f"[AUTHORITY] GUI mode request expired: {expired}")
            return self._mode_request

    def _authority_snapshot(self) -> ControlAuthority:
        with self._authority_lock:
            return self._authority

    def _update_authority(self, state: CachedState) -> ControlAuthority:
        request = self._mode_request_snapshot()
        with self._authority_lock:
            previous = self._authority
            reentry_pending = self._reentry_pending
            authority, pending_after = derive_control_authority(
                previous,
                state,
                reentry_pending=reentry_pending,
                mode_request=request,
            )
            self._reentry_pending = pending_after
            if authority is ControlAuthority.OFFBOARD_CONTROL and request in {
                "ARM_OFFBOARD",
                "OFFBOARD_REENTRY",
            }:
                self._mode_request = None
                self._mode_request_deadline = 0.0
            self._authority = authority

        if authority is not previous:
            self.log.write(
                f"[AUTHORITY] {previous.value} -> {authority.value} mode={state.mode}"
            )
            if authority is ControlAuthority.RC_OVERRIDE:
                self.controller.suspend_setpoint_stream()
                self.log.write(
                    "[RC_OVERRIDE] 遥控器已接管；GUI Offboard 操作已锁定，"
                    "不会自动切回 Offboard"
                )
            elif authority is ControlAuthority.OFFBOARD_CONTROL and previous is ControlAuthority.RC_OVERRIDE:
                self.log.write("[AUTHORITY] GUI regained control after explicit Offboard re-entry")
        return authority

    def _authority_label(self, authority: ControlAuthority) -> str:
        return self._tr({
            ControlAuthority.RC_CONTROL: "rc_control",
            ControlAuthority.OFFBOARD_CONTROL: "offboard_control",
            ControlAuthority.RC_OVERRIDE: "rc_override",
        }[authority])

    def _phase_intent_snapshot(self) -> str | None:
        with self._phase_lock:
            return self._phase_intent

    @staticmethod
    def _fmt(value: float | None, unit: str = "") -> str:
        return "—" if value is None else f"{value:.2f}{unit}"

    def _refresh(self) -> None:
        if self._closing:
            return
        state = self.controller.read_state()
        authority = self._update_authority(state)
        intent = self._phase_intent_snapshot()
        with self.controller._target_lock:
            target_snapshot = (
                list(self.controller.target_local_ned)
                if self.controller.target_local_ned is not None
                else None
            )
        phase = derive_flight_phase(
            state,
            intent,
            self.controller.origin,
            target_snapshot,
        )
        if not state.armed and intent is not None:
            with self._phase_lock:
                self._phase_intent = None
        if phase is not self._flight_phase:
            self.log.write(f"[STATE] {self._flight_phase.value} -> {phase.value}")
            self._flight_phase = phase
        self.status_vars["phase"].set(phase.value)
        self.status_vars["authority"].set(self._authority_label(authority))
        self.status_vars["connection"].set(
            f"{self._tr('connection')}: {self._tr('connected') if state.connected else self._tr('disconnected')}  |  {self._tr('heartbeat')}: {self._fmt(state.heartbeat_age, 's')}"
        )
        self.status_vars["px4"].set("CONNECTED" if state.connected else "—")
        self.status_vars["system"].set(f"SYS={state.system_id} COMP={state.component_id}" if state.connected else "—")
        self.status_vars["mode"].set(state.mode)
        self.status_vars["armed"].set(str(state.armed))
        self.status_vars["landed"].set(str(state.landed))
        self.status_vars["failsafe"].set(str(state.failsafe))
        self.status_vars["gcs"].set(str(state.gcs_connected))
        self.status_vars["offboard"].set(str(state.mode.upper() == "OFFBOARD"))
        self.status_vars["position_valid"].set(str(state.position_valid))
        self.status_vars["gps"].set(str(state.gps_active))
        self.status_vars["flow"].set(str(state.flow_active))
        self.status_vars["flow_quality"].set(str(state.flow_quality if state.flow_quality is not None else "—"))
        altitude = None if state.local_z is None or self.controller.origin is None else self.controller.origin[2] - state.local_z
        self.status_vars["actual"].set(f"N {self._fmt(state.local_x)}  E {self._fmt(state.local_y)}  Alt {self._fmt(altitude, 'm')}\nLocal z {self._fmt(state.local_z, 'm')}")
        self.status_vars["velocity"].set(f"vx {self._fmt(state.vx)}  vy {self._fmt(state.vy)}  vz {self._fmt(state.vz)} m/s")
        self.status_vars["attitude"].set(f"roll {self._fmt(state.roll, '°')}  pitch {self._fmt(state.pitch, '°')}  yaw {self._fmt(state.yaw, '°')}")
        target = target_snapshot
        if target is None:
            self.status_vars["target"].set("N —  E —  Alt —")
        else:
            target_alt = None if self.controller.origin is None else self.controller.origin[2] - target[2]
            self.status_vars["target"].set(f"N {target[0]:.2f}  E {target[1]:.2f}  Alt {target_alt:.2f}m\nLocal z {target[2]:.2f}")
        if state.error:
            self.status_vars["connection"].set(self.status_vars["connection"].get() + f"  | ERROR: {state.error}")
        permissions = button_permissions(state, phase, authority)
        self.connect_button.configure(
            state="normal" if phase is FlightPhase.DISCONNECTED and not state.busy else "disabled"
        )
        self.disconnect_button.configure(
            state="normal" if phase is FlightPhase.READY and not state.busy else "disabled"
        )
        for name, button in self.action_buttons.items():
            button.configure(state="normal" if permissions[name] else "disabled")
        self.root.after(200, self._refresh)

    def _drain_events(self) -> None:
        try:
            while True:
                line = self.events.get_nowait()
                self._event_history.append(line)
                self.event_text.configure(state="normal")
                self.event_text.insert("end", line + "\n")
                self.event_text.see("end")
                self.event_text.configure(state="disabled")
        except queue.Empty:
            pass
        if not self._closing:
            self.root.after(100, self._drain_events)

    def _on_close(self) -> None:
        state = self.controller.read_state()
        if state.busy:
            messagebox.showwarning(self._tr("safety"), self._tr("busy_warning"))
            return
        self._closing = True
        self.controller.close()
        self.log.write("[INFO] GUI closed")
        self.log.close()
        self.root.destroy()


def self_test() -> None:
    """Verify packet behavior and the feedback-driven GUI state matrix."""
    assert MIN_ALTITUDE_M == 0.3 and MAX_ALTITUDE_M == 3.0
    assert MIN_MOVE_M == 0.1 and MAX_MOVE_M == 5.0
    assert POSITION_ONLY_TYPE_MASK != 0
    assert not hasattr(FlightController, "_require_optical_flow_gate")
    for method_name in ("_require_connected", "arm", "takeoff", "move", "hover", "land", "disarm"):
        source = inspect.getsource(getattr(FlightController, method_name))
        assert "state.armed" not in source
        assert "state.failsafe" not in source
        assert "state.position_valid" not in source
        assert "state.mode.upper()" not in source

    origin = (0.0, 0.0, 0.0)
    target = [0.0, 0.0, -1.0]
    base = dict(
        connected=True,
        mode="OFFBOARD",
        local_x=0.0,
        local_y=0.0,
        local_z=0.0,
        vx=0.0,
        vy=0.0,
        vz=0.0,
        position_valid=True,
        busy=False,
    )
    def make_state(**overrides: object) -> CachedState:
        values = dict(base)
        values.update(overrides)
        return CachedState(**values)

    cases = (
        ("DISCONNECTED", CachedState(), None, FlightPhase.DISCONNECTED, set()),
        ("READY", make_state(armed=False, landed=True), None, FlightPhase.READY, {"ARM"}),
        (
            "ARMED",
            make_state(armed=True, landed=True),
            None,
            FlightPhase.ARMED,
            {"TAKEOFF", "LAND", "SAFE LAND"},
        ),
        (
            "TAKING_OFF",
            make_state(armed=True, landed=False, local_z=-0.2, vz=-0.5),
            "TAKING_OFF",
            FlightPhase.TAKING_OFF,
            {"LAND", "SAFE LAND"},
        ),
        (
            "FLYING",
            make_state(armed=True, landed=False, local_z=-1.0, vz=0.0),
            None,
            FlightPhase.FLYING,
            set(MOVEMENT_ACTIONS) | {"LAND", "SAFE LAND"},
        ),
        (
            "LANDING",
            make_state(armed=True, landed=False, mode="AUTO.LAND"),
            "LANDING",
            FlightPhase.LANDING,
            set(),
        ),
        (
            "LANDED",
            make_state(armed=True, landed=True, mode="AUTO.LAND"),
            "LANDING",
            FlightPhase.LANDED,
            {"DISARM"},
        ),
    )
    for name, state, intent, expected_phase, allowed in cases:
        phase = derive_flight_phase(state, intent, origin, target)
        assert phase is expected_phase, (name, phase, expected_phase)
        authority = (
            ControlAuthority.OFFBOARD_CONTROL
            if name in {"ARMED", "TAKING_OFF", "FLYING"}
            else ControlAuthority.RC_CONTROL
        )
        permissions = button_permissions(state, phase, authority)
        actual_allowed = {action for action, enabled in permissions.items() if enabled}
        assert actual_allowed == allowed, (name, actual_allowed, allowed)

    reached = make_state(armed=True, landed=False, local_z=-1.0, vz=0.0)
    assert derive_flight_phase(reached, "TAKING_OFF", origin, target) is FlightPhase.FLYING
    no_position = make_state(armed=False, landed=True, position_valid=False)
    assert not button_permissions(no_position, FlightPhase.READY)["ARM"]
    manual = make_state(armed=True, landed=False, mode="POSCTL")
    authority, pending = derive_control_authority(
        ControlAuthority.OFFBOARD_CONTROL,
        manual,
    )
    assert authority is ControlAuthority.RC_OVERRIDE and not pending
    override_permissions = button_permissions(
        manual,
        FlightPhase.FLYING,
        authority,
    )
    assert set(name for name, enabled in override_permissions.items() if enabled) == {
        "REENTER OFFBOARD"
    }
    reentry_state = make_state(armed=True, landed=False, mode="OFFBOARD")
    authority, pending = derive_control_authority(
        ControlAuthority.RC_OVERRIDE,
        reentry_state,
        reentry_pending=True,
        mode_request="OFFBOARD_REENTRY",
    )
    assert authority is ControlAuthority.OFFBOARD_CONTROL and not pending
    assert button_permissions(
        reentry_state,
        FlightPhase.FLYING,
        authority,
    )["HOVER"]
    print("GUI v2 self-test: packet gates OK; state-machine matrix OK; authority arbitration OK; no PX4 connection opened")


def main() -> None:
    parser = argparse.ArgumentParser(description="PX4 Offboard MAVLink packet sender v2")
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument("--self-test", action="store_true", help="run packet-sender validation test")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    root = tk.Tk()
    app = OffboardControlGui(root)
    app.connection_var.set(args.connection)
    root.mainloop()


if __name__ == "__main__":
    main()
