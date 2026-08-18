#!/usr/bin/env python3
"""PX4 SITL no-GPS optical-flow Offboard validation mission."""

from __future__ import annotations

import argparse
import math
import signal
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from enum import Enum, auto
from pathlib import Path

from pymavlink import mavutil

import generated_mavlink as px4_mavlink

from mission_controller_v4 import MavlinkClient, MissionLogger, VehicleSnapshot


# This PX4 tree publishes ESTIMATOR_SENSOR_FUSION_STATUS from the MAVLink
# development dialect. Use the matching generated dialect for direct evidence
# of GPS/optical-flow fusion state.
mavutil.mavlink = px4_mavlink


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
FUSION_INDEX_GPS = 0
FUSION_INDEX_OPTICAL_FLOW = 1
FUSION_INDEX_RANGE_FINDER = 5
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


class OffboardState(Enum):
    IDLE = auto()
    CONNECTING = auto()
    PREWARMING = auto()
    ARMING = auto()
    ENTERING_OFFBOARD = auto()
    TAKING_OFF = auto()
    HOVERING = auto()
    MOVING = auto()
    RETURNING = auto()
    HOLDING_TARGET = auto()
    LANDING = auto()
    DISARMING = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass(frozen=True)
class LocalPosition:
    x_m: float | None
    y_m: float | None
    z_m: float | None
    vx_m_s: float | None
    vy_m_s: float | None
    vz_m_s: float | None
    age_s: float | None


@dataclass(frozen=True)
class EstimatorEvidence:
    horizontal_position_valid: bool
    horizontal_velocity_valid: bool
    vertical_position_valid: bool
    vertical_velocity_valid: bool
    gps_intended: bool | None
    gps_active: bool | None
    optical_flow_intended: bool | None
    optical_flow_active: bool | None
    range_finder_active: bool | None
    flow_age_s: float | None
    good_flow_age_s: float | None
    flow_quality: int | None
    gps_fix_type: int | None
    gps_satellites_visible: int | None
    fusion_age_s: float | None


class OffboardMavlinkClient(MavlinkClient):
    """v4 MAVLink client extended with local-position state and setpoint streaming."""

    def __init__(self, connection_string: str, setpoint_rate_hz: float) -> None:
        super().__init__(connection_string)
        self.local_x_m: float | None = None
        self.local_y_m: float | None = None
        self.local_z_m: float | None = None
        self.local_vx_m_s: float | None = None
        self.local_vy_m_s: float | None = None
        self.local_vz_m_s: float | None = None
        self.last_local_position_monotonic: float | None = None
        self.estimator_status_flags = 0
        self.fusion_intended: list[int] | None = None
        self.fusion_active: list[int] | None = None
        self.last_fusion_status_monotonic: float | None = None
        self.flow_quality: int | None = None
        self.last_flow_monotonic: float | None = None
        self.last_good_flow_monotonic: float | None = None
        self.gps_fix_type: int | None = None
        self.gps_satellites_visible: int | None = None
        self.status_texts: list[str] = []
        self.px4_gcs_lost = False

        self.setpoint_rate_hz = setpoint_rate_hz
        self._setpoint_condition = threading.Condition()
        self._setpoint_target: tuple[float, float, float] | None = None
        self._setpoint_count = 0
        self._setpoint_thread: threading.Thread | None = None
        self._setpoint_stop_event = threading.Event()
        self._started_monotonic = time.monotonic()

    def close(self) -> None:
        self.stop_setpoint_stream()
        super().close()

    def local_position(self) -> LocalPosition:
        with self._condition:
            self._raise_thread_errors()
            age = None
            if self.last_local_position_monotonic is not None:
                age = time.monotonic() - self.last_local_position_monotonic
            return LocalPosition(
                x_m=self.local_x_m,
                y_m=self.local_y_m,
                z_m=self.local_z_m,
                vx_m_s=self.local_vx_m_s,
                vy_m_s=self.local_vy_m_s,
                vz_m_s=self.local_vz_m_s,
                age_s=age,
            )

    def estimator_evidence(self) -> EstimatorEvidence:
        with self._condition:
            self._raise_thread_errors()
            now = time.monotonic()
            flags = self.estimator_status_flags
            return EstimatorEvidence(
                horizontal_position_valid=bool(
                    flags
                    & (
                        px4_mavlink.ESTIMATOR_POS_HORIZ_REL
                        | px4_mavlink.ESTIMATOR_POS_HORIZ_ABS
                    )
                ),
                horizontal_velocity_valid=bool(
                    flags & px4_mavlink.ESTIMATOR_VELOCITY_HORIZ
                ),
                vertical_position_valid=bool(
                    flags
                    & (
                        px4_mavlink.ESTIMATOR_POS_VERT_ABS
                        | px4_mavlink.ESTIMATOR_POS_VERT_AGL
                    )
                ),
                vertical_velocity_valid=bool(
                    flags & px4_mavlink.ESTIMATOR_VELOCITY_VERT
                ),
                gps_intended=self._fusion_enabled(
                    self.fusion_intended, FUSION_INDEX_GPS
                ),
                gps_active=self._fusion_enabled(self.fusion_active, FUSION_INDEX_GPS),
                optical_flow_intended=self._fusion_enabled(
                    self.fusion_intended, FUSION_INDEX_OPTICAL_FLOW
                ),
                optical_flow_active=self._fusion_enabled(
                    self.fusion_active, FUSION_INDEX_OPTICAL_FLOW
                ),
                range_finder_active=self._fusion_enabled(
                    self.fusion_active, FUSION_INDEX_RANGE_FINDER
                ),
                flow_age_s=(
                    None
                    if self.last_flow_monotonic is None
                    else now - self.last_flow_monotonic
                ),
                good_flow_age_s=(
                    None
                    if self.last_good_flow_monotonic is None
                    else now - self.last_good_flow_monotonic
                ),
                flow_quality=self.flow_quality,
                gps_fix_type=self.gps_fix_type,
                gps_satellites_visible=self.gps_satellites_visible,
                fusion_age_s=(
                    None
                    if self.last_fusion_status_monotonic is None
                    else now - self.last_fusion_status_monotonic
                ),
            )

    def request_estimator_streams(self) -> None:
        requests = (
            (px4_mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2.0),
            (px4_mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0),
            (px4_mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, 10.0),
            (px4_mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS, 2.0),
            (px4_mavlink.MAVLINK_MSG_ID_ESTIMATOR_SENSOR_FUSION_STATUS, 2.0),
            (px4_mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2.0),
        )
        with self._send_lock:
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

    @staticmethod
    def _fusion_enabled(values: list[int] | None, index: int) -> bool | None:
        if values is None or len(values) <= index:
            return None
        return bool(values[index] & 1)

    def set_position_target(self, x_m: float, y_m: float, z_m: float) -> None:
        if not all(math.isfinite(value) for value in (x_m, y_m, z_m)):
            raise ValueError("Offboard位置目标必须为有限数值")
        with self._setpoint_condition:
            self._setpoint_target = (x_m, y_m, z_m)
            self._setpoint_condition.notify_all()

    def start_setpoint_stream(self) -> None:
        with self._setpoint_condition:
            if self._setpoint_target is None:
                raise RuntimeError("启动Offboard设定值流前必须设置位置目标")
            if self._setpoint_thread is not None and self._setpoint_thread.is_alive():
                return
            self._setpoint_stop_event.clear()
            self._setpoint_thread = threading.Thread(
                target=self._setpoint_loop,
                name="offboard-setpoint-sender",
                daemon=True,
            )
            self._setpoint_thread.start()

    def stop_setpoint_stream(self) -> None:
        self._setpoint_stop_event.set()
        with self._setpoint_condition:
            self._setpoint_condition.notify_all()
        if self._setpoint_thread is not None:
            self._setpoint_thread.join(timeout=2.0)
            self._setpoint_thread = None

    def wait_for_setpoint_count(self, minimum_count: int, timeout_s: float) -> int:
        deadline = time.monotonic() + timeout_s
        with self._setpoint_condition:
            while self._setpoint_count < minimum_count:
                self._raise_thread_errors()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"Offboard设定值预热超时：sent={self._setpoint_count}, "
                        f"required={minimum_count}"
                    )
                self._setpoint_condition.wait(timeout=remaining)
            return self._setpoint_count

    def setpoint_count(self) -> int:
        with self._setpoint_condition:
            return self._setpoint_count

    def _setpoint_loop(self) -> None:
        period_s = 1.0 / self.setpoint_rate_hz
        try:
            while not self._stop_event.is_set() and not self._setpoint_stop_event.is_set():
                with self._setpoint_condition:
                    target = self._setpoint_target
                if target is not None and self.target_system != 0:
                    time_boot_ms = int(
                        (time.monotonic() - self._started_monotonic) * 1000.0
                    ) & 0xFFFFFFFF
                    with self._send_lock:
                        self.connection.mav.set_position_target_local_ned_send(
                            time_boot_ms,
                            self.target_system,
                            self.target_component,
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                            POSITION_ONLY_TYPE_MASK,
                            target[0],
                            target[1],
                            target[2],
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        )
                    with self._setpoint_condition:
                        self._setpoint_count += 1
                        self._setpoint_condition.notify_all()
                self._setpoint_stop_event.wait(period_s)
        except Exception as error:
            if not self._setpoint_stop_event.is_set() and not self._stop_event.is_set():
                with self._condition:
                    self._sender_error = error
                    self._condition.notify_all()
                with self._setpoint_condition:
                    self._setpoint_condition.notify_all()

    def _receive_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                # The inherited client is not started separately; this is the only read path.
                message = self.connection.recv_match(blocking=True, timeout=0.5)
                if message is None or message.get_type() == "BAD_DATA":
                    continue

                message_type = message.get_type()
                now = time.monotonic()
                with self._condition:
                    if message_type == "HEARTBEAT" and message.get_srcSystem() != 255:
                        self.target_system = message.get_srcSystem()
                        self.target_component = message.get_srcComponent()
                        self.armed = bool(
                            message.base_mode
                            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )
                        self.mode = mavutil.mode_string_v10(message)
                        # Some pymavlink/PX4 combinations do not map main mode 6
                        # and return UNKNOWN even though PX4 is in Offboard.
                        px4_main_mode = (int(message.custom_mode) >> 16) & 0xFF
                        if px4_main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                            self.mode = "OFFBOARD"
                        self.failsafe = message.system_status in self.FAILSAFE_STATES
                        self.last_heartbeat_monotonic = now

                    elif message_type == "GLOBAL_POSITION_INT":
                        self.altitude_m = message.relative_alt / 1000.0
                        self.altitude_amsl_m = message.alt / 1000.0
                        self.vertical_speed_m_s = message.vz / 100.0

                    elif message_type == "LOCAL_POSITION_NED":
                        self.local_x_m = float(message.x)
                        self.local_y_m = float(message.y)
                        self.local_z_m = float(message.z)
                        self.local_vx_m_s = float(message.vx)
                        self.local_vy_m_s = float(message.vy)
                        self.local_vz_m_s = float(message.vz)
                        self.last_local_position_monotonic = now

                    elif message_type == "ESTIMATOR_STATUS":
                        self.estimator_status_flags = int(message.flags)

                    elif message_type == "ESTIMATOR_SENSOR_FUSION_STATUS":
                        self.fusion_intended = [int(value) for value in message.intended]
                        self.fusion_active = [int(value) for value in message.active]
                        self.last_fusion_status_monotonic = now

                    elif message_type == "OPTICAL_FLOW_RAD":
                        self.flow_quality = int(message.quality)
                        self.last_flow_monotonic = now
                        if self.flow_quality > 0:
                            self.last_good_flow_monotonic = now

                    elif message_type == "GPS_RAW_INT":
                        self.gps_fix_type = int(message.fix_type)
                        self.gps_satellites_visible = int(message.satellites_visible)

                    elif message_type == "STATUSTEXT":
                        text = str(message.text).rstrip("\\x00")
                        if text:
                            self.status_texts.append(text)
                            self.status_texts = self.status_texts[-20:]
                            normalized = text.lower()
                            if "gcs connection lost" in normalized:
                                self.px4_gcs_lost = True
                            elif "gcs connection regained" in normalized:
                                self.px4_gcs_lost = False

                    elif message_type == "ATTITUDE":
                        self.roll_deg = math.degrees(message.roll)
                        self.pitch_deg = math.degrees(message.pitch)
                        self.yaw_deg = math.degrees(message.yaw)

                    elif message_type == "EXTENDED_SYS_STATE":
                        self.landed = (
                            message.landed_state
                            == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                        )

                    elif message_type == "COMMAND_ACK":
                        self._ack_sequence += 1
                        self._acks[message.command] = (
                            self._ack_sequence,
                            message.result,
                        )

                    self._update_sequence += 1
                    self._condition.notify_all()
        except Exception as error:
            with self._condition:
                self._receiver_error = error
                self._condition.notify_all()


class OpticalFlowControllerV6:
    def __init__(
        self,
        connection_string: str,
        profile: str,
        altitude_m: float,
        move_north_m: float,
        hold_seconds: float,
        setpoint_rate_hz: float,
        logger: MissionLogger,
    ) -> None:
        self.client = OffboardMavlinkClient(connection_string, setpoint_rate_hz)
        self.profile = profile
        self.altitude_m = altitude_m
        self.move_north_m = move_north_m
        self.hold_seconds = hold_seconds
        self.setpoint_rate_hz = setpoint_rate_hz
        self.logger = logger
        self.state = OffboardState.IDLE
        self.stop_requested = threading.Event()
        self.land_requested = False
        self.origin: tuple[float, float, float] | None = None
        self.local_position_invalid_count = 0
        self.failsafe_count = 0
        self.gcs_lost_count = 0
        self.max_abs_roll_deg = 0.0
        self.max_abs_pitch_deg = 0.0
        self._failsafe_active = False
        self._gcs_lost_active = False
        self._local_invalid_active = False

    def run(self) -> None:
        self.client.start()
        try:
            self._connect()
            self._verify_estimator_gate()
            self._prewarm_setpoints()
            # With GPS disabled, the default disarmed LOITER/ALTCTL mode can
            # require a global/home estimate or manual input. Entering
            # Offboard while disarmed, after the required setpoint prewarm,
            # keeps the normal PX4 arming checks intact and lets the external
            # controller arm in the mode it will actually fly.
            self._enter_offboard()
            self._arm()
            self._reach_target("TAKEOFF", self._hover_target(), OffboardState.TAKING_OFF)
            self._hold("HOVER", OffboardState.HOVERING)
            if self.profile == "move":
                self._reach_target("MOVE", self._move_target(), OffboardState.MOVING)
                self._hold("TARGET", OffboardState.HOLDING_TARGET)
                self._reach_target(
                    "RETURN", self._hover_target(), OffboardState.RETURNING
                )
                self._hold("ORIGIN", OffboardState.HOLDING_TARGET)
            self._land()
            self._disarm()
            self._set_state(OffboardState.COMPLETE)
            self.logger.info(
                "[MISSION] Optical-flow Offboard mission complete "
                f"profile={self.profile} local_invalid={self.local_position_invalid_count} "
                f"failsafe_count={self.failsafe_count} gcs_lost_count={self.gcs_lost_count} "
                f"max_roll={self.max_abs_roll_deg:.2f}deg "
                f"max_pitch={self.max_abs_pitch_deg:.2f}deg"
            )
        except Exception:
            self._set_state(OffboardState.FAILED)
            self._emergency_land_if_needed()
            raise
        finally:
            self.client.close()

    def request_stop(self) -> None:
        self.stop_requested.set()

    def _connect(self) -> None:
        self._set_state(OffboardState.CONNECTING)
        self.logger.info(f"[INFO] Connecting PX4 through {self.client.connection_string}")
        self.client.wait_for(
            lambda snapshot: snapshot.last_heartbeat_age_s is not None,
            timeout_s=15.0,
            description="PX4 HEARTBEAT",
        )
        ready = self.client.wait_for(
            lambda snapshot: (
                snapshot.gcs_connected
                and snapshot.gcs_heartbeats_sent >= 3
                and snapshot.roll_deg is not None
                and snapshot.pitch_deg is not None
            ),
            timeout_s=10.0,
            description="GCS连接、姿态及至少3个GCS HEARTBEAT",
        )
        self.client.request_estimator_streams()
        deadline = time.monotonic() + 10.0
        local = self.client.local_position()
        while (
            local.x_m is None
            or local.y_m is None
            or local.z_m is None
            or local.age_s is None
            or local.age_s >= 1.0
        ):
            if time.monotonic() >= deadline:
                raise TimeoutError(f"等待有效LOCAL_POSITION_NED超时：{local}")
            sequence = self.client.update_sequence()
            self.client.wait_for_update(sequence, timeout_s=0.5)
            local = self.client.local_position()

        if ready.armed:
            raise RuntimeError("任务开始前飞行器必须处于Disarmed")
        self._require_healthy(ready, "CONNECT")
        self.origin = (local.x_m, local.y_m, local.z_m)
        self.logger.info(
            f"[OK] PX4 connected system={self.client.target_system} "
            f"component={self.client.target_component} mode={ready.mode}"
        )
        self.logger.info(
            f"[STATE] local_origin x={local.x_m:.3f} y={local.y_m:.3f} "
            f"z={local.z_m:.3f} armed={ready.armed} failsafe={ready.failsafe}"
        )

    def _verify_estimator_gate(self) -> None:
        deadline = time.monotonic() + 15.0
        next_report = 0.0
        sequence = self.client.update_sequence()
        while time.monotonic() < deadline:
            sequence = self.client.wait_for_update(sequence, timeout_s=0.5)
            snapshot = self.client.snapshot()
            local = self.client.local_position()
            evidence = self.client.estimator_evidence()
            self._require_healthy(snapshot, "ESTIMATOR_GATE")
            gps_unusable = evidence.gps_fix_type is None or evidence.gps_fix_type < 3
            passed = (
                local.age_s is not None
                and local.age_s < 1.0
                and evidence.horizontal_position_valid
                and evidence.horizontal_velocity_valid
                and evidence.vertical_position_valid
                and evidence.vertical_velocity_valid
                and evidence.gps_intended is False
                and evidence.gps_active is False
                and evidence.optical_flow_intended is True
                and evidence.optical_flow_active is True
                and evidence.good_flow_age_s is not None
                and evidence.good_flow_age_s < 1.0
                and evidence.fusion_age_s is not None
                and evidence.fusion_age_s < 2.0
                and gps_unusable
            )
            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    "[GATE] "
                    f"gps_active={evidence.gps_active} gps_fix={evidence.gps_fix_type} "
                    f"satellites={evidence.gps_satellites_visible} "
                    f"flow_active={evidence.optical_flow_active} "
                    f"flow_quality={evidence.flow_quality} "
                    f"good_flow_age={self._format(evidence.good_flow_age_s, 's')} "
                    f"hpos_valid={evidence.horizontal_position_valid} "
                    f"hvel_valid={evidence.horizontal_velocity_valid} "
                    f"vpos_valid={evidence.vertical_position_valid} "
                    f"vvel_valid={evidence.vertical_velocity_valid}"
                )
                next_report = now + 1.0
            if passed:
                self.logger.info(
                    "[OK] Optical-flow estimator gate passed: "
                    "GPS inactive, optical flow active, Local Position valid"
                )
                return
        raise TimeoutError("无GPS光流估计证据链在15秒内未全部满足，禁止ARM")

    def _prewarm_setpoints(self) -> None:
        self._set_state(OffboardState.PREWARMING)
        target = self._hover_target()
        self.client.set_position_target(*target)
        self.client.start_setpoint_stream()
        required = max(20, int(math.ceil(self.setpoint_rate_hz * 2.0)))
        sent = self.client.wait_for_setpoint_count(required, timeout_s=5.0)
        self.logger.info(
            f"[OK] Offboard setpoint prewarm sent={sent} rate={self.setpoint_rate_hz:.1f}Hz "
            f"target=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})"
        )

    def _arm(self) -> None:
        self._set_state(OffboardState.ARMING)
        try:
            result = self.client.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        except Exception:
            with self.client._condition:
                # Give the single receiver a short opportunity to consume the
                # STATUSTEXT that normally accompanies a denied arm command.
                self.client._condition.wait(timeout=0.5)
                recent_status = list(self.client.status_texts[-10:])
            if recent_status:
                self.logger.error("[PX4] Recent STATUSTEXT: " + " | ".join(recent_status))
            raise
        self.logger.info(f"[ACK] ARM command=400 result={result}")
        armed = self.client.wait_for(
            lambda snapshot: snapshot.armed,
            timeout_s=10.0,
            description="armed=True",
        )
        self._require_healthy(armed, "ARM")
        self.logger.info("[OK] ARM success")

    def _enter_offboard(self) -> None:
        self._set_state(OffboardState.ENTERING_OFFBOARD)
        result = self.client.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [
                float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                float(PX4_CUSTOM_MAIN_MODE_OFFBOARD),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
        self.logger.info(f"[ACK] OFFBOARD command=176 result={result}")
        offboard = self.client.wait_for(
            lambda snapshot: snapshot.mode.upper() == "OFFBOARD",
            timeout_s=8.0,
            description="mode=OFFBOARD",
        )
        self._require_healthy(offboard, "OFFBOARD")
        self.logger.info(
            f"[OK] OFFBOARD active offboard=True setpoints_sent={self.client.setpoint_count()}"
        )

    def _reach_target(
        self,
        name: str,
        target: tuple[float, float, float],
        state: OffboardState,
    ) -> None:
        self._set_state(state)
        self.client.set_position_target(*target)
        self.logger.info(
            f"[MISSION] {name} target_local_ned "
            f"x={target[0]:.2f} y={target[1]:.2f} z={target[2]:.2f}"
        )
        deadline = time.monotonic() + 35.0
        stable_since: float | None = None
        next_report = 0.0
        sequence = self.client.update_sequence()
        while time.monotonic() < deadline:
            self._check_stop()
            sequence = self.client.wait_for_update(sequence, timeout_s=0.5)
            snapshot = self.client.snapshot()
            local = self.client.local_position()
            self._require_offboard_healthy(snapshot, name)
            evidence = self.client.estimator_evidence()
            if local.x_m is None or local.y_m is None or local.z_m is None:
                continue
            error = math.sqrt(
                (local.x_m - target[0]) ** 2
                + (local.y_m - target[1]) ** 2
                + (local.z_m - target[2]) ** 2
            )
            speed = None
            if None not in (local.vx_m_s, local.vy_m_s, local.vz_m_s):
                speed = math.sqrt(
                    local.vx_m_s**2 + local.vy_m_s**2 + local.vz_m_s**2
                )
            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    f"[STATE] {name} position=({local.x_m:.2f},{local.y_m:.2f},"
                    f"{local.z_m:.2f}) error={error:.2f}m "
                    f"speed={self._format(speed, 'm/s')} mode={snapshot.mode} "
                    f"hpos_valid={evidence.horizontal_position_valid} "
                    f"flow_active={evidence.optical_flow_active} "
                    f"gps_active={evidence.gps_active} failsafe={snapshot.failsafe} "
                    f"gcs_connected={snapshot.gcs_connected}"
                )
                next_report = now + 1.0
            if error <= 0.25 and speed is not None and speed <= 0.25:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= 1.0:
                    self.logger.info(
                        f"[OK] {name} target reached position=({local.x_m:.2f},"
                        f"{local.y_m:.2f},{local.z_m:.2f}) error={error:.2f}m"
                    )
                    return
            else:
                stable_since = None
        raise TimeoutError(f"35秒内未稳定到达{name}目标")

    def _hold(self, name: str, state: OffboardState) -> None:
        self._set_state(state)
        started = time.monotonic()
        next_report = 0.0
        sequence = self.client.update_sequence()
        while time.monotonic() - started < self.hold_seconds:
            self._check_stop()
            sequence = self.client.wait_for_update(sequence, timeout_s=0.5)
            snapshot = self.client.snapshot()
            local = self.client.local_position()
            self._require_offboard_healthy(snapshot, name)
            evidence = self.client.estimator_evidence()
            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    f"[STATE] {name} hold={now-started:.1f}/{self.hold_seconds:.1f}s "
                    f"position=({self._format(local.x_m, 'm')},"
                    f"{self._format(local.y_m, 'm')},{self._format(local.z_m, 'm')}) "
                    f"velocity=({self._format(local.vx_m_s, 'm/s')},"
                    f"{self._format(local.vy_m_s, 'm/s')},"
                    f"{self._format(local.vz_m_s, 'm/s')}) "
                    f"hpos_valid={evidence.horizontal_position_valid} "
                    f"flow_active={evidence.optical_flow_active} "
                    f"gps_active={evidence.gps_active} "
                    f"failsafe={snapshot.failsafe} "
                    f"gcs_connected={snapshot.gcs_connected} "
                    f"setpoints_sent={self.client.setpoint_count()}"
                )
                next_report = now + 1.0
        self.logger.info(f"[OK] {name} holding complete")

    def _land(self) -> None:
        self._set_state(OffboardState.LANDING)
        self.logger.info("[MISSION] LAND")
        result = self.client.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            [0.0, 0.0, 0.0, math.nan, math.nan, math.nan, 0.0],
        )
        self.land_requested = True
        self.logger.info(f"[ACK] LAND command=21 result={result}")

        deadline = time.monotonic() + 40.0
        next_report = 0.0
        sequence = self.client.update_sequence()
        while time.monotonic() < deadline:
            sequence = self.client.wait_for_update(sequence, timeout_s=0.5)
            snapshot = self.client.snapshot()
            local = self.client.local_position()
            evidence = self.client.estimator_evidence()
            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    f"[STATE] LAND altitude={self._format(snapshot.altitude_m, 'm')} "
                    f"local_z={self._format(local.z_m, 'm')} armed={snapshot.armed} "
                    f"landed={snapshot.landed} mode={snapshot.mode} "
                    f"hpos_valid={evidence.horizontal_position_valid} "
                    f"flow_active={evidence.optical_flow_active} "
                    f"gps_active={evidence.gps_active} failsafe={snapshot.failsafe} "
                    f"gcs_connected={snapshot.gcs_connected}"
                )
                next_report = now + 1.0
            altitude_low = snapshot.altitude_m is not None and snapshot.altitude_m < 0.15
            if (snapshot.landed is True and not snapshot.armed) or (
                altitude_low and (snapshot.landed is True or not snapshot.armed)
            ):
                self.logger.info(
                    f"[OK] Landing complete altitude={self._format(snapshot.altitude_m, 'm')} "
                    f"landed={snapshot.landed}"
                )
                return
        raise TimeoutError("40秒内未确认落地")

    def _disarm(self) -> None:
        self._set_state(OffboardState.DISARMING)
        snapshot = self.client.snapshot()
        if snapshot.altitude_m is not None and snapshot.altitude_m >= 0.15:
            raise RuntimeError("高度未低于0.15m，禁止DISARM")
        if snapshot.landed is False:
            raise RuntimeError("PX4尚未确认落地，禁止DISARM")
        if snapshot.armed:
            result = self.client.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            self.logger.info(f"[ACK] DISARM command=400 result={result}")
        final = self.client.wait_for(
            lambda state: not state.armed,
            timeout_s=10.0,
            description="armed=False",
        )
        self.logger.info(f"[OK] DISARM success armed={final.armed}")

    def _emergency_land_if_needed(self) -> None:
        try:
            snapshot = self.client.snapshot()
            if snapshot.armed and not self.land_requested:
                self.logger.warning("[SAFETY] Failure while armed; sending LAND")
                self.client.send_command_long(
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    [0.0, 0.0, 0.0, math.nan, math.nan, math.nan, 0.0],
                    timeout_s=5.0,
                )
                self.land_requested = True
        except Exception as error:
            self.logger.error(f"[SAFETY] Emergency LAND failed: {error}")

    def _hover_target(self) -> tuple[float, float, float]:
        if self.origin is None:
            raise RuntimeError("尚未取得本地坐标原点")
        return (self.origin[0], self.origin[1], self.origin[2] - self.altitude_m)

    def _move_target(self) -> tuple[float, float, float]:
        hover = self._hover_target()
        return (hover[0] + self.move_north_m, hover[1], hover[2])

    def _check_stop(self) -> None:
        if self.stop_requested.is_set():
            raise RuntimeError("用户终止任务")

    def _require_healthy(self, snapshot: VehicleSnapshot, phase: str) -> None:
        if snapshot.roll_deg is not None:
            self.max_abs_roll_deg = max(self.max_abs_roll_deg, abs(snapshot.roll_deg))
        if snapshot.pitch_deg is not None:
            self.max_abs_pitch_deg = max(self.max_abs_pitch_deg, abs(snapshot.pitch_deg))
        if snapshot.failsafe and not self._failsafe_active:
            self.failsafe_count += 1
        self._failsafe_active = snapshot.failsafe
        with self.client._condition:
            sender_age = (
                None
                if self.client.last_gcs_heartbeat_sent_monotonic is None
                else time.monotonic() - self.client.last_gcs_heartbeat_sent_monotonic
            )
            sender_ok = (
                sender_age is not None
                and sender_age < 2.5
                and self.client._sender_error is None
            )
            px4_gcs_lost = self.client.px4_gcs_lost
        gcs_lost = px4_gcs_lost or not sender_ok
        if gcs_lost and not self._gcs_lost_active:
            self.gcs_lost_count += 1
        self._gcs_lost_active = gcs_lost
        if snapshot.failsafe:
            raise RuntimeError(f"{phase}阶段检测到PX4 failsafe")
        if px4_gcs_lost or not sender_ok:
            raise RuntimeError(f"{phase}阶段GCS连接状态异常")
        if (
            snapshot.roll_deg is not None and abs(snapshot.roll_deg) > 35.0
        ) or (
            snapshot.pitch_deg is not None and abs(snapshot.pitch_deg) > 35.0
        ):
            raise RuntimeError(f"{phase}阶段姿态倾角过大")

    def _require_offboard_healthy(
        self, snapshot: VehicleSnapshot, phase: str
    ) -> None:
        self._require_healthy(snapshot, phase)
        evidence = self.client.estimator_evidence()
        local = self.client.local_position()
        local_valid = (
            local.age_s is not None
            and local.age_s < 1.0
            and evidence.horizontal_position_valid
            and evidence.horizontal_velocity_valid
            and evidence.vertical_position_valid
            and evidence.vertical_velocity_valid
            and evidence.gps_active is False
            and evidence.good_flow_age_s is not None
            and evidence.good_flow_age_s < 1.0
            and evidence.fusion_age_s is not None
            and evidence.fusion_age_s < 2.0
        )
        if not local_valid and not self._local_invalid_active:
            self.local_position_invalid_count += 1
        self._local_invalid_active = not local_valid
        if not local_valid:
            raise RuntimeError(
                f"{phase}阶段无GPS光流Local Position失效：{evidence}"
            )
        if not snapshot.armed:
            raise RuntimeError(f"{phase}阶段意外解除武装")
        if snapshot.mode.upper() != "OFFBOARD":
            raise RuntimeError(f"{phase}阶段退出OFFBOARD，当前模式={snapshot.mode}")

    @staticmethod
    def _format(value: float | None, unit: str) -> str:
        return "unknown" if value is None else f"{value:.2f}{unit}"

    def _set_state(self, state: OffboardState) -> None:
        self.state = state
        self.logger.info(f"[MISSION] state={state.name}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="udp:127.0.0.1:14540")
    parser.add_argument("--profile", choices=("hover", "move"), default="hover")
    parser.add_argument("--altitude", type=float, default=1.0)
    parser.add_argument("--move-north", type=float, default=0.5)
    parser.add_argument("--hold-seconds", type=float, default=None)
    parser.add_argument("--setpoint-rate", type=float, default=10.0)
    parser.add_argument("--log-file", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if not 0.5 <= args.altitude <= 2.0:
        raise SystemExit("v6 --altitude必须在0.5到2.0m之间")
    if args.profile == "move" and not 0.1 <= abs(args.move_north) <= 1.0:
        raise SystemExit("v6 move配置的--move-north绝对值必须在0.1到1.0m之间")
    hold_seconds = args.hold_seconds
    if hold_seconds is None:
        hold_seconds = 10.0 if args.profile == "hover" else 5.0
    if not 1.0 <= hold_seconds <= 30.0:
        raise SystemExit("--hold-seconds必须在1到30秒之间")
    if not 5.0 <= args.setpoint_rate <= 50.0:
        raise SystemExit("--setpoint-rate必须在5到50Hz之间")

    default_log = (
        Path(__file__).resolve().parent
        / "logs"
        / f"mission_v6_optical_flow_{args.profile}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    logger = MissionLogger(args.log_file or default_log)
    controller = OpticalFlowControllerV6(
        args.connection,
        args.profile,
        args.altitude,
        args.move_north,
        hold_seconds,
        args.setpoint_rate,
        logger,
    )

    def handle_stop(_signum, _frame) -> None:
        logger.warning("[SAFETY] Interrupt received")
        controller.request_stop()

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    exit_code = 0
    try:
        controller.run()
    except KeyboardInterrupt:
        logger.error("[FAILED] Offboard mission interrupted")
        exit_code = 130
    except Exception as error:
        logger.error(f"[FAILED] {type(error).__name__}: {error}")
        exit_code = 1
    finally:
        logger.info(f"[INFO] Log saved to {logger.path}")
        logger.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
