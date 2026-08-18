#!/usr/bin/env python3
"""PX4 SITL Offboard mission: arm, hover, move, land, and disarm."""

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

from mission_controller_v4 import MavlinkClient, MissionLogger, VehicleSnapshot


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
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


class OffboardControllerV5:
    def __init__(
        self,
        connection_string: str,
        altitude_m: float,
        move_north_m: float,
        hold_seconds: float,
        setpoint_rate_hz: float,
        logger: MissionLogger,
    ) -> None:
        self.client = OffboardMavlinkClient(connection_string, setpoint_rate_hz)
        self.altitude_m = altitude_m
        self.move_north_m = move_north_m
        self.hold_seconds = hold_seconds
        self.setpoint_rate_hz = setpoint_rate_hz
        self.logger = logger
        self.state = OffboardState.IDLE
        self.stop_requested = threading.Event()
        self.land_requested = False
        self.origin: tuple[float, float, float] | None = None

    def run(self) -> None:
        self.client.start()
        try:
            self._connect()
            self._prewarm_setpoints()
            self._arm()
            self._enter_offboard()
            self._reach_target("TAKEOFF", self._hover_target(), OffboardState.TAKING_OFF)
            self._hold("HOVER", OffboardState.HOVERING)
            self._reach_target("MOVE", self._move_target(), OffboardState.MOVING)
            self._hold("TARGET", OffboardState.HOLDING_TARGET)
            self._land()
            self._disarm()
            self._set_state(OffboardState.COMPLETE)
            self.logger.info("[MISSION] Offboard mission complete")
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
        result = self.client.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
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
                    f"speed={self._format(speed, 'm/s')} mode={snapshot.mode}"
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
            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    f"[STATE] {name} hold={now-started:.1f}/{self.hold_seconds:.1f}s "
                    f"position=({self._format(local.x_m, 'm')},"
                    f"{self._format(local.y_m, 'm')},{self._format(local.z_m, 'm')}) "
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
            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    f"[STATE] LAND altitude={self._format(snapshot.altitude_m, 'm')} "
                    f"local_z={self._format(local.z_m, 'm')} armed={snapshot.armed} "
                    f"landed={snapshot.landed} mode={snapshot.mode}"
                )
                next_report = now + 1.0
            altitude_low = snapshot.altitude_m is not None and snapshot.altitude_m < 0.15
            if altitude_low and (snapshot.landed is True or not snapshot.armed):
                self.logger.info(
                    f"[OK] Landing complete altitude={snapshot.altitude_m:.2f}m "
                    f"landed={snapshot.landed}"
                )
                return
        raise TimeoutError("40秒内未确认落地")

    def _disarm(self) -> None:
        self._set_state(OffboardState.DISARMING)
        snapshot = self.client.snapshot()
        if snapshot.altitude_m is None or snapshot.altitude_m >= 0.15:
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
        if snapshot.failsafe:
            raise RuntimeError(f"{phase}阶段检测到PX4 failsafe")
        if not snapshot.gcs_connected:
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
    parser.add_argument("--altitude", type=float, default=2.0)
    parser.add_argument("--move-north", type=float, default=3.0)
    parser.add_argument("--hold-seconds", type=float, default=5.0)
    parser.add_argument("--setpoint-rate", type=float, default=10.0)
    parser.add_argument("--log-file", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if not 1.0 <= args.altitude <= 10.0:
        raise SystemExit("--altitude必须在1到10m之间")
    if not 0.5 <= abs(args.move_north) <= 20.0:
        raise SystemExit("--move-north绝对值必须在0.5到20m之间")
    if not 1.0 <= args.hold_seconds <= 60.0:
        raise SystemExit("--hold-seconds必须在1到60秒之间")
    if not 5.0 <= args.setpoint_rate <= 50.0:
        raise SystemExit("--setpoint-rate必须在5到50Hz之间")

    default_log = (
        Path(__file__).resolve().parent
        / "logs"
        / f"mission_v5_offboard_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    logger = MissionLogger(args.log_file or default_log)
    controller = OffboardControllerV5(
        args.connection,
        args.altitude,
        args.move_north,
        args.hold_seconds,
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
