#!/usr/bin/env python3
"""PX4 SITL full mission: arm, take off, hold, land, and disarm."""

from __future__ import annotations

import argparse
import logging
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from enum import Enum, auto
from pathlib import Path
from typing import Callable

from pymavlink import mavutil


class MissionState(Enum):
    IDLE = auto()
    CONNECTING = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOLDING = auto()
    LANDING = auto()
    DISARM = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass(frozen=True)
class VehicleSnapshot:
    armed: bool
    altitude_m: float | None
    altitude_amsl_m: float | None
    vertical_speed_m_s: float | None
    roll_deg: float | None
    pitch_deg: float | None
    yaw_deg: float | None
    mode: str
    failsafe: bool
    gcs_connected: bool
    landed: bool | None
    last_heartbeat_age_s: float | None
    gcs_heartbeats_sent: int


class MissionLogger:
    def __init__(self, log_path: Path) -> None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.path = log_path
        self.logger = logging.getLogger(f"mission_v4_{id(self)}")
        self.logger.setLevel(logging.INFO)
        self.logger.propagate = False

        formatter = logging.Formatter(
            fmt="[%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s",
            datefmt="%H:%M:%S",
        )
        console = logging.StreamHandler(sys.stdout)
        console.setFormatter(formatter)
        file_handler = logging.FileHandler(log_path, encoding="utf-8")
        file_handler.setFormatter(formatter)
        self.logger.addHandler(console)
        self.logger.addHandler(file_handler)

    def info(self, message: str) -> None:
        self.logger.info(message)

    def warning(self, message: str) -> None:
        self.logger.warning(message)

    def error(self, message: str) -> None:
        self.logger.error(message)

    def close(self) -> None:
        for handler in list(self.logger.handlers):
            handler.flush()
            handler.close()
            self.logger.removeHandler(handler)


class MavlinkClient:
    """A single MAVLink connection with exactly one receiving entry point."""

    ACCEPTED_RESULTS = {
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    }
    FAILSAFE_STATES = {
        mavutil.mavlink.MAV_STATE_CRITICAL,
        mavutil.mavlink.MAV_STATE_EMERGENCY,
        mavutil.mavlink.MAV_STATE_POWEROFF,
    }

    def __init__(self, connection_string: str) -> None:
        self.connection_string = connection_string
        self.connection = mavutil.mavlink_connection(
            connection_string,
            source_system=255,
            source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        )

        self.target_system = 0
        self.target_component = 0
        self.armed = False
        self.altitude_m: float | None = None
        self.altitude_amsl_m: float | None = None
        self.vertical_speed_m_s: float | None = None
        self.roll_deg: float | None = None
        self.pitch_deg: float | None = None
        self.yaw_deg: float | None = None
        self.mode = "UNKNOWN"
        self.failsafe = False
        self.landed: bool | None = None
        self.last_heartbeat_monotonic: float | None = None
        self.last_gcs_heartbeat_sent_monotonic: float | None = None
        self.gcs_heartbeats_sent = 0

        self._condition = threading.Condition()
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._receiver_error: Exception | None = None
        self._sender_error: Exception | None = None
        self._update_sequence = 0
        self._ack_sequence = 0
        self._acks: dict[int, tuple[int, int]] = {}
        self._receiver_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None

    def start(self) -> None:
        self._receiver_thread = threading.Thread(
            target=self._receive_loop,
            name="mavlink-receiver",
            daemon=True,
        )
        self._heartbeat_thread = threading.Thread(
            target=self._gcs_heartbeat_loop,
            name="gcs-heartbeat-sender",
            daemon=True,
        )
        self._receiver_thread.start()
        self._heartbeat_thread.start()

    def close(self) -> None:
        self._stop_event.set()
        with self._condition:
            self._condition.notify_all()
        if self._receiver_thread is not None:
            self._receiver_thread.join(timeout=2.0)
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=2.0)
        self.connection.close()

    def snapshot(self) -> VehicleSnapshot:
        with self._condition:
            self._raise_thread_errors()
            return self._snapshot_unlocked()

    def wait_for(
        self,
        predicate: Callable[[VehicleSnapshot], bool],
        timeout_s: float,
        description: str,
    ) -> VehicleSnapshot:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                self._raise_thread_errors()
                snapshot = self._snapshot_unlocked()
                if predicate(snapshot):
                    return snapshot
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"等待{description}超时；当前状态：{snapshot}")
                self._condition.wait(timeout=min(remaining, 1.0))

    def wait_for_update(self, previous_sequence: int, timeout_s: float) -> int:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._update_sequence <= previous_sequence:
                self._raise_thread_errors()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return self._update_sequence
                self._condition.wait(timeout=remaining)
            return self._update_sequence

    def update_sequence(self) -> int:
        with self._condition:
            return self._update_sequence

    def send_command_long(
        self,
        command: int,
        params: list[float],
        timeout_s: float = 5.0,
    ) -> int:
        if len(params) != 7:
            raise ValueError("COMMAND_LONG 必须包含7个参数")
        if self.target_system == 0:
            raise RuntimeError("尚未取得 PX4 目标系统ID")

        with self._condition:
            previous_ack_sequence = self._ack_sequence

        with self._send_lock:
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                command,
                0,
                *params,
            )

        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                self._raise_thread_errors()
                acknowledgement = self._acks.get(command)
                if acknowledgement is not None and acknowledgement[0] > previous_ack_sequence:
                    result = acknowledgement[1]
                    if result not in self.ACCEPTED_RESULTS:
                        raise RuntimeError(
                            f"PX4拒绝命令 command={command}, MAV_RESULT={result}"
                        )
                    return result
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"等待 command={command} 的 COMMAND_ACK 超时")
                self._condition.wait(timeout=remaining)

    def _receive_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                # This receiver thread is the program's only MAVLink read path.
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
                        self.failsafe = message.system_status in self.FAILSAFE_STATES
                        self.last_heartbeat_monotonic = now

                    elif message_type == "GLOBAL_POSITION_INT":
                        self.altitude_m = message.relative_alt / 1000.0
                        self.altitude_amsl_m = message.alt / 1000.0
                        self.vertical_speed_m_s = message.vz / 100.0

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

    def _gcs_heartbeat_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                with self._send_lock:
                    self.connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                with self._condition:
                    self.last_gcs_heartbeat_sent_monotonic = time.monotonic()
                    if self.target_system != 0:
                        self.gcs_heartbeats_sent += 1
                    self._condition.notify_all()
                self._stop_event.wait(1.0)
        except Exception as error:
            if not self._stop_event.is_set():
                with self._condition:
                    self._sender_error = error
                    self._condition.notify_all()

    def _snapshot_unlocked(self) -> VehicleSnapshot:
        now = time.monotonic()
        heartbeat_age = None
        if self.last_heartbeat_monotonic is not None:
            heartbeat_age = now - self.last_heartbeat_monotonic
        gcs_send_age = None
        if self.last_gcs_heartbeat_sent_monotonic is not None:
            gcs_send_age = now - self.last_gcs_heartbeat_sent_monotonic
        gcs_connected = (
            heartbeat_age is not None
            and heartbeat_age < 3.0
            and gcs_send_age is not None
            and gcs_send_age < 2.5
            and self._sender_error is None
        )
        return VehicleSnapshot(
            armed=self.armed,
            altitude_m=self.altitude_m,
            altitude_amsl_m=self.altitude_amsl_m,
            vertical_speed_m_s=self.vertical_speed_m_s,
            roll_deg=self.roll_deg,
            pitch_deg=self.pitch_deg,
            yaw_deg=self.yaw_deg,
            mode=self.mode,
            failsafe=self.failsafe,
            gcs_connected=gcs_connected,
            landed=self.landed,
            last_heartbeat_age_s=heartbeat_age,
            gcs_heartbeats_sent=self.gcs_heartbeats_sent,
        )

    def _raise_thread_errors(self) -> None:
        if self._receiver_error is not None:
            raise RuntimeError("MAVLink接收线程异常") from self._receiver_error
        if self._sender_error is not None:
            raise RuntimeError("GCS HEARTBEAT发送线程异常") from self._sender_error


class MissionControllerV4:
    def __init__(
        self,
        connection_string: str,
        target_altitude_m: float,
        hold_seconds: float,
        logger: MissionLogger,
    ) -> None:
        self.client = MavlinkClient(connection_string)
        self.target_altitude_m = target_altitude_m
        self.hold_seconds = hold_seconds
        self.logger = logger
        self.mission_state = MissionState.IDLE
        self.stop_requested = threading.Event()
        self.land_requested = False

    def run(self) -> None:
        self.client.start()
        try:
            self._connect()
            self._arm()
            self._takeoff()
            self._hold()
            self._land()
            self._disarm()
            self._set_state(MissionState.COMPLETE)
            self.logger.info("[MISSION] Mission complete")
        except Exception:
            self._set_state(MissionState.FAILED)
            self._emergency_land_if_needed()
            raise
        finally:
            self.client.close()

    def request_stop(self) -> None:
        self.stop_requested.set()

    def _connect(self) -> None:
        self._set_state(MissionState.CONNECTING)
        self.logger.info(f"[INFO] Connecting PX4 through {self.client.connection_string}")
        self.client.wait_for(
            lambda state: state.last_heartbeat_age_s is not None,
            timeout_s=15.0,
            description="PX4 HEARTBEAT",
        )
        ready = self.client.wait_for(
            lambda state: (
                state.gcs_connected
                and state.gcs_heartbeats_sent >= 3
                and state.altitude_m is not None
                and state.altitude_amsl_m is not None
                and state.roll_deg is not None
                and state.pitch_deg is not None
            ),
            timeout_s=10.0,
            description="GCS连接、位置、姿态及至少3个GCS HEARTBEAT",
        )
        self.logger.info(
            f"[OK] PX4 connected system={self.client.target_system} "
            f"component={self.client.target_component} mode={ready.mode}"
        )
        self.logger.info(
            f"[STATE] armed={ready.armed} failsafe={ready.failsafe} "
            f"gcs_connected={ready.gcs_connected}"
        )
        if ready.armed:
            raise RuntimeError("任务开始前飞行器必须处于 Disarmed")
        if ready.failsafe:
            raise RuntimeError("任务开始前 PX4 已处于 failsafe")

    def _arm(self) -> None:
        self._set_state(MissionState.ARMING)
        self.logger.info("[MISSION] ARMING")
        result = self.client.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.logger.info(f"[ACK] ARM command=400 result={result}")
        armed = self.client.wait_for(
            lambda state: state.armed,
            timeout_s=10.0,
            description="armed=True",
        )
        self._require_healthy(armed, "ARM")
        self.logger.info("[OK] ARM success")

    def _takeoff(self) -> None:
        self._set_state(MissionState.TAKEOFF)
        before_takeoff = self.client.snapshot()
        if before_takeoff.altitude_m is None or before_takeoff.altitude_amsl_m is None:
            raise RuntimeError("缺少起飞前高度数据")
        ground_amsl_m = before_takeoff.altitude_amsl_m - before_takeoff.altitude_m
        target_amsl_m = ground_amsl_m + self.target_altitude_m
        self.logger.info(f"[MISSION] TAKEOFF {self.target_altitude_m:.2f}m")
        result = self.client.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [0.0, 0.0, 0.0, math.nan, math.nan, math.nan, target_amsl_m],
        )
        self.logger.info(f"[ACK] TAKEOFF command=22 result={result}")

        deadline = time.monotonic() + 30.0
        stable_since: float | None = None
        next_report = 0.0
        sequence = self.client.update_sequence()
        while time.monotonic() < deadline:
            if self.stop_requested.is_set():
                raise RuntimeError("用户终止任务")
            sequence = self.client.wait_for_update(sequence, timeout_s=0.5)
            snapshot = self.client.snapshot()
            self._require_healthy(snapshot, "TAKEOFF")
            if not snapshot.armed:
                raise RuntimeError("起飞阶段意外解除武装")
            if snapshot.altitude_m is None:
                continue
            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    f"[STATE] Altitude={snapshot.altitude_m:.2f}m "
                    f"vz={self._format_value(snapshot.vertical_speed_m_s, 'm/s')} "
                    f"mode={snapshot.mode}"
                )
                next_report = now + 1.0
            altitude_ready = snapshot.altitude_m >= self.target_altitude_m - 0.20
            speed_ready = (
                snapshot.vertical_speed_m_s is not None
                and abs(snapshot.vertical_speed_m_s) <= 0.20
            )
            if altitude_ready and speed_ready:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= 1.0:
                    self.logger.info(
                        f"[OK] Takeoff success altitude={snapshot.altitude_m:.2f}m"
                    )
                    return
            else:
                stable_since = None
        raise TimeoutError("30秒内未达到2.5m目标高度范围并稳定")

    def _hold(self) -> None:
        self._set_state(MissionState.HOLDING)
        self.logger.info(f"[MISSION] HOLDING {self.hold_seconds:.1f}s")
        mode_result = self.client.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [
                float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                4.0,
                3.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
        self.logger.info(f"[ACK] AUTO.LOITER command=176 result={mode_result}")

        started = time.monotonic()
        next_report = 0.0
        sequence = self.client.update_sequence()
        while time.monotonic() - started < self.hold_seconds:
            if self.stop_requested.is_set():
                self.logger.warning("[SAFETY] Stop requested; entering LAND immediately")
                return
            sequence = self.client.wait_for_update(sequence, timeout_s=0.5)
            snapshot = self.client.snapshot()
            if snapshot.failsafe or not snapshot.gcs_connected:
                self.logger.warning(
                    f"[SAFETY] HOLD unhealthy: failsafe={snapshot.failsafe}, "
                    f"gcs_connected={snapshot.gcs_connected}; entering LAND"
                )
                return
            if not snapshot.armed:
                raise RuntimeError("悬停阶段意外解除武装")
            if self._tilt_excessive(snapshot):
                self.logger.warning("[SAFETY] Excessive attitude; entering LAND")
                return
            now = time.monotonic()
            if now >= next_report:
                elapsed = now - started
                self.logger.info(
                    f"[STATE] HOLD {elapsed:.1f}/{self.hold_seconds:.1f}s "
                    f"altitude={self._format_value(snapshot.altitude_m, 'm')} "
                    f"roll={self._format_value(snapshot.roll_deg, 'deg')} "
                    f"pitch={self._format_value(snapshot.pitch_deg, 'deg')} "
                    f"failsafe={snapshot.failsafe} gcs_connected={snapshot.gcs_connected}"
                )
                next_report = now + 1.0
        self.logger.info("[OK] Holding complete")

    def _land(self) -> None:
        self._set_state(MissionState.LANDING)
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
            if not snapshot.gcs_connected:
                self.logger.warning("[SAFETY] GCS heartbeat health degraded during LAND")
            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    f"[STATE] LAND altitude={self._format_value(snapshot.altitude_m, 'm')} "
                    f"armed={snapshot.armed} landed={snapshot.landed} mode={snapshot.mode}"
                )
                next_report = now + 1.0
            altitude_low = snapshot.altitude_m is not None and snapshot.altitude_m < 0.15
            landed_confirmed = snapshot.landed is True
            if altitude_low and (landed_confirmed or not snapshot.armed):
                self.logger.info(
                    f"[OK] Landing complete altitude={snapshot.altitude_m:.2f}m "
                    f"landed={snapshot.landed}"
                )
                return
        raise TimeoutError("40秒内未确认高度低于0.15m并落地")

    def _disarm(self) -> None:
        self._set_state(MissionState.DISARM)
        snapshot = self.client.snapshot()
        if snapshot.altitude_m is None or snapshot.altitude_m >= 0.15:
            raise RuntimeError("高度未低于0.15m，禁止发送DISARM")
        if snapshot.landed is False:
            raise RuntimeError("PX4尚未确认落地，禁止发送DISARM")

        if snapshot.armed:
            self.logger.info("[MISSION] DISARM")
            result = self.client.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            self.logger.info(f"[ACK] DISARM command=400 result={result}")
        else:
            self.logger.info("[MISSION] DISARM already completed automatically by PX4")

        final = self.client.wait_for(
            lambda state: (
                not state.armed
                and state.altitude_m is not None
                and state.altitude_m < 0.15
            ),
            timeout_s=10.0,
            description="armed=False且高度低于0.15m",
        )
        self.logger.info(
            f"[OK] DISARM success armed={final.armed} altitude={final.altitude_m:.2f}m"
        )

    def _emergency_land_if_needed(self) -> None:
        try:
            snapshot = self.client.snapshot()
            if snapshot.armed and not self.land_requested:
                self.logger.warning("[SAFETY] Mission failed while armed; sending LAND")
                self.client.send_command_long(
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    [0.0, 0.0, 0.0, math.nan, math.nan, math.nan, 0.0],
                    timeout_s=5.0,
                )
                self.land_requested = True
        except Exception as error:
            self.logger.error(f"[SAFETY] Emergency LAND failed: {error}")

    def _require_healthy(self, snapshot: VehicleSnapshot, phase: str) -> None:
        if snapshot.failsafe:
            raise RuntimeError(f"{phase}阶段检测到PX4 failsafe")
        if not snapshot.gcs_connected:
            raise RuntimeError(f"{phase}阶段GCS连接状态异常")
        if self._tilt_excessive(snapshot):
            raise RuntimeError(f"{phase}阶段姿态倾角过大")

    @staticmethod
    def _tilt_excessive(snapshot: VehicleSnapshot) -> bool:
        return (
            snapshot.roll_deg is not None
            and abs(snapshot.roll_deg) > 35.0
        ) or (
            snapshot.pitch_deg is not None
            and abs(snapshot.pitch_deg) > 35.0
        )

    @staticmethod
    def _format_value(value: float | None, unit: str) -> str:
        return "unknown" if value is None else f"{value:.2f}{unit}"

    def _set_state(self, state: MissionState) -> None:
        self.mission_state = state
        self.logger.info(f"[MISSION] state={state.name}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connection",
        default="udpin:0.0.0.0:14540",
        help="pymavlink连接串，默认监听PX4 SITL Onboard端口14540",
    )
    parser.add_argument("--altitude", type=float, default=2.5)
    parser.add_argument("--hold-seconds", type=float, default=10.0)
    parser.add_argument(
        "--log-file",
        type=Path,
        default=None,
        help="任务日志路径；默认写入python_control/logs",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.altitude <= 0.5:
        raise SystemExit("--altitude必须大于0.5m")
    if args.hold_seconds <= 0:
        raise SystemExit("--hold-seconds必须大于0")

    default_log = (
        Path(__file__).resolve().parent
        / "logs"
        / f"mission_v4_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    mission_logger = MissionLogger(args.log_file or default_log)
    controller = MissionControllerV4(
        args.connection,
        args.altitude,
        args.hold_seconds,
        mission_logger,
    )

    def handle_stop(_signum, _frame) -> None:
        mission_logger.warning("[SAFETY] Interrupt received")
        controller.request_stop()

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    exit_code = 0
    try:
        controller.run()
    except KeyboardInterrupt:
        mission_logger.error("[FAILED] Mission interrupted")
        exit_code = 130
    except Exception as error:
        mission_logger.error(f"[FAILED] {type(error).__name__}: {error}")
        exit_code = 1
    finally:
        mission_logger.info(f"[INFO] Log saved to {mission_logger.path}")
        mission_logger.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
