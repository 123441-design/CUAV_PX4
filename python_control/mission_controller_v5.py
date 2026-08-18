#!/usr/bin/env python3
"""PX4 SITL AUTO.MISSION: upload, arm, execute nearby mission, land, disarm."""

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


class MissionState(Enum):
    IDLE = auto()
    CONNECTING = auto()
    BUILDING_MISSION = auto()
    CLEARING_MISSION = auto()
    UPLOADING_MISSION = auto()
    VERIFYING_MISSION = auto()
    ARMING = auto()
    STARTING_AUTO_MISSION = auto()
    EXECUTING_MISSION = auto()
    LANDING = auto()
    DISARM = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass(frozen=True)
class MissionItem:
    seq: int
    command: int
    latitude_deg: float
    longitude_deg: float
    relative_altitude_m: float
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = math.nan
    current: int = 0
    autocontinue: int = 1
    frame: int = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT


@dataclass(frozen=True)
class MissionProgress:
    current_seq: int | None
    reached_sequences: frozenset[int]
    latitude_deg: float | None
    longitude_deg: float | None


class MissionMavlinkClient(MavlinkClient):
    """v4 client extended with Mission Protocol message distribution."""

    def __init__(self, connection_string: str) -> None:
        super().__init__(connection_string)
        self.latitude_deg: float | None = None
        self.longitude_deg: float | None = None
        self.mission_current_seq: int | None = None
        self.mission_reached_sequences: set[int] = set()

        self._mission_request_event_sequence = 0
        self._mission_requested_seq: int | None = None
        self._mission_request_was_int = False
        self._mission_ack_event_sequence = 0
        self._mission_ack_type: int | None = None
        self._mission_count_event_sequence = 0
        self._mission_count: int | None = None

    def clear_mission(self, timeout_s: float = 8.0) -> int:
        with self._condition:
            previous_ack = self._mission_ack_event_sequence
        with self._send_lock:
            self.connection.mav.mission_clear_all_send(
                self.target_system,
                self.target_component,
            )
        return self._wait_mission_ack(previous_ack, timeout_s, "MISSION_CLEAR_ALL")

    def upload_mission(
        self,
        items: list[MissionItem],
        timeout_s: float = 20.0,
    ) -> int:
        if not items:
            raise ValueError("Mission不能为空")
        if [item.seq for item in items] != list(range(len(items))):
            raise ValueError("Mission seq必须从0连续递增")

        with self._condition:
            request_cursor = self._mission_request_event_sequence
            ack_cursor = self._mission_ack_event_sequence

        with self._send_lock:
            self.connection.mav.mission_count_send(
                self.target_system,
                self.target_component,
                len(items),
            )

        deadline = time.monotonic() + timeout_s
        sent_sequences: set[int] = set()
        while time.monotonic() < deadline:
            with self._condition:
                self._raise_thread_errors()
                while (
                    self._mission_request_event_sequence <= request_cursor
                    and self._mission_ack_event_sequence <= ack_cursor
                ):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError("Mission上传握手超时")
                    self._condition.wait(timeout=remaining)

                if self._mission_ack_event_sequence > ack_cursor:
                    ack_cursor = self._mission_ack_event_sequence
                    ack_type = self._mission_ack_type
                    if ack_type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        raise RuntimeError(f"PX4拒绝Mission，MISSION_ACK type={ack_type}")
                    if len(sent_sequences) != len(items):
                        raise RuntimeError(
                            f"PX4提前接受Mission，仅发送{len(sent_sequences)}/{len(items)}项"
                        )
                    return int(ack_type)

                request_cursor = self._mission_request_event_sequence
                requested_seq = self._mission_requested_seq
                request_was_int = self._mission_request_was_int

            if requested_seq is None or not 0 <= requested_seq < len(items):
                raise RuntimeError(f"PX4请求了无效Mission seq={requested_seq}")
            if not request_was_int:
                raise RuntimeError("PX4使用MISSION_REQUEST而非MISSION_REQUEST_INT")

            self._send_mission_item_int(items[requested_seq])
            sent_sequences.add(requested_seq)

        raise TimeoutError("Mission上传未收到MISSION_ACK")

    def readback_mission_count(self, timeout_s: float = 8.0) -> int:
        with self._condition:
            previous_count_event = self._mission_count_event_sequence
        with self._send_lock:
            self.connection.mav.mission_request_list_send(
                self.target_system,
                self.target_component,
            )

        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._mission_count_event_sequence <= previous_count_event:
                self._raise_thread_errors()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("回读MISSION_COUNT超时")
                self._condition.wait(timeout=remaining)
            if self._mission_count is None:
                raise RuntimeError("MISSION_COUNT缺少count")
            return self._mission_count

    def set_current_mission_item(self, seq: int) -> None:
        with self._send_lock:
            self.connection.mav.mission_set_current_send(
                self.target_system,
                self.target_component,
                seq,
            )

    def wait_current_mission_item(self, seq: int, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self.mission_current_seq != seq:
                self._raise_thread_errors()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"等待MISSION_CURRENT seq={seq}超时")
                self._condition.wait(timeout=remaining)

    def mission_progress(self) -> MissionProgress:
        with self._condition:
            self._raise_thread_errors()
            return MissionProgress(
                current_seq=self.mission_current_seq,
                reached_sequences=frozenset(self.mission_reached_sequences),
                latitude_deg=self.latitude_deg,
                longitude_deg=self.longitude_deg,
            )

    def reset_mission_progress(self) -> None:
        with self._condition:
            self.mission_current_seq = None
            self.mission_reached_sequences.clear()

    def _send_mission_item_int(self, item: MissionItem) -> None:
        latitude_int = int(round(item.latitude_deg * 1e7))
        longitude_int = int(round(item.longitude_deg * 1e7))
        with self._send_lock:
            self.connection.mav.mission_item_int_send(
                self.target_system,
                self.target_component,
                item.seq,
                item.frame,
                item.command,
                item.current,
                item.autocontinue,
                item.param1,
                item.param2,
                item.param3,
                item.param4,
                latitude_int,
                longitude_int,
                item.relative_altitude_m,
            )

    def _wait_mission_ack(
        self,
        previous_event: int,
        timeout_s: float,
        operation: str,
    ) -> int:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._mission_ack_event_sequence <= previous_event:
                self._raise_thread_errors()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"{operation}等待MISSION_ACK超时")
                self._condition.wait(timeout=remaining)
            ack_type = self._mission_ack_type
            if ack_type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"{operation}失败，MISSION_ACK type={ack_type}")
            return int(ack_type)

    def _receive_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                # This receiver thread is v5's only MAVLink read path.
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
                        self.latitude_deg = message.lat / 1e7
                        self.longitude_deg = message.lon / 1e7
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

                    elif message_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                        self._mission_request_event_sequence += 1
                        self._mission_requested_seq = int(message.seq)
                        self._mission_request_was_int = message_type == "MISSION_REQUEST_INT"

                    elif message_type == "MISSION_ACK":
                        self._mission_ack_event_sequence += 1
                        self._mission_ack_type = int(message.type)

                    elif message_type == "MISSION_COUNT":
                        self._mission_count_event_sequence += 1
                        self._mission_count = int(message.count)

                    elif message_type == "MISSION_CURRENT":
                        self.mission_current_seq = int(message.seq)

                    elif message_type == "MISSION_ITEM_REACHED":
                        self.mission_reached_sequences.add(int(message.seq))

                    self._update_sequence += 1
                    self._condition.notify_all()
        except Exception as error:
            with self._condition:
                self._receiver_error = error
                self._condition.notify_all()


class MissionControllerV5:
    EARTH_RADIUS_M = 6_378_137.0

    def __init__(
        self,
        connection_string: str,
        target_altitude_m: float,
        north_offset_m: float,
        loiter_seconds: float,
        logger: MissionLogger,
    ) -> None:
        self.client = MissionMavlinkClient(connection_string)
        self.target_altitude_m = target_altitude_m
        self.north_offset_m = north_offset_m
        self.loiter_seconds = loiter_seconds
        self.logger = logger
        self.mission_state = MissionState.IDLE
        self.stop_requested = threading.Event()
        self.land_requested = False
        self.items: list[MissionItem] = []

    def run(self) -> None:
        self.client.start()
        try:
            self._connect()
            self._build_mission()
            self._upload_and_verify_mission()
            self._arm()
            self._start_auto_mission()
            self._monitor_mission()
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
        ready = self.client.wait_for(
            lambda state: (
                state.gcs_connected
                and state.gcs_heartbeats_sent >= 3
                and state.altitude_m is not None
                and state.altitude_amsl_m is not None
                and state.roll_deg is not None
                and state.pitch_deg is not None
            ),
            timeout_s=20.0,
            description="PX4、GCS心跳、位置和姿态",
        )
        progress = self.client.mission_progress()
        if progress.latitude_deg is None or progress.longitude_deg is None:
            raise RuntimeError("没有有效的GLOBAL_POSITION_INT经纬度")
        self.logger.info(
            f"[OK] PX4 connected system={self.client.target_system} "
            f"component={self.client.target_component} mode={ready.mode}"
        )
        self.logger.info(
            f"[STATE] armed={ready.armed} failsafe={ready.failsafe} "
            f"gcs_connected={ready.gcs_connected} "
            f"lat={progress.latitude_deg:.7f} lon={progress.longitude_deg:.7f}"
        )
        if ready.armed:
            raise RuntimeError("任务开始前必须Disarmed")
        if ready.failsafe:
            raise RuntimeError("任务开始前PX4已处于failsafe")

    def _build_mission(self) -> None:
        self._set_state(MissionState.BUILDING_MISSION)
        progress = self.client.mission_progress()
        if progress.latitude_deg is None or progress.longitude_deg is None:
            raise RuntimeError("无法获取Home附近坐标")

        home_lat = progress.latitude_deg
        home_lon = progress.longitude_deg
        waypoint_lat = home_lat + math.degrees(
            self.north_offset_m / self.EARTH_RADIUS_M
        )
        waypoint_lon = home_lon

        self.items = [
            MissionItem(
                seq=0,
                command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                latitude_deg=home_lat,
                longitude_deg=home_lon,
                relative_altitude_m=self.target_altitude_m,
                current=1,
            ),
            MissionItem(
                seq=1,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                latitude_deg=waypoint_lat,
                longitude_deg=waypoint_lon,
                relative_altitude_m=self.target_altitude_m,
                param1=0.0,
                param2=1.0,
            ),
            MissionItem(
                seq=2,
                command=mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME,
                latitude_deg=waypoint_lat,
                longitude_deg=waypoint_lon,
                relative_altitude_m=self.target_altitude_m,
                param1=self.loiter_seconds,
                param2=0.0,
                param3=2.0,
                param4=0.0,
            ),
            MissionItem(
                seq=3,
                command=mavutil.mavlink.MAV_CMD_NAV_LAND,
                latitude_deg=waypoint_lat,
                longitude_deg=waypoint_lon,
                relative_altitude_m=0.0,
            ),
        ]
        self.logger.info(
            f"[MISSION] Built {len(self.items)} items: TAKEOFF -> WAYPOINT "
            f"north {self.north_offset_m:.1f}m -> LOITER {self.loiter_seconds:.1f}s -> LAND"
        )
        for item in self.items:
            self.logger.info(
                f"[MISSION_ITEM] seq={item.seq} command={item.command} "
                f"lat={item.latitude_deg:.7f} lon={item.longitude_deg:.7f} "
                f"relative_alt={item.relative_altitude_m:.1f}m"
            )

    def _upload_and_verify_mission(self) -> None:
        self._set_state(MissionState.CLEARING_MISSION)
        clear_ack = self.client.clear_mission()
        self.logger.info(f"[ACK] MISSION_CLEAR_ALL type={clear_ack}")

        self._set_state(MissionState.UPLOADING_MISSION)
        upload_ack = self.client.upload_mission(self.items)
        self.logger.info(f"[ACK] MISSION_UPLOAD type={upload_ack}")

        self._set_state(MissionState.VERIFYING_MISSION)
        readback_count = self.client.readback_mission_count()
        self.logger.info(
            f"[VERIFY] PX4 MISSION_COUNT={readback_count}, expected={len(self.items)}"
        )
        if readback_count != len(self.items):
            raise RuntimeError(
                f"PX4回读Mission数量不一致：{readback_count}!={len(self.items)}"
            )

        self.client.reset_mission_progress()
        self.client.set_current_mission_item(0)
        self.client.wait_current_mission_item(0, timeout_s=8.0)
        self.logger.info("[OK] Mission upload and readback verification complete")

    def _arm(self) -> None:
        self._set_state(MissionState.ARMING)
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

    def _start_auto_mission(self) -> None:
        self._set_state(MissionState.STARTING_AUTO_MISSION)
        result = self.client.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [
                float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                4.0,
                4.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
        )
        self.logger.info(f"[ACK] AUTO.MISSION command=176 result={result}")
        mode = self.client.wait_for(
            lambda state: "MISSION" in state.mode.upper(),
            timeout_s=10.0,
            description="AUTO.MISSION模式",
        )
        self._require_healthy(mode, "AUTO.MISSION")
        self.logger.info(f"[OK] AUTO.MISSION active mode={mode.mode}")

    def _monitor_mission(self) -> None:
        self._set_state(MissionState.EXECUTING_MISSION)
        deadline = time.monotonic() + 120.0
        next_report = 0.0
        sequence = self.client.update_sequence()
        last_current: int | None = None
        reported_reached: set[int] = set()
        land_sequence = self.items[-1].seq
        airborne_seen = False
        land_phase_seen = False

        while time.monotonic() < deadline:
            if self.stop_requested.is_set():
                raise RuntimeError("用户终止任务")
            sequence = self.client.wait_for_update(sequence, timeout_s=0.5)
            snapshot = self.client.snapshot()
            progress = self.client.mission_progress()
            self._require_healthy(snapshot, "MISSION")

            if progress.current_seq != last_current:
                last_current = progress.current_seq
                self.logger.info(f"[MISSION_CURRENT] seq={last_current}")
                if last_current == land_sequence:
                    land_phase_seen = True
                    self._set_state(MissionState.LANDING)
                    self.land_requested = True

            if snapshot.landed is False and (
                snapshot.altitude_m is None or snapshot.altitude_m > 0.30
            ):
                airborne_seen = True

            new_reached = set(progress.reached_sequences) - reported_reached
            for reached_seq in sorted(new_reached):
                self.logger.info(f"[MISSION_ITEM_REACHED] seq={reached_seq}")
            reported_reached.update(new_reached)

            now = time.monotonic()
            if now >= next_report:
                self.logger.info(
                    f"[STATE] mission_seq={progress.current_seq} "
                    f"altitude={self._format_value(snapshot.altitude_m, 'm')} "
                    f"mode={snapshot.mode} armed={snapshot.armed} "
                    f"landed={snapshot.landed} failsafe={snapshot.failsafe} "
                    f"gcs_connected={snapshot.gcs_connected}"
                )
                next_report = now + 1.0

            altitude_low = snapshot.altitude_m is not None and snapshot.altitude_m < 0.15
            if (
                airborne_seen
                and land_phase_seen
                and altitude_low
                and snapshot.landed is True
            ):
                self.land_requested = True
                self.logger.info(
                    f"[OK] Mission LAND complete altitude={snapshot.altitude_m:.2f}m"
                )
                return

            if not snapshot.armed and not airborne_seen:
                raise RuntimeError("Mission在实际离地前意外解除武装")

        raise TimeoutError("120秒内Mission未完成LAND")

    def _disarm(self) -> None:
        self._set_state(MissionState.DISARM)
        snapshot = self.client.snapshot()
        if snapshot.altitude_m is None or snapshot.altitude_m >= 0.15:
            raise RuntimeError("高度未低于0.15m，禁止DISARM")
        if snapshot.landed is not True:
            raise RuntimeError("PX4尚未确认landed=True，禁止DISARM")

        if snapshot.armed:
            result = self.client.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            self.logger.info(f"[ACK] DISARM command=400 result={result}")
        else:
            self.logger.info("[MISSION] PX4 already auto-disarmed")

        final = self.client.wait_for(
            lambda state: not state.armed,
            timeout_s=10.0,
            description="armed=False",
        )
        self.logger.info(
            f"[OK] DISARM success armed={final.armed} "
            f"altitude={self._format_value(final.altitude_m, 'm')}"
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
            raise RuntimeError(f"{phase}阶段GCS连接异常")
        if (
            snapshot.roll_deg is not None and abs(snapshot.roll_deg) > 35.0
        ) or (
            snapshot.pitch_deg is not None and abs(snapshot.pitch_deg) > 35.0
        ):
            raise RuntimeError(f"{phase}阶段姿态倾角过大")

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
    )
    parser.add_argument("--altitude", type=float, default=3.0)
    parser.add_argument("--north-offset", type=float, default=5.0)
    parser.add_argument("--loiter-seconds", type=float, default=5.0)
    parser.add_argument("--log-file", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if not 2.5 <= args.altitude <= 10.0:
        raise SystemExit("--altitude必须在2.5到10m之间")
    if not 1.0 <= args.north_offset <= 20.0:
        raise SystemExit("--north-offset必须在1到20m之间")
    if not 1.0 <= args.loiter_seconds <= 60.0:
        raise SystemExit("--loiter-seconds必须在1到60秒之间")

    default_log = (
        Path(__file__).resolve().parent
        / "logs"
        / f"mission_v5_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    )
    logger = MissionLogger(args.log_file or default_log)
    controller = MissionControllerV5(
        args.connection,
        args.altitude,
        args.north_offset,
        args.loiter_seconds,
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
    except Exception as error:
        logger.error(f"[FAILED] {type(error).__name__}: {error}")
        exit_code = 1
    finally:
        logger.info(f"[INFO] Log saved to {logger.path}")
        logger.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
