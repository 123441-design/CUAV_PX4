#!/usr/bin/env python3
"""PX4 SITL MAVLink state-machine controller: ARM, take off, then hold."""

from __future__ import annotations

import argparse
import math
import signal
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto

from pymavlink import mavutil


class MissionState(Enum):
    CONNECTING = auto()
    WAITING_FOR_GCS_READY = auto()
    REQUESTING_ARM = auto()
    WAITING_FOR_ARM = auto()
    REQUESTING_TAKEOFF = auto()
    CLIMBING = auto()
    HOLDING = auto()
    COMPLETE = auto()
    FAILED = auto()


@dataclass(frozen=True)
class VehicleSnapshot:
    armed: bool
    altitude_m: float | None
    altitude_amsl_m: float | None
    vertical_speed_m_s: float | None
    mode: str
    last_heartbeat_age_s: float | None
    gcs_heartbeats_sent: int


class MavlinkClient:
    """One MAVLink connection with exactly one receiving thread."""

    ACCEPTED_RESULTS = {
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
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
        self.mode = "UNKNOWN"
        self.last_heartbeat_monotonic: float | None = None
        self.gcs_heartbeats_sent = 0

        self._condition = threading.Condition()
        self._send_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._receiver_error: Exception | None = None
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

    def wait_connected(self, timeout_s: float) -> None:
        self.wait_for(
            lambda snapshot: snapshot.last_heartbeat_age_s is not None,
            timeout_s,
            "PX4 HEARTBEAT",
        )

    def wait_for(
        self,
        predicate,
        timeout_s: float,
        description: str,
    ) -> VehicleSnapshot:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                self._raise_receiver_error()
                snapshot = self._snapshot_unlocked()
                if predicate(snapshot):
                    return snapshot
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"等待{description}超时；当前状态：{snapshot}")
                self._condition.wait(timeout=min(remaining, 1.0))

    def snapshot(self) -> VehicleSnapshot:
        with self._condition:
            self._raise_receiver_error()
            return self._snapshot_unlocked()

    def send_command_long(
        self,
        command: int,
        params: list[float],
        timeout_s: float = 5.0,
    ) -> int:
        if len(params) != 7:
            raise ValueError("COMMAND_LONG 必须包含7个参数")
        if self.target_system == 0:
            raise RuntimeError("尚未取得PX4目标系统ID")

        with self._condition:
            previous_sequence = self._ack_sequence

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
                self._raise_receiver_error()
                ack = self._acks.get(command)
                if ack is not None and ack[0] > previous_sequence:
                    result = ack[1]
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
                # This receiver thread is the only code path that reads MAVLink.
                message = self.connection.recv_match(blocking=True, timeout=0.5)
                if message is None or message.get_type() == "BAD_DATA":
                    continue

                message_type = message.get_type()
                now = time.monotonic()
                with self._condition:
                    if message_type == "HEARTBEAT":
                        if message.get_srcSystem() != 255:
                            self.target_system = message.get_srcSystem()
                            self.target_component = message.get_srcComponent()
                            self.armed = bool(
                                message.base_mode
                                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                            )
                            self.mode = mavutil.mode_string_v10(message)
                            self.last_heartbeat_monotonic = now

                    elif message_type == "GLOBAL_POSITION_INT":
                        self.altitude_m = message.relative_alt / 1000.0
                        self.altitude_amsl_m = message.alt / 1000.0
                        self.vertical_speed_m_s = message.vz / 100.0

                    elif message_type == "COMMAND_ACK":
                        self._ack_sequence += 1
                        self._acks[message.command] = (
                            self._ack_sequence,
                            message.result,
                        )

                    self._condition.notify_all()
        except Exception as error:
            with self._condition:
                self._receiver_error = error
                self._condition.notify_all()

    def _gcs_heartbeat_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                with self._send_lock:
                    self.connection.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        mavutil.mavlink.MAV_STATE_ACTIVE,
                    )
                if self.target_system != 0:
                    with self._condition:
                        self.gcs_heartbeats_sent += 1
                        self._condition.notify_all()
            except Exception:
                if not self._stop_event.is_set():
                    raise
            self._stop_event.wait(1.0)

    def _snapshot_unlocked(self) -> VehicleSnapshot:
        age = None
        if self.last_heartbeat_monotonic is not None:
            age = time.monotonic() - self.last_heartbeat_monotonic
        return VehicleSnapshot(
            armed=self.armed,
            altitude_m=self.altitude_m,
            altitude_amsl_m=self.altitude_amsl_m,
            vertical_speed_m_s=self.vertical_speed_m_s,
            mode=self.mode,
            last_heartbeat_age_s=age,
            gcs_heartbeats_sent=self.gcs_heartbeats_sent,
        )

    def _raise_receiver_error(self) -> None:
        if self._receiver_error is not None:
            raise RuntimeError("MAVLink接收线程异常") from self._receiver_error


class MissionControllerV3:
    def __init__(
        self,
        connection_string: str,
        target_altitude_m: float,
        hold_seconds: float,
    ) -> None:
        self.client = MavlinkClient(connection_string)
        self.target_altitude_m = target_altitude_m
        self.hold_seconds = hold_seconds
        self.state = MissionState.CONNECTING
        self.stop_requested = threading.Event()

    def run(self) -> None:
        self.client.start()
        try:
            self._set_state(MissionState.CONNECTING)
            self.client.wait_connected(timeout_s=15.0)
            snapshot = self.client.snapshot()
            print(
                f"[连接成功] system={self.client.target_system}, "
                f"component={self.client.target_component}, mode={snapshot.mode}"
            )

            self._set_state(MissionState.WAITING_FOR_GCS_READY)
            self.client.wait_for(
                lambda state: (
                    state.last_heartbeat_age_s is not None
                    and state.last_heartbeat_age_s < 2.0
                    and state.altitude_m is not None
                    and state.altitude_amsl_m is not None
                    and state.gcs_heartbeats_sent >= 3
                ),
                timeout_s=10.0,
                description="有效位置状态和至少3个GCS心跳",
            )
            print("[GCS状态确认] 已连续发送至少3个GCS HEARTBEAT")

            self._set_state(MissionState.REQUESTING_ARM)
            result = self.client.send_command_long(
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            print(f"[ARM ACK] command=400, result={result}")

            self._set_state(MissionState.WAITING_FOR_ARM)
            self.client.wait_for(
                lambda state: state.armed,
                timeout_s=10.0,
                description="armed=True",
            )
            print("[ARM状态确认] armed=True")

            before_takeoff = self.client.snapshot()
            assert before_takeoff.altitude_m is not None
            assert before_takeoff.altitude_amsl_m is not None
            ground_amsl_m = (
                before_takeoff.altitude_amsl_m - before_takeoff.altitude_m
            )
            command_altitude_amsl_m = ground_amsl_m + self.target_altitude_m

            self._set_state(MissionState.REQUESTING_TAKEOFF)
            result = self.client.send_command_long(
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                [
                    0.0,
                    0.0,
                    0.0,
                    math.nan,
                    math.nan,
                    math.nan,
                    command_altitude_amsl_m,
                ],
            )
            print(
                f"[TAKEOFF ACK] command=22, result={result}, "
                f"目标相对高度={self.target_altitude_m:.2f}m"
            )

            self._set_state(MissionState.CLIMBING)
            self._wait_until_target_altitude(timeout_s=30.0)

            self._set_state(MissionState.HOLDING)
            self._hold()
            self._set_state(MissionState.COMPLETE)
        except Exception:
            self._set_state(MissionState.FAILED)
            raise
        finally:
            self.client.close()

    def request_stop(self) -> None:
        self.stop_requested.set()

    def _wait_until_target_altitude(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        stable_since: float | None = None
        next_report = 0.0

        while time.monotonic() < deadline:
            snapshot = self.client.wait_for(
                lambda state: state.altitude_m is not None,
                timeout_s=min(2.0, max(0.1, deadline - time.monotonic())),
                description="高度反馈",
            )
            now = time.monotonic()
            altitude = snapshot.altitude_m
            vertical_speed = snapshot.vertical_speed_m_s
            assert altitude is not None

            if now >= next_report:
                speed_text = (
                    "unknown" if vertical_speed is None else f"{vertical_speed:.2f}m/s"
                )
                print(
                    f"[爬升] altitude={altitude:.2f}m, vz={speed_text}, "
                    f"mode={snapshot.mode}"
                )
                next_report = now + 1.0

            altitude_ready = altitude >= self.target_altitude_m - 0.12
            speed_ready = (
                vertical_speed is not None and abs(vertical_speed) <= 0.20
            )
            if altitude_ready and speed_ready:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= 1.0:
                    print(
                        f"[高度确认] altitude={altitude:.2f}m，"
                        "已到达2.5m目标范围并稳定"
                    )
                    return
            else:
                stable_since = None

            with self.client._condition:
                self.client._condition.wait(timeout=0.1)

        raise TimeoutError("30秒内未到达目标高度并稳定")

    def _hold(self) -> None:
        started = time.monotonic()
        next_report = 0.0
        duration_text = "持续保持，按Ctrl+C停止" if self.hold_seconds <= 0 else f"保持{self.hold_seconds:.1f}秒"
        print(f"[保持] 已进入PX4起飞后的保持状态：{duration_text}")

        while not self.stop_requested.is_set():
            now = time.monotonic()
            if self.hold_seconds > 0 and now - started >= self.hold_seconds:
                print("[保持完成] v3任务到此结束；本版本不会发送LAND或DISARM")
                return
            if now >= next_report:
                snapshot = self.client.snapshot()
                altitude_text = (
                    "unknown"
                    if snapshot.altitude_m is None
                    else f"{snapshot.altitude_m:.2f}m"
                )
                print(
                    f"[保持状态] armed={snapshot.armed}, mode={snapshot.mode}, "
                    f"altitude={altitude_text}"
                )
                next_report = now + 1.0
            self.stop_requested.wait(0.1)

    def _set_state(self, state: MissionState) -> None:
        self.state = state
        print(f"[状态机] {state.name}")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connection",
        default="udpin:0.0.0.0:14540",
        help="pymavlink连接串，默认监听PX4 SITL Onboard端口14540",
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=2.5,
        help="目标相对起飞高度，单位米，默认2.5",
    )
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=0.0,
        help="保持时间；0表示一直保持到Ctrl+C，默认0",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    if args.altitude <= 0.5:
        raise SystemExit("--altitude必须大于0.5m")
    if args.hold_seconds < 0:
        raise SystemExit("--hold-seconds不能小于0")

    controller = MissionControllerV3(
        args.connection,
        args.altitude,
        args.hold_seconds,
    )

    def handle_stop(_signum, _frame) -> None:
        print("\n[停止请求] 收到中断信号")
        controller.request_stop()

    signal.signal(signal.SIGINT, handle_stop)
    signal.signal(signal.SIGTERM, handle_stop)

    try:
        controller.run()
        snapshot = controller.client.snapshot()
        print(
            f"[v3成功] state={controller.state.name}, armed={snapshot.armed}, "
            f"altitude={snapshot.altitude_m}"
        )
        return 0
    except Exception as error:
        print(f"[v3失败] {type(error).__name__}: {error}")
        print("[安全提示] v3不包含自动LAND；若飞行器已起飞，请从PX4终端执行 commander land")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
