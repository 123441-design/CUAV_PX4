#!/usr/bin/env python3
"""PX4 SITL Offboard visual ground station (v1).

The GUI is intentionally a thin view/controller layer.  All MAVLink reads are
performed by the single receiver thread in ``mission_controller_v6_optical_flow``;
Tk callbacks only inspect cached state and enqueue commands.
"""

from __future__ import annotations

import argparse
import math
import queue
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
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
if not (PROJECT_CODE_DIR / "mission_controller_v6_optical_flow.py").exists():
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
    """Safety-gated API over the validated v6 MAVLink client."""

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

    def _set_busy(self, value: bool) -> None:
        with self._client_lock:
            self._busy = value

    def _submit(self, name: str, action: Callable[[], None]) -> None:
        with self._client_lock:
            if self._busy:
                self.log.write(f"[REJECT] {name}: another action is running")
                return
            self._busy = True

        def worker() -> None:
            try:
                with self._action_lock:
                    action()
            except Exception as exc:  # UI must remain alive after a rejected command
                self.log.write(f"[ERROR] {name}: {exc}")
            finally:
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

            client.request_estimator_streams()
            local_deadline = time.monotonic() + 10.0
            local = client.local_position()
            while (
                local.x_m is None
                or local.y_m is None
                or local.z_m is None
                or local.age_s is None
                or local.age_s >= 1.0
            ) and time.monotonic() < local_deadline:
                client.wait_for_update(client.update_sequence(), timeout_s=0.5)
                local = client.local_position()
            if local.x_m is None or local.y_m is None or local.z_m is None:
                raise TimeoutError("等待有效 LOCAL_POSITION_NED 超时")

            self.origin = (local.x_m, local.y_m, local.z_m)
            self._set_target((local.x_m, local.y_m, local.z_m))
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
        state = self.read_state()
        if state.armed:
            raise RuntimeError("飞行器 armed 时禁止 DISCONNECT，请先 LAND + DISARM")
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
        client = self._client_or_raise()
        client.set_position_target(*values)

    def _target(self) -> list[float]:
        with self._target_lock:
            if self.target_local_ned is None:
                raise RuntimeError("尚未建立 Local NED 目标")
            return list(self.target_local_ned)

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
        client = self._client_or_raise()
        state = self.read_state()
        if not state.gcs_connected:
            raise RuntimeError("PX4 heartbeat/GCS connection 尚未有效")
        if state.failsafe:
            raise RuntimeError("failsafe 激活，拒绝普通控制命令")
        if state.position_valid is not True:
            raise RuntimeError("Local Position 无效")
        return client

    @staticmethod
    def _require_optical_flow_gate(state: CachedState) -> None:
        # GPS is telemetry-only: this aircraft uses optical flow + Local NED.
        if state.flow_active is not True:
            raise RuntimeError(f"Optical Flow active={state.flow_active}，拒绝 ARM")
        if state.position_valid is not True:
            raise RuntimeError("Local Position invalid，拒绝 ARM")

    def arm_async(self) -> None:
        self._submit("ARM", self.arm)

    def arm(self) -> None:
        client = self._require_connected()
        state = self.read_state()
        if state.armed:
            raise RuntimeError("飞机已经 armed")
        self._require_optical_flow_gate(state)
        target = self._target()
        client.set_position_target(*target)
        client.start_setpoint_stream()
        required = max(20, int(math.ceil(client.setpoint_rate_hz * 2.0)))
        client.wait_for_setpoint_count(required, timeout_s=5.0)
        self.log.write("[MISSION] OFFBOARD prewarm complete")
        result = client.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            [float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED), float(PX4_CUSTOM_MAIN_MODE_OFFBOARD), 0, 0, 0, 0, 0],
        )
        self.log.write(f"[ACK] OFFBOARD command=176 result={result}")
        client.wait_for(lambda s: s.mode.upper() == "OFFBOARD", 8.0, "OFFBOARD")
        result = client.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1.0, 0, 0, 0, 0, 0, 0],
        )
        self.log.write(f"[ACK] ARM command=400 result={result}")
        client.wait_for(lambda s: s.armed, 10.0, "armed=True")
        self.log.write("[OK] ARM accepted; OFFBOARD active")

    def _validate_number(self, value: str, low: float, high: float, name: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} 必须是数字") from exc
        if not math.isfinite(number) or not low <= number <= high:
            raise ValueError(f"{name} 必须在 {low:g}～{high:g} 范围内")
        return number

    def takeoff_async(self, altitude_text: str) -> None:
        self._submit("TAKEOFF", lambda: self.takeoff(altitude_text))

    def takeoff(self, altitude_text: str) -> None:
        altitude = self._validate_number(altitude_text, MIN_ALTITUDE_M, MAX_ALTITUDE_M, "起飞高度")
        client = self._require_connected()
        state = self.read_state()
        if not state.armed or state.mode.upper() != "OFFBOARD":
            raise RuntimeError("TAKEOFF 需要 armed 且处于 OFFBOARD")
        if self.origin is None:
            raise RuntimeError("尚未建立 Local NED origin")
        target = self._target()
        target[2] = self.origin[2] - altitude
        self._set_target(tuple(target))
        self.log.write(f"[MISSION] TAKEOFF altitude={altitude:.2f}m target_z={target[2]:.3f}")

    def move_async(self, direction: str, distance_text: str) -> None:
        self._submit(direction, lambda: self.move(direction, distance_text))

    def move(self, direction: str, distance_text: str) -> None:
        distance = self._validate_number(distance_text, MIN_MOVE_M, MAX_MOVE_M, "移动距离")
        client = self._require_connected()
        state = self.read_state()
        if not state.armed or state.mode.upper() != "OFFBOARD":
            raise RuntimeError("移动需要 armed 且处于 OFFBOARD")
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
        self.log.write(f"[MISSION] MOVE {direction} {distance:.2f}m target=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})")

    def hover_async(self) -> None:
        self._submit("HOVER", self.hover)

    def hover(self) -> None:
        client = self._require_connected()
        state = self.read_state()
        if not state.armed or state.mode.upper() != "OFFBOARD":
            raise RuntimeError("HOVER 需要 armed 且处于 OFFBOARD")
        if None in (state.local_x, state.local_y, state.local_z):
            raise RuntimeError("当前 Local Position 无效")
        self._set_target((state.local_x, state.local_y, state.local_z))
        self.log.write("[MISSION] HOVER: current Local Position locked as target")

    def land_async(self) -> None:
        self._submit("LAND", lambda: self.land(safe=False))

    def land(self, safe: bool = False) -> None:
        client = self._client_or_raise() if safe else self._require_connected()
        state = self.read_state()
        if not state.armed:
            raise RuntimeError("飞机未 armed")
        result = client.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            [0, 0, 0, math.nan, math.nan, math.nan, 0],
        )
        self.log.write(f"[ACK] LAND command=21 result={result}")
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            state = self.read_state()
            if state.landed is True:
                self.log.write("[OK] Landing complete landed=True")
                return
            time.sleep(0.2)
        raise TimeoutError("等待 landed=True 超时")

    def disarm_async(self) -> None:
        self._submit("DISARM", self.disarm)

    def disarm(self) -> None:
        client = self._require_connected()
        state = self.read_state()
        if state.landed is not True:
            raise RuntimeError("只有 landed=True 才允许 DISARM")
        result = client.send_command_long(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [0, 0, 0, 0, 0, 0, 0],
        )
        self.log.write(f"[ACK] DISARM command=400 result={result}")
        client.wait_for(lambda s: not s.armed, 10.0, "armed=False")
        client.stop_setpoint_stream()
        self.log.write("[MISSION] COMPLETE")

    def safe_land_async(self) -> None:
        self._submit("SAFE LAND", lambda: self.land(safe=True))

    def close(self) -> None:
        with self._client_lock:
            client = self.client
            self.client = None
        if client is not None:
            client.close()


class OffboardControlGui:
    TRANSLATIONS = {
        "zh": {
            "window": "PX4 Offboard 地面控制站", "connection": "连接", "mavlink": "MAVLink 连接：",
            "connect": "连接", "disconnect": "断开连接", "language": "语言：", "flight_state": "飞行状态",
            "telemetry_target": "遥测 / 控制目标", "control": "控制面板", "event_log": "事件日志",
            "px4": "PX4", "system": "系统", "mode": "模式", "armed": "已解锁", "landed": "已着陆",
            "failsafe": "故障保护", "gcs": "GCS 连接", "offboard": "Offboard", "position_valid": "位置有效",
            "gps": "GPS（仅监视）", "flow": "光流 active", "flow_quality": "光流质量", "actual": "当前位置",
            "velocity": "速度", "attitude": "姿态", "target": "控制目标", "takeoff_altitude": "起飞高度（0.3–3.0 m）",
            "movement_distance": "移动距离（0.1–5.0 m）", "arm": "解锁 ARM", "takeoff": "起飞 TAKEOFF",
            "forward": "前进", "back": "后退", "left": "左移", "right": "右移", "up": "上升",
            "down": "下降", "hover": "悬停", "land": "降落 LAND", "safe_land": "安全降落",
            "disarm": "上锁 DISARM", "log": "日志：", "connected": "已连接", "disconnected": "未连接",
            "heartbeat": "心跳", "safety": "安全限制", "disconnect_warning": "飞行中不能断开连接，请先 LAND 并确认 landed=True。",
            "busy_warning": "当前仍有控制命令执行中，请等待完成。", "armed_warning": "飞行器仍处于 armed，不能退出。请先 LAND + DISARM。",
        },
        "en": {
            "window": "PX4 Offboard Ground Control Station", "connection": "Connection", "mavlink": "MAVLink connection:",
            "connect": "CONNECT", "disconnect": "DISCONNECT", "language": "Language:", "flight_state": "FLIGHT STATE",
            "telemetry_target": "TELEMETRY / TARGET", "control": "CONTROL", "event_log": "EVENT LOG",
            "px4": "PX4", "system": "System", "mode": "Mode", "armed": "Armed", "landed": "Landed",
            "failsafe": "Failsafe", "gcs": "GCS connected", "offboard": "Offboard", "position_valid": "Position valid",
            "gps": "GPS (monitor only)", "flow": "Optical flow active", "flow_quality": "Flow quality", "actual": "Actual position",
            "velocity": "Velocity", "attitude": "Attitude", "target": "Target", "takeoff_altitude": "Takeoff altitude (0.3–3.0 m)",
            "movement_distance": "Movement distance (0.1–5.0 m)", "arm": "ARM", "takeoff": "TAKEOFF",
            "forward": "FORWARD", "back": "BACK", "left": "LEFT", "right": "RIGHT", "up": "UP",
            "down": "DOWN", "hover": "HOVER", "land": "LAND", "safe_land": "SAFE LAND", "disarm": "DISARM",
            "log": "Log: ", "connected": "CONNECTED", "disconnected": "DISCONNECTED", "heartbeat": "heartbeat",
            "safety": "Safety restriction", "disconnect_warning": "Cannot disconnect while flying. LAND and confirm landed=True first.",
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

        self.connection_var = tk.StringVar(value=DEFAULT_CONNECTION)
        self.language_var = tk.StringVar(value="中文")
        self.altitude_var = tk.StringVar(value="1.00")
        self.distance_var = tk.StringVar(value="0.50")
        self.status_vars = {name: tk.StringVar(value="—") for name in (
            "connection", "px4", "mode", "armed", "landed", "failsafe", "gcs", "offboard",
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
        for i, key in enumerate(("px4", "system", "mode", "armed", "landed", "failsafe", "gcs", "offboard", "position_valid", "gps", "flow", "flow_quality")):
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
        self.action_buttons["ARM"] = ttk.Button(controls, text=self._tr("arm"), command=self.controller.arm_async)
        self.action_buttons["ARM"].grid(row=1, column=0, sticky="ew", pady=5)
        self.action_buttons["TAKEOFF"] = ttk.Button(controls, text=self._tr("takeoff"), command=self._takeoff)
        self.action_buttons["TAKEOFF"].grid(row=1, column=1, sticky="ew", padx=5, pady=5)
        ttk.Label(controls, text=self._tr("movement_distance")).grid(row=2, column=0, sticky="w", pady=(5, 4))
        ttk.Entry(controls, textvariable=self.distance_var, width=10).grid(row=2, column=1, sticky="w")
        dirs = (("FORWARD", 3, 1, "forward"), ("LEFT", 4, 0, "left"), ("HOVER", 4, 1, "hover"), ("RIGHT", 4, 2, "right"), ("BACK", 5, 1, "back"), ("UP", 3, 0, "up"), ("DOWN", 3, 2, "down"))
        for name, row, col, label_key in dirs:
            command = self.controller.hover_async if name == "HOVER" else (lambda n=name: self._move(n))
            self.action_buttons[name] = ttk.Button(controls, text=self._tr(label_key), command=command)
            self.action_buttons[name].grid(row=row, column=col, sticky="ew", padx=2, pady=2)
        self.action_buttons["LAND"] = ttk.Button(controls, text=self._tr("land"), command=self.controller.land_async)
        self.action_buttons["LAND"].grid(row=6, column=0, sticky="ew", pady=(8, 2))
        self.action_buttons["SAFE LAND"] = ttk.Button(controls, text=self._tr("safe_land"), command=self.controller.safe_land_async)
        self.action_buttons["SAFE LAND"].grid(row=6, column=1, sticky="ew", padx=4, pady=(8, 2))
        self.action_buttons["DISARM"] = ttk.Button(controls, text=self._tr("disarm"), command=self.controller.disarm_async)
        self.action_buttons["DISARM"].grid(row=7, column=0, columnspan=2, sticky="ew", pady=2)

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
        state = self.controller.read_state()
        if state.armed and state.landed is not True:
            messagebox.showwarning(self._tr("safety"), self._tr("disconnect_warning"))
            return
        self.controller.disconnect_async()

    def _takeoff(self) -> None:
        self.controller.takeoff_async(self.altitude_var.get())

    def _move(self, direction: str) -> None:
        self.controller.move_async(direction, self.distance_var.get())

    @staticmethod
    def _fmt(value: float | None, unit: str = "") -> str:
        return "—" if value is None else f"{value:.2f}{unit}"

    def _refresh(self) -> None:
        if self._closing:
            return
        state = self.controller.read_state()
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
        with self.controller._target_lock:
            target = list(self.controller.target_local_ned) if self.controller.target_local_ned else None
        if target is None:
            self.status_vars["target"].set("N —  E —  Alt —")
        else:
            target_alt = None if self.controller.origin is None else self.controller.origin[2] - target[2]
            self.status_vars["target"].set(f"N {target[0]:.2f}  E {target[1]:.2f}  Alt {target_alt:.2f}m\nLocal z {target[2]:.2f}")
        if state.error:
            self.status_vars["connection"].set(self.status_vars["connection"].get() + f"  | ERROR: {state.error}")
        ready = state.connected and not state.busy
        self.connect_button.configure(state="normal" if not state.connected and not state.busy else "disabled")
        self.disconnect_button.configure(state="normal" if ready and not state.armed else "disabled")
        self.action_buttons["ARM"].configure(state="normal" if ready and not state.armed else "disabled")
        flight_ready = ready and state.armed and state.mode.upper() == "OFFBOARD"
        for name in ("TAKEOFF", "FORWARD", "BACK", "LEFT", "RIGHT", "UP", "DOWN", "HOVER"):
            self.action_buttons[name].configure(state="normal" if flight_ready else "disabled")
        self.action_buttons["LAND"].configure(state="normal" if ready and state.armed else "disabled")
        self.action_buttons["SAFE LAND"].configure(state="normal" if ready and state.armed else "disabled")
        self.action_buttons["DISARM"].configure(state="normal" if ready and state.landed is True else "disabled")
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
        if state.armed:
            messagebox.showwarning(self._tr("safety"), self._tr("armed_warning"))
            return
        self._closing = True
        self.controller.close()
        self.log.write("[INFO] GUI closed")
        self.log.close()
        self.root.destroy()


def self_test() -> None:
    """Gate 1 smoke test: import GUI/backend and validate command limits."""
    assert MIN_ALTITUDE_M == 0.3 and MAX_ALTITUDE_M == 3.0
    assert MIN_MOVE_M == 0.1 and MAX_MOVE_M == 5.0
    assert POSITION_ONLY_TYPE_MASK != 0
    for gps_active in (None, False, True):
        FlightController._require_optical_flow_gate(CachedState(
            gps_active=gps_active,
            flow_active=True,
            position_valid=True,
        ))
    for invalid_state in (
        CachedState(gps_active=None, flow_active=False, position_valid=True),
        CachedState(gps_active=None, flow_active=True, position_valid=False),
    ):
        try:
            FlightController._require_optical_flow_gate(invalid_state)
        except RuntimeError:
            pass
        else:
            raise AssertionError("optical-flow/local-position gate must reject invalid state")
    print("GUI self-test: GPS gate disabled; optical flow/local position gates OK; no PX4 connection opened")


def main() -> None:
    parser = argparse.ArgumentParser(description="PX4 SITL Offboard Control Station v1")
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    parser.add_argument("--self-test", action="store_true", help="run Gate 1 import/validation test")
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
