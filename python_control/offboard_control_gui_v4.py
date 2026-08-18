#!/usr/bin/env python3
"""Minimal UDP MAVLink command panel for the V4 MyLink Bridge protocol."""

from __future__ import annotations

import argparse
import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2


DEFAULT_CONNECTION = "udp:127.0.0.1:14540"
DEFAULT_WIFI_IP = "127.0.0.1"
DEFAULT_UDP_PORT = 14540
DEFAULT_AMOUNT_M = 0.20
DEFAULT_TAKEOFF_M = 1.00
DEFAULT_HORIZONTAL_SPEED_M_S = 0.20
KEEPALIVE_INTERVAL_S = 0.10
HEARTBEAT_INTERVAL_S = 1.00
MAV_CMD_USER_1 = int(mavlink2.MAV_CMD_USER_1)

ACTION_TAKEOFF = 1
ACTION_UP = 2
ACTION_DOWN = 3
ACTION_FORWARD = 4
ACTION_BACK = 5
ACTION_LEFT = 6
ACTION_RIGHT = 7
ACTION_HOLD = 8

ACTION_NAMES = {
    ACTION_TAKEOFF: "TAKEOFF",
    ACTION_UP: "UP",
    ACTION_DOWN: "DOWN",
    ACTION_FORWARD: "FORWARD",
    ACTION_BACK: "BACK",
    ACTION_LEFT: "LEFT",
    ACTION_RIGHT: "RIGHT",
    ACTION_HOLD: "HOLD",
}

ACK_NAMES = {
    value: entry.name.removeprefix("MAV_RESULT_")
    for value, entry in mavlink2.enums["MAV_RESULT"].items()
}


@dataclass(frozen=True)
class PendingCommand:
    command: int
    action_name: str
    sent_at: float
    amount_m: float


class MinimalEventLog:
    """Write only TX action, RX ACK and latency records."""

    def __init__(self, sink):
        self._sink = sink
        self._path = Path(__file__).resolve().parent / "logs" / (
            f"gui_v4_{datetime.now():%Y%m%d_%H%M%S}.log"
        )
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self._path.open("a", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()

    @property
    def path(self) -> Path:
        return self._path

    def write(self, line: str) -> None:
        stamped = f"{datetime.now():%H:%M:%S.%f} {line}"
        with self._lock:
            self._file.write(stamped + "\n")
        self._sink(stamped)

    def close(self) -> None:
        with self._lock:
            if not self._file.closed:
                self._file.close()


class MavlinkCommandClient:
    """One UDP connection, one receive loop, and one pending ACK."""

    def __init__(self, connection_string: str, event_log: MinimalEventLog, on_state):
        self.connection_string = connection_string.strip() or DEFAULT_CONNECTION
        self.event_log = event_log
        self.on_state = on_state
        self._connection = None
        self._stop = threading.Event()
        self._tx_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending_by_command: dict[int, PendingCommand] = {}
        self._receive_thread: threading.Thread | None = None
        self._transmit_thread: threading.Thread | None = None
        self._ping_sequence = 0
        self.target_system = 1
        self.target_component = 1

    @property
    def connected(self) -> bool:
        return self._connection is not None and not self._stop.is_set()

    def connect(self) -> None:
        if self.connected:
            return
        connection = mavutil.mavlink_connection(
            self.connection_string,
            source_system=42,
            source_component=191,
            dialect="common",
            autoreconnect=True,
            force_connected=True,
        )
        self._connection = connection
        self._stop.clear()
        connection.wait_heartbeat(timeout=8.0)
        self.target_system = int(connection.target_system or 1)
        self.target_component = int(connection.target_component or 1)
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()
        self._transmit_thread = threading.Thread(target=self._transmit_loop, daemon=True)
        self._transmit_thread.start()
        self.on_state(True)

    def disconnect(self) -> None:
        self._stop.set()
        thread = self._receive_thread
        if thread is not None:
            thread.join(timeout=1.0)
        self._receive_thread = None
        transmit_thread = self._transmit_thread
        if transmit_thread is not None:
            transmit_thread.join(timeout=1.0)
        self._transmit_thread = None
        connection = self._connection
        self._connection = None
        if connection is not None:
            connection.close()
        with self._pending_lock:
            self._pending_by_command.clear()
        self.on_state(False)

    def send_action(self, action: int, amount_m: float, speed_limit_m_s: float) -> bool:
        action_name = ACTION_NAMES[action]
        if not self.connected:
            self.event_log.write(f"REJECT {action_name} reason=DISCONNECTED")
            return False
        with self._pending_lock:
            if MAV_CMD_USER_1 in self._pending_by_command:
                self.event_log.write(f"REJECT {action_name} reason=WAITING_ACK")
                return False
            pending = PendingCommand(
                MAV_CMD_USER_1,
                action_name,
                time.monotonic(),
                amount_m,
            )
            self._pending_by_command[MAV_CMD_USER_1] = pending
        try:
            with self._tx_lock:
                self._connection.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    MAV_CMD_USER_1,
                    0,
                    float(action),
                    float(amount_m),
                    float(speed_limit_m_s),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            self.event_log.write(
                f"TX {action_name} amount={amount_m:.2f} speed_limit={speed_limit_m_s:.2f}"
            )
            return True
        except Exception as error:
            with self._pending_lock:
                self._pending_by_command.pop(MAV_CMD_USER_1, None)
            self.event_log.write(f"REJECT {action_name} reason=TX_ERROR:{error}")
            return False

    def send_land(self) -> bool:
        if not self.connected:
            self.event_log.write("REJECT LAND reason=DISCONNECTED")
            return False
        land_command = int(mavlink2.MAV_CMD_NAV_LAND)
        with self._pending_lock:
            if land_command in self._pending_by_command:
                self.event_log.write("REJECT LAND reason=WAITING_ACK")
                return False
            self._pending_by_command[land_command] = PendingCommand(
                land_command, "LAND", time.monotonic(), 0.0
            )
        try:
            with self._tx_lock:
                self._connection.mav.command_long_send(
                    self.target_system,
                    self.target_component,
                    land_command,
                    0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                )
            self.event_log.write("TX LAND amount=0.00 speed_limit=0.00")
            return True
        except Exception as error:
            with self._pending_lock:
                self._pending_by_command.pop(land_command, None)
            self.event_log.write(f"REJECT LAND reason=TX_ERROR:{error}")
            return False

    def _receive_loop(self) -> None:
        while not self._stop.is_set():
            connection = self._connection
            if connection is None:
                return
            try:
                message = connection.recv_match(blocking=True, timeout=0.2)
            except Exception:
                continue
            if message is None or message.get_type() != "COMMAND_ACK":
                continue
            command = int(message.command)
            with self._pending_lock:
                pending = self._pending_by_command.pop(command, None)
                if pending is None:
                    continue
            latency_ms = (time.monotonic() - pending.sent_at) * 1000.0
            result = int(message.result)
            self.event_log.write(
                f"RX {pending.action_name} ACK={ACK_NAMES.get(result, str(result))} "
                f"latency={latency_ms:.0f}ms"
            )

    def _transmit_loop(self) -> None:
        next_heartbeat = 0.0
        next_keepalive = 0.0
        while not self._stop.is_set():
            connection = self._connection
            if connection is None:
                return
            now = time.monotonic()
            try:
                with self._tx_lock:
                    if now >= next_heartbeat:
                        connection.mav.heartbeat_send(
                            mavlink2.MAV_TYPE_GCS,
                            mavlink2.MAV_AUTOPILOT_INVALID,
                            0,
                            0,
                            mavlink2.MAV_STATE_ACTIVE,
                        )
                        next_heartbeat = now + HEARTBEAT_INTERVAL_S
                    if now >= next_keepalive:
                        connection.mav.ping_send(
                            time.time_ns() // 1000,
                            self._ping_sequence,
                            0,
                            0,
                        )
                        self._ping_sequence = (self._ping_sequence + 1) & 0xFFFFFFFF
                        next_keepalive = now + KEEPALIVE_INTERVAL_S
            except Exception as error:
                self.event_log.write(f"TX_KEEPALIVE_ERROR {error}")
                next_heartbeat = now + HEARTBEAT_INTERVAL_S
                next_keepalive = now + KEEPALIVE_INTERVAL_S
            self._stop.wait(0.02)


class OffboardControlGuiV4(tk.Tk):
    def __init__(self, connection: str):
        super().__init__()
        self.title("PX4 MyLink V4")
        self.geometry("540x600")
        self.minsize(480, 540)
        self._ui_queue: queue.Queue[tuple[str, object]] = queue.Queue()
        self._event_log = MinimalEventLog(self._append_log)
        self._client = MavlinkCommandClient(connection, self._event_log, self._queue_state)
        endpoint = connection.removeprefix("udp:")
        host, separator, port = endpoint.rpartition(":")
        self._wifi_ip_var = tk.StringVar(value=host if separator else DEFAULT_WIFI_IP)
        self._udp_port_var = tk.StringVar(value=port if separator else str(DEFAULT_UDP_PORT))
        self._amount_var = tk.StringVar(value=f"{DEFAULT_AMOUNT_M:.2f}")
        self._takeoff_var = tk.StringVar(value=f"{DEFAULT_TAKEOFF_M:.2f}")
        self._status_var = tk.StringVar(value="Disconnected")
        self._build_ui()
        self.after(50, self._drain_ui_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        root = ttk.Frame(self, padding=16)
        root.pack(fill=tk.BOTH, expand=True)
        ttk.Label(root, text="MAVLink UDP").pack(anchor=tk.W)
        connection_row = ttk.Frame(root)
        connection_row.pack(fill=tk.X, pady=(4, 12))
        ttk.Label(connection_row, text="WiFi IP").pack(side=tk.LEFT)
        ttk.Entry(connection_row, textvariable=self._wifi_ip_var).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 10))
        ttk.Label(connection_row, text="UDP Port").pack(side=tk.LEFT)
        ttk.Entry(connection_row, textvariable=self._udp_port_var, width=7).pack(side=tk.LEFT, padx=(6, 0))
        self._connect_button = ttk.Button(connection_row, text="连接", command=self._connect)
        self._connect_button.pack(side=tk.LEFT, padx=(8, 0))
        self._disconnect_button = ttk.Button(connection_row, text="断开", command=self._disconnect, state=tk.DISABLED)
        self._disconnect_button.pack(side=tk.LEFT, padx=(6, 0))

        status_row = ttk.Frame(root)
        status_row.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(status_row, textvariable=self._status_var).pack(side=tk.LEFT)
        ttk.Label(status_row, text="起飞高度 (m)").pack(side=tk.RIGHT)
        ttk.Entry(status_row, textvariable=self._takeoff_var, width=6).pack(side=tk.RIGHT, padx=(6, 12))
        ttk.Label(status_row, text="步进 (m)").pack(side=tk.RIGHT)
        ttk.Entry(status_row, textvariable=self._amount_var, width=6).pack(side=tk.RIGHT, padx=(6, 12))

        controls = ttk.Frame(root)
        controls.pack(fill=tk.BOTH, expand=True)
        for row in range(5):
            controls.rowconfigure(row, weight=1)
        for column in range(3):
            controls.columnconfigure(column, weight=1)

        self._button(controls, "起飞", lambda: self._send_action(ACTION_TAKEOFF), 0, 1)
        self._button(controls, "上升", lambda: self._send_action(ACTION_UP), 1, 1)
        self._button(controls, "定点", lambda: self._send_action(ACTION_HOLD, 0.0, 0.0), 2, 1)
        self._button(controls, "下降", lambda: self._send_action(ACTION_DOWN), 3, 1)
        self._button(controls, "左移", lambda: self._send_action(ACTION_LEFT), 2, 0)
        self._button(controls, "右移", lambda: self._send_action(ACTION_RIGHT), 2, 2)
        self._button(controls, "前进", lambda: self._send_action(ACTION_FORWARD), 3, 0)
        self._button(controls, "后退", lambda: self._send_action(ACTION_BACK), 3, 2)
        self._button(controls, "降落", self._send_land, 4, 1)

        ttk.Label(root, text="ACK / 通信日志").pack(anchor=tk.W, pady=(12, 4))
        self._log_text = tk.Text(root, height=8, state=tk.DISABLED, wrap=tk.NONE)
        self._log_text.pack(fill=tk.BOTH, expand=True)

    def _button(self, parent, text, command, row, column) -> None:
        ttk.Button(parent, text=text, command=command).grid(
            row=row, column=column, sticky="nsew", padx=5, pady=5
        )

    def _append_log(self, line: str) -> None:
        self._ui_queue.put(("log", line))

    def _queue_state(self, connected: bool) -> None:
        self._ui_queue.put(("state", connected))

    def _drain_ui_queue(self) -> None:
        try:
            while True:
                kind, value = self._ui_queue.get_nowait()
                if kind == "log":
                    self._log_text.configure(state=tk.NORMAL)
                    self._log_text.insert(tk.END, str(value) + "\n")
                    self._log_text.see(tk.END)
                    self._log_text.configure(state=tk.DISABLED)
                elif kind == "state":
                    connected = bool(value)
                    self._status_var.set("Connected" if connected else "Disconnected")
                    self._connect_button.configure(state=tk.DISABLED if connected else tk.NORMAL)
                    self._disconnect_button.configure(state=tk.NORMAL if connected else tk.DISABLED)
        except queue.Empty:
            pass
        self.after(50, self._drain_ui_queue)

    def _connect(self) -> None:
        self._connect_button.configure(state=tk.DISABLED)

        def worker() -> None:
            try:
                host = self._wifi_ip_var.get().strip() or DEFAULT_WIFI_IP
                port = int(self._udp_port_var.get())
                if not 1 <= port <= 65535:
                    raise ValueError("UDP Port 必须在 1 到 65535 之间")
                self._client.connection_string = f"udp:{host}:{port}"
                self._client.connect()
            except Exception as error:
                self._queue_state(False)
                self._append_log(f"CONNECT_ERROR {error}")
                self.after(0, lambda: self._connect_button.configure(state=tk.NORMAL))

        threading.Thread(target=worker, daemon=True).start()

    def _disconnect(self) -> None:
        self._client.disconnect()

    def _amount(self, action: int) -> float:
        amount = float(self._takeoff_var.get() if action == ACTION_TAKEOFF else self._amount_var.get())
        if amount <= 0.0:
            raise ValueError("amount must be positive")
        return amount

    def _send_action(self, action: int, amount: float | None = None, speed: float | None = None) -> None:
        try:
            amount_value = self._amount(action) if amount is None else amount
        except ValueError as error:
            messagebox.showerror("输入错误", str(error))
            return
        horizontal = action in {ACTION_FORWARD, ACTION_BACK, ACTION_LEFT, ACTION_RIGHT}
        speed_value = DEFAULT_HORIZONTAL_SPEED_M_S if horizontal else 0.0
        if speed is not None:
            speed_value = speed
        self._client.send_action(action, amount_value, speed_value)

    def _send_land(self) -> None:
        self._client.send_land()

    def _on_connection_state(self, _connected: bool) -> None:
        self._queue_state(_connected)

    def _on_close(self) -> None:
        self._client.disconnect()
        self._event_log.close()
        self.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default=DEFAULT_CONNECTION)
    args = parser.parse_args()
    app = OffboardControlGuiV4(args.connection)
    app.mainloop()


if __name__ == "__main__":
    main()
