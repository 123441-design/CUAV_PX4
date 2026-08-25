#!/usr/bin/env python3
"""Relay four x500_cart Gazebo lidars to the Windows V3 GUI over UDP.

This program forwards measured distances only. It does not infer top contact.
The V3 GUI remains the sole MAVLink peer of mylink_bridge on UDP 14541.
"""

from __future__ import annotations

import argparse
import math
import socket
import struct
import threading
import time
from dataclasses import dataclass

from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node


SENSOR_NAMES = ("cart_lidar_fr", "cart_lidar_fl", "cart_lidar_rr", "cart_lidar_rl")
MAGIC = b"V3LD"
VERSION = 1
PACKET = struct.Struct("!4sBBHQ4H")


@dataclass
class Sample:
    distance_m: float = math.nan
    generation: int = 0


def wsl_default_gateway() -> str | None:
    """Return the Windows host gateway in WSL NAT mode, if present."""
    try:
        with open("/proc/net/route", encoding="ascii") as routes:
            for line in routes:
                fields = line.split()
                if len(fields) >= 3 and fields[1] == "00000000":
                    raw = bytes.fromhex(fields[2])
                    return socket.inet_ntoa(raw[::-1])
    except (OSError, ValueError):
        pass
    return None


class CartLidarRelay:
    def __init__(self, destinations: list[tuple[str, int]], rate_hz: float) -> None:
        self._destinations = destinations
        self._period = 1.0 / rate_hz
        self._node = Node()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._samples = [Sample() for _ in SENSOR_NAMES]
        self._lock = threading.Lock()
        self._sequence = 0
        self._last_emitted_generations = (0, 0, 0, 0)
        self._topics: list[str] = []

    def discover_and_subscribe(self) -> None:
        while True:
            advertised = tuple(self._node.topic_list())
            topics: list[str] = []
            for sensor_id, sensor_name in enumerate(SENSOR_NAMES):
                suffix = f"/sensor/{sensor_name}/scan"
                matches = [topic for topic in advertised if topic.endswith(suffix)]
                if len(matches) != 1:
                    break
                topic = matches[0]
                self._node.subscribe(LaserScan, topic, self._callback(sensor_id))
                topics.append(topic)
            if len(topics) == len(SENSOR_NAMES):
                self._topics = topics
                print("Subscribed to four measured Gazebo lidar topics:")
                for name, topic in zip(SENSOR_NAMES, topics):
                    print(f"  {name}: {topic}")
                return
            print("Waiting for x500_cart lidar topics ...", flush=True)
            time.sleep(1.0)

    def _callback(self, sensor_id: int):
        def receive(message: LaserScan, *_unused) -> None:
            distance = math.nan
            if message.ranges:
                candidate = float(message.ranges[0])
                if (
                    math.isfinite(candidate)
                    and candidate >= float(message.range_min)
                    and candidate < float(message.range_max)
                ):
                    distance = candidate
            with self._lock:
                generation = self._samples[sensor_id].generation + 1
                self._samples[sensor_id] = Sample(distance, generation)
        return receive

    def _make_packet(self) -> tuple[bytes, tuple[float, float, float, float], int] | None:
        timestamp_us = time.monotonic_ns() // 1000
        valid_mask = 0
        millimetres: list[int] = []
        distances: list[float] = []
        with self._lock:
            samples = tuple(self._samples)
            generations = tuple(sample.generation for sample in samples)
            if any(
                generation <= last_generation
                for generation, last_generation in zip(generations, self._last_emitted_generations)
            ):
                return None
            self._last_emitted_generations = generations
        for sensor_id, sample in enumerate(samples):
            valid = (
                math.isfinite(sample.distance_m)
                and 0.0 <= sample.distance_m <= 65.535
            )
            if valid:
                valid_mask |= 1 << sensor_id
                millimetres.append(min(0xFFFF, round(sample.distance_m * 1000.0)))
                distances.append(sample.distance_m)
            else:
                millimetres.append(0)
                distances.append(math.nan)
        self._sequence = (self._sequence + 1) & 0x0FFF
        payload = PACKET.pack(
            MAGIC,
            VERSION,
            valid_mask,
            self._sequence,
            timestamp_us,
            *millimetres,
        )
        return payload, tuple(distances), valid_mask

    def run(self) -> None:
        self.discover_and_subscribe()
        next_send = time.monotonic()
        next_report = next_send
        last_distances = (math.nan,) * 4
        last_mask = 0
        while True:
            now = time.monotonic()
            if now >= next_send:
                packet = self._make_packet()
                if packet is not None:
                    payload, last_distances, last_mask = packet
                    for destination in self._destinations:
                        self._socket.sendto(payload, destination)
                next_send += self._period
                if next_send < now:
                    next_send = now + self._period
            if now >= next_report:
                values = " ".join(
                    f"{name}={distance:.3f}m" if last_mask & (1 << index) else f"{name}=INVALID"
                    for index, (name, distance) in enumerate(zip(SENSOR_NAMES, last_distances))
                )
                print(f"seq={self._sequence:04d} mask=0x{last_mask:X} {values}", flush=True)
                next_report = now + 1.0
            time.sleep(min(0.01, max(0.0, next_send - time.monotonic())))


def destinations(hosts: list[str] | None, port: int) -> list[tuple[str, int]]:
    candidates = hosts or ["127.0.0.1"]
    if hosts is None:
        gateway = wsl_default_gateway()
        if gateway and gateway not in candidates:
            candidates.append(gateway)
    return [(socket.gethostbyname(host), port) for host in candidates]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", action="append", help="Windows GUI host; repeat for multiple destinations")
    parser.add_argument("--port", type=int, default=14600, help="V3 GUI lidar UDP port")
    parser.add_argument("--rate", type=float, default=20.0, help="forwarding rate in Hz")
    args = parser.parse_args()
    if not (1 <= args.port <= 65535):
        parser.error("--port must be between 1 and 65535")
    if not (1.0 <= args.rate <= 100.0):
        parser.error("--rate must be between 1 and 100 Hz")
    targets = destinations(args.host, args.port)
    print("UDP destinations: " + ", ".join(f"{host}:{port}" for host, port in targets))
    try:
        CartLidarRelay(targets, args.rate).run()
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
