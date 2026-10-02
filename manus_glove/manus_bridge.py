from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class ManusFrame:
    side: str
    keypoints: np.ndarray
    ergonomics: dict[str, float]
    sample_id: int
    received_at: float


def parse_glove_yaml(text: str) -> tuple[Optional[str], Optional[np.ndarray], dict[str, float]]:
    """Parse one glove YAML message into a normalized side + keypoint array."""
    side = None
    keypoints = np.zeros((25, 7), dtype=float)
    ergonomics: dict[str, float] = {}

    lines = text.strip().splitlines()
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("side:"):
            side = stripped.split(":", 1)[1].strip().lower()
            break

    if side not in {"left", "right"}:
        return None, None, ergonomics

    current_node_id = -1
    in_position = False
    in_orientation = False
    px = py = pz = 0.0
    ox = oy = oz = 0.0
    ow = 1.0

    for line in lines:
        stripped = line.strip()

        if stripped.startswith("- node_id:"):
            if 0 <= current_node_id < 25:
                keypoints[current_node_id] = [px, py, pz, ow, ox, oy, oz]
            current_node_id = int(stripped.split(":", 1)[1].strip())
            in_position = False
            in_orientation = False
            px = py = pz = 0.0
            ox = oy = oz = 0.0
            ow = 1.0
            continue

        if current_node_id < 0:
            continue

        if stripped == "position:":
            in_position = True
            in_orientation = False
            continue
        if stripped == "orientation:":
            in_orientation = True
            in_position = False
            continue

        if stripped.startswith(("parent_node_id:", "joint_type:", "chain_type:", "pose:")):
            in_position = False
            in_orientation = False
            continue

        if in_position:
            if stripped.startswith("x:"):
                px = float(stripped.split(":", 1)[1].strip())
            elif stripped.startswith("y:"):
                py = float(stripped.split(":", 1)[1].strip())
            elif stripped.startswith("z:"):
                pz = float(stripped.split(":", 1)[1].strip())
        elif in_orientation:
            if stripped.startswith("x:"):
                ox = float(stripped.split(":", 1)[1].strip())
            elif stripped.startswith("y:"):
                oy = float(stripped.split(":", 1)[1].strip())
            elif stripped.startswith("z:"):
                oz = float(stripped.split(":", 1)[1].strip())
            elif stripped.startswith("w:"):
                ow = float(stripped.split(":", 1)[1].strip())

        if stripped.startswith(("ergonomics_count:", "ergonomics:")):
            break

    if 0 <= current_node_id < 25:
        keypoints[current_node_id] = [px, py, pz, ow, ox, oy, oz]

    current_ergo_type: Optional[str] = None
    in_ergonomics = False
    for line in lines:
        stripped = line.strip()
        if stripped == "ergonomics:":
            in_ergonomics = True
            current_ergo_type = None
            continue
        if not in_ergonomics:
            continue
        if stripped.startswith("raw_sensor_orientation:"):
            break
        if stripped.startswith("- type:"):
            current_ergo_type = stripped.split(":", 1)[1].strip()
            continue
        if stripped.startswith("type:"):
            current_ergo_type = stripped.split(":", 1)[1].strip()
            continue
        if stripped.startswith("value:") and current_ergo_type:
            ergonomics[current_ergo_type] = float(stripped.split(":", 1)[1].strip())
            current_ergo_type = None

    return side, keypoints, ergonomics


class ManusGloveBridgeClient:
    """TCP client for the Manus glove bridge used in the docker workflow."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 9090,
        connect_timeout_s: float = 2.0,
        reconnect_delay_s: float = 1.0,
    ) -> None:
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.reconnect_delay_s = reconnect_delay_s

        self._lock = threading.Lock()
        self._frames = {"left": None, "right": None}
        self._sample_counts = {"left": 0, "right": 0}
        self._announced_sides = set()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True, name="manus-bridge-client")
        self._thread.start()

    def close(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def get_latest(self, side: str) -> Optional[ManusFrame]:
        side_key = side.lower()
        if side_key not in self._frames:
            raise ValueError(f"Unsupported glove side: {side}")
        with self._lock:
            frame = self._frames[side_key]
            if frame is None:
                return None
            return ManusFrame(
                side=frame.side,
                keypoints=frame.keypoints.copy(),
                ergonomics=frame.ergonomics.copy(),
                sample_id=frame.sample_id,
                received_at=frame.received_at,
            )

    def wait_for_first_frame(self, side: str, timeout_s: float) -> Optional[ManusFrame]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._stop_event.is_set():
            frame = self.get_latest(side)
            if frame is not None:
                return frame
            time.sleep(0.05)
        return self.get_latest(side)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            sock = self._connect()
            if sock is None:
                break
            try:
                self._recv_until_disconnect(sock)
            finally:
                try:
                    sock.close()
                except OSError:
                    pass

            if not self._stop_event.is_set():
                print(
                    f"[manus_bridge] bridge disconnected, retrying in {self.reconnect_delay_s:.1f}s..."
                )
                time.sleep(self.reconnect_delay_s)

    def _connect(self) -> Optional[socket.socket]:
        first_attempt = True
        while not self._stop_event.is_set():
            try:
                sock = socket.create_connection(
                    (self.host, self.port), timeout=self.connect_timeout_s
                )
                sock.settimeout(1.0)
                print(f"[manus_bridge] connected to {self.host}:{self.port}")
                return sock
            except OSError as exc:
                if first_attempt:
                    print(f"[manus_bridge] waiting for bridge at {self.host}:{self.port} ({exc})")
                    first_attempt = False
                time.sleep(self.reconnect_delay_s)
        return None

    def _recv_until_disconnect(self, sock: socket.socket) -> None:
        buffer = ""
        message_buffers = {0: "", 1: ""}

        while not self._stop_event.is_set():
            try:
                payload = sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break

            if not payload:
                break

            buffer += payload.decode("utf-8", errors="replace")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                glove_index = -1
                stripped = line
                if line.startswith("GLOVE0:"):
                    glove_index = 0
                    stripped = line[7:]
                elif line.startswith("GLOVE1:"):
                    glove_index = 1
                    stripped = line[7:]

                if glove_index < 0:
                    continue

                if stripped == "---":
                    if message_buffers[glove_index]:
                        side, keypoints, ergonomics = parse_glove_yaml(message_buffers[glove_index])
                        if side is not None and keypoints is not None:
                            self._update_frame(side, keypoints, ergonomics)
                    message_buffers[glove_index] = ""
                else:
                    message_buffers[glove_index] += stripped + "\n"

    def _update_frame(self, side: str, keypoints: np.ndarray, ergonomics: dict[str, float]) -> None:
        with self._lock:
            self._sample_counts[side] += 1
            self._frames[side] = ManusFrame(
                side=side,
                keypoints=keypoints.copy(),
                ergonomics=ergonomics.copy(),
                sample_id=self._sample_counts[side],
                received_at=time.time(),
            )
            should_announce = side not in self._announced_sides
            if should_announce:
                self._announced_sides.add(side)
        if should_announce:
            print(f"[manus_bridge] first {side} glove frame received")
