#!/usr/bin/env python3
"""
fake_bridge.py — Stand-in for the docker glove_bridge when no gloves are attached.

Serves the same wire format as docker/glove_bridge.c (`ros2 topic echo` YAML of
the ManusGlove message, every line prefixed with GLOVE0:/GLOVE1:, messages
separated by `---`) with two synthetic hands opening and closing.

Usage: python3 tools/fake_bridge.py [port [rate_hz]]
  port:    default 9090
  rate_hz: default 30
"""

import math
import socket
import sys
import threading
import time

# Raw Manus node layout: 0 = wrist, then 4 or 5 nodes per finger, base -> tip.
FINGERS = [
    # name,    first node, base offset (x, y),  segment lengths (m)
    ("Thumb",  1,  (0.025, 0.015), (0.045, 0.035, 0.030)),
    ("Index",  5,  (0.022, 0.020), (0.065, 0.040, 0.025, 0.020)),
    ("Middle", 10, (0.007, 0.020), (0.065, 0.045, 0.028, 0.022)),
    ("Ring",   15, (-0.008, 0.020), (0.060, 0.040, 0.026, 0.020)),
    ("Pinky",  20, (-0.022, 0.018), (0.055, 0.032, 0.020, 0.018)),
]

# MANUS ergonomics channels (flexion in degrees) reported per finger.
ERGO_JOINTS = ("MCPStretch", "PIPStretch", "DIPStretch")


def _quat_x(angle):
    return (math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0)


def hand_pose(side, t):
    """Return ([(x, y, z, qw, qx, qy, qz)] * 25, {ergonomics}) for one hand.

    Palm in the XY plane, fingers along +y, curling toward -z. Fingers close in
    a wave so each one is visibly distinct.
    """
    mirror = 1.0 if side == "Left" else -1.0
    phase = 0.0 if side == "Left" else math.pi / 2
    nodes = [(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)] * 25
    ergo = {}

    for fi, (name, first, (bx, by), lengths) in enumerate(FINGERS):
        curl = 0.5 - 0.5 * math.cos(1.5 * t + phase - 0.4 * fi)  # 0 open .. 1 closed
        x, y, z = mirror * bx, by, 0.0
        nodes[first] = (x, y, z, 1.0, 0.0, 0.0, 0.0)

        # Metacarpal stays rigid for the long fingers; the thumb flexes from its base.
        bends = [0.0, 1.2, 1.4, 0.9] if name != "Thumb" else [0.4, 0.6, 0.7]
        splay = mirror * (0.6 if name == "Thumb" else 0.08 * (fi - 2.5))
        angle = 0.0
        for si, seg in enumerate(lengths):
            angle += bends[si] * curl
            dy = seg * math.cos(angle)
            dz = -seg * math.sin(angle)
            x += dy * math.sin(splay)
            y += dy * math.cos(splay)
            z += dz
            nodes[first + si + 1] = (x, y, z, *_quat_x(-angle))

        for ji, joint in enumerate(ERGO_JOINTS):
            ergo[f"{name}{joint}"] = math.degrees(bends[min(ji + 1, len(bends) - 1)] * curl)
        ergo[f"{name}Spread" if name != "Thumb" else "ThumbMCPSpread"] = 0.0

    return nodes, ergo


def glove_message(glove_id, side, t):
    """Render one ManusGlove message the way `ros2 topic echo` prints it."""
    nodes, ergo = hand_pose(side, t)
    out = [
        f"glove_id: {glove_id}",
        f"side: {side}",
        f"raw_node_count: {len(nodes)}",
        "raw_nodes:",
    ]
    for i, (px, py, pz, qw, qx, qy, qz) in enumerate(nodes):
        out += [
            f"- node_id: {i}",
            f"  parent_node_id: {0 if i in (0, 1, 5, 10, 15, 20) else i - 1}",
            "  joint_type: Invalid",
            "  chain_type: Hand",
            "  pose:",
            "    position:",
            f"      x: {px:.6f}",
            f"      y: {py:.6f}",
            f"      z: {pz:.6f}",
            "    orientation:",
            f"      x: {qx:.6f}",
            f"      y: {qy:.6f}",
            f"      z: {qz:.6f}",
            f"      w: {qw:.6f}",
        ]
    out += [f"ergonomics_count: {len(ergo)}", "ergonomics:"]
    for name, value in ergo.items():
        out += [f"- type: {name}", f"  value: {value:.4f}"]
    out += ["raw_sensor_orientation:", "  x: 0.0", "  y: 0.0", "  z: 0.0", "  w: 1.0", "---"]
    return out


class FakeBridge:
    def __init__(self, port, rate_hz):
        self.port = port
        self.period = 1.0 / rate_hz
        self.clients = []
        self.lock = threading.Lock()

    def accept_loop(self, server):
        while True:
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                self.clients.append(conn)
                n = len(self.clients)
            print(f"[fake_bridge] client connected from {addr[0]} ({n} total)", flush=True)

    def broadcast(self, data):
        with self.lock:
            for conn in list(self.clients):
                try:
                    conn.sendall(data)
                except OSError:
                    conn.close()
                    self.clients.remove(conn)

    def serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", self.port))
        server.listen(8)
        print(f"[fake_bridge] listening on port {self.port}", flush=True)
        threading.Thread(target=self.accept_loop, args=(server,), daemon=True).start()

        start = time.monotonic()
        next_tick = start
        while True:
            t = time.monotonic() - start
            lines = []
            for prefix, glove_id, side in (("GLOVE0:", 1, "Left"), ("GLOVE1:", 2, "Right")):
                lines += [prefix + line for line in glove_message(glove_id, side, t)]
            self.broadcast(("\n".join(lines) + "\n").encode())
            next_tick += self.period
            time.sleep(max(0.0, next_tick - time.monotonic()))


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 9090
    rate_hz = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    try:
        FakeBridge(port, rate_hz).serve()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
