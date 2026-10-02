#!/usr/bin/env python3
"""
glove_web.py — Connects to glove_bridge over TCP, parses YAML,
and pushes joint data to the browser via WebSocket.

Usage: python3 web/glove_web.py [bridge_host [bridge_port [ws_port]]]
  bridge_host: default "127.0.0.1"
  bridge_port: default 9090
  ws_port:     default 8765
"""

import sys
import os
import json
import socket
import signal
import time
import threading
import asyncio
import http.server
import functools

import websockets


def parse_glove_yaml(text):
    """Parse YAML glove message into (side, keypoints_list).

    Returns (side, keypoints) where side is 'Left'/'Right' and keypoints is
    a list of 25 entries, each [x, y, z, qw, qx, qy, qz].
    """
    side = None
    keypoints = [[0.0] * 7 for _ in range(25)]

    lines = text.strip().split('\n')
    for line in lines:
        s = line.strip()
        if s.startswith("side:"):
            side = s.split(":", 1)[1].strip()
            break

    if side is None:
        return None, None

    node_id = -1
    in_pos = False
    in_ori = False
    px = py = pz = 0.0
    ox = oy = oz = 0.0
    ow = 1.0

    for line in lines:
        s = line.strip()

        if s.startswith("- node_id:"):
            if 0 <= node_id < 25:
                keypoints[node_id] = [px, py, pz, ow, ox, oy, oz]
            node_id = int(s.split(":", 1)[1].strip())
            in_pos = in_ori = False
            px = py = pz = 0.0
            ox = oy = oz = 0.0
            ow = 1.0
            continue

        if node_id < 0:
            continue

        if s == "position:":
            in_pos, in_ori = True, False
            continue
        elif s == "orientation:":
            in_pos, in_ori = False, True
            continue

        if s.startswith(("parent_node_id:", "joint_type:", "chain_type:", "pose:")):
            in_pos = in_ori = False
            continue

        if in_pos:
            if s.startswith("x:"):   px = float(s.split(":", 1)[1])
            elif s.startswith("y:"): py = float(s.split(":", 1)[1])
            elif s.startswith("z:"): pz = float(s.split(":", 1)[1])
        elif in_ori:
            if s.startswith("x:"):   ox = float(s.split(":", 1)[1])
            elif s.startswith("y:"): oy = float(s.split(":", 1)[1])
            elif s.startswith("z:"): oz = float(s.split(":", 1)[1])
            elif s.startswith("w:"): ow = float(s.split(":", 1)[1])

        if s.startswith(("ergonomics_count:", "ergonomics:")):
            break

    if 0 <= node_id < 25:
        keypoints[node_id] = [px, py, pz, ow, ox, oy, oz]

    return side, keypoints


class GloveBridgeClient:
    """TCP client that connects to glove_bridge and parses YAML into keypoints."""

    def __init__(self, host="127.0.0.1", port=9090):
        self.host = host
        self.port = port
        self.running = True
        self.lock = threading.Lock()
        self.left = None
        self.right = None

    def connect(self):
        for i in range(30):
            if not self.running:
                return None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.host, self.port))
                print(f"Connected to bridge at {self.host}:{self.port}")
                return sock
            except ConnectionRefusedError:
                if i == 0:
                    print(f"Waiting for bridge at {self.host}:{self.port}...")
                sock.close()
                time.sleep(1)
        return None

    def run(self, sock):
        buf = ""
        msg_buf = {0: "", 1: ""}
        try:
            while self.running:
                data = sock.recv(65536)
                if not data:
                    print("Bridge disconnected.")
                    break
                buf += data.decode("utf-8", errors="replace")
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    gi = -1
                    payload = line
                    if line.startswith("GLOVE0:"):
                        gi, payload = 0, line[7:]
                    elif line.startswith("GLOVE1:"):
                        gi, payload = 1, line[7:]
                    if gi < 0:
                        continue
                    if payload == "---":
                        if msg_buf[gi]:
                            side, kp = parse_glove_yaml(msg_buf[gi])
                            if side is not None:
                                with self.lock:
                                    if side == "Left":
                                        self.left = kp
                                    else:
                                        self.right = kp
                        msg_buf[gi] = ""
                    else:
                        msg_buf[gi] += payload + "\n"
        except Exception as e:
            if self.running:
                print(f"Recv error: {e}")
        finally:
            self.running = False

    def snapshot(self):
        with self.lock:
            return self.left, self.right


async def ws_handler(websocket, client):
    """Push latest glove data to each connected browser at ~30Hz."""
    try:
        while client.running:
            left, right = client.snapshot()
            msg = {}
            if left is not None:
                msg["left"] = left
            if right is not None:
                msg["right"] = right
            if msg:
                await websocket.send(json.dumps(msg))
            await asyncio.sleep(0.033)
    except websockets.exceptions.ConnectionClosed:
        pass


def start_http_server(directory, port):
    """Serve index.html on the given port."""
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=directory)
    httpd = http.server.HTTPServer(("0.0.0.0", port), handler)
    httpd.serve_forever()


def main():
    bridge_host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    bridge_port = int(sys.argv[2]) if len(sys.argv) > 2 else 9090
    ws_port = int(sys.argv[3]) if len(sys.argv) > 3 else 8765
    http_port = 8000

    client = GloveBridgeClient(bridge_host, bridge_port)

    def sig_handler(sig, frame):
        client.running = False

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    # Connect to bridge
    sock = client.connect()
    if sock is None:
        print("Could not connect to bridge.")
        return

    # Start TCP recv thread
    tcp_thread = threading.Thread(target=client.run, args=(sock,), daemon=True)
    tcp_thread.start()

    # Start HTTP server for index.html
    script_dir = os.path.dirname(os.path.abspath(__file__))
    http_thread = threading.Thread(
        target=start_http_server, args=(script_dir, http_port), daemon=True
    )
    http_thread.start()
    print(f"Open http://localhost:{http_port} in your browser")

    # Run WebSocket server on main thread
    print(f"WebSocket server on ws://localhost:{ws_port}")

    async def serve():
        async with websockets.serve(
            lambda ws: ws_handler(ws, client), "0.0.0.0", ws_port
        ):
            while client.running:
                await asyncio.sleep(0.1)

    asyncio.run(serve())

    sock.close()
    tcp_thread.join(timeout=2)
    print("Done.")


if __name__ == "__main__":
    main()
