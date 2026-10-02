"""Helpers for managing the Manus glove docker bridge from Python entrypoints.

The bridge is a docker-compose stack that runs the Manus SDK + a TCP relay on
host port 9090. Multiple launchers need to ensure it's up before they
start consuming glove data, so the start/teardown logic lives here.
"""

from __future__ import annotations

import atexit
import socket
import subprocess
import time
from pathlib import Path


DEFAULT_COMPOSE_DIR = Path(__file__).resolve().parent.parent / "docker"


def start_manus_bridge(
    compose_dir: str | Path = DEFAULT_COMPOSE_DIR,
    host: str = "127.0.0.1",
    port: int = 9090,
    start_timeout: float = 30.0,
) -> None:
    """Down→up the docker bridge stack, wait for the port to accept
    connections, and register an atexit hook to bring it back down."""
    compose_file = Path(compose_dir).expanduser().resolve() / "docker-compose.yml"
    if not compose_file.exists():
        raise SystemExit(f"Docker compose file not found: {compose_file}")

    def _compose(*cargs: str) -> None:
        subprocess.run(["docker", "compose", "-f", str(compose_file), *cargs], check=True)

    _compose("down", "-t", "1")
    _compose("up", "-d", "--no-build", "--pull", "never")

    deadline = time.monotonic() + start_timeout
    while time.monotonic() < deadline:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(0.5)
            if sock.connect_ex((host, port)) == 0:
                atexit.register(lambda: _compose("down", "-t", "1"))
                return
        time.sleep(0.25)
    raise TimeoutError(f"Timed out waiting for Manus bridge at {host}:{port}")
