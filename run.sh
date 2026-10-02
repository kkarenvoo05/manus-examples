#!/usr/bin/env bash
# Entry point for the Manus glove examples. Run `./run.sh help` for usage.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$ROOT/docker/docker-compose.yml"
BRIDGE_PORT="${BRIDGE_PORT:-9090}"

usage() {
    cat <<EOF
Usage: ./run.sh <command> [args]

  web [--fake] [bridge_host]    Browser glove viewer at http://localhost:8000
                                  --fake       use tools/fake_bridge.py instead of real gloves
                                  bridge_host  machine running the bridge (default 127.0.0.1)
  bridge [up|down|logs|build]   Manage the Manus SDK + glove_bridge container (default: up)
  fake [port [rate_hz]]         Serve a synthetic two-hand glove stream (default :9090, 30 Hz)

Env: BRIDGE_PORT (default 9090)
EOF
}

port_open() {
    python3 - "$1" "$2" <<'EOF'
import socket, sys
try:
    socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=0.5).close()
except OSError:
    sys.exit(1)
EOF
}

wait_for_bridge() {
    local host="$1" timeout="$2" deadline=$((SECONDS + $2))
    while ((SECONDS < deadline)); do
        port_open "$host" "$BRIDGE_PORT" && return 0
        sleep 0.25
    done
    echo "Timed out after ${timeout}s waiting for glove bridge at $host:$BRIDGE_PORT" >&2
    return 1
}

compose() {
    docker compose -f "$COMPOSE_FILE" "$@"
}

bridge() {
    local action="${1:-up}"
    if ! command -v docker >/dev/null; then
        echo "docker not found. The bridge must run on the Linux machine the gloves are plugged into;" >&2
        echo "pass that machine's hostname to './run.sh web <host>', or use './run.sh web --fake'." >&2
        exit 1
    fi
    case "$action" in
        up)
            compose up -d
            wait_for_bridge 127.0.0.1 30
            echo "Bridge up on :$BRIDGE_PORT"
            ;;
        down)  compose down -t 1 ;;
        logs)  compose logs -f ;;
        build) compose build ;;
        *) echo "Unknown bridge action: $action" >&2; exit 1 ;;
    esac
}

web() {
    local fake=0 host="127.0.0.1" arg
    for arg in "$@"; do
        case "$arg" in
            --fake) fake=1 ;;
            -h|--help) usage; exit 0 ;;
            *) host="$arg" ;;
        esac
    done

    if ((fake)); then
        if port_open 127.0.0.1 "$BRIDGE_PORT"; then
            echo "Port $BRIDGE_PORT is already in use; stop whatever is running there first." >&2
            exit 1
        fi
        host="127.0.0.1"
        python3 "$ROOT/tools/fake_bridge.py" "$BRIDGE_PORT" &
        FAKE_PID=$!
        trap 'kill "$FAKE_PID" 2>/dev/null || true' EXIT
        wait_for_bridge "$host" 10
    elif ! port_open "$host" "$BRIDGE_PORT"; then
        if [[ "$host" == "127.0.0.1" || "$host" == "localhost" ]] && command -v docker >/dev/null; then
            echo "No bridge on :$BRIDGE_PORT, starting docker bridge..."
            bridge up
        else
            echo "Glove bridge not reachable at $host:$BRIDGE_PORT." >&2
            echo "Start it with './run.sh bridge' on the glove machine, or try './run.sh web --fake'." >&2
            exit 1
        fi
    fi

    cd "$ROOT"
    uv run --quiet python web/glove_web.py "$host" "$BRIDGE_PORT"
}

cmd="${1:-help}"
shift || true
case "$cmd" in
    web)    web "$@" ;;
    bridge) bridge "$@" ;;
    fake)   python3 "$ROOT/tools/fake_bridge.py" "$@" ;;
    help|-h|--help) usage ;;
    *) echo "Unknown command: $cmd" >&2; usage >&2; exit 1 ;;
esac
