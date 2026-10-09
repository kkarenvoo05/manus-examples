#!/usr/bin/env bash
# Record one glove take with everything needed to explain gaps and twitches:
# glove topics + /manus_telemetry in one bag, the kernel log (USB/dongle
# disconnects), the publisher log and a snapshot of the host. Run on the glove
# machine from the repo, after docker/publisher_telemetry/install_in_container.sh:
#
#   tools/record_glove_take.sh take_001 [container]
#
# Stop with Ctrl+C. Output: ~/manus-recordings/<take>/ (OUT_ROOT to change).
# Then: python3 tools/diagnose_glove_telemetry.py ~/manus-recordings/<take>/bag \
#           --kernel-log ~/manus-recordings/<take>/kernel.log
#
# Env: DOCKER (default "sudo docker"), WS (default /opt/ros2_ws),
#      CAPTURE_SYNC=<session_id> to also run tools/record_capture_sync.py,
#      ALLOW_NO_TELEMETRY=1 to record with a stock (unpatched) publisher.
set -euo pipefail

TAKE="${1:?usage: $0 <take_name> [container]}"
CONTAINER="${2:-${MANUS_CONTAINER:-manus-bridge-arm64}}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
read -r -a DOCKER <<< "${DOCKER:-sudo docker}"
WS="${WS:-/opt/ros2_ws}"
OUT="${OUT_ROOT:-$HOME/manus-recordings}/$TAKE"
IN="/tmp/manus_recordings/$TAKE"
ROS="source /opt/ros/humble/setup.bash && source $WS/install/setup.bash"
TOPICS=(/manus_glove_0 /manus_glove_1 /manus_telemetry)

if [[ -e "$OUT" ]]; then
    echo "$OUT already exists; pick a new take name." >&2
    exit 1
fi
if ! "${DOCKER[@]}" exec "$CONTAINER" bash -c "$ROS && ros2 topic list" > /tmp/manus_topics.$$ 2>&1; then
    echo "Cannot list ROS topics in container '$CONTAINER':" >&2
    cat /tmp/manus_topics.$$ >&2
    exit 1
fi
for topic in /manus_glove_0 /manus_glove_1; do
    grep -qx "$topic" /tmp/manus_topics.$$ || echo "WARNING: $topic is not being published (glove off or unpaired?)" >&2
done
if ! grep -qx /manus_telemetry /tmp/manus_topics.$$; then
    if [[ "${ALLOW_NO_TELEMETRY:-0}" != 1 ]]; then
        echo "/manus_telemetry is not being published. Run docker/publisher_telemetry/install_in_container.sh" >&2
        echo "first, or set ALLOW_NO_TELEMETRY=1 to record without it." >&2
        exit 1
    fi
    TOPICS=(/manus_glove_0 /manus_glove_1)
fi
rm -f /tmp/manus_topics.$$
mkdir -p "$OUT"

# Host snapshot: what was plugged in and running.
{
    echo "take: $TAKE"
    echo "start_local: $(date --iso-8601=ns)"
    echo "start_unix_ns: $(date +%s%N)"
    echo "host: $(hostname)  kernel: $(uname -r)"
    echo "container: $CONTAINER  image: $("${DOCKER[@]}" inspect -f '{{.Config.Image}}' "$CONTAINER" 2>/dev/null)"
    echo "repo commit: $(git -C "$ROOT" rev-parse HEAD 2>/dev/null) $(git -C "$ROOT" status --porcelain 2>/dev/null | wc -l) uncommitted"
    echo "topics recorded: ${TOPICS[*]}"
    echo; echo "== processes in container (glove_bridge / topic echo add CPU load)"
    "${DOCKER[@]}" exec "$CONTAINER" bash -c "ps -eo pid,pcpu,etime,args | grep -E 'manus|glove_bridge|ros2' | grep -v grep" || true
    echo; echo "== lsusb -t"; lsusb -t 2>&1 || true
    echo; echo "== MANUS USB devices (vendor 3325)"; lsusb -d 3325: 2>&1 || true
    echo; echo "== load"; uptime
} > "$OUT/host_snapshot.txt"

# Kernel log: USB disconnect/reset lines are direct evidence of dongle dropouts.
KERNEL_PID=""
if sudo -n true 2>/dev/null || sudo -v; then
    sudo dmesg --follow --time-format iso > "$OUT/kernel.log" 2>&1 &
    KERNEL_PID=$!
else
    echo "WARNING: no sudo; kernel log (USB disconnects) not recorded." >&2
fi

SYNC_PID=""
if [[ -n "${CAPTURE_SYNC:-}" ]]; then
    "${DOCKER[@]}" exec -u root "$CONTAINER" mkdir -p /opt/manus_capture/tools /opt/manus_capture/manus_glove
    "${DOCKER[@]}" cp "$ROOT/tools/record_capture_sync.py" "$CONTAINER:/opt/manus_capture/tools/"
    "${DOCKER[@]}" cp "$ROOT/manus_glove/capture_sync.py" "$CONTAINER:/opt/manus_capture/manus_glove/"
    "${DOCKER[@]}" exec "$CONTAINER" bash -c "$ROS && exec python3 /opt/manus_capture/tools/record_capture_sync.py \
        --output $IN/capture_sync --session-id '$CAPTURE_SYNC' --take-id '$TAKE' --topics ${TOPICS[*]}" \
        > "$OUT/capture_sync.log" 2>&1 &
    SYNC_PID=$!
fi

START_ISO="$(date --iso-8601=seconds)"
cleanup() {
    trap - INT TERM
    [[ -n "$SYNC_PID" ]] && { "${DOCKER[@]}" exec "$CONTAINER" pkill -INT -f "[r]ecord_capture_sync.py" || true; wait "$SYNC_PID" || true; }
    [[ -n "$KERNEL_PID" ]] && { sudo kill "$KERNEL_PID" 2>/dev/null || kill "$KERNEL_PID" 2>/dev/null || true; }
    echo "end_unix_ns: $(date +%s%N)" >> "$OUT/host_snapshot.txt"
    echo "Copying recording out of the container ..."
    "${DOCKER[@]}" cp "$CONTAINER:$IN/bag" "$OUT/bag"
    [[ -n "$SYNC_PID" ]] && "${DOCKER[@]}" cp "$CONTAINER:$IN/capture_sync" "$OUT/capture_sync" || true
    "${DOCKER[@]}" exec "$CONTAINER" bash -c "cat /tmp/manus_data_publisher.log 2>/dev/null" > "$OUT/publisher.log" || true
    "${DOCKER[@]}" logs --since "$START_ISO" "$CONTAINER" > "$OUT/container.log" 2>&1 || true
    [[ "${DOCKER[0]}" == sudo ]] && sudo chown -R "$(id -u):$(id -g)" "$OUT" || true  # docker cp ran as root
    "${DOCKER[@]}" exec "$CONTAINER" bash -c "$ROS && ros2 bag info $IN/bag" | tee "$OUT/bag_info.txt" || true
    echo
    echo "Saved $OUT"
    echo "Analyse with:"
    echo "  python3 tools/diagnose_glove_telemetry.py $OUT/bag --kernel-log $OUT/kernel.log"
}
trap cleanup EXIT

echo "Recording ${TOPICS[*]} to $OUT. Press Ctrl+C to stop."
"${DOCKER[@]}" exec -it "$CONTAINER" bash -c "$ROS && mkdir -p $IN && exec ros2 bag record -o $IN/bag ${TOPICS[*]}" || true
