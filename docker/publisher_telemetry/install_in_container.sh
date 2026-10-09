#!/usr/bin/env bash
# Patch, rebuild and restart manus_data_publisher inside a running container so
# it also publishes /manus_telemetry. Run on the glove machine from the repo:
#
#   docker/publisher_telemetry/install_in_container.sh [container]
#
# Re-run after pulling updates or after the container is recreated. Undo with
#   REVERT=1 docker/publisher_telemetry/install_in_container.sh [container]
#
# Env: DOCKER (default "sudo docker"), WS (ROS workspace in the container,
# default /opt/ros2_ws), EXEC_USER (container user for the build, default the
# container's default user), RESTART=0 to skip restarting the publisher.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${1:-${MANUS_CONTAINER:-manus-bridge-arm64}}"
read -r -a DOCKER <<< "${DOCKER:-sudo docker}"
WS="${WS:-/opt/ros2_ws}"
STAGE=/tmp/manus_publisher_telemetry
USER_ARGS=()
[[ -n "${EXEC_USER:-}" ]] && USER_ARGS=(-u "$EXEC_USER")

in_container() {
    "${DOCKER[@]}" exec "${USER_ARGS[@]}" "$CONTAINER" bash -c "$1"
}

if ! "${DOCKER[@]}" inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
    echo "Container '$CONTAINER' is not running. Containers on this machine:" >&2
    "${DOCKER[@]}" ps -a --format '  {{.Names}}  [{{.Status}}]  ({{.Image}})' >&2 || true
    echo "Start a stopped one with '${DOCKER[*]} start <name>', or the repo's bridge with" >&2
    echo "'./run.sh bridge' (container rllg2-manus-bridge), then run: $0 <name>" >&2
    exit 1
fi

PKG="$WS/src/manus_ros2"
if ! in_container "test -f '$PKG/src/ManusDataPublisher.cpp'"; then
    echo "No manus_ros2 package at $PKG in $CONTAINER. Set WS to the workspace that contains src/manus_ros2." >&2
    in_container "find / -path /proc -prune -o -name ManusDataPublisher.cpp -print 2>/dev/null | grep -v /build/" >&2 || true
    exit 1
fi

"${DOCKER[@]}" exec -u root "$CONTAINER" bash -c "rm -rf $STAGE && mkdir -p $STAGE && chmod 777 $STAGE"
for f in ManusTelemetry.hpp ManusTelemetry.cpp apply_patch.py; do
    "${DOCKER[@]}" cp "$HERE/$f" "$CONTAINER:$STAGE/$f"
done
"${DOCKER[@]}" exec -u root "$CONTAINER" chmod -R a+rX "$STAGE"

if [[ "${REVERT:-0}" == 1 ]]; then
    in_container "python3 $STAGE/apply_patch.py '$PKG' --revert"
else
    in_container "python3 $STAGE/apply_patch.py '$PKG'"
fi

echo "Building manus_ros2 in $WS ..."
in_container "source /opt/ros/humble/setup.bash && cd '$WS' && colcon build --packages-select manus_ros2"

if [[ "${RESTART:-1}" != 1 ]]; then
    echo "Built. Restart manus_data_publisher yourself to load it."
    exit 0
fi

echo "Restarting manus_data_publisher ..."
"${DOCKER[@]}" exec -u root "$CONTAINER" bash -c "pkill -INT -f '[l]ib/manus_ros2/manus_data_publisher' || true; sleep 2; pkill -KILL -f '[l]ib/manus_ros2/manus_data_publisher' || true"
"${DOCKER[@]}" exec -d "${USER_ARGS[@]}" "$CONTAINER" bash -c \
    "source /opt/ros/humble/setup.bash && source '$WS/install/setup.bash' && exec ros2 run manus_ros2 manus_data_publisher > /tmp/manus_data_publisher.log 2>&1"

if [[ "${REVERT:-0}" == 1 ]]; then
    echo "Reverted to the stock publisher (log: /tmp/manus_data_publisher.log in the container)."
    exit 0
fi

echo "Waiting for /manus_telemetry ..."
if in_container "source /opt/ros/humble/setup.bash && source '$WS/install/setup.bash' && \
        timeout 30 ros2 topic echo --once --field data /manus_telemetry | head -c 400"; then
    echo
    echo "OK: /manus_telemetry is publishing. Publisher log: /tmp/manus_data_publisher.log in the container."
else
    echo "No /manus_telemetry message within 30 s. Check the publisher log:" >&2
    in_container "tail -20 /tmp/manus_data_publisher.log" >&2 || true
    exit 1
fi
