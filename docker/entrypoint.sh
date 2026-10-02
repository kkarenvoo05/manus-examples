#!/bin/bash
set -e

fix_usb_permissions() {
    # Hotplugged USB/hidraw nodes inherit host permissions, not container udev rules.
    sudo chmod 666 /dev/bus/usb/*/* 2>/dev/null || true
    sudo chmod 666 /dev/hidraw* 2>/dev/null || true
}

fix_usb_permissions

# Keep permissions in sync for devices plugged in after the container starts.
(
    while true; do
        fix_usb_permissions
        sleep 2
    done
) &

source /opt/ros/humble/setup.bash
source /opt/ros2_ws/install/setup.bash

# Start manus_data_publisher in background
ros2 run manus_ros2 manus_data_publisher &
echo "Starting glove bridge."
glove_bridge &

exec "$@"
