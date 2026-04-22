#!/bin/bash

# JetRacer ROS 2 Bootstrap Script
# Starts the Sentinel node for controller-driven mission launching.

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROS_DISTRO="humble" # Default to humble, adjust if needed

echo "--- JetRacer Bootstrap Initializing ---"

# 1. Source ROS 2 Base
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
else
    echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found."
    exit 1
fi

# 2. Source Local Workspace
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
else
    echo "WARNING: Local workspace setup.bash not found. Did you run 'colcon build'?"
fi

# 3. Hardware check
if [ ! -c "/dev/input/js0" ]; then
    echo "CRITICAL: Gamepad not found at /dev/input/js0"
    # We continue anyway so the OLED can show the fault
fi

# 4. Launch Sentinel
export PYTHONUNBUFFERED=1
ros2 launch jetracer_bringup sentinel.launch.py
