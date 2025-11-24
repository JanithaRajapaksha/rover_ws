#!/bin/bash
# ==============================================
# Script: start_rover.sh
# Purpose: Start main ROS2 launch inside Docker container
# Only launches ROS2 if container was not running
# ==============================================

CONTAINER_NAME="ros_humble"
WORKSPACE_PATH="/rover/rover_ws"
PACKAGE_NAME="rover1"
MAIN_LAUNCH="full_robot.launch.py"

# Step 1: Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "🔹 Container not running. Starting it and launching ROS2..."
    docker start ${CONTAINER_NAME}

    # Step 2: Execute ROS2 launch only if container was just started
    docker exec -d ${CONTAINER_NAME} bash -c "
        cd ${WORKSPACE_PATH} && \
        source install/setup.bash && \
        echo '✅ Environment sourced, launching main ROS2...' && \
        ros2 launch ${PACKAGE_NAME} ${MAIN_LAUNCH}
    "
else
    echo "🔹 Container already running. Skipping ROS2 launch."
fi
