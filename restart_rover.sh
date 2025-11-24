#!/bin/bash
# ==============================================
# Script: restart_rover.sh
# Purpose: Restart ROS2 nodes inside container
# Loops kill commands to ensure all nodes are stopped
# ==============================================

CONTAINER_NAME="main_rover"
WORKSPACE_PATH="/rover/rover_ws"
PACKAGE_NAME="rover1"
MAIN_LAUNCH="launch_robot.launch.py"

echo "🔄 Restarting ROS2 nodes inside container ${CONTAINER_NAME}..."

# # Loop 3 times to ensure nodes are stopped
# for i in {1..3}; do
#     echo "🛑 Killing ROS2 nodes (attempt $i)..."
#     docker exec ${CONTAINER_NAME} pkill -f "ros2"
#     docker exec ${CONTAINER_NAME} pkill -f "robot_state_publisher"
#     docker exec ${CONTAINER_NAME} pkill -f "twist_mux"
#     sleep 0.5
# done

# # Wait until all nodes are fully stopped
# echo "⏳ Waiting for all ROS2 nodes to stop..."
# while docker exec ${CONTAINER_NAME} pgrep -f "ros2|robot_state_publisher|twist_mux" >/dev/null; do
#     sleep 0.5
# done

# Kill any ros2 node processes
docker exec ${CONTAINER_NAME} bash -c "pkill -f controller_manager"
docker exec ${CONTAINER_NAME} bash -c "pkill -f robot_state_publisher"
docker exec ${CONTAINER_NAME} bash -c "pkill -f joint_broad"
docker exec ${CONTAINER_NAME} bash -c "pkill -f diff_cont"
docker exec ${CONTAINER_NAME} bash -c "pkill -f twist_mux"

echo "✅ All ROS2 nodes stopped. Relaunching launch file..."

# Relaunch main launch file
docker exec -d ${CONTAINER_NAME} bash -c "
    cd ${WORKSPACE_PATH} && \
    source install/setup.bash && \
    echo '✅ Environment sourced, launching main ROS2...' && \
    ros2 launch ${PACKAGE_NAME} ${MAIN_LAUNCH}
"

echo "✅ ROS2 nodes restarted successfully."
