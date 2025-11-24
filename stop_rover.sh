#!/bin/bash
# ==============================================
# Script: stop_rover.sh
# Purpose: Stop all ROS2 nodes inside the container
# ==============================================

CONTAINER_NAME="main_rover"

echo "🛑 Stopping all ROS2 nodes inside container ${CONTAINER_NAME}..."

# Kill any ros2 node processes
docker exec ${CONTAINER_NAME} bash -c "pkill -f controller_manager"
docker exec ${CONTAINER_NAME} bash -c "pkill -f robot_state_publisher"
docker exec ${CONTAINER_NAME} bash -c "pkill -f joint_broad"
docker exec ${CONTAINER_NAME} bash -c "pkill -f diff_cont"
docker exec ${CONTAINER_NAME} bash -c "pkill -f twist_mux"


echo "✅ All ROS2 nodes stopped."
