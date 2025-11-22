#!/bin/bash
# ==============================================
# Script: stop_rover.sh
# Purpose: Stop all ROS2 nodes inside the container
# ==============================================

CONTAINER_NAME="main_rover"

echo "🛑 Stopping all ROS2 nodes inside container ${CONTAINER_NAME}..."

# Kill any ros2 launch processes
docker exec ${CONTAINER_NAME} pkill -f "ros2"

# Kill robot_state_publisher and twist_mux
docker exec ${CONTAINER_NAME} pkill -f "robot_state_publisher"
docker exec ${CONTAINER_NAME} pkill -f "twist_mux"

echo "✅ All ROS2 nodes stopped."
