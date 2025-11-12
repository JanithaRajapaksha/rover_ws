#!/usr/bin/env python3
"""Launch file to start the UWB + ToF nodes used by the "come to me" demo.

Starts these nodes:
 - `uwb/pose_pub.py`       -> uwb reader publishing `/uwb_distance`
 - `uwb/direction_tester.py` -> test controller that publishes `/cmd_vel_scaled`
 - `uwb/come_to_me_mux.py` -> command multiplexer publishing `/cmd_vel_tracking`
 - `camera/obstacle_avoidance.py` -> ToF PID obstacle avoidance publishing `/cmd_vel_tof` and `/tof_distances`

Notes:
 - This launch assumes the Python nodes are installed as executables in their packages
   (typical after `colcon build` / `pip install` via `ament_python`).
 - If you run from source without installing, either install the packages or adapt the
   launch to use `ExecuteProcess` with the direct script paths.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments (tweak defaults as needed)
    uwb_port_arg = DeclareLaunchArgument('uwb_port', default_value='/dev/ttyUSB0', description='Serial port for UWB')
    uwb_baud_arg = DeclareLaunchArgument('uwb_baud', default_value='115200', description='Baudrate for UWB')
    uwb_topic_arg = DeclareLaunchArgument('uwb_topic', default_value='/uwb_distance', description='UWB publish topic')

    mux_threshold_arg = DeclareLaunchArgument('obstacle_threshold_mm', default_value='500.0', description='ToF threshold in mm for obstacle mux')
    mux_max_angular_arg = DeclareLaunchArgument('max_angular_z', default_value='0.2', description='Max angular z used by mux/avoidance')
    scaled_timeout_arg = DeclareLaunchArgument('scaled_timeout', default_value='2.0', description='Timeout for scaled cmd_vel')

    # Nodes
    pose_pub_node = Node(
        package='uwb',
        executable='pose_pub.py',
        name='uwb_reader',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('uwb_port'),
            'baudrate': LaunchConfiguration('uwb_baud'),
            'publish_topic': LaunchConfiguration('uwb_topic')
        }]
    )

    direction_tester_node = Node(
        package='uwb',
        executable='direction_tester.py',
        name='direction_tester',
        output='screen'
    )

    cmd_mux_node = Node(
        package='uwb',
        executable='come_to_me_mux.py',
        name='cmd_mux_node',
        output='screen',
        parameters=[{
            'obstacle_threshold_mm': LaunchConfiguration('obstacle_threshold_mm'),
            'max_angular_z': LaunchConfiguration('max_angular_z'),
            'scaled_timeout': LaunchConfiguration('scaled_timeout')
        }]
    )

    tof_pid_node = Node(
        package='camera',
        executable='obstacle_avoidance.py',
        name='tof_pid_node',
        output='screen'
    )

    ld = LaunchDescription()
    # declare args
    ld.add_action(uwb_port_arg)
    ld.add_action(uwb_baud_arg)
    ld.add_action(uwb_topic_arg)
    ld.add_action(mux_threshold_arg)
    ld.add_action(mux_max_angular_arg)
    ld.add_action(scaled_timeout_arg)

    # add nodes
    ld.add_action(pose_pub_node)
    ld.add_action(direction_tester_node)
    ld.add_action(cmd_mux_node)
    ld.add_action(tof_pid_node)

    return ld
