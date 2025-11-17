#!/usr/bin/env python3
"""Launch the person tracker, ToF obstacle avoidance, and the command mux.

This starts three nodes from the `camera` package:
 - `track_and_follow.py` (publishes cmd_vel_mp)
 - `obstacle_avoidance.py` (publishes cmd_vel_tof and /tof_distances)
 - `track_and_follow_with_obs_avoidance.py` (mux -> cmd_vel_tracking)

Usage:
  ros2 launch camera track_and_follow_all.launch.py

You can override the obstacle threshold (mm) with the launch argument
`obstacle_threshold_mm` (default 500.0).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    obstacle_threshold = LaunchConfiguration('obstacle_threshold_mm', default='500.0')
    log_level = LaunchConfiguration('log_level', default='WARN')

    declare_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold_mm', default_value='500.0', description='ToF obstacle threshold in mm'
    )
    declare_log_arg = DeclareLaunchArgument(
        'log_level', default_value='WARN', description='ROS logger level for launched nodes (DEBUG|INFO|WARN|ERROR|FATAL)'
    )

    # Nodes to launch
    tracker_node = Node(
        package='camera',
        executable='track_and_follow.py',
        name='person_tracker_pid_node',
        output='screen',
        # pass ros arguments to set the node log level
        arguments=['--ros-args', '--log-level', log_level],
    )

    tof_node = Node(
        package='camera',
        executable='obstacle_avoidace.py',
        name='tof_pid_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
    )

    camera_node = Node(
        package='camera',
        executable='obstacle_avoidance_camera.py',
        name='camera_publisher_node',
        output='screen',)

    mux_node = Node(
        package='camera',
        executable='track_and_follow_with_obs_avoidance.py',
        name='cmd_mux_node',
        output='screen',
        parameters=[{'obstacle_threshold_mm': obstacle_threshold}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    ld = LaunchDescription()
    ld.add_action(declare_threshold_arg)
    ld.add_action(declare_log_arg)
    ld.add_action(tracker_node)
    ld.add_action(tof_node)
    ld.add_action(mux_node)

    return ld
