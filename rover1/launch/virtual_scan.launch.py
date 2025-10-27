# file: virtual_scan_launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '/virtual_scan',
                'sensor_msgs/msg/LaserScan',
                '{"header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "base_link"},'
                ' "angle_min": 0.0, "angle_max": 6.283185307, "angle_increment": 0.785398163,'
                ' "time_increment": 0.0, "scan_time": 0.1, "range_min": 0.01, "range_max": 2.0,'
                ' "ranges": [0.6,0.6,0.6,0.6,0.6,0.6,0.6,0.6],'
                ' "intensities": [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]}',
                '-r', '100'
            ],
            output='screen'
        )
    ])
