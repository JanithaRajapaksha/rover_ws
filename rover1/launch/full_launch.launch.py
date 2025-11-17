"""Combined launch file that includes three other launch files.

Included launches:
 - rover1/launch/launch_robot.launch.py
 - camera/launch/track_and_follow_all.launch.py
 - uwb/launch/come_to_me_all.launch.py

Run with:
  ros2 launch rover1 full_launch.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	pkg_rover1 = get_package_share_directory('rover1')
	pkg_camera = get_package_share_directory('camera')
	pkg_uwb = get_package_share_directory('uwb')

	launch_files = [
		os.path.join(pkg_rover1, 'launch', 'launch_robot.launch.py'),
		os.path.join(pkg_camera, 'launch', 'track_and_follow_all.launch.py'),
		os.path.join(pkg_uwb, 'launch', 'come_to_me_all.launch.py'),
	]

	actions = []
	for lf in launch_files:
		actions.append(LogInfo(msg=f"Including launch file: {lf}"))
		actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(lf)))

	return LaunchDescription(actions)

