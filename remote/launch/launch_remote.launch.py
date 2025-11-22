("""Launch file to run the websocket reset server and the mode controller node.

This file launches two processes using ExecuteProcess so it works when running
from the workspace (uninstalled) as well as from an installed package.
""")

from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
	this_dir = Path(__file__).resolve().parent
	pkg_root = this_dir.parent

	ws_script = str(pkg_root / 'tools' / 'ws_reset_server.py')
	mode_script = str(pkg_root / 'remote' / 'mode_controller.py')

	ws_proc = ExecuteProcess(
		cmd=['python3', ws_script],
		name='ws_reset_server',
		output='screen',
		emulate_tty=True,
	)

	mode_proc = ExecuteProcess(
		cmd=['python3', mode_script],
		name='mode_controller',
		output='screen',
		emulate_tty=True,
	)

	return LaunchDescription([
		ws_proc,
		mode_proc,
	])

