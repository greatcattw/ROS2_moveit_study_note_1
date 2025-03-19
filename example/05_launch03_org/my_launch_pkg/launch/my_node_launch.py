from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='my_launch_pkg',
			executable='exe_pkg_03',
			#name='my_node',
			output='screen',
		),
	])
