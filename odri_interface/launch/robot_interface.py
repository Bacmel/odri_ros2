import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory('odri_interface'), 'config', 'params.yaml')

    node = Node(package='odri_interface',
                name='robot_interface',
                executable='robot_interface',
                output='screen',
                emulate_tty=True,
                parameters=[config])
    ld.add_action(node)

    return ld
