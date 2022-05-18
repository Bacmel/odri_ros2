import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    odri_iface_config = os.path.join(get_package_share_directory('odri_interface'), 'config', 'params.yaml')

    odri_iface_node = Node(package='odri_interface',
                           name='robot_interface',
                           executable='robot_interface',
                           parameters=[odri_iface_config],
                           output='screen',
                           remappings=[("robot_state", "/odri/robot_state"), ("robot_command", "/odri/robot_command")])

    odri_example_node = Node(package='odri_examples',
                             name='example_robot',
                             executable='example_robot',
                             output='screen',
                             remappings=[("robot_state", "/odri/robot_state"),
                                         ("robot_command", "/odri/robot_command")])

    reconfigure = Node(package='rqt_reconfigure', name='rqt_reconfigure', executable='rqt_reconfigure')

    ld.add_action(odri_iface_node)
    ld.add_action(odri_example_node)
    ld.add_action(reconfigure)

    return ld
