import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    ld = LaunchDescription()
    odri_iface_config = os.path.join(get_package_share_directory('odri_interface'), 'config', 'params.yaml')
    trajectory_config = os.path.join(get_package_share_directory('odri_identification'), 'config',
                                     'joint_by_joint_trapezoidal_trajectory.csv')

    odri_iface_node = Node(package='odri_interface',
                           name='robot_interface',
                           executable='robot_interface',
                           parameters=[odri_iface_config],
                           output='screen',
                           remappings=[("robot_state", "/odri/robot_state"), ("robot_command", "/odri/robot_command")])

    odri_identification_node = Node(package='odri_identification',
                                    name='flying_arm_identification',
                                    executable='flying_arm_identification',
                                    parameters=[{"trajectory_csv_path": trajectory_config}],
                                    output='screen',
                                    remappings=[("robot_state", "/odri/robot_state"),
                                                ("robot_command", "/odri/robot_command")])

    reconfigure = Node(package='rqt_reconfigure', name='rqt_reconfigure', executable='rqt_reconfigure')

    ld.add_action(odri_iface_node)
    ld.add_action(odri_identification_node)
    ld.add_action(reconfigure)

    return ld
