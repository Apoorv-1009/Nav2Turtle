import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('group6'),
        'config',
        'waypoint_params.yaml'
    )

    node = Node(
        package = 'group6',
        name = 'robot_controller',
        executable = 'robot_controller',
        parameters = [config]
    )

    ld.add_action(node)
    return ld