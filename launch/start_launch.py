# example.launch.py

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # args that can be set from the command line or a default will be used


    # include another launch file
    # launch_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory('demo_nodes_cpp'),
    #             'launch/topics/talker_listener.launch.py'))
    # )

    # and use args to set parameters
    node_with_parameters = Node(
        package='robot_interface',
        executable='robot_interface_node',
        name='robot',
        output='screen',
        parameters=[{
            "dev_name": '/dev/CH340_3',
        }]
    )

    return LaunchDescription([
        node_with_parameters,
    ])
