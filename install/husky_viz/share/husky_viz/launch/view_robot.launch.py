import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import launch


def generate_launch_description():

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("husky_viz"), "rviz", "test.rviz"]
    )
    
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            node_rviz,
        ]
    )
