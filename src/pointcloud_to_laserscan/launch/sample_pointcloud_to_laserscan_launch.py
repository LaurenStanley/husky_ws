from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', 'velodyne_points'),
                        ('scan', 'scan')],
            parameters=[{
                'target_frame': 'velodyne',
                'transform_tolerance': 0.01,
                'min_height': -0.75,
                'max_height': 10.0,
                'angle_min': -3.1415,
                'angle_max': 3.1415,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.9,
                'range_max': 120.0,
                'use_inf': False,
                'inf_epsilon': -1.0,
                'use_sim_time': True,
            }],
            name='pointcloud_to_laserscan'
        )
    ])
