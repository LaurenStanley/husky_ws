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
        # Node(
        #     package='pointcloud_to_laserscan', executable='dummy_pointcloud_publisher',
        #     remappings=[('cloud', [LaunchConfiguration(variable_name='scanner'), '/velodyne_points'])],
        #     parameters=[{'cloud_frame_id': 'velodyne', 'cloud_extent': 2.0, 'cloud_size': 500}],
        #     name='cloud_publisher'
        #),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'velodyne']
        # ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            # remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/velodyne_points']),
            #             ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            remappings=[('cloud_in', 'velodyne_points'),
                        ('scan', 'scan')],
            parameters=[{
                'target_frame': 'velodyne',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 10.0,
                #'angle_min': -1.5708,  # -M_PI/2
                #'angle_max': 1.5708,  # M_PI/2
                'angle_min': -3.1415,
                'angle_max': 3.1415,
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.25,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                'use_sim_time': True,
            }],
            name='pointcloud_to_laserscan'
        )
    ])
