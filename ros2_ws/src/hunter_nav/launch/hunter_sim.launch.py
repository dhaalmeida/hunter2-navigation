from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pointcloud_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[
            ('cloud_in', '/velodyne_points'),
        ],
        parameters=[{
            'range_max': 30.0,
            'range_min': 0.1,
            'use_inf': False,
            'min_height': -0.2,
            'max_height': 0.5,
            'target_frame': 'velodyne'
        }]
    )

    obstacle_node = Node(
        package='obstacle_detection_ros2',
        executable='cpp_code',
        name='obstacle_detection',
        output='screen',
        remappings=[
            ('scanner/scan', '/scan'),
        ]
    )

    navigation_node = Node(
        package='hunter_nav',
        executable='hunter_nav',
        name='hunter_navigation',
        output='screen'
    )

    return LaunchDescription([
        pointcloud_node,
        obstacle_node,
        navigation_node
    ])