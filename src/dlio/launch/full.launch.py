import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('dlio')
    config_file = os.path.join(pkg_dir, 'config', 'config.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to launch RViz2'
    )

    odom_node = Node(
        package='dlio',
        executable='dlio_node',
        name='small_dlio_odom',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/livox/imu', '/livox/imu'),
            ('/livox/lidar', '/livox/lidar'),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'dlio.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        odom_node,
        rviz_node,
    ])
