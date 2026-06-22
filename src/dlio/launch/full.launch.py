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
    backend_config_file = os.path.join(pkg_dir, 'config', 'backend.yaml')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to launch RViz2'
    )

    backend_arg = DeclareLaunchArgument(
        'backend', default_value='false',
        description='Whether to launch optional loop-closure/backend nodes'
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

    loop_detector_node = Node(
        package='dlio',
        executable='loop_detector_node',
        name='small_dlio_loop_detector',
        output='screen',
        parameters=[backend_config_file],
        condition=IfCondition(LaunchConfiguration('backend')),
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
        backend_arg,
        odom_node,
        loop_detector_node,
        rviz_node,
    ])
