"""Start saved-map localization and named semantic navigation together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ackermann_line_following_controller.semantic_map import (
    load_landmarks,
    map_identity,
)


def _launch(context):
    """Reject incompatible landmark data before starting any processes."""
    map_file = LaunchConfiguration('map_file').perform(context)
    database = LaunchConfiguration('database').perform(context)
    load_landmarks(database, map_identity(map_file))
    share = get_package_share_directory('ackermann_line_following_bringup')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'indoor_navigation.launch.py')
            ),
            launch_arguments={
                'mode': 'amcl',
                'map_file': map_file,
                'use_rviz': LaunchConfiguration('use_rviz'),
                'enable_rgbd': LaunchConfiguration('enable_rgbd'),
            }.items(),
        ),
        Node(
            package='ackermann_line_following_controller',
            executable='semantic_navigation',
            name='semantic_navigation',
            output='screen',
            arguments=['--map', map_file, '--database', database],
            parameters=[{'use_sim_time': True}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Use the measured demonstration map and manually recorded landmarks."""
    share = get_package_share_directory('ackermann_line_following_bringup')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'map_file',
                default_value=os.path.join(share, 'maps', 'indoor_slam.yaml'),
            ),
            DeclareLaunchArgument(
                'database',
                default_value=os.path.join(share, 'maps', 'indoor_landmarks.json'),
            ),
            DeclareLaunchArgument('use_rviz', default_value='false'),
            DeclareLaunchArgument('enable_rgbd', default_value='false'),
            OpaqueFunction(function=_launch),
        ]
    )
