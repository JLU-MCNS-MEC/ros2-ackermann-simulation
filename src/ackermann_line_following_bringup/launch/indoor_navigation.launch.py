"""Map and navigate a 100 square metre indoor scene without ground truth."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch(context):
    """Validate saved-map input before starting the simulator."""
    mode = LaunchConfiguration('mode').perform(context)
    map_file = LaunchConfiguration('map_file').perform(context)
    if mode == 'amcl' and not Path(map_file).is_file():
        raise ValueError('AMCL requires an existing map_file saved from SLAM')
    bringup = get_package_share_directory('ackermann_line_following_bringup')
    description = get_package_share_directory('ackermann_line_following_description')
    world = os.path.join(description, 'worlds', 'indoor_100sqm.sdf')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'nav2_waypoint_nav.launch.py')
            ),
            launch_arguments={
            'localization_mode': mode,
            'rviz_config': os.path.join(bringup, 'rviz', 'indoor_semantic.rviz'),
                'map_file': map_file,
                'world_name': 'indoor_100sqm',
                'gz_args': ['-r -s --headless-rendering ', world],
                'start_x': '-3.5',
                'start_y': '-3.5',
                'use_ekf_localization': 'true',
                'send_waypoints': 'false',
                'allow_reversing': 'true',
                'enable_rgbd': LaunchConfiguration('enable_rgbd'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Expose mapping and saved-map localization as separate modes."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode', default_value='mapping', choices=['mapping', 'amcl']
            ),
            DeclareLaunchArgument('map_file', default_value=''),
            DeclareLaunchArgument('enable_rgbd', default_value='false'),
            DeclareLaunchArgument('use_rviz', default_value='false'),
            OpaqueFunction(function=_launch),
        ]
    )
