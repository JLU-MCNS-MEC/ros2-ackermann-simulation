"""Launch the waypoint tracker and the obstacle demonstration world."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Build the trajectory tracking demonstration launch."""
    description_share = get_package_share_directory(
        'ackermann_line_following_description'
    )
    bringup_share = get_package_share_directory('ackermann_line_following_bringup')
    world_file = os.path.join(description_share, 'worlds', 'waypoint_obstacle.sdf')
    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')

    return LaunchDescription(
        [
            DeclareLaunchArgument('target_speed', default_value='0.18'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(sim_launch),
                launch_arguments={
                    'world_name': 'waypoint_obstacle',
                    'gz_args': ['-r ', world_file],
                    'start_x': '-3.5',
                    'start_y': '0.0',
                    'start_z': '0.02',
                    'enable_line_follower': 'false',
                    'enable_waypoint_nav': 'true',
                    'target_speed': LaunchConfiguration('target_speed'),
                }.items(),
            ),
        ]
    )
