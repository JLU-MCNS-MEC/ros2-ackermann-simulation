"""Select repeatable static-map navigation scenarios for the Ackermann car."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _scenario_launch(context, *args, **kwargs):
    """Resolve a named route and include the common Nav2 launch."""
    del args, kwargs
    scenario = LaunchConfiguration('scenario').perform(context).strip().lower()
    send_waypoints = LaunchConfiguration('send_waypoints')
    use_rviz = LaunchConfiguration('use_rviz')
    target_speed = LaunchConfiguration('target_speed')
    use_diagnostics = LaunchConfiguration('use_diagnostics')
    use_rqt_plots = LaunchConfiguration('use_rqt_plots')
    external_gz_args = LaunchConfiguration('gz_args').perform(context).strip()

    bringup_share = get_package_share_directory('ackermann_line_following_bringup')
    description_share = get_package_share_directory(
        'ackermann_line_following_description'
    )
    controller_share = get_package_share_directory(
        'ackermann_line_following_controller'
    )
    scenarios = {
        'straight': ('nav2_short_straight.csv', 'waypoint_obstacle.yaml'),
        'offset': ('nav2_offset_goal.csv', 'waypoint_obstacle.yaml'),
        'obstacle': ('nav2_trajectory.csv', 'waypoint_obstacle.yaml'),
        'unknown_obstacle': (
            'nav2_unknown_obstacle.csv',
            'free_navigation.yaml',
        ),
    }
    if scenario not in scenarios:
        available = ', '.join(sorted(scenarios))
        raise RuntimeError(
            f'Unknown static-map scenario {scenario!r}; choose: {available}'
        )

    world = os.path.join(
        description_share, 'worlds', 'waypoint_obstacle.sdf'
    )
    route_name, map_name = scenarios[scenario]
    route = os.path.join(controller_share, 'config', route_name)
    map_file = os.path.join(
        bringup_share, 'maps', map_name
    )
    nav_launch = os.path.join(
        bringup_share, 'launch', 'nav2_waypoint_nav.launch.py'
    )
    gz_args = external_gz_args or f'-r {world}'
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                'world_name': 'waypoint_obstacle',
                'map_file': map_file,
                'gz_args': gz_args,
                'waypoint_file': route,
                'target_speed': target_speed,
                'send_waypoints': send_waypoints,
                'use_rviz': use_rviz,
                'use_diagnostics': use_diagnostics,
                'use_rqt_plots': use_rqt_plots,
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Build the scenario selector and common Nav2 launch."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'scenario',
                default_value='unknown_obstacle',
                description=(
                    'One of straight, offset, obstacle or unknown_obstacle.'
                ),
            ),
            DeclareLaunchArgument(
                'send_waypoints',
                default_value='true',
                description='Send the selected route to NavigateThroughPoses.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='Open RViz with map, costmaps, paths and sensors.',
            ),
            DeclareLaunchArgument(
                'target_speed',
                default_value='0.18',
                description='Desired speed passed to the Nav2 parameter set.',
            ),
            DeclareLaunchArgument(
                'use_diagnostics',
                default_value='true',
                description=(
                    'Publish navigation telemetry and the RViz summary.'
                ),
            ),
            DeclareLaunchArgument(
                'use_rqt_plots',
                default_value='false',
                description='Open live speed and decision curves.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value='',
                description=(
                    'Optional Gazebo arguments; empty selects the scenario world.'
                ),
            ),
            OpaqueFunction(function=_scenario_launch),
        ]
    )
