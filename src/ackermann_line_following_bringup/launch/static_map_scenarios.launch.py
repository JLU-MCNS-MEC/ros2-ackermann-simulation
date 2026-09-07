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
    record_experiment = LaunchConfiguration('record_experiment')
    experiment_result_file = LaunchConfiguration('experiment_result_file')
    external_gz_args = LaunchConfiguration('gz_args').perform(context).strip()

    bringup_share = get_package_share_directory('ackermann_line_following_bringup')
    description_share = get_package_share_directory(
        'ackermann_line_following_description'
    )
    controller_share = get_package_share_directory(
        'ackermann_line_following_controller'
    )
    scenarios = {
        'straight': (
            'nav2_short_straight.csv',
            'waypoint_obstacle.yaml',
            'waypoint_obstacle',
            '-3.5',
            '0.0',
        ),
        'offset': (
            'nav2_offset_goal.csv',
            'waypoint_obstacle.yaml',
            'waypoint_obstacle',
            '-3.5',
            '0.0',
        ),
        'obstacle': (
            'nav2_trajectory.csv',
            'waypoint_obstacle.yaml',
            'waypoint_obstacle',
            '-3.5',
            '0.0',
        ),
        'unknown_obstacle': (
            'nav2_unknown_obstacle.csv',
            'free_navigation.yaml',
            'unknown_obstacle',
            '-3.5',
            '0.0',
        ),
        'complex_static': (
            'nav2_complex_static.csv',
            'complex_static.yaml',
            'complex_static',
            '-13.0',
            '-8.0',
        ),
    }
    if scenario not in scenarios:
        available = ', '.join(sorted(scenarios))
        raise RuntimeError(
            f'Unknown static-map scenario {scenario!r}; choose: {available}'
        )

    route_name, map_name, world_name, start_x, start_y = scenarios[scenario]
    world = os.path.join(description_share, 'worlds', f'{world_name}.sdf')
    rviz_name = (
        'perception_large.rviz'
        if scenario == 'complex_static'
        else 'perception.rviz'
    )
    rviz_config = os.path.join(bringup_share, 'rviz', rviz_name)
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
                'world_name': world_name,
                'map_file': map_file,
                'gz_args': gz_args,
                'waypoint_file': route,
                'rviz_config': rviz_config,
                'start_x': start_x,
                'start_y': start_y,
                'target_speed': target_speed,
                'send_waypoints': send_waypoints,
                'use_rviz': use_rviz,
                'use_diagnostics': use_diagnostics,
                'use_rqt_plots': use_rqt_plots,
                'record_experiment': record_experiment,
                'experiment_name': scenario,
                'experiment_result_file': experiment_result_file,
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
                    'One of straight, offset, obstacle, unknown_obstacle or '
                    'complex_static.'
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
                'record_experiment',
                default_value='false',
                description='Write CSV samples and a JSON scenario summary.',
            ),
            DeclareLaunchArgument(
                'experiment_result_file',
                default_value='/tmp/ackermann_navigation_experiment.json',
                description='Path for the aggregate experiment report.',
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
