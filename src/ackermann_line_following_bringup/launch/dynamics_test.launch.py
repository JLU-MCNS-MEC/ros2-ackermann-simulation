"""Launch the reversible Ackermann dynamics acceptance test."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the open-field test track, adapter, test node and RViz."""
    bringup_share = get_package_share_directory('ackermann_line_following_bringup')
    description_share = get_package_share_directory(
        'ackermann_line_following_description'
    )
    world = os.path.join(description_share, 'worlds', 'dynamics_test.sdf')
    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')
    rviz_config = os.path.join(bringup_share, 'rviz', 'dynamics.rviz')
    gz_args = LaunchConfiguration('gz_args')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world_name': 'dynamics_test',
            'gz_args': gz_args,
            'start_x': '-2.0',
            'start_y': '0.0',
            'start_z': '0.02',
            'enable_line_follower': 'false',
            'enable_waypoint_nav': 'false',
            'enable_ackermann_adapter': 'true',
        }.items(),
    )

    dynamics_test = Node(
        condition=IfCondition(LaunchConfiguration('run_test')),
        package='ackermann_line_following_controller',
        executable='ackermann_dynamics_test',
        name='ackermann_dynamics_test',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'result_file': LaunchConfiguration('result_file'),
                'segment_scale': LaunchConfiguration('segment_scale'),
            }
        ],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2_dynamics',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'run_test',
                default_value='true',
                description='Run the forward/reverse acceptance sequence.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='Show the measured dynamics path in RViz.',
            ),
            DeclareLaunchArgument(
                'result_file',
                default_value='/tmp/ackermann_dynamics_result.json',
                description='JSON file written by the dynamics test node.',
            ),
            DeclareLaunchArgument(
                'segment_scale',
                default_value='1.0',
                description='Scale each test segment duration for slow hosts.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value=f'-r {world}',
                description='Gazebo arguments; add -s for a headless run.',
            ),
            sim,
            dynamics_test,
            rviz,
        ]
    )
