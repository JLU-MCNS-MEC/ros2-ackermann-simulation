"""Run wheel odometry and IMU fusion without Gazebo ground-truth TF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start a headless dynamics world and the planar EKF."""
    bringup_share = get_package_share_directory(
        'ackermann_line_following_bringup'
    )
    description_share = get_package_share_directory(
        'ackermann_line_following_description'
    )
    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')
    default_world = os.path.join(
        description_share, 'worlds', 'dynamics_test.sdf'
    )
    ekf_config = os.path.join(bringup_share, 'config', 'ekf_sim.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world_name': 'dynamics_test',
            'gz_args': LaunchConfiguration('gz_args'),
            'enable_line_follower': 'false',
            'enable_waypoint_nav': 'false',
            'enable_ackermann_adapter': 'false',
            'enable_rgbd': 'false',
            'ground_truth_tf_output': '/tf_ground_truth',
            'start_x': '0.0',
            'start_y': '0.0',
            'start_z': '0.02',
        }.items(),
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gz_args',
                default_value=f'-r -s {default_world}',
                description='Gazebo arguments for the EKF regression world.',
            ),
            simulation,
            ekf,
        ]
    )
