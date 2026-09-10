"""Convert a final Nav2 safety command into a real Ackermann command."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the hardware-neutral command boundary used by the real car."""
    adapter = Node(
        package='ackermann_line_following_controller',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'wheelbase': LaunchConfiguration('wheelbase'),
                'max_speed': LaunchConfiguration('max_speed'),
                'max_steering': LaunchConfiguration('max_steering'),
                'minimum_speed': LaunchConfiguration('minimum_speed'),
                'command_timeout': LaunchConfiguration('command_timeout'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument(
                'input_topic',
                default_value='/cmd_vel_safe',
                description=(
                    'Final Twist after smoothing and collision safety.'
                ),
            ),
            DeclareLaunchArgument(
                'output_topic',
                default_value='/drive',
                description=(
                    'AckermannDriveStamped consumed by the chassis driver.'
                ),
            ),
            DeclareLaunchArgument('wheelbase', default_value='0.56'),
            DeclareLaunchArgument('max_speed', default_value='0.6'),
            DeclareLaunchArgument('max_steering', default_value='0.55'),
            DeclareLaunchArgument(
                'minimum_speed',
                default_value='0.000001',
                description=(
                    'Numerical stationary threshold; preserve low-speed curvature.'
                ),
            ),
            DeclareLaunchArgument(
                'command_timeout',
                default_value='0.25',
                description=(
                    'Publish a stop when final Twist becomes stale (s).'
                ),
            ),
            DeclareLaunchArgument('publish_rate', default_value='50.0'),
            adapter,
        ]
    )
