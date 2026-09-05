"""Start Gazebo Harmonic, a swappable Ackermann model, bridges and line control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the complete simulation launch description."""
    description_share = get_package_share_directory(
        'ackermann_line_following_description'
    )
    gz_share = get_package_share_directory('ros_gz_sim')

    default_model = os.path.join(
        description_share,
        'urdf',
        'ackermann_car.urdf.xacro',
    )
    default_world = os.path.join(description_share, 'worlds', 'line_track.sdf')

    entity_name = LaunchConfiguration('entity_name')
    robot_description_file = LaunchConfiguration('robot_description_file')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_z = LaunchConfiguration('start_z')
    line_speed = LaunchConfiguration('line_speed')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', robot_description_file]),
            value_type=str,
        ),
        'use_sim_time': True,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args'),
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_spawn_model.launch.py')
        ),
        launch_arguments={
            'world': 'line_track',
            'topic': 'robot_description',
            'entity_name': entity_name,
            'allow_renaming': 'False',
            'x': start_x,
            'y': start_y,
            'z': start_z,
        }.items(),
    )

    # The native Gazebo Ackermann system uses Twist. The controller keeps the
    # public interface as AckermannDriveStamped and the adapter performs the
    # kinematic conversion before this bridge.
    command_bridge_topic = [
        '/model/',
        entity_name,
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    ]
    odometry_bridge_topic = [
        '/model/',
        entity_name,
        '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    ]
    joint_state_topic = [
        '/world/line_track/model/',
        entity_name,
        '/joint_state',
    ]
    joint_state_bridge_topic = joint_state_topic + [
        '@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ['/model/', entity_name, '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
            command_bridge_topic,
            odometry_bridge_topic,
            joint_state_bridge_topic,
        ],
        remappings=[
            (joint_state_topic, '/joint_states'),
            (['/model/', entity_name, '/tf'], '/tf'),
        ],
        output='screen',
    )

    ackermann_to_twist = Node(
        package='ackermann_line_following_controller',
        executable='ackermann_to_twist',
        name='ackermann_to_twist',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'input_topic': '/cmd_ackermann',
                'output_topic': command_bridge_topic[:2] + [
                    '/cmd_vel',
                ],
                'wheelbase': 0.56,
            }
        ],
    )

    line_follower = Node(
        condition=IfCondition(LaunchConfiguration('enable_line_follower')),
        package='ackermann_line_following_controller',
        executable='line_follower',
        name='line_follower',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'image_topic': '/camera/image_raw',
                'command_topic': '/cmd_ackermann',
                'speed': line_speed,
                'lost_line_speed': 0.03,
                'threshold': 80,
                'roi_top': 0.15,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'entity_name',
                default_value='ackermann_car',
                description='Gazebo entity name; bridge topics follow this name.',
            ),
            DeclareLaunchArgument(
                'robot_description_file',
                default_value=default_model,
                description='URDF/Xacro model to spawn.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value=f'-r {default_world}',
                description='Arguments passed to Gazebo Sim; use -s for headless mode.',
            ),
            DeclareLaunchArgument('start_x', default_value='-2.8'),
            DeclareLaunchArgument('enable_line_follower', default_value='true'),
            DeclareLaunchArgument('start_y', default_value='0.0'),
            DeclareLaunchArgument('start_z', default_value='0.02'),
            DeclareLaunchArgument(
                'line_speed',
                default_value='0.18',
                description='Forward speed for the built-in line follower (m/s).',
            ),
            gazebo,
            robot_state_publisher,
            spawn,
            bridge,
            ackermann_to_twist,
            line_follower,
        ]
    )
