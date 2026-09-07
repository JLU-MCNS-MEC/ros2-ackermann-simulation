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
    controller_share = get_package_share_directory(
        'ackermann_line_following_controller'
    )
    gz_share = get_package_share_directory('ros_gz_sim')

    default_model = os.path.join(
        description_share,
        'urdf',
        'ackermann_car.urdf.xacro',
    )
    default_world = os.path.join(description_share, 'worlds', 'line_track.sdf')
    default_waypoint_file = os.path.join(
        controller_share,
        'config',
        'straight_trajectory.csv',
    )

    entity_name = LaunchConfiguration('entity_name')
    world_name = LaunchConfiguration('world_name')
    robot_description_file = LaunchConfiguration('robot_description_file')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_z = LaunchConfiguration('start_z')
    line_speed = LaunchConfiguration('line_speed')
    waypoint_file = LaunchConfiguration('waypoint_file')
    target_speed = LaunchConfiguration('target_speed')
    enable_ackermann_adapter = LaunchConfiguration('enable_ackermann_adapter')
    enable_rgbd = LaunchConfiguration('enable_rgbd')
    max_speed = LaunchConfiguration('max_speed')
    max_acceleration = LaunchConfiguration('max_acceleration')
    max_deceleration = LaunchConfiguration('max_deceleration')
    max_steering = LaunchConfiguration('max_steering')
    max_steering_rate = LaunchConfiguration('max_steering_rate')
    command_timeout = LaunchConfiguration('command_timeout')
    control_rate = LaunchConfiguration('control_rate')
    ground_truth_tf_output = LaunchConfiguration('ground_truth_tf_output')

    robot_description = {
        'robot_description': ParameterValue(
            Command([
                'xacro ', robot_description_file,
                ' enable_rgbd:=', enable_rgbd,
            ]),
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
            'world': world_name,
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
    odometry_topic = [
        '/model/',
        entity_name,
        '/odometry',
    ]
    odometry_bridge_topic = odometry_topic + [
        '@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    ]
    # The native Ackermann plugin publishes its wheel-integrated estimate on
    # the fixed auxiliary topic configured in the URDF. Keep it bridged for
    # encoder-slip experiments while the standard odometry topic comes from
    # Gazebo's model-pose OdometryPublisher.
    wheel_odometry_topic = [
        '/model/ackermann_car/wheel_odometry',
    ]
    wheel_odometry_bridge_topic = wheel_odometry_topic + [
        '@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    ]
    wheel_tf_topic = [
        '/model/ackermann_car/wheel_tf',
    ]
    ground_truth_tf_topic = [
        '/model/',
        entity_name,
        '/ground_truth_tf',
    ]
    joint_state_topic = [
        '/world/',
        world_name,
        '/model/',
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
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rgbd/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            wheel_tf_topic + [
                '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            command_bridge_topic,
            odometry_bridge_topic,
            wheel_odometry_bridge_topic,
            ground_truth_tf_topic + [
                '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            joint_state_bridge_topic,
        ],
        remappings=[
            (joint_state_topic, '/joint_states'),
            # Nav2 consumes the model-pose transform. The wheel-integrated TF
            # remains visible on /tf_wheel for comparison and debugging.
            (ground_truth_tf_topic, ground_truth_tf_output),
            (wheel_tf_topic, '/tf_wheel'),
        ],
        output='screen',
    )

    ackermann_to_twist = Node(
        condition=IfCondition(enable_ackermann_adapter),
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
                'max_speed': max_speed,
                'max_acceleration': max_acceleration,
                'max_deceleration': max_deceleration,
                'max_steering': max_steering,
                'max_steering_rate': max_steering_rate,
                'command_timeout': command_timeout,
                'control_rate': control_rate,
            }
        ],
    )

    waypoint_tracker = Node(
        condition=IfCondition(LaunchConfiguration('enable_waypoint_nav')),
        package='ackermann_line_following_controller',
        executable='waypoint_tracker',
        name='waypoint_tracker',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'waypoint_file': waypoint_file,
                'odom_topic': odometry_topic[:],
                'scan_topic': '/scan',
                'command_topic': '/cmd_ackermann',
                'target_speed': target_speed,
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
                'world_name',
                default_value='line_track',
                description='Gazebo world name used by model spawn and joint bridge.',
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
            DeclareLaunchArgument('enable_waypoint_nav', default_value='false'),
            DeclareLaunchArgument(
                'enable_ackermann_adapter',
                default_value='true',
                description=(
                    'Convert AckermannDriveStamped to the Gazebo Twist topic. '
                    'Disable when Nav2 publishes the final Twist.'
                ),
            ),
            DeclareLaunchArgument(
                'enable_rgbd',
                default_value='true',
                description='Spawn the RGB-D sensor; disable it when unused.',
            ),
            DeclareLaunchArgument(
                'ground_truth_tf_output',
                default_value='/tf',
                description=(
                    'Remap Gazebo pose TF away from /tf when EKF owns odom TF.'
                ),
            ),
            DeclareLaunchArgument('target_speed', default_value='0.18'),
            DeclareLaunchArgument(
                'max_speed',
                default_value='0.6',
                description='Symmetric forward/reverse speed limit (m/s).',
            ),
            DeclareLaunchArgument(
                'max_acceleration',
                default_value='1.5',
                description='Positive acceleration limit (m/s^2).',
            ),
            DeclareLaunchArgument(
                'max_deceleration',
                default_value='1.5',
                description='Positive braking limit (m/s^2).',
            ),
            DeclareLaunchArgument(
                'max_steering',
                default_value='0.55',
                description='Absolute front steering limit (rad).',
            ),
            DeclareLaunchArgument(
                'max_steering_rate',
                default_value='2.0',
                description='Absolute steering rate limit (rad/s).',
            ),
            DeclareLaunchArgument(
                'command_timeout',
                default_value='0.5',
                description='Stop if no Ackermann command arrives (s).',
            ),
            DeclareLaunchArgument(
                'control_rate',
                default_value='50.0',
                description='Ackermann command output rate (Hz).',
            ),
            DeclareLaunchArgument(
                'waypoint_file',
                default_value=default_waypoint_file,
                description='CSV waypoint file in the odom frame.',
            ),
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
            waypoint_tracker,
        ]
    )
