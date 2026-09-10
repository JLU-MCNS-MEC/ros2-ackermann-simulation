"""Run the Ackermann world with the official Nav2 navigation components."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, IfElseSubstitution, LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    """Build Gazebo, map server, Nav2 and the CSV route sender."""
    bringup_share = get_package_share_directory('ackermann_line_following_bringup')
    description_share = get_package_share_directory(
        'ackermann_line_following_description'
    )
    controller_share = get_package_share_directory(
        'ackermann_line_following_controller'
    )
    nav2_share = get_package_share_directory('nav2_bringup')
    sim_launch = os.path.join(bringup_share, 'launch', 'sim.launch.py')
    navigation_launch = os.path.join(nav2_share, 'launch', 'navigation_launch.py')
    default_world = os.path.join(
        description_share, 'worlds', 'waypoint_obstacle.sdf'
    )
    params = os.path.join(bringup_share, 'config', 'nav2_ackermann_params.yaml')
    ackermann_to_pose_bt = os.path.join(
        bringup_share, 'behavior_trees', 'navigate_to_pose_ackermann.xml'
    )
    ackermann_through_poses_bt = os.path.join(
        bringup_share, 'behavior_trees', 'navigate_through_poses_ackermann.xml'
    )
    default_map_yaml = os.path.join(
        bringup_share, 'maps', 'waypoint_obstacle.yaml'
    )
    default_waypoint_file = os.path.join(
        controller_share, 'config', 'nav2_trajectory.csv'
    )
    default_rviz_config = os.path.join(bringup_share, 'rviz', 'perception.rviz')

    world_name = LaunchConfiguration('world_name')
    entity_name = LaunchConfiguration('entity_name')
    target_speed = LaunchConfiguration('target_speed')
    gz_args = LaunchConfiguration('gz_args')
    map_file = LaunchConfiguration('map_file')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_z = LaunchConfiguration('start_z')
    rviz_config = LaunchConfiguration('rviz_config')
    use_ekf_localization = IfElseSubstitution(
        EqualsSubstitution(LaunchConfiguration('localization_mode'), 'static'),
        if_value=LaunchConfiguration('use_ekf_localization'), else_value='true',
    )
    odom_topic = IfElseSubstitution(
        use_ekf_localization,
        if_value='/odometry/filtered',
        else_value='/model/ackermann_car/odometry',
    )
    ground_truth_tf_output = IfElseSubstitution(
        use_ekf_localization,
        if_value='/tf_ground_truth',
        else_value='/tf',
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'world_name': world_name,
            'entity_name': entity_name,
            'gz_args': gz_args,
            'start_x': start_x,
            'start_y': start_y,
            'start_z': start_z,
            'enable_line_follower': 'false',
            'enable_waypoint_nav': 'false',
            'enable_ackermann_adapter': 'false',
            'enable_rgbd': LaunchConfiguration('enable_rgbd'),
            'bridge_lidar_points': 'false',
            'target_speed': target_speed,
            'ground_truth_tf_output': ground_truth_tf_output,
        }.items(),
    )

    # Gazebo's model-pose OdometryPublisher reports the world pose directly,
    # so this simulation uses coincident map and odom frames. On hardware this
    # static transform is replaced by robot_localization + SLAM/AMCL.
    map_to_odom = Node(
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization_mode'), 'static')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=[
            '--x', IfElseSubstitution(
                use_ekf_localization,
                if_value=start_x,
                else_value='0.0',
            ),
            '--y', IfElseSubstitution(
                use_ekf_localization,
                if_value=start_y,
                else_value='0.0',
            ),
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )
    ekf_config = os.path.join(bringup_share, 'config', 'ekf_sim.yaml')
    ekf = Node(
        condition=IfCondition(use_ekf_localization),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odometry/filtered')],
    )
    map_server = Node(
        condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('localization_mode'), 'mapping')),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'yaml_filename': map_file,
            }
        ],
    )
    map_lifecycle = Node(
        condition=IfCondition(NotEqualsSubstitution(LaunchConfiguration('localization_mode'), 'mapping')),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
            }
        ],
    )

    amcl = Node(
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization_mode'), 'amcl')),
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[params, {'use_sim_time': True, 'scan_topic': '/scan_slam'}],
    )
    amcl_lifecycle = Node(
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization_mode'), 'amcl')),
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_amcl', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'node_names': ['amcl']}],
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch',
            'online_async_launch.py')),
        condition=IfCondition(EqualsSubstitution(LaunchConfiguration('localization_mode'), 'mapping')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(
                bringup_share, 'config', 'slam_indoor.yaml'),
        }.items(),
    )
    slam_scan = ComposableNode(
        package='pointcloud_to_laserscan',
        plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
        name='slam_scan_projection',
        extra_arguments=[{'use_intra_process_comms': True}],
        remappings=[('cloud_in', '/scan/points'), ('scan', '/scan_slam')],
        parameters=[{'use_sim_time': True, 'target_frame': 'lidar_link',
                     'transform_tolerance': 0.2, 'min_height': -0.10,
                     'max_height': 0.50, 'angle_min': -3.141592653589793,
                     'angle_max': 3.141592653589793,
                     'angle_increment': 0.006135923151543, 'scan_time': 0.1,
                     'range_min': 0.12, 'range_max': 12.0, 'use_inf': True}],
    )
    control_boundary = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'sim2real_control.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    chassis_adapter = Node(
        package='ackermann_line_following_controller',
        executable='ackermann_to_twist', name='navigation_chassis_adapter',
        parameters=[{'use_sim_time': True, 'input_topic': '/drive',
                     'output_topic': ['/model/', entity_name, '/cmd_vel'],
                     'command_timeout': 0.25}],
    )

    # Gazebo's multi-layer GPU lidar publishes useful obstacle returns in the
    # PointCloud2 stream. Project a ground-filtered height band to LaserScan
    # for Nav2 plugins that require a two-dimensional scan.
    lidar_scan_projection = ComposableNode(
        package='pointcloud_to_laserscan',
        plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
        name='mid360_scan_projection',
        extra_arguments=[{'use_intra_process_comms': True}],
        remappings=[
            ('cloud_in', '/scan/points'),
            ('scan', '/scan_nav'),
        ],
        parameters=[
            {
                'use_sim_time': True,
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.1,
                'min_height': 0.08,
                'max_height': 1.80,
                'angle_min': -3.141592653589793,
                'angle_max': 3.141592653589793,
                'angle_increment': 0.006135923151543,
                'scan_time': 0.1,
                'range_min': 0.12,
                'range_max': 12.0,
                'use_inf': True,
            }
        ],
    )
    # Keep the large point cloud and both scan consumers inside one process.
    # This bypasses lossy inter-process delivery before the safety scan exists.
    lidar_pipeline = ComposableNodeContainer(
        name='lidar_pipeline', namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='ros_gz_bridge', plugin='ros_gz_bridge::RosGzBridge',
                name='pointcloud_bridge',
                parameters=[{'use_sim_time': True, 'config_file': os.path.join(
                    bringup_share, 'config', 'lidar_bridge.yaml')}],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            slam_scan, lidar_scan_projection,
        ],
        output='screen',
    )
    lidar_ground_filter = Node(
        package='pcl_ros',
        executable='filter_passthrough_node',
        name='mid360_ground_filter',
        output='screen',
        remappings=[
            ('input', '/scan/points'),
            ('output', '/scan/points_obstacles'),
        ],
        parameters=[
            {
                'use_sim_time': True,
                'filter_field_name': 'z',
                'filter_limit_min': 0.08,
                'filter_limit_max': 1.80,
                'filter_limit_negative': False,
                'keep_organized': False,
            }
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            # navigation_launch.py applies its own namespace/autostart rewrite;
            # these rewrites select separate Ackermann-safe recovery trees for
            # single poses and through-poses without hard-coding an install
            # prefix in the YAML file.
            'params_file': RewrittenYaml(
                source_file=params,
                param_rewrites={
                    (
                        'bt_navigator.ros__parameters.'
                        'default_nav_to_pose_bt_xml'
                    ): ackermann_to_pose_bt,
                    (
                        'bt_navigator.ros__parameters.'
                        'default_nav_through_poses_bt_xml'
                    ): ackermann_through_poses_bt,
                    (
                        'controller_server.ros__parameters.'
                        'FollowPath.desired_linear_vel'
                    ): target_speed,
                    'bt_navigator.ros__parameters.odom_topic': odom_topic,
                    'velocity_smoother.ros__parameters.odom_topic': odom_topic,
                    'collision_monitor.ros__parameters.cmd_vel_out_topic': '/cmd_vel_safe',
                    'planner_server.ros__parameters.GridBased.motion_model_for_search': IfElseSubstitution(
                        LaunchConfiguration('allow_reversing'),
                        if_value='REEDS_SHEPP', else_value='DUBIN'),
                    'controller_server.ros__parameters.FollowPath.allow_reversing': LaunchConfiguration('allow_reversing'),
                },
                root_key='',
                convert_types=True,
            ),
            'use_composition': 'False',
            'use_respawn': 'False',
            'log_level': 'info',
        }.items(),
    )

    route_sender = Node(
        condition=IfCondition(LaunchConfiguration('send_waypoints')),
        package='ackermann_line_following_controller',
        executable='nav2_waypoint_sender',
        name='nav2_waypoint_sender',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'waypoint_file': LaunchConfiguration('waypoint_file'),
                'frame_id': 'map',
            }
        ],
    )

    goal_pose_optimizer = Node(
        package='ackermann_line_following_controller',
        executable='goal_pose_optimizer',
        name='goal_pose_optimizer',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'input_topic': '/goal_pose_auto',
                'output_topic': '/goal_pose',
                'base_frame': 'base_footprint',
                'allow_reverse': LaunchConfiguration('allow_reversing'),
            }
        ],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    diagnostics = Node(
        condition=IfCondition(LaunchConfiguration('use_diagnostics')),
        package='ackermann_line_following_controller',
        executable='navigation_diagnostics',
        name='navigation_diagnostics',
        output='screen',
        parameters=[{'use_sim_time': True, 'odom_topic': odom_topic}],
    )
    navigation_plots = Node(
        condition=IfCondition(LaunchConfiguration('use_rqt_plots')),
        package='ackermann_line_following_controller',
        executable='navigation_plotter',
        name='navigation_plotter',
        output='screen',
    )
    experiment = Node(
        condition=IfCondition(LaunchConfiguration('record_experiment')),
        package='ackermann_line_following_controller',
        executable='navigation_experiment',
        name='navigation_experiment',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'scenario': LaunchConfiguration('experiment_name'),
                'result_file': LaunchConfiguration('experiment_result_file'),
                'odom_topic': odom_topic,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'localization_mode', default_value='static',
                choices=['static', 'mapping', 'amcl'],
                description='Static legacy baseline, online SLAM or saved-map AMCL.',
            ),
            DeclareLaunchArgument('enable_rgbd', default_value='false'),
            DeclareLaunchArgument('allow_reversing', default_value='false'),
            DeclareLaunchArgument(
                'world_name',
                default_value='waypoint_obstacle',
                description='Gazebo world name used for model topics.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value=f'-r {default_world}',
                description='Gazebo arguments; add -s for a headless server run.',
            ),
            DeclareLaunchArgument(
                'map_file',
                default_value=default_map_yaml,
                description='Static map YAML matching world_file.',
            ),
            DeclareLaunchArgument(
                'entity_name',
                default_value='ackermann_car',
                description='Gazebo Ackermann model name.',
            ),
            DeclareLaunchArgument(
                'waypoint_file',
                default_value=default_waypoint_file,
                description='CSV route sent through Nav2 NavigateThroughPoses.',
            ),
            DeclareLaunchArgument('start_x', default_value='-3.5'),
            DeclareLaunchArgument('start_y', default_value='0.0'),
            DeclareLaunchArgument('start_z', default_value='0.02'),
            DeclareLaunchArgument(
                'rviz_config',
                default_value=default_rviz_config,
                description='RViz configuration for this navigation scenario.',
            ),
            DeclareLaunchArgument('target_speed', default_value='0.22'),
            DeclareLaunchArgument(
                'use_ekf_localization',
                default_value='false',
                description=(
                    'Fuse wheel odometry and IMU instead of using Gazebo '
                    'ground-truth TF.'
                ),
            ),
            DeclareLaunchArgument(
                'send_waypoints',
                default_value='true',
                description='Send the CSV route through nav2_simple_commander.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='false',
                description='Start RViz with the perception and Nav2 displays.',
            ),
            DeclareLaunchArgument(
                'use_diagnostics',
                default_value='true',
                description=(
                    'Publish scalar navigation telemetry and an RViz summary.'
                ),
            ),
            DeclareLaunchArgument(
                'use_rqt_plots',
                default_value='false',
                description='Open live speed and navigation decision plots.',
            ),
            DeclareLaunchArgument(
                'record_experiment',
                default_value='false',
                description='Record route samples and aggregate metrics.',
            ),
            DeclareLaunchArgument(
                'experiment_name',
                default_value='unnamed',
                description='Scenario label stored in the experiment report.',
            ),
            DeclareLaunchArgument(
                'experiment_result_file',
                default_value='/tmp/ackermann_navigation_experiment.json',
                description='JSON result path; CSV samples use the same stem.',
            ),
            sim,
            ekf,
            map_to_odom,
            map_server,
            map_lifecycle,
            amcl,
            amcl_lifecycle,
            slam,
            lidar_pipeline,
            control_boundary,
            chassis_adapter,
            lidar_ground_filter,
            navigation,
            goal_pose_optimizer,
            route_sender,
            diagnostics,
            rviz,
            navigation_plots,
            experiment,
        ]
    )
