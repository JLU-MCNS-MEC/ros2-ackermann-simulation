"""Run the Ackermann world with the official Nav2 navigation components."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
            'enable_rgbd': 'false',
            'target_speed': target_speed,
        }.items(),
    )

    # Gazebo's model-pose OdometryPublisher reports the world pose directly,
    # so this simulation uses coincident map and odom frames. On hardware this
    # static transform is replaced by robot_localization + SLAM/AMCL.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
    )
    map_server = Node(
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

    # Gazebo's multi-layer GPU lidar publishes useful obstacle returns in the
    # PointCloud2 stream. Project a ground-filtered height band to LaserScan
    # for Nav2 plugins that require a two-dimensional scan.
    lidar_scan_projection = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='mid360_scan_projection',
        output='screen',
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
        parameters=[{'use_sim_time': True}],
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
            }
        ],
    )

    return LaunchDescription(
        [
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
            map_to_odom,
            map_server,
            map_lifecycle,
            lidar_scan_projection,
            lidar_ground_filter,
            navigation,
            route_sender,
            diagnostics,
            rviz,
            navigation_plots,
            experiment,
        ]
    )
