"""Check that perception and stationary testing remain launchable."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument


def test_sim_launch_exposes_stationary_mode():
    path = Path(__file__).parents[1] / 'launch' / 'sim.launch.py'
    spec = importlib.util.spec_from_file_location('sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    names = {entity.name for entity in description.entities
             if isinstance(entity, DeclareLaunchArgument)}
    assert {
        'enable_line_follower',
        'enable_waypoint_nav',
        'enable_rgbd',
        'start_x',
        'start_y',
        'gz_args',
        'target_speed',
        'waypoint_file',
        'world_name',
        'max_speed',
        'max_acceleration',
        'max_deceleration',
        'max_steering',
        'max_steering_rate',
        'command_timeout',
        'control_rate',
        'ground_truth_tf_output',
    } <= names


def test_sim_launch_uses_mid360_scan_and_pointcloud_bridge():
    bringup_dir = Path(__file__).parents[1]
    urdf = (
        bringup_dir.parent / 'ackermann_line_following_description'
        / 'urdf' / 'ackermann_car.urdf.xacro'
    ).read_text(encoding='utf-8')
    bridge = (bringup_dir / 'launch' / 'sim.launch.py').read_text(
        encoding='utf-8'
    )
    assert '<sensor name="mid360" type="gpu_lidar">' in urdf
    assert '<vertical>' in urdf
    assert '<samples>20</samples>' in urdf
    assert (
        '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        in bridge
    )
    assert '<sensor name="imu" type="imu">' in urdf
    assert '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU' in bridge
    worlds = bringup_dir.parent / 'ackermann_line_following_description' / 'worlds'
    for world in worlds.glob('*.sdf'):
        assert 'gz-sim-imu-system' in world.read_text(encoding='utf-8')


def test_rviz_configs_color_mid360_points_by_height():
    bringup_dir = Path(__file__).parents[1]
    for config_name in ('dynamics.rviz', 'perception.rviz'):
        config = (bringup_dir / 'rviz' / config_name).read_text(
            encoding='utf-8'
        )
        assert 'Name: MID360 PointCloud' in config
        assert 'Color Transformer: AxisColor' in config
        assert 'Topic: /scan/points' in config


def test_navigation_rviz_uses_best_effort_projected_scan():
    config = (
        Path(__file__).parents[1] / 'rviz' / 'perception.rviz'
    ).read_text(encoding='utf-8')
    lidar_start = config.index('Name: Lidar')
    lidar_end = config.index('Name: RGBD PointCloud')
    lidar_config = config[lidar_start:lidar_end]
    assert 'Value: /scan_nav' in lidar_config
    assert 'Reliability Policy: Best Effort' in lidar_config


def test_nav2_launch_uses_open_source_navigation_stack():
    path = Path(__file__).parents[1] / 'launch' / 'nav2_waypoint_nav.launch.py'
    spec = importlib.util.spec_from_file_location('nav2_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    names = {entity.name for entity in description.entities
             if isinstance(entity, DeclareLaunchArgument)}
    assert {
        'waypoint_file',
        'send_waypoints',
        'use_rviz',
        'use_diagnostics',
        'use_rqt_plots',
        'entity_name',
        'use_ekf_localization',
    } <= names

    launch_text = path.read_text(encoding='utf-8')
    assert "package='pointcloud_to_laserscan'" in launch_text
    assert "('cloud_in', '/scan/points')" in launch_text
    assert "('scan', '/scan_nav')" in launch_text


def test_nav2_uses_projected_scan_and_3d_voxel_layer():
    bringup_dir = Path(__file__).parents[1]
    params = (bringup_dir / 'config' / 'nav2_ackermann_params.yaml').read_text(
        encoding='utf-8'
    )
    assert 'topic: "/scan_nav"' in params
    assert 'plugin: "nav2_costmap_2d::VoxelLayer"' in params
    assert 'topic: /scan/points' in params
    assert 'polygons: ["PolygonStop", "PolygonSlow", "FootprintApproach"]' in params


def test_unknown_obstacle_scenario_uses_free_static_map():
    bringup_dir = Path(__file__).parents[1]
    scenario_launch = (
        bringup_dir / 'launch' / 'static_map_scenarios.launch.py'
    ).read_text(encoding='utf-8')
    free_map = (bringup_dir / 'maps' / 'free_navigation.yaml').read_text(
        encoding='utf-8'
    )
    assert "'unknown_obstacle'" in scenario_launch
    assert "'free_navigation.yaml'" in scenario_launch
    assert "'unknown_obstacle'," in scenario_launch
    assert 'image: free_navigation.pgm' in free_map

    pgm_lines = (bringup_dir / 'maps' / 'free_navigation.pgm').read_text(
        encoding='ascii'
    ).splitlines()
    pixels = [int(value) for line in pgm_lines[4:] for value in line.split()]
    width, height = 200, 160
    interior = [
        pixels[y * width + x]
        for y in range(2, height - 2)
        for x in range(2, width - 2)
    ]
    assert len(pixels) == width * height
    assert set(interior) == {254}


def test_unknown_obstacle_world_contains_only_the_test_obstacle():
    """Unknown-obstacle baseline is not confounded by display objects."""
    world = (
        Path(__file__).parents[2]
        / 'ackermann_line_following_description'
        / 'worlds'
        / 'unknown_obstacle.sdf'
    ).read_text(encoding='utf-8')
    assert '<world name="unknown_obstacle">' in world
    assert '<model name="center_obstacle">' in world
    assert 'left_obstacle' not in world
    assert 'right_obstacle' not in world


def test_complex_static_scenario_has_large_aligned_map_and_route():
    """Large performance scenario packages matching map, world and route."""
    bringup_dir = Path(__file__).parents[1]
    pgm = (bringup_dir / 'maps' / 'complex_static.pgm').read_bytes()
    header, raster = pgm.split(b'255\n', maxsplit=1)
    assert header.startswith(b'P5\n')
    assert b'600 400' in header
    assert len(raster) == 600 * 400
    assert {0, 254} <= set(raster)

    source_root = Path(__file__).parents[2]
    world = (
        source_root
        / 'ackermann_line_following_description'
        / 'worlds'
        / 'complex_static.sdf'
    ).read_text(encoding='utf-8')
    route = (
        source_root
        / 'ackermann_line_following_controller'
        / 'config'
        / 'nav2_complex_static.csv'
    ).read_text(encoding='utf-8')
    assert '<world name="complex_static">' in world
    assert 'shelf_a' in world and 'shelf_e' in world
    assert '-13.0, -8.0' in route and '13.0, 8.0' in route
    large_rviz = (
        bringup_dir / 'rviz' / 'perception_large.rviz'
    ).read_text(encoding='utf-8')
    assert 'Distance: 24' in large_rviz


def test_dynamics_launch_exposes_report_and_rviz_controls():
    path = Path(__file__).parents[1] / 'launch' / 'dynamics_test.launch.py'
    spec = importlib.util.spec_from_file_location('dynamics_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    names = {entity.name for entity in description.entities
             if isinstance(entity, DeclareLaunchArgument)}
    assert {'run_test', 'use_rviz', 'result_file', 'segment_scale'} <= names


def test_static_scenario_launch_exposes_scenario_selector():
    path = Path(__file__).parents[1] / 'launch' / 'static_map_scenarios.launch.py'
    spec = importlib.util.spec_from_file_location('static_scenarios_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    names = {entity.name for entity in description.entities
             if isinstance(entity, DeclareLaunchArgument)}
    assert {
        'scenario',
        'send_waypoints',
        'use_rviz',
        'target_speed',
        'use_diagnostics',
        'use_rqt_plots',
        'record_experiment',
        'experiment_result_file',
        'gz_args',
    } <= names


def test_navigation_launch_wires_diagnostics_and_live_plots():
    """Navigation launch includes telemetry, plots and its RViz marker."""
    bringup_dir = Path(__file__).parents[1]
    launch_text = (
        bringup_dir / 'launch' / 'nav2_waypoint_nav.launch.py'
    ).read_text(encoding='utf-8')
    rviz = (bringup_dir / 'rviz' / 'perception.rviz').read_text(
        encoding='utf-8'
    )
    assert "executable='navigation_diagnostics'" in launch_text
    assert "executable='navigation_plotter'" in launch_text
    assert "executable='navigation_experiment'" in launch_text
    assert "executable='filter_passthrough_node'" in launch_text
    assert "('output', '/scan/points_obstacles')" in launch_text
    assert 'FollowPath.desired_linear_vel' in launch_text
    assert 'Topic: /nav_diagnostics/summary' in rviz
    assert 'Topic: /scan/points_obstacles' in rviz


def test_sim2real_launch_exposes_hardware_command_contract():
    path = Path(__file__).parents[1] / 'launch' / 'sim2real_control.launch.py'
    spec = importlib.util.spec_from_file_location('sim2real_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    names = {
        entity.name
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert {
        'use_sim_time',
        'input_topic',
        'output_topic',
        'wheelbase',
        'max_speed',
        'max_steering',
        'minimum_speed',
        'command_timeout',
        'publish_rate',
    } <= names

    launch_text = path.read_text(encoding='utf-8')
    assert "executable='twist_to_ackermann'" in launch_text
    assert "default_value='/cmd_vel_safe'" in launch_text
    assert "default_value='/drive'" in launch_text


def test_ekf_launch_removes_ground_truth_tf_ownership():
    bringup_dir = Path(__file__).parents[1]
    launch_text = (
        bringup_dir / 'launch' / 'ekf_localization_test.launch.py'
    ).read_text(encoding='utf-8')
    config = (bringup_dir / 'config' / 'ekf_sim.yaml').read_text(
        encoding='utf-8'
    )
    assert "'ground_truth_tf_output': '/tf_ground_truth'" in launch_text
    assert "package='robot_localization'" in launch_text
    assert 'odom0: /model/ackermann_car/wheel_odometry' in config
    assert 'imu0: /imu/data_raw' in config
    assert 'publish_tf: true' in config

    nav_launch = (
        bringup_dir / 'launch' / 'nav2_waypoint_nav.launch.py'
    ).read_text(encoding='utf-8')
    assert "if_value='/odometry/filtered'" in nav_launch
    assert "if_value='/tf_ground_truth'" in nav_launch
    assert "package='robot_localization'" in nav_launch
