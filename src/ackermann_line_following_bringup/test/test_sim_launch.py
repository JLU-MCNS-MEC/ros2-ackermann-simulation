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


def test_rviz_configs_color_mid360_points_by_height():
    bringup_dir = Path(__file__).parents[1]
    for config_name in ('dynamics.rviz', 'perception.rviz'):
        config = (bringup_dir / 'rviz' / config_name).read_text(
            encoding='utf-8'
        )
        assert 'Name: MID360 PointCloud' in config
        assert 'Color Transformer: AxisColor' in config
        assert 'Topic: /scan/points' in config


def test_nav2_launch_uses_open_source_navigation_stack():
    path = Path(__file__).parents[1] / 'launch' / 'nav2_waypoint_nav.launch.py'
    spec = importlib.util.spec_from_file_location('nav2_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    names = {entity.name for entity in description.entities
             if isinstance(entity, DeclareLaunchArgument)}
    assert {'waypoint_file', 'send_waypoints', 'use_rviz', 'entity_name'} <= names


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
        'gz_args',
    } <= names
