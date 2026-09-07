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
    } <= names


def test_nav2_launch_uses_open_source_navigation_stack():
    path = Path(__file__).parents[1] / 'launch' / 'nav2_waypoint_nav.launch.py'
    spec = importlib.util.spec_from_file_location('nav2_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    names = {entity.name for entity in description.entities
             if isinstance(entity, DeclareLaunchArgument)}
    assert {'waypoint_file', 'send_waypoints', 'use_rviz', 'entity_name'} <= names
