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
    assert {'enable_line_follower', 'start_x', 'start_y', 'gz_args'} <= names
