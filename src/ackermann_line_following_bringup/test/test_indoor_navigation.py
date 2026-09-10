"""Validate indoor launch modes, TF ownership and packaged scene geometry."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
import pytest
import yaml


ROOT = Path(__file__).parents[1]


def indoor_module():
    spec = importlib.util.spec_from_file_location(
        'indoor_launch', ROOT / 'launch' / 'indoor_navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_indoor_mapping_launch_requires_no_prior_map():
    context = LaunchContext()
    context.launch_configurations.update(mode='mapping', map_file='')
    assert len(indoor_module()._launch(context)) == 1
    assert indoor_module().generate_launch_description().entities


def test_indoor_localization_rejects_missing_map(tmp_path):
    context = LaunchContext()
    context.launch_configurations.update(mode='amcl', map_file=str(tmp_path / 'missing'))
    with pytest.raises(ValueError, match='existing map_file'):
        indoor_module()._launch(context)
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\n')
    context.launch_configurations['map_file'] = str(path)
    assert len(indoor_module()._launch(context)) == 1


def test_indoor_scene_has_100sqm_interior_and_articulated_furniture():
    world = ET.parse(ROOT.parent / 'ackermann_line_following_description'
                     / 'worlds' / 'indoor_100sqm.sdf').getroot().find('world')
    models = {model.attrib['name']: model for model in world.findall('model')}
    assert float(models['wall_east'].findtext('pose').split()[0]) == 5.1
    assert float(models['wall_west'].findtext('pose').split()[0]) == -5.1
    assert len(models['office_table'].findall('link')) == 5
    assert len(models['red_chair'].findall('link')) == 6
    assert len(models['storage_shelf'].findall('link')) == 7


def test_slam_uses_separate_scan_and_control_boundary():
    config = yaml.safe_load((ROOT / 'config' / 'slam_indoor.yaml').read_text())
    assert config['slam_toolbox']['ros__parameters']['scan_topic'] == '/scan_slam'
    text = (ROOT / 'launch' / 'nav2_waypoint_nav.launch.py').read_text()
    assert "EqualsSubstitution(LaunchConfiguration('localization_mode'), 'static')" in text
    assert "'input_topic': '/drive'" in text
    assert "cmd_vel_out_topic': '/cmd_vel_safe'" in text
