"""Validate indoor launch modes, TF ownership and packaged scene geometry."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
import pytest
import yaml


ROOT = Path(__file__).parents[1]


def indoor_module(name='indoor_navigation'):
    spec = importlib.util.spec_from_file_location(
        'indoor_launch', ROOT / 'launch' / f'{name}.launch.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_indoor_mapping_launch_requires_no_prior_map():
    context = LaunchContext()
    context.launch_configurations.update(mode='mapping', map_file='')
    assert len(indoor_module()._launch(context)) == 1
    assert indoor_module().generate_launch_description().entities


@pytest.mark.parametrize('name', ['indoor_navigation', 'indoor_semantic'])
def test_indoor_speed_default_and_forwarding(name):
    module = indoor_module(name)
    argument = next(entity for entity in module.generate_launch_description().entities
                    if getattr(entity, 'name', None) == 'target_speed')
    assert ''.join(part.perform(LaunchContext()) for part in argument.default_value) == '0.35'
    text = (ROOT / 'launch' / f'{name}.launch.py').read_text()
    assert "'target_speed': LaunchConfiguration('target_speed')" in text


def test_indoor_localization_rejects_missing_map(tmp_path):
    context = LaunchContext()
    context.launch_configurations.update(
        mode='amcl', map_file=str(tmp_path / 'missing')
    )
    with pytest.raises(ValueError, match='existing map_file'):
        indoor_module()._launch(context)
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\n')
    context.launch_configurations['map_file'] = str(path)
    assert len(indoor_module()._launch(context)) == 1


def test_indoor_scene_has_100sqm_interior_and_articulated_furniture():
    world = (
        ET.parse(
            ROOT.parent
            / 'ackermann_line_following_description'
            / 'worlds'
            / 'indoor_100sqm.sdf'
        )
        .getroot()
        .find('world')
    )
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
    assert (
        "EqualsSubstitution(LaunchConfiguration('localization_mode'), 'static')" in text
    )
    assert "'input_topic': '/drive'" in text
    assert "cmd_vel_out_topic': '/cmd_vel_safe'" in text


def test_progress_threshold_allows_safety_scaled_approach():
    config = yaml.safe_load((ROOT / 'config' / 'nav2_ackermann_params.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    safety = config['collision_monitor']['ros__parameters']
    progress = controller['progress_checker']
    speed = (controller['FollowPath']['min_approach_linear_velocity']
             * safety['PolygonSlow']['slowdown_ratio'])
    assert 0 < progress['required_movement_radius'] < speed * progress['movement_time_allowance']
    assert progress['movement_time_allowance'] <= 15.0
    assert config['bt_navigator']['ros__parameters']['default_server_timeout'] == 500


def test_lidar_scan_pipeline_uses_intra_process_transport():
    text = (ROOT / 'launch' / 'nav2_waypoint_nav.launch.py').read_text()
    assert "'bridge_lidar_points': 'false'" in text
    assert 'slam_scan, lidar_scan_projection' in text
    assert text.count("'use_intra_process_comms': True") == 3
    bridge = yaml.safe_load((ROOT / 'config' / 'lidar_bridge.yaml').read_text())
    assert len(bridge) == 1
    assert bridge[0]['topic_name'] == '/scan/points'
    assert bridge[0]['direction'] == 'GZ_TO_ROS'


def test_semantic_demo_loads_sensor_map_and_recorded_landmarks():
    context = LaunchContext()
    context.launch_configurations.update(
        map_file=str(ROOT / 'maps' / 'indoor_slam.yaml'),
        database=str(ROOT / 'maps' / 'indoor_landmarks.json'),
    )
    module = indoor_module('indoor_semantic')
    assert module.generate_launch_description().entities
    assert len(module._launch(context)) == 2


def test_semantic_demo_rejects_missing_database(tmp_path):
    context = LaunchContext()
    context.launch_configurations.update(
        map_file=str(ROOT / 'maps' / 'indoor_slam.yaml'),
        database=str(tmp_path / 'missing.json'),
    )
    with pytest.raises(FileNotFoundError):
        indoor_module('indoor_semantic')._launch(context)


def test_indoor_rviz_uses_map_for_initial_pose_and_named_goals():
    config = yaml.safe_load((ROOT / 'rviz' / 'indoor_semantic.rviz').read_text())
    manager = config['Visualization Manager']
    assert manager['Global Options']['Fixed Frame'] == 'map'
    assert any(tool['Class'] == 'nav2_rviz_plugins/GoalTool' for tool in manager['Tools'])
    # GoalTool emits a Qt signal; the Nav2 panel owns the action client.
    assert any(panel['Class'] == 'nav2_rviz_plugins/Navigation 2'
               for panel in config['Panels'])
    marker = next(display for display in manager['Displays']
                  if display.get('Name') == 'Semantic Approach Poses')
    assert marker['Topic']['Durability Policy'] == 'Transient Local'


def test_visual_navigation_launch_supports_record_and_shadow(tmp_path):
    module = indoor_module('visual_navigation')
    context = LaunchContext()
    context.launch_configurations.update(
        mode='record', dataset=str(tmp_path / 'dataset'), rate='5.0',
        image_topic='/rgbd/image', teacher_topic='/cmd_vel_smoothed',
        plan_topic='/plan', model='', metrics=str(tmp_path / 'metrics.json'),
    )
    assert module._launch(context)[0].node_executable == 'visual_dataset_recorder'
    model = tmp_path / 'policy.yml'
    model.write_text('placeholder', encoding='utf-8')
    context.launch_configurations.update(mode='shadow', model=str(model))
    assert module._launch(context)[0].node_executable == 'visual_policy'


def test_visual_navigation_launch_rejects_bad_mode_and_model(tmp_path):
    module = indoor_module('visual_navigation')
    context = LaunchContext()
    context.launch_configurations.update(
        mode='bad', dataset=str(tmp_path), rate='5.0',
        image_topic='/rgbd/image', teacher_topic='/cmd_vel_smoothed',
        plan_topic='/plan', model='', metrics=str(tmp_path / 'metrics.json'),
    )
    with pytest.raises(ValueError, match='mode'):
        module._launch(context)
    context.launch_configurations['mode'] = 'shadow'
    with pytest.raises(FileNotFoundError, match='does not exist'):
        module._launch(context)
