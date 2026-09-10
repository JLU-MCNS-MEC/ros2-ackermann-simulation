"""Semantic map persistence, query ambiguity and coordinate binding tests."""

import json
import math

from builtin_interfaces.msg import Time
import pytest

from ackermann_line_following_controller.semantic_map import (
    load_landmarks,
    map_identity,
    resolve_landmark,
    save_landmarks,
    validate_landmark,
)
from ackermann_line_following_controller.semantic_navigation_node import goal_pose


def landmark(name='红椅子', aliases=None):
    return {
        'name': name,
        'aliases': aliases or ['椅子'],
        'source': 'manual_pose',
        'approach_pose': [1.0, 2.0, math.pi / 2],
    }


def test_roundtrip_and_goal_pose(tmp_path):
    path = str(tmp_path / 'nested' / 'semantic.json')
    save_landmarks(path, 'map-a', [landmark()])
    items = load_landmarks(path, 'map-a')
    item = resolve_landmark(items, ' 椅子 ')
    assert item['source'] == 'manual_pose'
    pose = goal_pose(item, Time(sec=5))
    assert pose.header.frame_id == 'map'
    assert pose.header.stamp.sec == 5
    assert pose.pose.position.x == 1
    assert pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))


@pytest.mark.parametrize('query', ['', '不存在'])
def test_unknown_or_empty_query(query):
    with pytest.raises(ValueError):
        resolve_landmark([landmark()], query)


def test_ambiguous_alias_requires_instance_name():
    items = [landmark(), landmark('蓝椅子')]
    with pytest.raises(ValueError, match='Ambiguous'):
        resolve_landmark(items, '椅子')
    assert resolve_landmark(items, '蓝椅子')['name'] == '蓝椅子'


@pytest.mark.parametrize(
    'change',
    [
        {'name': ''},
        {'source': 'gazebo_truth'},
        {'aliases': 'chair'},
        {'aliases': ['']},
        {'approach_pose': [0, 1]},
        {'approach_pose': [0, 1, float('nan')]},
        {'approach_pose': [True, 0, 0]},
    ],
)
def test_invalid_landmark_rejected(change):
    with pytest.raises(ValueError):
        validate_landmark({**landmark(), **change})


def test_nonobject_landmark_rejected():
    with pytest.raises(ValueError):
        validate_landmark(None)


def test_wrong_map_and_duplicate_names_rejected(tmp_path):
    path = str(tmp_path / 'semantic.json')
    save_landmarks(path, 'map-a', [landmark()])
    with pytest.raises(ValueError, match='different'):
        load_landmarks(path, 'map-b')
    with pytest.raises(ValueError, match='Duplicate'):
        save_landmarks(path, 'map-a', [landmark(), landmark()])
    assert len(load_landmarks(path, 'map-a')) == 1


@pytest.mark.parametrize(
    'data',
    [
        [],
        {'version': 2},
        {'version': 1, 'map_id': 'a', 'frame_id': 'map', 'landmarks': None},
    ],
)
def test_invalid_database(tmp_path, data):
    path = tmp_path / 'semantic.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_landmarks(str(path), 'a')


def test_map_identity_tracks_pixels_and_geometry(tmp_path):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n1 1\n255\n\xff')
    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n')
    identity = map_identity(str(map_file))
    image.write_bytes(b'P5\n1 1\n255\n\x00')
    assert identity != map_identity(str(map_file))
    identity = map_identity(str(map_file))
    map_file.write_text('image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n')
    assert identity != map_identity(str(map_file))
    map_file.write_text('[]')
    with pytest.raises(ValueError):
        map_identity(str(map_file))


@pytest.mark.parametrize(
    'content',
    [
        'image: [',
        'image: []\nresolution: 0.1\norigin: [0,0,0]',
        'image: x\nresolution: 0\norigin: [0,0,0]',
        'image: x\nresolution: 0.1\norigin: [0,0]',
    ],
)
def test_invalid_map_geometry(tmp_path, content):
    path = tmp_path / 'map.yaml'
    path.write_text(content)
    with pytest.raises(ValueError):
        map_identity(str(path))
