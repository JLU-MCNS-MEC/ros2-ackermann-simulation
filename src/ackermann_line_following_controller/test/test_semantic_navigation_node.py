"""Exercise semantic command execution using a mocked navigation backend."""

import json
from types import SimpleNamespace
from unittest.mock import MagicMock

from geometry_msgs.msg import TransformStamped
from nav2_simple_commander.robot_navigator import TaskResult
import pytest
import rclpy
from std_msgs.msg import String

from ackermann_line_following_controller import semantic_navigation_node as module
from ackermann_line_following_controller.semantic_map import (
    save_landmarks,
    load_landmarks,
)


def options(tmp_path):
    return SimpleNamespace(
        database=str(tmp_path / 'semantic.json'),
        timeout=180,
        record='入口',
        alias=['起点'],
    )


def test_record_localized_pose_and_reject_duplicate(tmp_path, monkeypatch):
    args = options(tmp_path)
    navigator = MagicMock()
    navigator.get_clock.return_value.now.return_value.nanoseconds = 5_000_000_000
    transform = TransformStamped()
    transform.header.stamp.sec = 5
    transform.transform.translation.x = 1.5
    transform.transform.rotation.w = 1.0
    buffer = MagicMock()
    buffer.lookup_transform.return_value = transform
    monkeypatch.setattr(module, 'Buffer', lambda: buffer)
    monkeypatch.setattr(module, 'TransformListener', MagicMock())
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'spin_once', lambda *a, **k: None)
    module.record_landmark(navigator, args, 'map-a')
    assert load_landmarks(args.database, 'map-a')[0]['approach_pose'] == [1.5, 0, 0]
    with pytest.raises(ValueError, match='already exists'):
        module.record_landmark(navigator, args, 'map-a')


def test_record_requires_live_localization(tmp_path, monkeypatch):
    monkeypatch.setattr(module, 'Buffer', MagicMock())
    monkeypatch.setattr(module, 'TransformListener', MagicMock())
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    with pytest.raises(RuntimeError, match='localize first'):
        module.record_landmark(MagicMock(), options(tmp_path), 'a')


@pytest.mark.parametrize(
    'query,ready,accepted,result,expected',
    [
        ('入口', True, True, TaskResult.SUCCEEDED, 'succeeded'),
        ('入口', True, True, TaskResult.FAILED, 'failed'),
        ('入口', True, True, TaskResult.CANCELED, 'canceled'),
        ('入口', False, True, TaskResult.SUCCEEDED, 'rejected'),
        ('入口', True, False, TaskResult.SUCCEEDED, 'rejected'),
        ('不存在', True, True, TaskResult.SUCCEEDED, 'rejected'),
    ],
)
def test_serve_goal_outcomes(
    tmp_path, monkeypatch, query, ready, accepted, result, expected
):
    args = options(tmp_path)
    save_landmarks(
        args.database,
        'a',
        [{'name': '入口', 'source': 'manual_pose', 'approach_pose': [0, 0, 0]}],
    )
    navigator = MagicMock()
    navigator.get_clock.return_value.now.return_value.to_msg.return_value = (
        module.rclpy.time.Time().to_msg()
    )
    navigator.nav_to_pose_client.server_is_ready.return_value = ready
    navigator.goToPose.return_value = accepted
    navigator.isTaskComplete.return_value = True
    navigator.getResult.return_value = result
    callbacks = []
    navigator.create_subscription.side_effect = lambda _t, _n, cb, _q: callbacks.append(
        cb
    )
    iterations = iter([True, True, False])
    monkeypatch.setattr(rclpy, 'ok', lambda: next(iterations))
    calls = [0]

    def spin(*args, **kwargs):
        if calls[0] == 0:
            callbacks[0](String(data=query))
        calls[0] += 1

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    module.serve(navigator, args, 'a')
    messages = [
        call
        for call in navigator.create_publisher.return_value.publish.call_args_list
        if isinstance(call.args[0], String)
    ]
    assert json.loads(messages[-1].args[0].data)['state'] == expected


def test_serve_busy_and_cancel(tmp_path, monkeypatch):
    args = options(tmp_path)
    save_landmarks(
        args.database,
        'a',
        [{'name': '入口', 'source': 'manual_pose', 'approach_pose': [0, 0, 0]}],
    )
    navigator = MagicMock()
    navigator.get_clock.return_value.now.return_value.to_msg.return_value = (
        module.rclpy.time.Time().to_msg()
    )
    navigator.isTaskComplete.return_value = False
    callbacks = []
    navigator.create_subscription.side_effect = lambda _t, _n, cb, _q: callbacks.append(
        cb
    )
    iterations = iter([True, True, False])
    monkeypatch.setattr(rclpy, 'ok', lambda: next(iterations))
    calls = [0]

    def spin(*args, **kwargs):
        callbacks[0](String(data='入口'))
        if calls[0] == 1:
            callbacks[0](String(data='__cancel__'))
        calls[0] += 1

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    module.serve(navigator, args, 'a')
    navigator.cancelTask.assert_called_once()
    states = [
        json.loads(call.args[0].data)['state']
        for call in navigator.create_publisher.return_value.publish.call_args_list
        if isinstance(call.args[0], String)
    ]
    assert 'rejected' in states


def test_empty_and_named_markers():
    stamp = rclpy.time.Time().to_msg()
    assert not module.landmark_markers([], stamp).markers
    markers = module.landmark_markers(
        [{'name': '桌旁', 'approach_pose': [1, 2, 0]}], stamp
    ).markers
    assert markers[0].text == '桌旁'
    assert markers[0].pose.position.x == 1
    assert markers[0].header.frame_id == 'map'


@pytest.mark.parametrize('record', [False, True])
def test_main_routes_modes_and_cleans_up(monkeypatch, record):
    navigator = MagicMock()
    monkeypatch.setattr(module, 'BasicNavigator', lambda **kwargs: navigator)
    monkeypatch.setattr(module, 'map_identity', lambda path: 'map-a')
    recorder, server = MagicMock(), MagicMock()
    monkeypatch.setattr(module, 'record_landmark', recorder)
    monkeypatch.setattr(module, 'serve', server)
    monkeypatch.setattr(rclpy, 'init', MagicMock())
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    shutdown = MagicMock()
    monkeypatch.setattr(rclpy, 'shutdown', shutdown)
    args = ['semantic_navigation', '--map', 'map.yaml', '--database', 'landmarks.json']
    if record:
        args += ['--record', '入口']
    module.main(args)
    (recorder if record else server).assert_called_once()
    navigator.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_main_reports_invalid_database(monkeypatch):
    navigator = MagicMock()
    monkeypatch.setattr(module, 'BasicNavigator', lambda **kwargs: navigator)
    monkeypatch.setattr(
        module, 'map_identity', MagicMock(side_effect=ValueError('bad map'))
    )
    monkeypatch.setattr(rclpy, 'init', MagicMock())
    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    with pytest.raises(SystemExit) as error:
        module.main(
            ['semantic_navigation', '--map', 'map.yaml', '--database', 'landmarks.json']
        )
    assert error.value.code == 1
    navigator.destroy_node.assert_called_once()


def test_main_rejects_invalid_timeout():
    with pytest.raises(SystemExit) as error:
        module.main(
            [
                'semantic_navigation',
                '--map',
                'map.yaml',
                '--database',
                'landmarks.json',
                '--timeout',
                'nan',
            ]
        )
    assert error.value.code == 2
