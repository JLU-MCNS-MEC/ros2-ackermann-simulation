"""Regression runner validation without commanding a robot."""

import json
from unittest.mock import MagicMock

from nav2_simple_commander.robot_navigator import TaskResult
from nav_msgs.msg import Odometry
import pytest
from rclpy.time import Time
from std_msgs.msg import Float64

from ackermann_line_following_controller import navigation_regression as module


@pytest.mark.parametrize('data', [[], {}, [None], [{'name': 'x', 'pose': [0, 0]}],
                                  [{'name': 'x', 'pose': [True, 0, 0]}],
                                  [{'name': 'x', 'pose': [float('nan'), 0, 0]}]])
def test_invalid_routes(tmp_path, data):
    path = tmp_path / 'route.json'
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        module.load_route(path)


def test_route_and_summary(tmp_path):
    goals = [{'name': 'desk', 'pose': [0, 1, 0]}]
    path = tmp_path / 'route.json'
    path.write_text(json.dumps(goals))
    assert module.load_route(path) == goals
    assert not module.summarize_trials([], 20)['all_passed']
    trial = {'success': True, 'wall_duration_s': 2, 'recoveries': 1}
    assert module.summarize_trials([trial], 1)['all_passed']
    assert module.summarize_trials([trial], 2)['success_rate_of_requested'] == 0.5


def test_sensor_health_gaps_and_repeated_timestamps():
    health = module.SensorHealth()
    assert health.report(0) == {}
    health.observe('imu', 1, 1)
    health.observe('imu', 1.1, 1.01)
    health.observe('imu', 1.2, 1.01)
    report = health.report(2)['imu']
    assert report['count'] == 3
    assert report['nonmonotonic_stamps'] == 1
    assert report['wall_rate_hz'] == pytest.approx(10)
    assert report['last_receipt_age_s'] == pytest.approx(0.8)


@pytest.mark.parametrize('accepted,outcome,count', [
    (True, TaskResult.SUCCEEDED, 2), (True, TaskResult.FAILED, 1),
    (False, TaskResult.FAILED, 1),
])
def test_runner_success_failure_rejection(tmp_path, monkeypatch, accepted, outcome, count):
    nav = MagicMock()
    callbacks = {}
    nav.create_subscription.side_effect = lambda typ, name, cb, qos: callbacks.update({name: cb})
    nav.get_clock.return_value.now.return_value = Time(seconds=1)
    nav.goToPose.return_value = accepted
    nav.getResult.return_value = outcome
    nav.isTaskComplete.side_effect = [False, True] * 2
    nav.getFeedback.return_value.number_of_recoveries = 0

    def spin(*args, **kwargs):
        for name in ['/scan_nav', '/imu/data_raw', '/odometry/filtered']:
            callbacks[name](Odometry())
        for name in module.NavigationExperimentNode.VALUE_NAMES:
            callbacks['/nav_diagnostics/' + name](Float64(data=0.1))

    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    path = tmp_path / 'result.json'
    result = module.run_trials(nav, [{'name': 'goal', 'pose': [0, 0, 0]}], 2, 1, path)
    assert result == (outcome == TaskResult.SUCCEEDED)
    report = json.loads(path.read_text())
    assert report['completed_trials'] == count
    assert nav.destroy_subscription.call_count == 12


@pytest.mark.parametrize('passed', [True, False])
def test_main_lifecycle_and_exit(tmp_path, monkeypatch, passed):
    route = tmp_path / 'route.json'
    route.write_text('[{"name":"goal","pose":[0,0,0]}]')
    nav = MagicMock()
    monkeypatch.setattr(module, 'BasicNavigator', lambda **kwargs: nav)
    monkeypatch.setattr(module.rclpy, 'init', MagicMock())
    shutdown = MagicMock()
    monkeypatch.setattr(module.rclpy, 'shutdown', shutdown)
    monkeypatch.setattr(module, 'run_trials', lambda *args: passed)
    args = ['--route', str(route), '--output', str(tmp_path / 'out.json')]
    if passed:
        module.main(args)
    else:
        with pytest.raises(SystemExit):
            module.main(args)
    nav.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_main_rejects_invalid_limits_and_existing_result(tmp_path):
    with pytest.raises(SystemExit):
        module.main(['--route', 'unused', '--output', 'unused', '--count', '0'])
    path = tmp_path / 'route.json'
    path.write_text('[{"name":"goal","pose":[0,0,0]}]')
    with pytest.raises(FileExistsError):
        module.main(['--route', str(path), '--output', str(path)])
