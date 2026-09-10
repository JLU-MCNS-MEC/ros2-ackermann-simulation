"""Check the final safety boundary without publishing to a running vehicle."""

from types import SimpleNamespace
from unittest.mock import MagicMock

from geometry_msgs.msg import Twist
import pytest
from rclpy.time import Time

from ackermann_line_following_controller import twist_to_ackermann_node as module


def boundary():
    return SimpleNamespace(
        wheelbase=0.56, max_speed=0.6, max_steering=0.55,
        minimum_speed=1.0e-6, command_timeout=0.25,
        _speed=0.0, _steering=0.0, _last_command_time=None,
        _reported_stationary_turn=False, get_logger=MagicMock(),
        publisher=MagicMock(), get_clock=MagicMock(),
    )


def test_callback_preserves_safety_scaled_turn_and_accepts_stop():
    node = boundary()
    message = Twist()
    message.linear.x = 0.0175
    message.angular.z = 0.01
    module.TwistToAckermannNode.command_callback(node, message)
    assert node._speed == pytest.approx(0.0175)
    assert node._steering > 0.0
    node.get_logger.return_value.warning.assert_not_called()
    module.TwistToAckermannNode.command_callback(node, Twist())
    assert (node._speed, node._steering) == (0.0, 0.0)


def test_invalid_command_does_not_refresh_watchdog():
    node = boundary()
    message = Twist()
    message.linear.x = float('nan')
    module.TwistToAckermannNode.command_callback(node, message)
    assert node._last_command_time is None
    node.get_logger.return_value.warning.assert_called_once()


@pytest.mark.parametrize('last_time,expected', [(None, 0.0), (1.0, 0.0), (1.9, 0.1)])
def test_watchdog_forces_stop(last_time, expected, monkeypatch):
    node = boundary()
    node._last_command_time = last_time
    node._speed = 0.1
    node._steering = 0.3
    node.get_clock.return_value.now.return_value = Time()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 2.0)
    module.TwistToAckermannNode._publish(node)
    message = node.publisher.publish.call_args.args[0]
    assert message.drive.speed == pytest.approx(expected)
    assert message.drive.steering_angle == pytest.approx(0.3 if expected else 0.0)


def test_stationary_rotation_warns_only_once():
    node = boundary()
    message = Twist()
    message.angular.z = 0.5
    for _ in range(2):
        module.TwistToAckermannNode.command_callback(node, message)
    assert (node._speed, node._steering) == (0.0, 0.0)
    node.get_logger.return_value.warning.assert_called_once()


def test_shutdown_publishes_zero():
    node = boundary()
    node.get_clock.return_value.now.return_value = Time()
    module.TwistToAckermannNode.publish_stop(node)
    message = node.publisher.publish.call_args.args[0]
    assert message.drive.speed == 0.0
    assert message.drive.steering_angle == 0.0
