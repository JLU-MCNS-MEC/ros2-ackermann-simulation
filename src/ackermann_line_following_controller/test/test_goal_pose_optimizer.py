"""Test position-first Ackermann goal orientation selection."""

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy

from ackermann_line_following_controller.goal_pose_optimizer import (
    GoalPoseOptimizer,
    optimized_goal_pose,
    preferred_goal_yaw,
)


def test_prefers_straight_forward_and_reverse_arrivals():
    """Goals on the vehicle axis do not create unnecessary turns."""
    assert preferred_goal_yaw(0, 0, 0, 2, 0) == pytest.approx(0.0)
    assert preferred_goal_yaw(0, 0, 0, -2, 0) == pytest.approx(0.0, abs=1e-6)


def test_side_goal_points_along_travel_direction():
    """A lateral target receives a useful arrival direction."""
    assert preferred_goal_yaw(0, 0, 0, 0, 2) == pytest.approx(math.pi / 2)
    assert preferred_goal_yaw(
        0, 0, 0, 0, 2, allow_reverse=False
    ) == pytest.approx(math.pi / 2)


def test_near_goal_keeps_current_heading():
    """Tiny position corrections do not flip the goal orientation."""
    assert preferred_goal_yaw(0, 0, 0.7, 0.1, 0.1) == pytest.approx(0.7)


def test_preferred_goal_yaw_rejects_invalid_values():
    """Nonfinite poses and negative tolerances are rejected."""
    with pytest.raises(ValueError, match='finite'):
        preferred_goal_yaw(0, 0, float('nan'), 1, 1)
    with pytest.raises(ValueError, match='nonnegative'):
        preferred_goal_yaw(0, 0, 0, 1, 1, position_tolerance=-0.1)


def test_optimized_pose_preserves_position_and_input():
    """Optimization copies the goal instead of mutating the caller's message."""
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = -2.0
    goal.pose.orientation.w = 1.0
    output = optimized_goal_pose(goal, 0.0, 0.0, 0.0)
    assert output.pose.position.x == -2.0
    assert abs(output.pose.orientation.z) < 1.0e-6
    assert output.pose.orientation.w == pytest.approx(1.0)
    assert goal.pose.orientation.w == 1.0


def test_optimized_pose_requires_frame():
    """A frameless RViz goal is not safe to relay."""
    with pytest.raises(ValueError, match='frame_id'):
        optimized_goal_pose(PoseStamped(), 0.0, 0.0, 0.0)


@pytest.fixture
def optimizer_node():
    """Create a real ROS node while replacing only its external interfaces."""
    if not rclpy.ok():
        rclpy.init(args=[])
    node = GoalPoseOptimizer()
    node.publisher = MagicMock()
    node.debug_publisher = MagicMock()
    yield node
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_goal_callback_relays_optimized_pose(optimizer_node):
    """A valid TF produces one identical debug and navigation goal."""
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    optimizer_node.buffer = MagicMock()
    optimizer_node.buffer.lookup_transform.return_value = transform
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = 2.0

    optimizer_node._goal_callback(goal)

    optimizer_node.publisher.publish.assert_called_once()
    optimizer_node.debug_publisher.publish.assert_called_once()
    output = optimizer_node.publisher.publish.call_args.args[0]
    assert output.header.frame_id == 'map'
    assert output.pose.position.x == 2.0
    assert output.pose.orientation.w == pytest.approx(1.0)


def test_goal_callback_drops_invalid_goal(optimizer_node):
    """A goal without a frame is reported and never reaches Nav2."""
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    optimizer_node.buffer = MagicMock()
    optimizer_node.buffer.lookup_transform.return_value = transform

    optimizer_node._goal_callback(PoseStamped())

    optimizer_node.publisher.publish.assert_not_called()
    optimizer_node.debug_publisher.publish.assert_not_called()
