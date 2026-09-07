import math

from ackermann_line_following_controller.nav2_waypoint_sender import (
    default_waypoint_file,
    poses_from_waypoints,
    wait_for_map,
)
from ackermann_line_following_controller.waypoint_tracker_node import Waypoint
import pytest
import rclpy


def test_poses_from_waypoints_uses_segment_heading() -> None:
    poses = poses_from_waypoints(
        [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0), Waypoint(1.0, 1.0)]
    )
    assert [pose.header.frame_id for pose in poses] == ['map', 'map', 'map']
    assert poses[0].pose.orientation.w == pytest.approx(1.0)
    assert poses[1].pose.orientation.z == pytest.approx(math.sin(math.pi / 4))
    assert poses[2].pose.orientation.z == pytest.approx(math.sin(math.pi / 4))


def test_poses_from_waypoints_requires_a_route() -> None:
    with pytest.raises(ValueError, match='At least two'):
        poses_from_waypoints([Waypoint(0.0, 0.0)])


def test_default_waypoint_file_is_packaged() -> None:
    assert default_waypoint_file().endswith(
        'ackermann_line_following_controller/config/nav2_trajectory.csv'
    )


def test_wait_for_map_times_out_without_a_ros_context(monkeypatch) -> None:
    class FakeNavigator:
        def create_subscription(self, *_args):
            return object()

        def destroy_subscription(self, _subscription):
            return None

    monkeypatch.setattr(rclpy, 'ok', lambda: False)
    assert wait_for_map(FakeNavigator(), timeout_sec=0.0) is False
