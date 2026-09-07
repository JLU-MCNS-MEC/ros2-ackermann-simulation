import math

import pytest
from sensor_msgs.msg import LaserScan

from ackermann_line_following_controller.waypoint_tracker_node import (
    Pose2D,
    Waypoint,
    load_waypoints,
    pure_pursuit_steering,
    quaternion_to_yaw,
    sector_min_range,
    select_lookahead_target,
)


def test_load_waypoints_skips_comments(tmp_path) -> None:
    waypoint_file = tmp_path / 'route.csv'
    waypoint_file.write_text(
        '# demo\n0.0, 0.0\n1.0 0.5 # corner\n',
        encoding='utf-8',
    )
    assert load_waypoints(str(waypoint_file)) == [
        Waypoint(0.0, 0.0),
        Waypoint(1.0, 0.5),
    ]


def test_load_waypoints_requires_two_finite_points(tmp_path) -> None:
    waypoint_file = tmp_path / 'route.csv'
    waypoint_file.write_text('0.0,0.0\n', encoding='utf-8')
    with pytest.raises(ValueError, match='At least two'):
        load_waypoints(str(waypoint_file))


def test_target_selection_and_pure_pursuit() -> None:
    route = [Waypoint(0.0, 0.0), Waypoint(1.0, 0.0), Waypoint(2.0, 1.0)]
    pose = Pose2D(0.0, 0.0, 0.0)
    target, index, goal_distance = select_lookahead_target(route, pose, 0.8)
    assert index == 1
    assert target == Waypoint(1.0, 0.0)
    assert goal_distance == pytest.approx(math.sqrt(5.0))
    assert pure_pursuit_steering(
        pose,
        Waypoint(1.0, 1.0),
        wheelbase=0.56,
        max_steering=0.48,
    ) == pytest.approx(0.48)


def test_sector_min_range_uses_range_max_for_missing_returns() -> None:
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = math.pi / 2.0
    scan.range_min = 0.12
    scan.range_max = 12.0
    scan.ranges = [float('inf'), float('inf'), 2.0, 1.5, float('inf')]
    assert sector_min_range(scan, -0.1, 0.1) == pytest.approx(2.0)
    assert sector_min_range(scan, 1.4, 1.8) == pytest.approx(1.5)
    assert sector_min_range(scan, -2.0, -1.8) == pytest.approx(12.0)


def test_quaternion_to_yaw() -> None:
    assert quaternion_to_yaw(
        0.0,
        0.0,
        math.sin(math.pi / 4),
        math.cos(math.pi / 4),
    ) == pytest.approx(math.pi / 2)
