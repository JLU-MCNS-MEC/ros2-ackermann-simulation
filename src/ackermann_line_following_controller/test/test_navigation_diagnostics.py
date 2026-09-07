"""Unit tests for navigation telemetry calculations."""

import math
import pytest
from sensor_msgs.msg import LaserScan

from ackermann_line_following_controller.navigation_diagnostics_node import (
    path_metrics,
    scan_clearances,
    steering_from_twist,
)
from ackermann_line_following_controller.navigation_plotter import plot_groups


def test_steering_from_twist_uses_ackermann_bicycle_geometry() -> None:
    """Twist conversion follows the bicycle model and validates geometry."""
    steering = steering_from_twist(0.2, 0.1, 0.56)
    assert steering == pytest.approx(math.atan(0.28))
    assert steering_from_twist(0.0, 1.0, 0.56) == 0.0
    with pytest.raises(ValueError, match='wheelbase must be positive'):
        steering_from_twist(0.2, 0.1, 0.0)


def test_path_metrics_projects_onto_segments_and_handles_missing_path(
) -> None:
    """Cross-track error projects onto segments and tolerates no plan."""
    metrics = path_metrics(
        1.0, 0.4, [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    )
    assert metrics.cross_track_error == pytest.approx(0.4)
    assert metrics.distance_to_goal == pytest.approx(math.hypot(1.0, 1.6))
    missing = path_metrics(0.0, 0.0, [])
    assert math.isnan(missing.cross_track_error)
    assert math.isnan(missing.distance_to_goal)


def test_scan_clearances_separates_front_from_full_mid360_scan() -> None:
    """Front clearance excludes a nearer return behind the vehicle."""
    scan = LaserScan()
    scan.angle_min = -math.pi
    scan.angle_increment = math.pi / 2.0
    scan.range_min = 0.1
    scan.range_max = 12.0
    scan.ranges = [0.3, 2.0, 0.8, float('inf'), 1.5]
    nearest, front = scan_clearances(scan, math.radians(30.0))
    assert nearest == pytest.approx(0.3)
    assert front == pytest.approx(0.8)


def test_plot_groups_cover_control_and_safety_decisions() -> None:
    """Live plots include the requested speed and decision measurements."""
    groups = plot_groups()
    topics = {topic for _, group_topics in groups for topic in group_topics}
    assert len(groups) == 2
    assert '/nav_diagnostics/speed/measured/data' in topics
    assert '/nav_diagnostics/obstacle/front_clearance/data' in topics
    assert '/nav_diagnostics/decision/action_code/data' in topics
