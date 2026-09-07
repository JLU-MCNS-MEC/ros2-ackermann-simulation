"""Unit tests for repeatable navigation experiment metrics."""

import math

from ackermann_line_following_controller.navigation_experiment_node import (
    ExperimentAccumulator,
)
import pytest


def sample_values(**overrides: float) -> dict[str, float]:
    """Return one complete diagnostic input snapshot."""
    values = {
        'speed/commanded': 0.2,
        'speed/smoothed': 0.19,
        'speed/applied': 0.18,
        'speed/measured': 0.16,
        'navigation/cross_track_error': 0.1,
        'navigation/distance_to_goal': 2.0,
        'obstacle/front_clearance': 1.5,
        'obstacle/nearest_clearance': 0.8,
        'decision/action_code': 0.0,
    }
    values.update(overrides)
    return values


def test_accumulator_reports_route_quality_and_safety_metrics() -> None:
    """Summary contains distance, tracking error and minimum clearance."""
    recorder = ExperimentAccumulator('obstacle')
    recorder.begin(10.0)
    recorder.add_pose(0.0, 0.0)
    recorder.add_pose(0.3, 0.4)
    recorder.add_sample(10.1, sample_values())
    recorder.add_sample(
        10.2,
        sample_values(**{
            'speed/measured': 0.14,
            'navigation/cross_track_error': -0.3,
            'obstacle/front_clearance': 0.7,
            'decision/action_code': 2.0,
        }),
    )

    report = recorder.summary('SUCCEEDED', 12.0)

    assert report['success'] is True
    assert report['duration_s'] == pytest.approx(2.0)
    assert report['effective_sample_rate_hz'] == pytest.approx(1.0)
    assert report['time_to_first_command_s'] == pytest.approx(0.1)
    assert report['time_to_first_motion_s'] == pytest.approx(0.1)
    assert report['distance_travelled_m'] == pytest.approx(0.5)
    assert report['mean_abs_cross_track_error_m'] == pytest.approx(0.2)
    assert report['max_abs_cross_track_error_m'] == pytest.approx(0.3)
    assert report['minimum_front_clearance_m'] == pytest.approx(0.7)
    assert report['decision_transition_count'] == 1
    assert report['decision_sample_counts']['SLOWDOWN'] == 1


def test_accumulator_ignores_pose_before_start_and_missing_data() -> None:
    """Pre-run odometry and unavailable diagnostics do not corrupt metrics."""
    recorder = ExperimentAccumulator('straight')
    recorder.add_pose(5.0, 5.0)
    recorder.begin(1.0)
    values = sample_values(**{
        'navigation/cross_track_error': math.nan,
        'obstacle/front_clearance': math.nan,
        'obstacle/nearest_clearance': math.nan,
        'decision/action_code': math.nan,
    })
    recorder.add_sample(1.1, values)

    report = recorder.summary('FAILED', 1.5)

    assert report['success'] is False
    assert report['distance_travelled_m'] == 0.0
    assert report['mean_abs_cross_track_error_m'] is None
    assert report['minimum_front_clearance_m'] is None
    assert report['decision_sample_counts']['CLEAR'] == 1


def test_accumulator_validates_lifecycle() -> None:
    """Invalid labels, timestamps and premature summaries fail clearly."""
    with pytest.raises(ValueError, match='scenario'):
        ExperimentAccumulator('  ')
    recorder = ExperimentAccumulator('offset')
    with pytest.raises(ValueError, match='timestamp'):
        recorder.begin(math.nan)
    with pytest.raises(RuntimeError, match='has not started'):
        recorder.summary('INCOMPLETE', 1.0)
