import math

from ackermann_line_following_controller.dynamics_test_node import (
    default_segments,
    evaluate_segment,
    MotionSegment,
    normalize_angle,
    PoseSample,
)
import pytest


def sample(x: float, y: float, yaw: float, speed: float) -> PoseSample:
    return PoseSample(0.0, x, y, yaw, speed)


def test_default_sequence_covers_forward_reverse_and_turning() -> None:
    segments = default_segments()
    assert any(segment.expected_motion == 'forward' for segment in segments)
    assert any(segment.expected_motion == 'reverse' for segment in segments)
    assert {segment.expected_yaw_sign for segment in segments} >= {-1, 0, 1}


def test_evaluate_segment_checks_signed_distance_and_yaw() -> None:
    segment = MotionSegment('forward_left', 1.0, 0.2, 0.2, 'forward', 1)
    report = evaluate_segment(
        segment,
        sample(0.0, 0.0, 0.0, 0.2),
        sample(0.25, 0.02, 0.10, 0.2),
    )
    assert report['passed'] is True
    assert report['longitudinal_distance'] == pytest.approx(0.25)

    reverse = MotionSegment('reverse', 1.0, -0.2, 0.0, 'reverse')
    reverse_report = evaluate_segment(
        reverse,
        sample(0.0, 0.0, math.pi / 2.0, -0.2),
        sample(0.0, -0.2, math.pi / 2.0, -0.2),
    )
    assert reverse_report['passed'] is True


def test_evaluate_segment_rejects_wrong_direction_and_invalid_motion() -> None:
    segment = MotionSegment('forward', 1.0, 0.2, 0.0, 'forward')
    report = evaluate_segment(
        segment,
        sample(0.0, 0.0, 0.0, 0.2),
        sample(-0.2, 0.0, 0.2, 0.2),
    )
    assert report['passed'] is False

    with pytest.raises(ValueError, match='Unsupported'):
        evaluate_segment(
            MotionSegment('bad', 1.0, 0.0, 0.0, 'sideways'),
            sample(0.0, 0.0, 0.0, 0.0),
            sample(0.0, 0.0, 0.0, 0.0),
        )


def test_normalize_angle_wraps_at_pi() -> None:
    assert normalize_angle(3.0 * math.pi) == pytest.approx(-math.pi)
