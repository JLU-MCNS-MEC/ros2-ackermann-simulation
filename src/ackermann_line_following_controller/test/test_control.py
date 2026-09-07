import math

from ackermann_line_following_controller.control import (
    ackermann_to_yaw_rate,
    AckermannCommandLimiter,
    clamp,
    compute_pid_steering,
    twist_to_ackermann,
)
from ackermann_line_following_controller.line_follower_node import detect_line_error
import pytest


def test_clamp_and_invalid_bounds() -> None:
    assert clamp(-2.0, -1.0, 1.0) == -1.0
    assert clamp(0.5, -1.0, 1.0) == 0.5
    with pytest.raises(ValueError):
        clamp(0.0, 2.0, 1.0)


def test_pid_is_bounded_and_integral_is_anti_windup() -> None:
    steering, integral = compute_pid_steering(
        error=2.0,
        previous_error=0.0,
        integral=0.9,
        dt=0.1,
        kp=1.0,
        ki=1.0,
        kd=0.0,
        integral_limit=1.0,
        max_steering=0.4,
    )
    assert steering == pytest.approx(-0.4)
    assert integral == pytest.approx(1.0)


def test_ackermann_conversion() -> None:
    assert ackermann_to_yaw_rate(1.0, 0.0, 0.56) == 0.0
    assert ackermann_to_yaw_rate(0.56, math.atan(1.0), 0.56) == pytest.approx(1.0)
    with pytest.raises(ValueError):
        ackermann_to_yaw_rate(1.0, 0.0, 0.0)


def test_twist_conversion_round_trip_forward_and_reverse() -> None:
    for speed, steering in ((0.4, 0.3), (-0.25, -0.2)):
        yaw_rate = ackermann_to_yaw_rate(speed, steering, 0.56)
        converted_speed, converted_steering = twist_to_ackermann(
            speed, yaw_rate, 0.56, 0.6, 0.55
        )
        assert converted_speed == pytest.approx(speed)
        assert converted_steering == pytest.approx(steering)


def test_twist_conversion_clamps_and_rejects_stationary_rotation() -> None:
    speed, steering = twist_to_ackermann(2.0, 10.0, 0.56, 0.6, 0.55)
    assert speed == pytest.approx(0.6)
    assert steering == pytest.approx(0.55)
    assert twist_to_ackermann(0.0, 1.0, 0.56, 0.6, 0.55) == (0.0, 0.0)

    with pytest.raises(ValueError, match='finite'):
        twist_to_ackermann(float('nan'), 0.0, 0.56, 0.6, 0.55)
    with pytest.raises(ValueError, match='wheelbase'):
        twist_to_ackermann(0.2, 0.0, 0.0, 0.6, 0.55)


def test_command_limiter_limits_forward_acceleration_and_steering_rate() -> None:
    limiter = AckermannCommandLimiter(
        max_acceleration=1.0,
        max_steering_rate=1.0,
    )

    speed, steering = limiter.update(0.6, 0.55, 0.1)

    assert speed == pytest.approx(0.1)
    assert steering == pytest.approx(0.1)


def test_command_limiter_brakes_through_zero_before_reverse() -> None:
    limiter = AckermannCommandLimiter(speed=0.4)

    speed, _ = limiter.update(-0.4, 0.0, 0.1)

    assert speed == pytest.approx(0.25)
    for _ in range(4):
        speed, _ = limiter.update(-0.4, 0.0, 0.1)
    assert speed < 0.0


def test_command_limiter_clamps_targets_and_rejects_invalid_dt() -> None:
    limiter = AckermannCommandLimiter(max_speed=0.3, max_steering=0.4)
    speed, steering = limiter.update(2.0, -2.0, 1.0)
    assert speed == pytest.approx(0.3)
    assert steering == pytest.approx(-0.4)

    with pytest.raises(ValueError, match='dt'):
        limiter.update(0.0, 0.0, 0.0)


def test_dark_line_centroid_error() -> None:
    import numpy as np

    image = np.full((100, 200, 3), 220, dtype=np.uint8)
    image[60:95, 140:150] = 20
    error = detect_line_error(image, threshold=100, roi_top=0.5, min_pixels=20)
    assert error == pytest.approx(0.45, abs=0.02)


def test_line_detection_returns_none_without_line() -> None:
    import numpy as np

    image = np.full((100, 200, 3), 220, dtype=np.uint8)
    assert detect_line_error(image, threshold=100, roi_top=0.5, min_pixels=20) is None


def test_line_detection_ignores_bright_blue_chassis() -> None:
    import numpy as np

    image = np.full((100, 200, 3), 220, dtype=np.uint8)
    image[60:100, :] = (190, 65, 20)
    assert detect_line_error(image, threshold=100, roi_top=0.5, min_pixels=20) is None
