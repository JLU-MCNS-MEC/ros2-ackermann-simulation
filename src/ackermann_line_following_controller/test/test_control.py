import math

import pytest

from ackermann_line_following_controller.control import (
    ackermann_to_yaw_rate,
    clamp,
    compute_pid_steering,
)
from ackermann_line_following_controller.line_follower_node import detect_line_error


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
