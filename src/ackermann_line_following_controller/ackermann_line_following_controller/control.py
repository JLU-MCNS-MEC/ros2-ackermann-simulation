"""Pure control helpers kept independent from ROS for easy testing."""

import math


def clamp(value: float, lower: float, upper: float) -> float:
    """Return *value* constrained to the inclusive interval."""
    if lower > upper:
        raise ValueError('lower bound must not exceed upper bound')
    return max(lower, min(value, upper))


def compute_pid_steering(
    error: float,
    previous_error: float,
    integral: float,
    dt: float,
    kp: float,
    ki: float,
    kd: float,
    integral_limit: float,
    max_steering: float,
    steering_sign: float = -1.0,
) -> tuple[float, float]:
    """Compute a bounded steering angle from a normalized image error.

    The camera error is positive when the line is to the right of the image
    center.  The default sign maps that to a right turn in the ROS convention,
    where positive yaw is left.
    """
    if dt <= 0.0:
        raise ValueError('dt must be positive')
    if integral_limit < 0.0:
        raise ValueError('integral_limit must be non-negative')
    if max_steering < 0.0:
        raise ValueError('max_steering must be non-negative')

    new_integral = clamp(
        integral + error * dt,
        -integral_limit,
        integral_limit,
    )
    derivative = (error - previous_error) / dt
    raw_control = kp * error + ki * new_integral + kd * derivative
    steering = clamp(
        steering_sign * raw_control,
        -max_steering,
        max_steering,
    )
    return steering, new_integral


def ackermann_to_yaw_rate(
    speed: float,
    steering_angle: float,
    wheelbase: float,
) -> float:
    """Convert front-wheel steering into the planar body yaw rate."""
    if wheelbase <= 0.0:
        raise ValueError('wheelbase must be positive')
    return speed * math.tan(steering_angle) / wheelbase
