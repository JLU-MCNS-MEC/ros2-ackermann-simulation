"""Pure control helpers kept independent from ROS for easy testing."""

from dataclasses import dataclass
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


def twist_to_ackermann(
    speed: float,
    yaw_rate: float,
    wheelbase: float,
    max_speed: float,
    max_steering: float,
    minimum_speed: float = 1.0e-6,
) -> tuple[float, float]:
    """Convert a planar body twist into a bounded Ackermann command.

    Preserve curvature even when safety monitoring scales motion below 0.02
    m/s. Only numerically stationary commands are stopped; never synthesize
    forward motion to realize an in-place rotation.
    """
    values = (
        speed,
        yaw_rate,
        wheelbase,
        max_speed,
        max_steering,
        minimum_speed,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Twist conversion inputs must be finite')
    if wheelbase <= 0.0:
        raise ValueError('wheelbase must be positive')
    if max_speed <= 0.0:
        raise ValueError('max_speed must be positive')
    if max_steering <= 0.0:
        raise ValueError('max_steering must be positive')
    if minimum_speed < 0.0:
        raise ValueError('minimum_speed must be non-negative')

    bounded_speed = clamp(speed, -max_speed, max_speed)
    if abs(bounded_speed) <= max(minimum_speed, 1.0e-9):
        return 0.0, 0.0
    steering = math.atan(wheelbase * yaw_rate / bounded_speed)
    return bounded_speed, clamp(steering, -max_steering, max_steering)


@dataclass
class AckermannCommandLimiter:
    """Apply the longitudinal and steering limits of the simulated chassis.

    The Gazebo Ackermann system accepts a planar ``Twist`` command, but a real
    drive controller cannot jump from full forward to full reverse or change
    the steering angle instantaneously.  Keeping this stateful limiter outside
    the ROS node makes the command dynamics deterministic and unit-testable.
    """

    max_speed: float = 0.6
    max_acceleration: float = 1.5
    max_deceleration: float = 1.5
    max_steering: float = 0.55
    max_steering_rate: float = 2.0
    speed: float = 0.0
    steering_angle: float = 0.0

    def __post_init__(self) -> None:
        """Validate the physical limits before accepting a command."""
        for name in (
            'max_speed',
            'max_acceleration',
            'max_deceleration',
            'max_steering',
            'max_steering_rate',
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        self.speed = float(self.speed)
        self.steering_angle = float(self.steering_angle)
        if not (math.isfinite(self.speed) and math.isfinite(self.steering_angle)):
            raise ValueError('initial command state must be finite')
        self.speed = clamp(self.speed, -self.max_speed, self.max_speed)
        self.steering_angle = clamp(
            self.steering_angle,
            -self.max_steering,
            self.max_steering,
        )

    def reset(self) -> None:
        """Reset the command state to a stopped, straight configuration."""
        self.speed = 0.0
        self.steering_angle = 0.0

    def update(
        self,
        target_speed: float,
        target_steering: float,
        dt: float,
    ) -> tuple[float, float]:
        """Move the current command toward a bounded target over ``dt``.

        A sign change is handled as braking through zero before acceleration in
        the opposite direction.  This is the minimum behavior expected from a
        drive-by-wire controller and makes reverse commands safe to replay.
        """
        if not math.isfinite(target_speed) or not math.isfinite(target_steering):
            raise ValueError('target command must be finite')
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError('dt must be finite and positive')

        target_speed = clamp(target_speed, -self.max_speed, self.max_speed)
        target_steering = clamp(
            target_steering,
            -self.max_steering,
            self.max_steering,
        )

        speed_delta = target_speed - self.speed
        slowing_down = (
            abs(target_speed) < abs(self.speed)
            or target_speed * self.speed < 0.0
        )
        speed_limit = (
            self.max_deceleration if slowing_down else self.max_acceleration
        )
        self.speed += clamp(
            speed_delta,
            -speed_limit * dt,
            speed_limit * dt,
        )
        steering_delta = target_steering - self.steering_angle
        self.steering_angle += clamp(
            steering_delta,
            -self.max_steering_rate * dt,
            self.max_steering_rate * dt,
        )

        if abs(target_speed) < 1.0e-9 and abs(self.speed) < speed_limit * dt:
            self.speed = 0.0
        if (
            abs(target_steering) < 1.0e-9
            and abs(self.steering_angle) < self.max_steering_rate * dt
        ):
            self.steering_angle = 0.0
        return self.speed, self.steering_angle
