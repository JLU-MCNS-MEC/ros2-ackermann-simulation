"""Expose Nav2 Twist commands through a hardware-neutral Ackermann topic."""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .control import twist_to_ackermann


class TwistToAckermannNode(Node):
    """Convert the final safety-filtered Twist into a chassis command."""

    def __init__(self) -> None:
        """Configure the topic boundary, vehicle limits and watchdog."""
        super().__init__('twist_to_ackermann')
        self.declare_parameter('input_topic', '/cmd_vel_safe')
        self.declare_parameter('output_topic', '/drive')
        self.declare_parameter('wheelbase', 0.56)
        self.declare_parameter('max_speed', 0.6)
        self.declare_parameter('max_steering', 0.55)
        self.declare_parameter('minimum_speed', 0.02)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('publish_rate', 50.0)

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.minimum_speed = float(self.get_parameter('minimum_speed').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        publish_rate = float(self.get_parameter('publish_rate').value)
        if (
            not math.isfinite(self.command_timeout)
            or self.command_timeout <= 0.0
        ):
            raise ValueError('command_timeout must be finite and positive')
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(
            AckermannDriveStamped, output_topic, 10
        )
        self.subscription = self.create_subscription(
            Twist, input_topic, self.command_callback, 10
        )
        self._last_command_time: float | None = None
        self._speed = 0.0
        self._steering = 0.0
        self._reported_stationary_turn = False
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            f'Converting safety output {input_topic} to Ackermann '
            f'{output_topic}'
        )

    def command_callback(self, message: Twist) -> None:
        """Validate and cache the most recent final velocity command."""
        speed = float(message.linear.x)
        yaw_rate = float(message.angular.z)
        try:
            self._speed, self._steering = twist_to_ackermann(
                speed,
                yaw_rate,
                self.wheelbase,
                self.max_speed,
                self.max_steering,
                self.minimum_speed,
            )
        except ValueError as error:
            self.get_logger().warning(f'Ignoring invalid Twist command: {error}')
            return

        stationary_turn = (
            abs(speed) < self.minimum_speed and abs(yaw_rate) > 1.0e-4
        )
        if stationary_turn and not self._reported_stationary_turn:
            self.get_logger().warning(
                'Ackermann chassis cannot rotate in place; dropping angular '
                'velocity while stopped'
            )
        self._reported_stationary_turn = stationary_turn
        self._last_command_time = time.monotonic()

    def _publish(self) -> None:
        """Publish at a stable rate and force a stop after input timeout."""
        timed_out = (
            self._last_command_time is None
            or time.monotonic() - self._last_command_time > self.command_timeout
        )
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        if not timed_out:
            message.drive.speed = self._speed
            message.drive.steering_angle = self._steering
        self.publisher.publish(message)

    def publish_stop(self) -> None:
        """Publish an explicit zero command during orderly shutdown."""
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)


def main(args=None) -> None:
    """Run the Twist-to-Ackermann hardware boundary."""
    rclpy.init(args=args)
    node = TwistToAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
