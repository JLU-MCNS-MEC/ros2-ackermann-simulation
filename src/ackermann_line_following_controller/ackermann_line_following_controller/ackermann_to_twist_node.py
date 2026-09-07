"""Convert a standard Ackermann command into Gazebo's Twist interface."""

import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .control import ackermann_to_yaw_rate, AckermannCommandLimiter


class AckermannToTwistNode(Node):
    """Bridge the model-independent Ackermann topic to Gazebo Sim."""

    def __init__(self) -> None:
        super().__init__('ackermann_to_twist')
        self.declare_parameter('input_topic', '/cmd_ackermann')
        self.declare_parameter('output_topic', '/model/ackermann_car/cmd_vel')
        self.declare_parameter('wheelbase', 0.56)
        self.declare_parameter('max_speed', 0.6)
        self.declare_parameter('max_acceleration', 1.5)
        self.declare_parameter('max_deceleration', 1.5)
        self.declare_parameter('max_steering', 0.55)
        self.declare_parameter('max_steering_rate', 2.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('control_rate', 50.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value
        )
        control_rate = float(self.get_parameter('control_rate').value)
        if self.command_timeout <= 0.0:
            raise ValueError('command_timeout must be positive')
        if control_rate <= 0.0:
            raise ValueError('control_rate must be positive')

        self.limiter = AckermannCommandLimiter(
            max_speed=float(self.get_parameter('max_speed').value),
            max_acceleration=float(
                self.get_parameter('max_acceleration').value
            ),
            max_deceleration=float(
                self.get_parameter('max_deceleration').value
            ),
            max_steering=float(self.get_parameter('max_steering').value),
            max_steering_rate=float(
                self.get_parameter('max_steering_rate').value
            ),
        )
        self._target_speed = 0.0
        self._target_steering = 0.0
        self._last_command_time = time.monotonic()
        self._last_update_time = time.monotonic()

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            input_topic,
            self.command_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / control_rate,
            self._publish_limited_command,
        )
        self.get_logger().info(
            f'Converting {input_topic} to {output_topic} with wheelbase '
            f'{self.wheelbase:.3f} m'
        )

    def command_callback(self, message: AckermannDriveStamped) -> None:
        """Cache a finite target command for the dynamics limiter."""
        speed = float(message.drive.speed)
        steering = float(message.drive.steering_angle)
        if not (math.isfinite(speed) and math.isfinite(steering)):
            self.get_logger().warning('Ignoring non-finite Ackermann command')
            return

        self._target_speed = speed
        self._target_steering = steering
        self._last_command_time = time.monotonic()

    def _publish_limited_command(self) -> None:
        """Publish a rate-limited Twist and stop after command timeout."""
        now = time.monotonic()
        dt = min(max(now - self._last_update_time, 1.0e-3), 0.1)
        self._last_update_time = now
        if now - self._last_command_time > self.command_timeout:
            target_speed = 0.0
            target_steering = 0.0
        else:
            target_speed = self._target_speed
            target_steering = self._target_steering

        speed, steering = self.limiter.update(
            target_speed,
            target_steering,
            dt,
        )

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = ackermann_to_yaw_rate(speed, steering, self.wheelbase)
        self.publisher.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannToTwistNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            stop = Twist()
            node.publisher.publish(stop)
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            # A launch shutdown can interrupt destruction after the stop
            # command was sent.  Keep the adapter from reporting a spurious
            # failure during an otherwise clean Ctrl-C.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
