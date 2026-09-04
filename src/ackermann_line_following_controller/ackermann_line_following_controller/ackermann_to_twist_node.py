"""Convert a standard Ackermann command into Gazebo's Twist interface."""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node

from .control import ackermann_to_yaw_rate


class AckermannToTwistNode(Node):
    """Bridge the model-independent Ackermann topic to Gazebo Sim."""

    def __init__(self) -> None:
        super().__init__('ackermann_to_twist')
        self.declare_parameter('input_topic', '/cmd_ackermann')
        self.declare_parameter('output_topic', '/model/ackermann_car/cmd_vel')
        self.declare_parameter('wheelbase', 0.56)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            input_topic,
            self.command_callback,
            10,
        )
        self.get_logger().info(
            f'Converting {input_topic} to {output_topic} with wheelbase '
            f'{self.wheelbase:.3f} m'
        )

    def command_callback(self, message: AckermannDriveStamped) -> None:
        """Convert speed and steering angle into planar velocity."""
        speed = float(message.drive.speed)
        steering = float(message.drive.steering_angle)
        if not (math.isfinite(speed) and math.isfinite(steering)):
            self.get_logger().warning('Ignoring non-finite Ackermann command')
            return

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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
