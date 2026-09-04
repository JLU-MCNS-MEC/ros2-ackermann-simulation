"""Simple camera-based dark-line follower for the demo track."""

import time
from typing import Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .control import compute_pid_steering


def detect_line_error(
    image: np.ndarray,
    threshold: int,
    roi_top: float,
    min_pixels: int,
) -> Optional[float]:
    """Find the normalized horizontal error of a dark line in an image.

    The demo track has a dark line on a light floor.  Only the lower region of
    the image is used so chassis/sensor geometry above the floor is ignored.
    ``None`` means that the line could not be trusted.
    """
    if image.size == 0 or image.ndim not in (2, 3):
        return None

    if image.ndim == 3:
        # Use HSV brightness rather than grayscale.  The demo chassis is blue;
        # grayscale would make its dark-looking red channel look like the
        # black track marking and pull the centroid toward the car body.
        brightness = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)[:, :, 2]
    else:
        brightness = image

    bounded_roi_top = max(0.0, min(float(roi_top), 0.95))
    start_row = int(brightness.shape[0] * bounded_roi_top)
    roi = brightness[start_row:]
    if roi.size == 0:
        return None

    threshold_value = max(0, min(int(threshold), 255))
    mask = cv2.inRange(roi, 0, threshold_value)
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    moments = cv2.moments(mask, binaryImage=True)
    if moments['m00'] < max(1, int(min_pixels)):
        return None

    centroid_x = moments['m10'] / moments['m00']
    image_center_x = brightness.shape[1] / 2.0
    if image_center_x <= 0.0:
        return None
    return float((centroid_x - image_center_x) / image_center_x)


class LineFollowerNode(Node):
    """Publish an Ackermann command from a camera image."""

    def __init__(self) -> None:
        super().__init__('line_follower')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('command_topic', '/cmd_ackermann')
        self.declare_parameter('speed', 0.30)
        self.declare_parameter('lost_line_speed', 0.06)
        self.declare_parameter('threshold', 105)
        self.declare_parameter('roi_top', 0.52)
        self.declare_parameter('min_pixels', 40)
        self.declare_parameter('kp', 0.55)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('integral_limit', 1.0)
        self.declare_parameter('max_steering', 0.48)
        self.declare_parameter('steering_sign', -1.0)
        self.declare_parameter('watchdog_timeout', 0.75)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.speed = float(self.get_parameter('speed').value)
        self.lost_line_speed = float(self.get_parameter('lost_line_speed').value)
        self.threshold = int(self.get_parameter('threshold').value)
        self.roi_top = float(self.get_parameter('roi_top').value)
        self.min_pixels = int(self.get_parameter('min_pixels').value)
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.integral_limit = float(self.get_parameter('integral_limit').value)
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.steering_sign = float(self.get_parameter('steering_sign').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            self.command_topic,
            10,
        )
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)

        self.previous_error = 0.0
        self.integral = 0.0
        self.last_steering = 0.0
        self.last_control_time = time.monotonic()
        self.last_image_time = time.monotonic()
        self.have_seen_line = False
        self.watchdog_stopped = False

        self.get_logger().info(
            f'Following {self.image_topic}; publishing {self.command_topic}'
        )

    def publish_command(self, speed: float, steering: float) -> None:
        """Publish one bounded high-level drive command."""
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed = float(speed)
        message.drive.steering_angle = float(steering)
        self.publisher.publish(message)

    def image_callback(self, message: Image) -> None:
        """Process one camera frame and update the PID command."""
        now = time.monotonic()
        self.last_image_time = now
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            self.get_logger().warning(f'Unable to convert camera frame: {error}')
            return

        line_error = detect_line_error(
            image,
            self.threshold,
            self.roi_top,
            self.min_pixels,
        )
        dt = max(now - self.last_control_time, 1.0e-3)
        self.last_control_time = now

        if line_error is None:
            # Keep a small forward speed only after the line has been found;
            # this lets the car recover briefly without driving blind at start.
            speed = self.lost_line_speed if self.have_seen_line else 0.0
            self.publish_command(speed, self.last_steering)
            return

        steering, self.integral = compute_pid_steering(
            error=line_error,
            previous_error=self.previous_error,
            integral=self.integral,
            dt=dt,
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            integral_limit=self.integral_limit,
            max_steering=self.max_steering,
            steering_sign=self.steering_sign,
        )
        self.previous_error = line_error
        self.last_steering = steering
        self.have_seen_line = True
        self.watchdog_stopped = False
        self.publish_command(self.speed, steering)

    def watchdog_callback(self) -> None:
        """Stop the vehicle when the camera stream disappears."""
        if time.monotonic() - self.last_image_time <= self.watchdog_timeout:
            return
        if not self.watchdog_stopped:
            self.get_logger().warning('Camera timeout; publishing a stop command')
            self.publish_command(0.0, 0.0)
            self.watchdog_stopped = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_command(0.0, 0.0)
        except (rclpy.exceptions.RCLError, RuntimeError):
            # The launch system may invalidate the context before this
            # process receives SIGINT.  It is already safe to exit then.
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
