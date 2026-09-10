"""Run a trained visual policy beside Nav2 without taking control authority."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

from .visual_policy import (
    OpenCvVisualPolicy,
    ShadowMetrics,
    draw_policy_overlay,
    draw_status_overlay,
    quaternion_to_yaw,
    relative_goal,
)


class VisualPolicyNode(Node):
    """Publish visual commands on an isolated shadow topic for comparison."""

    def __init__(self) -> None:
        """Load a policy and connect camera, goal and teacher inputs."""
        super().__init__('visual_policy')
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/rgbd/image')
        self.declare_parameter('teacher_topic', '/cmd_vel_smoothed')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter(
            'output_topic', '/visual_navigation/cmd_vel_shadow'
        )
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('inference_rate', 5.0)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter(
            'metrics_file', '/tmp/visual_navigation_shadow_metrics.json'
        )
        model_path = str(self.get_parameter('model_path').value)
        if not model_path:
            raise ValueError('model_path must name a trained visual policy')
        inference_rate = float(self.get_parameter('inference_rate').value)
        self.watchdog_timeout = float(
            self.get_parameter('watchdog_timeout').value
        )
        if not math.isfinite(inference_rate) or inference_rate <= 0.0:
            raise ValueError('inference_rate must be positive and finite')
        if (
            not math.isfinite(self.watchdog_timeout)
            or self.watchdog_timeout <= 0
        ):
            raise ValueError('watchdog_timeout must be positive and finite')
        self.inference_period = 1.0 / inference_rate
        self.policy = OpenCvVisualPolicy(model_path)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.metrics_file = Path(str(self.get_parameter('metrics_file').value))
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_plan = None
        self.latest_plan_wall_time = 0.0
        self.latest_teacher = None
        self.latest_teacher_wall_time = 0.0
        self.latest_image = None
        self.last_image_wall_time = 0.0
        self.watchdog_stopped = False
        self.metrics = ShadowMetrics()

        reliable_sensor_qos = QoSProfile(depth=3)
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._image_callback,
            reliable_sensor_qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('teacher_topic').value),
            self._teacher_callback,
            10,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter('plan_topic').value),
            self._plan_callback,
            10,
        )
        self.command_publisher = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10
        )
        self.debug_publisher = self.create_publisher(
            Image, '/visual_navigation/debug_image', 3
        )
        self.linear_error_publisher = self.create_publisher(
            Float64, '/visual_navigation/linear_error', 10
        )
        self.angular_error_publisher = self.create_publisher(
            Float64, '/visual_navigation/angular_error', 10
        )
        self.create_timer(self.inference_period, self._infer)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            'Visual policy is in shadow mode; it has no control authority'
        )

    def _teacher_callback(self, message: Twist) -> None:
        self.latest_teacher = message
        self.latest_teacher_wall_time = time.monotonic()

    def _plan_callback(self, message: NavPath) -> None:
        if message.poses:
            self.latest_plan = message
            self.latest_plan_wall_time = time.monotonic()

    def _image_callback(self, message: Image) -> None:
        self.latest_image = message
        self.last_image_wall_time = time.monotonic()
        self.watchdog_stopped = False

    def _infer(self) -> None:
        if self.latest_image is None:
            return
        if (
            time.monotonic() - self.last_image_wall_time
            > self.watchdog_timeout
        ):
            return
        message = self.latest_image
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError:
            return
        if self.latest_plan is None or not self.latest_plan.poses:
            self.command_publisher.publish(Twist())
            self._publish_debug(
                message,
                draw_status_overlay(image, 'WAITING FOR NAVIGATION GOAL'),
            )
            return
        if time.monotonic() - self.latest_plan_wall_time > 2.0:
            self.command_publisher.publish(Twist())
            self._publish_debug(
                message, draw_status_overlay(image, 'WAITING FOR FRESH PLAN')
            )
            return
        plan = self.latest_plan
        try:
            transform = self.tf_buffer.lookup_transform(
                plan.header.frame_id,
                self.base_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.1),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            yaw = quaternion_to_yaw(
                rotation.x, rotation.y, rotation.z, rotation.w
            )
            goal = plan.poses[-1].pose.position
            distance, bearing = relative_goal(
                translation.x, translation.y, yaw, goal.x, goal.y
            )
            predicted = self.policy.predict(image, distance, bearing)
        except (TransformException, ValueError):
            self.command_publisher.publish(Twist())
            self._publish_debug(
                message,
                draw_status_overlay(image, 'WAITING FOR IMAGE-TIME TF'),
            )
            return
        command = Twist()
        command.linear.x, command.angular.z = predicted
        self.command_publisher.publish(command)
        teacher = None
        if (
            self.latest_teacher is not None
            and time.monotonic() - self.latest_teacher_wall_time
            <= self.watchdog_timeout
        ):
            teacher = (
                float(self.latest_teacher.linear.x),
                float(self.latest_teacher.angular.z),
            )
            self.metrics.add(predicted, teacher)
            linear_error = Float64()
            linear_error.data = abs(predicted[0] - teacher[0])
            angular_error = Float64()
            angular_error.data = abs(predicted[1] - teacher[1])
            self.linear_error_publisher.publish(linear_error)
            self.angular_error_publisher.publish(angular_error)
        overlay = draw_policy_overlay(
            image, distance, bearing, predicted, teacher
        )
        self._publish_debug(message, overlay)

    def _publish_debug(self, source: Image, image) -> None:
        debug_message = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        debug_message.header = source.header
        self.debug_publisher.publish(debug_message)

    def _watchdog(self) -> None:
        if (
            time.monotonic() - self.last_image_wall_time
            <= self.watchdog_timeout
        ):
            return
        if not self.watchdog_stopped:
            self.command_publisher.publish(Twist())
            self.watchdog_stopped = True

    def write_metrics(self) -> None:
        """Persist the shadow comparison without claiming control success."""
        report = self.metrics.summary()
        report.update(
            mode='shadow',
            control_authority=False,
            model_path=str(self.policy.model_path),
        )
        self.metrics_file.parent.mkdir(parents=True, exist_ok=True)
        self.metrics_file.write_text(
            json.dumps(report, indent=2), encoding='utf-8'
        )

    def destroy_node(self):
        """Write metrics before releasing ROS resources."""
        self.write_metrics()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
