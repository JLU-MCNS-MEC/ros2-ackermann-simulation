"""Record Nav2 demonstrations for visual imitation learning."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

from .visual_policy import quaternion_to_yaw, relative_goal


class VisualDatasetRecorder(Node):
    """Save fresh visual tuples without changing vehicle control."""

    def __init__(self) -> None:
        """Create subscriptions and an explicitly bounded-rate recorder."""
        super().__init__('visual_dataset_recorder')
        self.declare_parameter(
            'output_directory', '/tmp/visual_navigation_dataset'
        )
        self.declare_parameter('image_topic', '/rgbd/image')
        self.declare_parameter('teacher_topic', '/cmd_vel_smoothed')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('sample_rate', 5.0)
        self.declare_parameter('maximum_age', 0.3)
        self.declare_parameter('minimum_command', 0.01)
        self.declare_parameter('jpeg_quality', 90)

        sample_rate = float(self.get_parameter('sample_rate').value)
        self.maximum_age = float(self.get_parameter('maximum_age').value)
        self.minimum_command = float(
            self.get_parameter('minimum_command').value
        )
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        if not math.isfinite(sample_rate) or sample_rate <= 0.0:
            raise ValueError('sample_rate must be positive and finite')
        if not math.isfinite(self.maximum_age) or self.maximum_age <= 0.0:
            raise ValueError('maximum_age must be positive and finite')
        if self.minimum_command < 0.0:
            raise ValueError('minimum_command must be nonnegative')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 1 and 100')

        self.output_directory = Path(
            str(self.get_parameter('output_directory').value)
        ).expanduser()
        self.image_directory = self.output_directory / 'images'
        self.image_directory.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.output_directory / 'samples.jsonl'
        with self.manifest_path.open(encoding='utf-8') as stream:
            self.sample_count = sum(1 for _ in stream)
        self.manifest = self.manifest_path.open('a', encoding='utf-8')
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_image = None
        self.latest_image_wall_time = 0.0
        self.latest_teacher = None
        self.latest_teacher_wall_time = 0.0
        self.latest_plan = None
        self.skipped = {}

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
        self.create_timer(1.0 / sample_rate, self._sample)
        self._write_metadata(sample_rate)
        self.get_logger().info(
            f'Recording visual demonstrations in {self.output_directory}'
        )

    def _write_metadata(self, sample_rate: float) -> None:
        metadata = {
            'format_version': 1,
            'image_topic': str(self.get_parameter('image_topic').value),
            'teacher_topic': str(self.get_parameter('teacher_topic').value),
            'plan_topic': str(self.get_parameter('plan_topic').value),
            'base_frame': self.base_frame,
            'sample_rate_hz': sample_rate,
            'control_authority': 'nav2_teacher_only',
        }
        (self.output_directory / 'metadata.json').write_text(
            json.dumps(metadata, indent=2), encoding='utf-8'
        )

    def _skip(self, reason: str) -> None:
        self.skipped[reason] = self.skipped.get(reason, 0) + 1

    def _image_callback(self, message: Image) -> None:
        self.latest_image = message
        self.latest_image_wall_time = time.monotonic()

    def _teacher_callback(self, message: Twist) -> None:
        self.latest_teacher = message
        self.latest_teacher_wall_time = time.monotonic()

    def _plan_callback(self, message: NavPath) -> None:
        if message.poses:
            self.latest_plan = message

    def _sample(self) -> None:
        now = time.monotonic()
        if self.latest_image is None or self.latest_teacher is None:
            self._skip('missing_image_or_teacher')
            return
        if self.latest_plan is None or not self.latest_plan.poses:
            self._skip('missing_plan')
            return
        if max(
            now - self.latest_image_wall_time,
            now - self.latest_teacher_wall_time,
        ) > self.maximum_age:
            self._skip('stale_input')
            return
        teacher = self.latest_teacher
        if (
            abs(float(teacher.linear.x)) + abs(float(teacher.angular.z))
            < self.minimum_command
        ):
            self._skip('idle_command')
            return
        image_message = self.latest_image
        plan = self.latest_plan
        try:
            transform = self.tf_buffer.lookup_transform(
                plan.header.frame_id,
                self.base_frame,
                Time.from_msg(image_message.header.stamp),
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
            image = self.bridge.imgmsg_to_cv2(
                image_message, desired_encoding='bgr8'
            )
        except (TransformException, CvBridgeError, ValueError) as error:
            self._skip(type(error).__name__)
            return

        filename = f'frame_{self.sample_count:07d}.jpg'
        image_path = self.image_directory / filename
        if not cv2.imwrite(
            str(image_path),
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        ):
            self._skip('image_write_failed')
            return
        stamp = image_message.header.stamp
        record = {
            'image': f'images/{filename}',
            'stamp_s': stamp.sec + stamp.nanosec * 1.0e-9,
            'frame_id': image_message.header.frame_id,
            'goal_distance_m': distance,
            'goal_bearing_rad': bearing,
            'teacher_linear_x': float(teacher.linear.x),
            'teacher_angular_z': float(teacher.angular.z),
        }
        self.manifest.write(json.dumps(record, allow_nan=False) + '\n')
        self.manifest.flush()
        self.sample_count += 1

    def close(self) -> None:
        """Flush the manifest and write capture statistics."""
        if self.manifest.closed:
            return
        self.manifest.flush()
        self.manifest.close()
        statistics = {
            'sample_count': self.sample_count,
            'skipped': self.skipped,
        }
        (self.output_directory / 'capture_stats.json').write_text(
            json.dumps(statistics, indent=2), encoding='utf-8'
        )

    def destroy_node(self):
        """Close dataset files before releasing the ROS node."""
        self.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualDatasetRecorder()
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
