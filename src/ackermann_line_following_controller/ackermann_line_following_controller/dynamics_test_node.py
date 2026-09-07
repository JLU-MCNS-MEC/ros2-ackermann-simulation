"""Exercise the Ackermann drive in forward and reverse directions.

The test follows the same command boundary used by the line follower:
``AckermannDriveStamped`` is converted by :mod:`ackermann_to_twist_node` and
then consumed by Gazebo's native Ackermann system.  It intentionally checks
the motion through odometry instead of declaring success when a command was
published.  The measured path and phase marker are also useful in RViz.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Optional

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


@dataclass(frozen=True)
class MotionSegment:
    """One constant Ackermann command and its expected motion direction."""

    name: str
    duration: float
    speed: float
    steering_angle: float
    expected_motion: str
    expected_yaw_sign: int = 0


@dataclass(frozen=True)
class PoseSample:
    """Planar odometry sample used for segment evaluation."""

    stamp: float
    x: float
    y: float
    yaw: float
    body_speed: float


def default_segments() -> tuple[MotionSegment, ...]:
    """Return a short open-field forward/reverse acceptance sequence."""
    return (
        MotionSegment('forward_straight', 2.5, 0.20, 0.00, 'forward'),
        MotionSegment('forward_left', 2.5, 0.18, 0.28, 'forward', 1),
        MotionSegment('forward_right', 2.5, 0.18, -0.28, 'forward', -1),
        MotionSegment('brake_to_stop', 1.2, 0.00, 0.00, 'stop'),
        MotionSegment('reverse_straight', 2.5, -0.18, 0.00, 'reverse'),
        MotionSegment('reverse_left', 2.5, -0.16, 0.28, 'reverse', -1),
        MotionSegment('reverse_right', 2.5, -0.16, -0.28, 'reverse', 1),
        MotionSegment('final_stop', 1.0, 0.00, 0.00, 'stop'),
    )


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the interval ``[-pi, pi)``."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def evaluate_segment(
    segment: MotionSegment,
    start: PoseSample,
    end: PoseSample,
    minimum_distance: float = 0.12,
    minimum_yaw_change: float = 0.04,
    maximum_stop_speed: float = 0.05,
) -> dict[str, object]:
    """Evaluate signed travel, yaw direction and stopping speed."""
    dx = end.x - start.x
    dy = end.y - start.y
    longitudinal = math.cos(start.yaw) * dx + math.sin(start.yaw) * dy
    lateral = -math.sin(start.yaw) * dx + math.cos(start.yaw) * dy
    yaw_change = normalize_angle(end.yaw - start.yaw)

    if segment.expected_motion == 'forward':
        motion_passed = longitudinal >= minimum_distance
    elif segment.expected_motion == 'reverse':
        motion_passed = longitudinal <= -minimum_distance
    elif segment.expected_motion == 'stop':
        motion_passed = abs(end.body_speed) <= maximum_stop_speed
    else:
        raise ValueError(
            f'Unsupported expected motion: {segment.expected_motion}'
        )

    yaw_passed = True
    if segment.expected_yaw_sign:
        yaw_passed = (
            abs(yaw_change) >= minimum_yaw_change
            and math.copysign(1.0, yaw_change) == segment.expected_yaw_sign
        )

    return {
        'name': segment.name,
        'command_speed': segment.speed,
        'command_steering_angle': segment.steering_angle,
        'duration': segment.duration,
        'longitudinal_distance': longitudinal,
        'lateral_distance': lateral,
        'yaw_change': yaw_change,
        'terminal_body_speed': end.body_speed,
        'motion_passed': motion_passed,
        'yaw_passed': yaw_passed,
        'passed': motion_passed and yaw_passed,
    }


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert a quaternion into a planar yaw angle."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return an XYZW quaternion for a planar heading."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class AckermannDynamicsTestNode(Node):
    """Run the motion sequence and write a machine-readable report."""

    def __init__(self) -> None:
        super().__init__('ackermann_dynamics_test')
        self.declare_parameter(
            'odom_topic', '/model/ackermann_car/odometry'
        )
        self.declare_parameter('command_topic', '/cmd_ackermann')
        self.declare_parameter(
            'result_file', '/tmp/ackermann_dynamics_result.json'
        )
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('startup_timeout', 15.0)
        self.declare_parameter('segment_scale', 1.0)

        control_rate = float(self.get_parameter('control_rate').value)
        self.startup_timeout = float(
            self.get_parameter('startup_timeout').value
        )
        segment_scale = float(self.get_parameter('segment_scale').value)
        if control_rate <= 0.0:
            raise ValueError('control_rate must be positive')
        if self.startup_timeout <= 0.0:
            raise ValueError('startup_timeout must be positive')
        if segment_scale <= 0.0:
            raise ValueError('segment_scale must be positive')

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.result_file = str(self.get_parameter('result_file').value)
        self.segments = tuple(
            MotionSegment(
                segment.name,
                segment.duration * segment_scale,
                segment.speed,
                segment.steering_angle,
                segment.expected_motion,
                segment.expected_yaw_sign,
            )
            for segment in default_segments()
        )
        self.segment_index = -1
        self.segment_started_at: Optional[float] = None
        self.segment_start_pose: Optional[PoseSample] = None
        self.latest_pose: Optional[PoseSample] = None
        self.last_path_stamp = 0.0
        self.started_at = time.monotonic()
        self.finished = False
        self.passed = False
        self.reports: list[dict[str, object]] = []
        self.failure_reason = ''

        command_topic = str(self.get_parameter('command_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.command_publisher = self.create_publisher(
            AckermannDriveStamped,
            command_topic,
            10,
        )
        path_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publisher = self.create_publisher(
            PathMessage,
            '/dynamics_trajectory',
            path_qos,
        )
        self.phase_publisher = self.create_publisher(
            Marker,
            '/dynamics_phase',
            10,
        )
        self.path = PathMessage()
        self.path.header.frame_id = self.frame_id
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / control_rate,
            self.control_callback,
        )
        self.get_logger().info(
            f'Running {len(self.segments)} Ackermann dynamics segments; '
            f'odometry={odom_topic}'
        )

    def odom_callback(self, message: Odometry) -> None:
        """Cache the latest model pose and append a bounded-rate path."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        sample = PoseSample(
            stamp=time.monotonic(),
            x=float(position.x),
            y=float(position.y),
            yaw=quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
            body_speed=float(message.twist.twist.linear.x),
        )
        self.latest_pose = sample
        if sample.stamp - self.last_path_stamp < 0.05:
            return
        self.last_path_stamp = sample.stamp
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = sample.x
        pose.pose.position.y = sample.y
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = _quaternion_from_yaw(sample.yaw)
        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        self.path_publisher.publish(self.path)

    def _publish_command(self, speed: float, steering_angle: float) -> None:
        """Publish one standard Ackermann command to the adapter."""
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed = float(speed)
        message.drive.steering_angle = float(steering_angle)
        self.command_publisher.publish(message)

    def _publish_phase_marker(self, segment: MotionSegment) -> None:
        """Show the active command in RViz above the vehicle."""
        if self.latest_pose is None:
            return
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ackermann_dynamics'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.latest_pose.x
        marker.pose.position.y = self.latest_pose.y
        marker.pose.position.z = 0.55
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.a = 1.0
        if segment.expected_motion == 'reverse':
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.55, 0.10
        elif segment.expected_motion == 'forward':
            marker.color.r, marker.color.g, marker.color.b = 0.10, 1.0, 0.20
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.90, 0.90, 0.90
        marker.text = (
            f'{segment.name}: v={segment.speed:+.2f} '
            f'delta={segment.steering_angle:+.2f}'
        )
        self.phase_publisher.publish(marker)

    def _start_segment(self, now: float) -> None:
        """Start timing the next command from the current odometry pose."""
        self.segment_started_at = now
        self.segment_start_pose = self.latest_pose
        segment = self.segments[self.segment_index]
        self.get_logger().info(
            f'Starting {segment.name}: speed={segment.speed:+.2f} m/s, '
            f'steering={segment.steering_angle:+.2f} rad'
        )

    def _finish(self, reason: str = '') -> None:
        """Stop the vehicle, write the report and let ``main`` exit."""
        self._publish_command(0.0, 0.0)
        self.failure_reason = reason
        self.passed = not reason and all(
            bool(report['passed']) for report in self.reports
        )
        payload = {
            'passed': self.passed,
            'segments': self.reports,
            'failure_reason': self.failure_reason,
        }
        if self.result_file:
            try:
                result_path = Path(self.result_file)
                result_path.parent.mkdir(parents=True, exist_ok=True)
                result_path.write_text(
                    json.dumps(payload, indent=2) + '\n',
                    encoding='utf-8',
                )
            except OSError as error:
                self.get_logger().error(f'Unable to write dynamics report: {error}')
                self.passed = False
                payload['passed'] = False
        self.finished = True
        if self.passed:
            self.get_logger().info(
                f'Ackermann dynamics PASS; report={self.result_file}'
            )
        else:
            self.get_logger().error(
                f'Ackermann dynamics FAIL: {self.failure_reason or "segment check failed"}'
            )

    def control_callback(self) -> None:
        """Advance the sequence and publish the active test command."""
        if self.finished:
            return
        now = time.monotonic()
        if self.latest_pose is None:
            self._publish_command(0.0, 0.0)
            if now - self.started_at > self.startup_timeout:
                self._finish('odometry did not arrive before startup timeout')
            return

        if self.segment_index < 0:
            self.segment_index = 0
            self._start_segment(now)

        assert self.segment_started_at is not None
        assert self.segment_start_pose is not None
        segment = self.segments[self.segment_index]
        if now - self.segment_started_at >= segment.duration:
            report = evaluate_segment(
                segment,
                self.segment_start_pose,
                self.latest_pose,
            )
            self.reports.append(report)
            status = 'PASS' if report['passed'] else 'FAIL'
            self.get_logger().info(
                f'{status} {segment.name}: '
                f'ds={report["longitudinal_distance"]:+.3f} m, '
                f'dyaw={report["yaw_change"]:+.3f} rad'
            )
            if not report['passed']:
                self._finish(f'{segment.name} motion check failed')
                return

            self.segment_index += 1
            if self.segment_index >= len(self.segments):
                self._finish()
                return
            self._start_segment(now)
            segment = self.segments[self.segment_index]

        self._publish_command(segment.speed, segment.steering_angle)
        self._publish_phase_marker(segment)


def main(args=None) -> None:
    """Run the acceptance sequence and return a failing process on failure."""
    rclpy.init(args=args)
    node = AckermannDynamicsTestNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        passed = node.passed
    except KeyboardInterrupt:
        passed = False
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        if rclpy.ok():
            rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
