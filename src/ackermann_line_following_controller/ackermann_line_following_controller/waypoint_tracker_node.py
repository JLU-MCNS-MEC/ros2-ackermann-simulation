"""Track planar waypoints with Ackermann pure pursuit and laser avoidance."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

from .control import clamp


@dataclass(frozen=True)
class Waypoint:
    """A planar target expressed in the odometry frame."""

    x: float
    y: float


@dataclass(frozen=True)
class Pose2D:
    """The part of an odometry pose needed by the tracker."""

    x: float
    y: float
    yaw: float


def load_waypoints(filename: str) -> list[Waypoint]:
    """Read ``x,y`` waypoints from a small CSV or whitespace file."""
    if not filename:
        return []

    waypoints: list[Waypoint] = []
    for line_number, raw_line in enumerate(
        Path(filename).read_text(encoding='utf-8').splitlines(),
        start=1,
    ):
        line = raw_line.split('#', 1)[0].strip()
        if not line:
            continue
        fields = line.replace(',', ' ').split()
        if len(fields) < 2:
            raise ValueError(f'Waypoint line {line_number} needs x and y')
        try:
            x, y = float(fields[0]), float(fields[1])
        except ValueError as error:
            raise ValueError(f'Invalid waypoint line {line_number}') from error
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f'Waypoint line {line_number} is not finite')
        waypoints.append(Waypoint(x, y))

    if len(waypoints) < 2:
        raise ValueError('At least two waypoints are required')
    return waypoints


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert a unit quaternion to a planar yaw angle."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def select_lookahead_target(
    waypoints: Sequence[Waypoint],
    pose: Pose2D,
    lookahead_distance: float,
) -> tuple[Waypoint, int, float]:
    """Return the nearest forward path point beyond the lookahead distance."""
    if not waypoints:
        raise ValueError('waypoints must not be empty')
    nearest_index = min(
        range(len(waypoints)),
        key=lambda index: math.hypot(
            waypoints[index].x - pose.x,
            waypoints[index].y - pose.y,
        ),
    )
    target_index = nearest_index
    for index in range(nearest_index, len(waypoints)):
        distance = math.hypot(
            waypoints[index].x - pose.x,
            waypoints[index].y - pose.y,
        )
        if distance >= lookahead_distance:
            target_index = index
            break
    target = waypoints[target_index]
    goal = waypoints[-1]
    goal_distance = math.hypot(goal.x - pose.x, goal.y - pose.y)
    return target, target_index, goal_distance


def pure_pursuit_steering(
    pose: Pose2D,
    target: Waypoint,
    wheelbase: float,
    max_steering: float,
) -> float:
    """Calculate a bounded Ackermann steering angle for one target point."""
    if wheelbase <= 0.0:
        raise ValueError('wheelbase must be positive')
    dx = target.x - pose.x
    dy = target.y - pose.y
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    distance_squared = max(local_x * local_x + local_y * local_y, 1.0e-9)
    curvature_angle = math.atan2(2.0 * wheelbase * local_y, distance_squared)
    return clamp(curvature_angle, -max_steering, max_steering)


def sector_min_range(
    scan: LaserScan,
    start_angle: float,
    end_angle: float,
) -> float:
    """Return a sector minimum, treating missing returns as ``range_max``."""
    if not scan.ranges or scan.angle_increment <= 0.0:
        return float(scan.range_max)

    lower = min(start_angle, end_angle)
    upper = max(start_angle, end_angle)
    values: list[float] = []
    for index, value in enumerate(scan.ranges):
        angle = scan.angle_min + index * scan.angle_increment
        if lower <= angle <= upper and math.isfinite(value):
            values.append(clamp(float(value), scan.range_min, scan.range_max))
    return min(values, default=float(scan.range_max))


class WaypointTrackerNode(Node):
    """Follow a waypoint file and temporarily override it around obstacles."""

    def __init__(self) -> None:
        super().__init__('waypoint_tracker')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('odom_topic', '/model/ackermann_car/odometry')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('command_topic', '/cmd_ackermann')
        self.declare_parameter('trajectory_topic', '/trajectory')
        self.declare_parameter('target_topic', '/waypoint_target')
        self.declare_parameter('wheelbase', 0.56)
        self.declare_parameter('target_speed', 0.18)
        self.declare_parameter('minimum_speed', 0.05)
        self.declare_parameter('lookahead_distance', 0.65)
        self.declare_parameter('max_steering', 0.48)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('obstacle_slow_distance', 1.20)
        self.declare_parameter('obstacle_stop_distance', 0.72)
        self.declare_parameter('obstacle_side_stop_distance', 0.58)
        self.declare_parameter('avoidance_speed', 0.12)
        self.declare_parameter('avoidance_hold_time', 4.0)
        self.declare_parameter('scan_timeout', 0.60)
        self.declare_parameter('odom_timeout', 0.60)
        self.declare_parameter('control_rate', 20.0)

        waypoint_file = str(self.get_parameter('waypoint_file').value)
        try:
            self.waypoints = load_waypoints(waypoint_file)
        except (OSError, ValueError) as error:
            self.waypoints = []
            self.get_logger().error(f'Unable to load waypoints: {error}')

        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)
        trajectory_topic = str(self.get_parameter('trajectory_topic').value)
        target_topic = str(self.get_parameter('target_topic').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.minimum_speed = float(self.get_parameter('minimum_speed').value)
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value
        )
        self.max_steering = float(self.get_parameter('max_steering').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.obstacle_slow_distance = float(
            self.get_parameter('obstacle_slow_distance').value
        )
        self.obstacle_stop_distance = float(
            self.get_parameter('obstacle_stop_distance').value
        )
        self.obstacle_side_stop_distance = float(
            self.get_parameter('obstacle_side_stop_distance').value
        )
        self.avoidance_speed = float(
            self.get_parameter('avoidance_speed').value
        )
        self.avoidance_hold_time = float(
            self.get_parameter('avoidance_hold_time').value
        )
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)
        control_rate = max(1.0, float(self.get_parameter('control_rate').value))

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
            trajectory_topic,
            path_qos,
        )
        self.target_publisher = self.create_publisher(Marker, target_topic, 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / control_rate, self.control_callback)

        self.pose: Optional[Pose2D] = None
        self.scan: Optional[LaserScan] = None
        self.last_odom_time = 0.0
        self.last_scan_time = 0.0
        self.avoidance_direction = 0
        self.avoidance_until = 0.0
        self.goal_reported = False
        self._publish_trajectory()
        self.get_logger().info(
            f'Tracking {len(self.waypoints)} waypoints; '
            f'odom={self.odom_topic}, scan={self.scan_topic}'
        )

    def odom_callback(self, message: Odometry) -> None:
        """Cache the latest planar odometry pose."""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.pose = Pose2D(
            float(position.x),
            float(position.y),
            quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )
        self.last_odom_time = time.monotonic()

    def scan_callback(self, message: LaserScan) -> None:
        """Cache the latest laser scan used by the safety guard."""
        self.scan = message
        self.last_scan_time = time.monotonic()

    def publish_command(self, speed: float, steering: float) -> None:
        """Publish one bounded Ackermann command."""
        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.drive.speed = float(max(0.0, speed))
        command.drive.steering_angle = float(
            clamp(steering, -self.max_steering, self.max_steering)
        )
        self.command_publisher.publish(command)

    def _publish_trajectory(self) -> None:
        """Publish the path once so RViz can show the loaded route."""
        path = PathMessage()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_publisher.publish(path)

    def _publish_target(self, target: Waypoint) -> None:
        """Publish a sphere at the currently selected target."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'waypoint_tracker'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target.x
        marker.pose.position.y = target.y
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.05
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        self.target_publisher.publish(marker)

    def _stop_for_missing_data(self, now: float) -> bool:
        """Stop until both pose and scan streams are fresh."""
        if not self.waypoints:
            self.publish_command(0.0, 0.0)
            return True
        if (
            self.pose is None
            or now - self.last_odom_time > self.odom_timeout
            or self.scan is None
            or now - self.last_scan_time > self.scan_timeout
        ):
            self.publish_command(0.0, 0.0)
            return True
        return False

    def control_callback(self) -> None:
        """Run pure pursuit and apply the reactive obstacle override."""
        now = time.monotonic()
        if self._stop_for_missing_data(now):
            return
        assert self.pose is not None
        assert self.scan is not None

        target, _, goal_distance = select_lookahead_target(
            self.waypoints,
            self.pose,
            self.lookahead_distance,
        )
        self._publish_target(target)
        if goal_distance <= self.goal_tolerance:
            if not self.goal_reported:
                self.get_logger().info('Final waypoint reached; stopping')
                self.goal_reported = True
            self.publish_command(0.0, 0.0)
            return
        self.goal_reported = False

        pursuit_steering = pure_pursuit_steering(
            self.pose,
            target,
            self.wheelbase,
            self.max_steering,
        )
        front_min = sector_min_range(self.scan, -math.radians(28), math.radians(28))
        left_min = sector_min_range(self.scan, math.radians(42), math.radians(138))
        right_min = sector_min_range(self.scan, -math.radians(138), -math.radians(42))

        if self.avoidance_until > now:
            if (
                front_min < self.obstacle_stop_distance
                and left_min < self.obstacle_side_stop_distance
                and right_min < self.obstacle_side_stop_distance
            ):
                self.publish_command(0.0, 0.0)
                return
            self.publish_command(self.avoidance_speed, self.avoidance_direction * self.max_steering)
            return

        if front_min < self.obstacle_stop_distance:
            if (
                left_min < self.obstacle_side_stop_distance
                and right_min < self.obstacle_side_stop_distance
            ):
                self.publish_command(0.0, 0.0)
                return
            self.avoidance_direction = 1 if left_min >= right_min else -1
            self.avoidance_until = now + self.avoidance_hold_time
            direction_name = (
                'left' if self.avoidance_direction > 0 else 'right'
            )
            self.get_logger().warning(
                f'Obstacle at {front_min:.2f} m; avoiding {direction_name}'
            )
            self.publish_command(
                self.avoidance_speed,
                self.avoidance_direction * self.max_steering,
            )
            return

        if front_min < self.obstacle_slow_distance:
            clearance_direction = 1 if left_min >= right_min else -1
            steering = pursuit_steering + 0.18 * clearance_direction
            self.publish_command(
                max(self.minimum_speed, 0.45 * self.target_speed),
                clamp(steering, -self.max_steering, self.max_steering),
            )
            return

        self.publish_command(self.target_speed, pursuit_steering)


def main(args=None) -> None:
    """Run the waypoint tracker node."""
    rclpy.init(args=args)
    node = WaypointTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if rclpy.ok():
                node.publish_command(0.0, 0.0)
        except (rclpy.exceptions.RCLError, RuntimeError):
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
