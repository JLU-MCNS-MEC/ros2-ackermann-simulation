"""Publish compact navigation telemetry for plots and RViz."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


ACTION_NAMES = {
    CollisionMonitorState.DO_NOTHING: 'CLEAR',
    CollisionMonitorState.STOP: 'STOP',
    CollisionMonitorState.SLOWDOWN: 'SLOWDOWN',
    CollisionMonitorState.APPROACH: 'APPROACH',
    CollisionMonitorState.LIMIT: 'LIMIT',
}


@dataclass(frozen=True)
class PathMetrics:
    """Distance measurements between the vehicle and a planned path."""

    cross_track_error: float
    distance_to_goal: float


def steering_from_twist(
    linear_speed: float,
    yaw_rate: float,
    wheelbase: float,
) -> float:
    """Infer the bicycle steering angle represented by a planar Twist."""
    if wheelbase <= 0.0:
        raise ValueError('wheelbase must be positive')
    if abs(linear_speed) < 1.0e-4:
        return 0.0
    return math.atan(wheelbase * yaw_rate / linear_speed)


def path_metrics(
    x: float,
    y: float,
    points: Sequence[tuple[float, float]],
) -> PathMetrics:
    """Return nearest path distance and direct distance to its final pose."""
    if not points:
        return PathMetrics(float('nan'), float('nan'))

    goal_x, goal_y = points[-1]
    goal_distance = math.hypot(goal_x - x, goal_y - y)
    if len(points) == 1:
        return PathMetrics(goal_distance, goal_distance)

    minimum_distance = float('inf')
    for start, end in zip(points, points[1:]):
        start_x, start_y = start
        end_x, end_y = end
        dx = end_x - start_x
        dy = end_y - start_y
        length_squared = dx * dx + dy * dy
        if length_squared <= 1.0e-12:
            projection = 0.0
        else:
            projection = max(
                0.0,
                min(
                    1.0,
                    (
                        (x - start_x) * dx + (y - start_y) * dy
                    ) / length_squared,
                ),
            )
        nearest_x = start_x + projection * dx
        nearest_y = start_y + projection * dy
        minimum_distance = min(
            minimum_distance,
            math.hypot(nearest_x - x, nearest_y - y),
        )
    return PathMetrics(minimum_distance, goal_distance)


def transform_point_2d(
    x: float,
    y: float,
    translation_x: float,
    translation_y: float,
    quaternion_x: float,
    quaternion_y: float,
    quaternion_z: float,
    quaternion_w: float,
) -> tuple[float, float]:
    """Apply a rigid transform to a planar point."""
    yaw = math.atan2(
        2.0 * (
            quaternion_w * quaternion_z
            + quaternion_x * quaternion_y
        ),
        1.0 - 2.0 * (
            quaternion_y * quaternion_y
            + quaternion_z * quaternion_z
        ),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        translation_x + cosine * x - sine * y,
        translation_y + sine * x + cosine * y,
    )


def scan_clearances(
    scan: LaserScan,
    front_half_angle: float,
) -> tuple[float, float]:
    """Return nearest 360-degree and forward-sector finite lidar ranges."""
    nearest = float(scan.range_max)
    front = float(scan.range_max)
    for index, raw_range in enumerate(scan.ranges):
        if not math.isfinite(raw_range) or raw_range < scan.range_min:
            continue
        value = min(float(raw_range), float(scan.range_max))
        angle = scan.angle_min + index * scan.angle_increment
        nearest = min(nearest, value)
        normalized_angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(normalized_angle) <= front_half_angle:
            front = min(front, value)
    return nearest, front


class NavigationDiagnosticsNode(Node):
    """Aggregate controller, perception and safety decisions as scalar data."""

    def __init__(self) -> None:
        """Connect the navigation data sources and scalar outputs."""
        super().__init__('navigation_diagnostics')
        self.declare_parameter('wheelbase', 0.56)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('front_sector_degrees', 30.0)
        self.declare_parameter('odom_topic', '/model/ackermann_car/odometry')
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        front_degrees = float(
            self.get_parameter('front_sector_degrees').value
        )
        if self.wheelbase <= 0.0:
            raise ValueError('wheelbase must be positive')
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')
        if not 0.0 < front_degrees <= 180.0:
            raise ValueError('front_sector_degrees must be in (0, 180]')
        self.front_half_angle = math.radians(front_degrees)

        self.values = {
            'speed/commanded': 0.0,
            'speed/smoothed': 0.0,
            'speed/applied': 0.0,
            'speed/measured': 0.0,
            'steering/commanded': 0.0,
            'steering/applied': 0.0,
            'yaw_rate/measured': 0.0,
            'navigation/cross_track_error': float('nan'),
            'navigation/distance_to_goal': float('nan'),
            'obstacle/front_clearance': float('nan'),
            'obstacle/nearest_clearance': float('nan'),
            'decision/action_code': 0.0,
            'decision/speed_ratio': 1.0,
        }
        self.scalar_publishers = {
            name: self.create_publisher(
                Float64, f'/nav_diagnostics/{name}', 10
            )
            for name in self.values
        }
        self.marker_publisher = self.create_publisher(
            Marker, '/nav_diagnostics/summary', 10
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_xy: tuple[float, float] | None = None
        self.pose_frame = ''
        self.path_frame = ''
        self.path_points: list[tuple[float, float]] = []
        self.action_name = 'CLEAR'
        self.action_polygon = '-'
        self.create_subscription(
            Twist, '/cmd_vel_nav', self._command_callback, 10
        )
        self.create_subscription(
            Twist, '/cmd_vel_smoothed', self._smoothed_callback, 10
        )
        self.create_subscription(
            Twist, '/model/ackermann_car/cmd_vel', self._applied_callback, 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._odom_callback,
            10,
        )
        self.create_subscription(Path, '/plan', self._path_callback, 10)
        self.create_subscription(
            LaserScan,
            '/scan_nav',
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CollisionMonitorState,
            '/collision_monitor_state',
            self._collision_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)
        self.get_logger().info(
            'Publishing speed, steering, path, lidar and collision telemetry '
            'under /nav_diagnostics'
        )

    def _set_twist(self, prefix: str, message: Twist) -> None:
        speed = float(message.linear.x)
        self.values[f'speed/{prefix}'] = speed
        if prefix in ('commanded', 'applied'):
            self.values[f'steering/{prefix}'] = steering_from_twist(
                speed, float(message.angular.z), self.wheelbase
            )

    def _command_callback(self, message: Twist) -> None:
        self._set_twist('commanded', message)

    def _smoothed_callback(self, message: Twist) -> None:
        self._set_twist('smoothed', message)

    def _applied_callback(self, message: Twist) -> None:
        self._set_twist('applied', message)
        commanded = self.values['speed/commanded']
        self.values['decision/speed_ratio'] = (
            abs(float(message.linear.x) / commanded)
            if abs(commanded) > 1.0e-4
            else 1.0
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.pose_xy = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        self.pose_frame = message.header.frame_id
        self.values['speed/measured'] = float(message.twist.twist.linear.x)
        self.values['yaw_rate/measured'] = float(message.twist.twist.angular.z)
        self._update_path_metrics()

    def _path_callback(self, message: Path) -> None:
        self.path_frame = message.header.frame_id
        self.path_points = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ]
        self._update_path_metrics()

    def _update_path_metrics(self) -> None:
        if self.pose_xy is None:
            return
        pose_xy = self.pose_xy
        if (
            self.path_frame
            and self.pose_frame
            and self.path_frame != self.pose_frame
        ):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.path_frame, self.pose_frame, Time()
                ).transform
            except TransformException:
                return
            pose_xy = transform_point_2d(
                *pose_xy,
                float(transform.translation.x),
                float(transform.translation.y),
                float(transform.rotation.x),
                float(transform.rotation.y),
                float(transform.rotation.z),
                float(transform.rotation.w),
            )
        metrics = path_metrics(*pose_xy, self.path_points)
        self.values['navigation/cross_track_error'] = metrics.cross_track_error
        self.values['navigation/distance_to_goal'] = metrics.distance_to_goal

    def _scan_callback(self, message: LaserScan) -> None:
        nearest, front = scan_clearances(message, self.front_half_angle)
        self.values['obstacle/nearest_clearance'] = nearest
        self.values['obstacle/front_clearance'] = front

    def _collision_callback(self, message: CollisionMonitorState) -> None:
        self.values['decision/action_code'] = float(message.action_type)
        self.action_name = ACTION_NAMES.get(message.action_type, 'UNKNOWN')
        self.action_polygon = message.polygon_name or '-'

    @staticmethod
    def _format_value(value: float, suffix: str) -> str:
        return f'{value:.2f}{suffix}' if math.isfinite(value) else '--'

    def _publish(self) -> None:
        for name, value in self.values.items():
            message = Float64()
            message.data = float(value)
            self.scalar_publishers[name].publish(message)
        self._publish_marker()
        self._publish_diagnostic_status()

    def _publish_marker(self) -> None:
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'navigation_diagnostics'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.65
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.18
        marker.color.r = 0.20
        marker.color.g = 1.0 if self.action_name == 'CLEAR' else 0.65
        marker.color.b = 0.95
        marker.color.a = 1.0
        value = self.values
        front_clearance = self._format_value(
            value['obstacle/front_clearance'], ' m'
        )
        nearest_clearance = self._format_value(
            value['obstacle/nearest_clearance'], ' m'
        )
        path_error = self._format_value(
            value['navigation/cross_track_error'], ' m'
        )
        goal_distance = self._format_value(
            value['navigation/distance_to_goal'], ' m'
        )
        marker.text = (
            f'Collision: {self.action_name} [{self.action_polygon}]\n'
            f'Speed cmd/smooth/actual: {value["speed/commanded"]:.2f} / '
            f'{value["speed/smoothed"]:.2f} / '
            f'{value["speed/measured"]:.2f} m/s\n'
            f'Steering cmd: {value["steering/commanded"]:.2f} rad  '
            f'Yaw rate: {value["yaw_rate/measured"]:.2f} rad/s\n'
            f'Lidar front/nearest: {front_clearance} / {nearest_clearance}\n'
            f'Path error/goal: {path_error} / {goal_distance}'
        )
        self.marker_publisher.publish(marker)

    def _publish_diagnostic_status(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'Ackermann navigation decision'
        status.hardware_id = 'ackermann_car'
        status.level = (
            DiagnosticStatus.OK
            if self.action_name == 'CLEAR'
            else DiagnosticStatus.WARN
        )
        status.message = f'{self.action_name}: {self.action_polygon}'
        status.values = [
            KeyValue(key=name, value=self._format_value(value, ''))
            for name, value in self.values.items()
        ]
        array.status = [status]
        self.diagnostic_publisher.publish(array)


def main(args=None) -> None:
    """Run the navigation telemetry aggregator."""
    rclpy.init(args=args)
    node = NavigationDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
