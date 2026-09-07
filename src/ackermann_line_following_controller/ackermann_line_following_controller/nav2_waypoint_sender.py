"""Send a CSV route to the official Nav2 NavigateThroughPoses action.

This node intentionally contains only application glue: CSV parsing, pose
orientation and RViz markers. Planning, obstacle handling and velocity control
are provided by Nav2's Smac Hybrid-A*, Regulated Pure Pursuit, costmaps and
Collision Monitor plugins.
"""

from __future__ import annotations

import math
import os
import time
from typing import Sequence

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from .waypoint_tracker_node import load_waypoints, Waypoint


def _yaw_between(start: Waypoint, end: Waypoint, fallback: float = 0.0) -> float:
    """Return the path heading from ``start`` to ``end``."""
    dx = end.x - start.x
    dy = end.y - start.y
    if math.hypot(dx, dy) < 1.0e-9:
        return fallback
    return math.atan2(dy, dx)


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """Return an XYZW quaternion for a planar heading."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def poses_from_waypoints(
    waypoints: Sequence[Waypoint], frame_id: str = 'map'
) -> list[PoseStamped]:
    """Convert CSV waypoints into Nav2 goal poses with tangent headings."""
    if len(waypoints) < 2:
        raise ValueError('At least two waypoints are required')

    poses: list[PoseStamped] = []
    previous_yaw = 0.0
    for index, waypoint in enumerate(waypoints):
        if index + 1 < len(waypoints):
            yaw = _yaw_between(waypoint, waypoints[index + 1], previous_yaw)
        else:
            yaw = previous_yaw
        previous_yaw = yaw

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = _quaternion_from_yaw(yaw)
        poses.append(pose)
    return poses


def _publish_path(publisher, poses: Sequence[PoseStamped], frame_id: str) -> None:
    """Publish the requested route for RViz inspection."""
    path = PathMessage()
    path.header.frame_id = frame_id
    path.poses = list(poses)
    for pose in path.poses:
        pose.header = path.header
    publisher.publish(path)


def _publish_target(publisher, pose: PoseStamped, frame_id: str) -> None:
    """Publish a highlighted current Nav2 waypoint."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = 'nav2_waypoint_sender'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose = pose.pose
    marker.pose.position.z = 0.15
    marker.scale.x = 0.22
    marker.scale.y = 0.22
    marker.scale.z = 0.22
    marker.color.r = 0.15
    marker.color.g = 0.95
    marker.color.b = 0.25
    marker.color.a = 1.0
    publisher.publish(marker)


def default_waypoint_file() -> str:
    """Return the packaged Nav2 route used when no file is supplied."""
    return os.path.join(
        get_package_share_directory('ackermann_line_following_controller'),
        'config',
        'nav2_trajectory.csv',
    )


def wait_for_map(navigator: BasicNavigator, timeout_sec: float = 30.0) -> bool:
    """Wait until the transient-local map server has published a map."""
    received = False

    def map_callback(_message: OccupancyGrid) -> None:
        nonlocal received
        received = True

    qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    subscription = navigator.create_subscription(
        OccupancyGrid, '/map', map_callback, qos
    )
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not received and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
    navigator.destroy_subscription(subscription)
    return received


def main(args=None) -> None:
    """Send the configured route and report the Nav2 action result."""
    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='nav2_waypoint_sender')
    route_succeeded = False
    navigator.declare_parameter('waypoint_file', default_waypoint_file())
    navigator.declare_parameter('frame_id', 'map')
    # BasicNavigator uses the literal ``robot_localization`` to mean that
    # localization is supplied by a non-lifecycle node. The simulation has a
    # static map->odom transform, so there is no AMCL lifecycle node to wait
    # for; hardware can override this with ``amcl`` when appropriate.
    navigator.declare_parameter('localizer', 'robot_localization')
    waypoint_file = str(navigator.get_parameter('waypoint_file').value)
    frame_id = str(navigator.get_parameter('frame_id').value)
    localizer = str(navigator.get_parameter('localizer').value)

    try:
        waypoints = load_waypoints(waypoint_file)
        poses = poses_from_waypoints(waypoints, frame_id)
    except (OSError, ValueError) as error:
        navigator.get_logger().error(f'Unable to load Nav2 route: {error}')
        navigator.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    path_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    path_publisher = navigator.create_publisher(PathMessage, '/trajectory', path_qos)
    target_publisher = navigator.create_publisher(Marker, '/waypoint_target', 10)
    status_publisher = navigator.create_publisher(
        String, '/navigation_experiment/task_status', path_qos
    )

    def publish_status(value: str) -> None:
        message = String()
        message.data = value
        status_publisher.publish(message)

    _publish_path(path_publisher, poses, frame_id)

    navigator.get_logger().info(
        f'Sending {len(poses)} poses to Nav2 NavigateThroughPoses '
        f'(Smac Hybrid-A* + Regulated Pure Pursuit)'
    )
    # There is no AMCL in this odometry-only demo. A static map->odom transform
    # is supplied by nav2_waypoint_nav.launch.py, so only bt_navigator must be
    # waited on here. NavigateThroughPoses keeps the route continuous, which
    # is required for an Ackermann chassis at a corner.
    navigator.waitUntilNav2Active(localizer=localizer)
    navigator.get_logger().info('Waiting for the transient-local /map message...')
    if not wait_for_map(navigator):
        navigator.get_logger().error(
            'Map server did not publish /map before the timeout; refusing to send a route'
        )
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(1)
    if not navigator.goThroughPoses(poses):
        navigator.get_logger().error('Nav2 rejected NavigateThroughPoses goal')
        publish_status('FAILED')
    else:
        publish_status('RUNNING')
        last_waypoint = -1
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback is None:
                continue
            # NavigateThroughPoses reports how many poses remain, while the
            # FollowWaypoints action reports current_waypoint. Supporting both
            # keeps this small sender useful with either stock Nav2 action.
            if hasattr(feedback, 'number_of_poses_remaining'):
                remaining = int(feedback.number_of_poses_remaining)
                current = max(0, min(len(poses) - 1, len(poses) - remaining - 1))
            else:
                current = min(int(feedback.current_waypoint), len(poses) - 1)
            if current != last_waypoint:
                last_waypoint = current
                navigator.get_logger().info(
                    f'Nav2 tracking route point {current + 1}/{len(poses)}'
                )
            _publish_target(target_publisher, poses[current], frame_id)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('Nav2 route completed successfully')
            route_succeeded = True
            publish_status('SUCCEEDED')
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warning('Nav2 route was canceled')
            publish_status('CANCELED')
        else:
            navigator.get_logger().error(f'Nav2 route failed: {result.name}')
            publish_status('FAILED')

    navigator.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if not route_succeeded:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
