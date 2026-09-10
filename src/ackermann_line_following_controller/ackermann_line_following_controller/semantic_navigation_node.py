"""Record named approach poses and navigate to map-bound semantic goals."""

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .semantic_map import load_landmarks, map_identity, resolve_landmark, save_landmarks


def goal_pose(item: dict, stamp) -> PoseStamped:
    """Convert the annotated approach pose, not an object's centre, to a goal."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    x, y, yaw = item['approach_pose']
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def record_landmark(navigator, args, identity: str) -> None:
    """Annotate the current localized vehicle pose as a named stopping place."""
    items = (
        load_landmarks(args.database, identity) if Path(args.database).exists() else []
    )
    if any(item['name'].casefold() == args.record.strip().casefold() for item in items):
        raise ValueError('Name already exists; choose a new landmark name')
    buffer = Buffer()
    listener = TransformListener(buffer, navigator)
    deadline = time.monotonic() + 15.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(navigator, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
        except TransformException:
            continue
        age = (
            navigator.get_clock().now().nanoseconds
            - rclpy.time.Time.from_msg(transform.header.stamp).nanoseconds
        ) / 1e9
        if age < -0.1 or age > 1.0:
            continue
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = transform.transform.translation
        items.append(
            {
                'name': args.record.strip(),
                'aliases': args.alias,
                'source': 'manual_pose',
                'approach_pose': [p.x, p.y, yaw],
                'observed_at': transform.header.stamp.sec,
            }
        )
        save_landmarks(args.database, identity, items)
        navigator.get_logger().info(f'Recorded manual approach pose: {args.record}')
        listener.unregister()
        return
    raise RuntimeError('No fresh map -> base_footprint transform; localize first')


def landmark_markers(items: list[dict], stamp) -> MarkerArray:
    """Show named stopping places in RViz without claiming object geometry."""
    markers = MarkerArray()
    for index, item in enumerate(items):
        pose = goal_pose(item, stamp)
        marker = Marker()
        marker.header = pose.header
        marker.ns, marker.id = 'semantic_approach_poses', index
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = 0.8
        marker.scale.z = 0.25
        marker.color.r, marker.color.g, marker.color.a = 0.2, 1.0, 1.0
        marker.text = item['name']
        markers.markers.append(marker)
    return markers


def serve(navigator, args, identity: str) -> None:
    """Serve exact names on /semantic_goal, with explicit status and cancellation."""
    items = load_landmarks(args.database, identity)
    publisher = navigator.create_publisher(
        String,
        '/semantic_navigation/status',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    marker_publisher = navigator.create_publisher(
        MarkerArray,
        '/semantic_navigation/landmarks',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    marker_publisher.publish(
        landmark_markers(items, navigator.get_clock().now().to_msg())
    )
    pending = []
    active = None
    cancel_requested = False

    def status(state, detail=''):
        publisher.publish(
            String(
                data=json.dumps({'state': state, 'detail': detail}, ensure_ascii=False)
            )
        )

    def receive(message):
        nonlocal cancel_requested
        if message.data.strip() == '__cancel__':
            cancel_requested = True
        elif active is not None or pending:
            status('rejected', 'Busy; cancel the current goal first')
        else:
            pending.append(message.data)

    subscription = navigator.create_subscription(String, '/semantic_goal', receive, 10)
    status('ready', ', '.join(item['name'] for item in items))
    started = 0.0
    while rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.1)
        if cancel_requested:
            pending.clear()
            if active is not None:
                navigator.cancelTask()
            cancel_requested = False
        if active is not None:
            if time.monotonic() - started > args.timeout:
                navigator.cancelTask()
                status('timeout', active)
                active = None
            elif navigator.isTaskComplete():
                result = navigator.getResult()
                state = {
                    TaskResult.SUCCEEDED: 'succeeded',
                    TaskResult.CANCELED: 'canceled',
                }.get(result, 'failed')
                status(state, active)
                active = None
        elif pending:
            query = pending.pop(0)
            try:
                item = resolve_landmark(items, query)
                if not navigator.nav_to_pose_client.server_is_ready():
                    raise ValueError('Navigation server unavailable')
                pose = goal_pose(item, navigator.get_clock().now().to_msg())
                # Nav2 performs footprint and Ackermann reachability checks.
                # A named pose can become unreachable when the scene changes.
                active = item['name']
                status('planning', active)
                if not navigator.goToPose(pose):
                    status('rejected', active)
                    active = None
                else:
                    started = time.monotonic()
                    status('navigating', active)
            except ValueError as error:
                active = None
                status('rejected', str(error))
    navigator.destroy_subscription(subscription)


def main(args=None) -> None:
    """Run manual annotation or semantic navigation against a saved map."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, dest='map_file')
    parser.add_argument('--database', required=True)
    parser.add_argument(
        '--record', help='Record current localized approach pose under this name'
    )
    parser.add_argument('--alias', action='append', default=[])
    parser.add_argument('--timeout', type=float, default=180.0)
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    if not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='semantic_navigation')
    try:
        identity = map_identity(options.map_file)
        if options.record is not None:
            record_landmark(navigator, options, identity)
        else:
            serve(navigator, options, identity)
    except (OSError, ValueError, RuntimeError) as error:
        navigator.get_logger().error(str(error))
        raise SystemExit(1) from error
    except KeyboardInterrupt:
        if navigator.goal_handle is not None:
            navigator.cancelTask()
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
