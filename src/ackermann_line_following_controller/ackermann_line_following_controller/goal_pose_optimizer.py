"""Adapt position-first RViz goals to short Ackermann-compatible poses."""

from __future__ import annotations

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .visual_policy import normalize_angle, quaternion_to_yaw


def preferred_goal_yaw(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    goal_x: float,
    goal_y: float,
    allow_reverse: bool = True,
    position_tolerance: float = 0.25,
) -> float:
    """Choose the forward or reverse arrival yaw needing less initial turn."""
    values = (robot_x, robot_y, robot_yaw, goal_x, goal_y, position_tolerance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('poses and position_tolerance must be finite')
    if position_tolerance < 0.0:
        raise ValueError('position_tolerance must be nonnegative')
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    if math.hypot(dx, dy) <= position_tolerance:
        return normalize_angle(robot_yaw)
    forward_yaw = math.atan2(dy, dx)
    if not allow_reverse:
        return forward_yaw
    reverse_yaw = normalize_angle(forward_yaw + math.pi)
    forward_turn = abs(normalize_angle(forward_yaw - robot_yaw))
    reverse_turn = abs(normalize_angle(reverse_yaw - robot_yaw))
    return reverse_yaw if reverse_turn < forward_turn else forward_yaw


def optimized_goal_pose(
    goal: PoseStamped,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    allow_reverse: bool = True,
) -> PoseStamped:
    """Copy a goal and replace only its orientation with a shorter approach."""
    if not goal.header.frame_id:
        raise ValueError('goal frame_id must not be empty')
    output = deepcopy(goal)
    yaw = preferred_goal_yaw(
        robot_x,
        robot_y,
        robot_yaw,
        goal.pose.position.x,
        goal.pose.position.y,
        allow_reverse,
    )
    output.pose.orientation.x = 0.0
    output.pose.orientation.y = 0.0
    output.pose.orientation.z = math.sin(yaw / 2.0)
    output.pose.orientation.w = math.cos(yaw / 2.0)
    return output


class GoalPoseOptimizer(Node):
    """Relay position-first goals after choosing a short arrival direction."""

    def __init__(self) -> None:
        """Connect an isolated RViz input to Nav2's standard goal topic."""
        super().__init__('goal_pose_optimizer')
        self.declare_parameter('input_topic', '/goal_pose_auto')
        self.declare_parameter('output_topic', '/goal_pose')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('allow_reverse', True)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('output_topic').value), 10
        )
        self.debug_publisher = self.create_publisher(
            PoseStamped, '/goal_pose_optimized', 10
        )
        self.subscription = self.create_subscription(
            PoseStamped,
            str(self.get_parameter('input_topic').value),
            self._goal_callback,
            10,
        )
        self.get_logger().info(
            'Position-first goals are relayed to Nav2 with an optimized yaw'
        )

    def _goal_callback(self, goal: PoseStamped) -> None:
        try:
            transform = self.buffer.lookup_transform(
                goal.header.frame_id,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            robot_yaw = quaternion_to_yaw(
                rotation.x, rotation.y, rotation.z, rotation.w
            )
            output = optimized_goal_pose(
                goal,
                translation.x,
                translation.y,
                robot_yaw,
                self.allow_reverse,
            )
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f'Goal was not relayed: {error}')
            return
        output.header.stamp = self.get_clock().now().to_msg()
        self.debug_publisher.publish(output)
        self.publisher.publish(output)


def main(args=None) -> None:
    """Run the position-first goal adapter."""
    rclpy.init(args=args)
    node = GoalPoseOptimizer()
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
