"""Run explicit map-frame goals and write actual Nav2 outcomes as JSON.

Example: python3 scripts/indoor_route_check.py --goals '[[5,0,0],[6,2,1.57]]'
"""

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goals', required=True)
    parser.add_argument('--output', default='/tmp/indoor_route_result.json')
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    goals = json.loads(args.goals)
    if not isinstance(goals, list) or not goals or any(
        not isinstance(p, list) or len(p) != 3 or any(
            not isinstance(v, (float, int)) or not math.isfinite(v) for v in p
        ) for p in goals
    ):
        parser.error('goals must be a nonempty list of finite [x, y, yaw]')
    rclpy.init()
    navigator = BasicNavigator(node_name='indoor_route_check')
    navigator.set_parameters([Parameter('use_sim_time', value=True)])
    results = []
    try:
        if not navigator.nav_to_pose_client.wait_for_server(timeout_sec=30):
            raise RuntimeError('Nav2 action unavailable')
        for x, y, yaw in goals:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = navigator.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            started = time.monotonic()
            accepted = navigator.goToPose(pose)
            timed_out = False
            while accepted and not navigator.isTaskComplete():
                if time.monotonic() - started > args.timeout:
                    navigator.cancelTask()
                    timed_out = True
                    break
            succeeded = accepted and not timed_out and navigator.getResult() == TaskResult.SUCCEEDED
            results.append({'goal': [x, y, yaw], 'succeeded': succeeded,
                            'timeout': timed_out, 'seconds': time.monotonic() - started})
            print(json.dumps(results[-1]), flush=True)
            if not succeeded:
                break
    finally:
        Path(args.output).write_text(json.dumps(results, indent=2) + '\n')
        navigator.destroy_node()
        rclpy.shutdown()
    if len(results) != len(goals) or not all(r['succeeded'] for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
