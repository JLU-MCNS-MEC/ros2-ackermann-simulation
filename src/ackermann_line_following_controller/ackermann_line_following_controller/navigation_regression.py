"""Run bounded, repeatable navigation trials against an already localized robot."""

import argparse
import json
import math
from pathlib import Path
import statistics
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64

from .navigation_experiment_node import ExperimentAccumulator, NavigationExperimentNode


def load_route(path):
    """Require named, finite planar goals before commanding any motion."""
    goals = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(goals, list) or not goals:
        raise ValueError('Route requires a nonempty goal list')
    for goal in goals:
        if not isinstance(goal, dict) or not isinstance(goal.get('name'), str):
            raise ValueError('Each goal requires a name')
        pose = goal.get('pose')
        if not goal['name'].strip() or not isinstance(pose, list) or len(pose) != 3:
            raise ValueError('Each goal requires a name and [x, y, yaw]')
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not math.isfinite(v) for v in pose):
            raise ValueError('Goal coordinates must be finite numbers')
    return goals


def summarize_trials(trials, requested):
    """Keep aborted/incomplete suites distinct from fully successful suites."""
    times = sorted(t['wall_duration_s'] for t in trials)
    successes = sum(t['success'] for t in trials)
    return {
        'requested_trials': requested, 'completed_trials': len(trials),
        'successes': successes, 'success_rate_of_requested': successes / requested,
        'complete': len(trials) == requested,
        'all_passed': len(trials) == requested and successes == requested,
        'median_wall_duration_s': statistics.median(times) if times else None,
        'p95_wall_duration_s': times[math.ceil(0.95 * len(times)) - 1] if times else None,
        'recovery_count': sum(t['recoveries'] for t in trials),
    }


class SensorHealth:
    """Measure receipt gaps and sample counts without assuming reliable transport."""

    def __init__(self):
        self.streams = {}

    def observe(self, name, received, stamp):
        """Track each sensor independently; timestamps are in sensor clock units."""
        state = self.streams.setdefault(name, {
            'count': 0, 'first_wall': received, 'last_wall': received,
            'first_stamp': stamp, 'last_stamp': stamp,
            'max_wall_gap_s': 0.0, 'nonmonotonic_stamps': 0,
        })
        if state['count']:
            state['max_wall_gap_s'] = max(state['max_wall_gap_s'], received - state['last_wall'])
            state['nonmonotonic_stamps'] += int(stamp <= state['last_stamp'])
        state['count'] += 1
        state['last_wall'], state['last_stamp'] = received, stamp

    def report(self, now):
        """Return rates in both wall and sensor time, and final receipt age."""
        return {name: dict(state,
                          wall_rate_hz=(state['count'] - 1) / max(
                              1e-9, state['last_wall'] - state['first_wall']),
                          sensor_rate_hz=(state['count'] - 1) / max(
                              1e-9, state['last_stamp'] - state['first_stamp']),
                          last_receipt_age_s=max(0.0, now - state['last_wall']))
                for name, state in self.streams.items()}


def run_trials(navigator, goals, count, timeout, output):
    """Record every result; cancel on timeout and stop the suite on first failure."""
    values = {}
    health = SensorHealth()
    recorder = ExperimentAccumulator('indoor_regression')
    subscriptions = []
    trials = []
    required = ('/scan_nav', '/imu/data_raw', '/odometry/filtered')

    def sensor(name, message):
        stamp = message.header.stamp
        health.observe(name, time.monotonic(), stamp.sec + stamp.nanosec * 1e-9)
        if isinstance(message, Odometry):
            recorder.add_pose(message.pose.pose.position.x, message.pose.pose.position.y)

    for name in NavigationExperimentNode.VALUE_NAMES:
        subscriptions.append(navigator.create_subscription(
            Float64, '/nav_diagnostics/' + name,
            lambda m, key=name: values.update({key: (m.data, time.monotonic())}), 10))
    for topic, message_type in zip(required, (LaserScan, Imu, Odometry)):
        subscriptions.append(navigator.create_subscription(
            message_type, topic, lambda m, key=topic: sensor(key, m), qos_profile_sensor_data))

    def save():
        payload = dict(summarize_trials(trials, count), trials=trials,
                       sensor_health=health.report(time.monotonic()),
                       manual_interventions='not automatically observable',
                       clearance_definition='scan origin range, not body clearance')
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding='utf-8')
        temporary.replace(output)

    try:
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            rclpy.spin_once(navigator, timeout_sec=0.1)
            if (navigator.nav_to_pose_client.server_is_ready()
                    and all(k in health.streams for k in required)):
                break
        else:
            raise RuntimeError('Navigation server or required sensor streams unavailable')
        for index in range(count):
            if any(time.monotonic() - health.streams[k]['last_wall'] > 1.0 for k in required):
                raise RuntimeError('Required sensor stream stale before goal')
            goal = goals[index % len(goals)]
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = navigator.get_clock().now().to_msg()
            x, y, yaw = goal['pose']
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            start = time.monotonic()
            sim_start = navigator.get_clock().now().nanoseconds / 1e9
            recorder.begin(start)
            recoveries, reverse, moving, samples = 0, 0, 0, 0
            accepted = navigator.goToPose(pose)
            timed_out = False
            while accepted and not navigator.isTaskComplete():
                now = time.monotonic()
                fresh = {k: values[k][0] if k in values and now - values[k][1] < 0.5
                         else math.nan for k in NavigationExperimentNode.VALUE_NAMES}
                recorder.add_sample(now, fresh)
                speed = fresh['speed/measured']
                reverse += int(speed < -0.001)
                moving += int(abs(speed) > 0.001)
                samples += 1
                feedback = navigator.getFeedback()
                if feedback:
                    recoveries = max(recoveries, feedback.number_of_recoveries)
                if now - start > timeout:
                    navigator.cancelTask()
                    timed_out = True
                    break
            result = navigator.getResult() if accepted else TaskResult.FAILED
            outcome = 'TIMEOUT' if timed_out else result.name
            report = recorder.summary(outcome, time.monotonic())
            report.update(index=index + 1, goal=goal, recoveries=recoveries,
                          wall_duration_s=time.monotonic() - start,
                          sim_duration_s=navigator.get_clock().now().nanoseconds / 1e9 - sim_start,
                          reverse_fraction_of_moving_samples=reverse / moving if moving else None,
                          diagnostic_samples=samples)
            trials.append(report)
            save()
            print(json.dumps(report, allow_nan=False), flush=True)
            if not report['success']:
                break
    except BaseException:
        navigator.cancelTask()
        save()
        raise
    finally:
        for subscription in subscriptions:
            navigator.destroy_subscription(subscription)
    return summarize_trials(trials, count)['all_passed']


def main(args=None):
    """Execute a route file; the operator must localize the robot beforehand."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--timeout', type=float, default=150.0)
    parser.add_argument('--use-sim-time', action='store_true')
    options = parser.parse_args(args)
    if options.count <= 0 or not math.isfinite(options.timeout) or options.timeout <= 0:
        parser.error('count and timeout must be positive and finite')
    goals = load_route(options.route)
    output = Path(options.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        stream.write('{}\n')
    rclpy.init(args=[])
    navigator = BasicNavigator(node_name='navigation_regression')
    navigator.set_parameters([Parameter('use_sim_time', value=options.use_sim_time)])
    try:
        passed = run_trials(navigator, goals, options.count, options.timeout, output)
    finally:
        navigator.destroy_node()
        rclpy.shutdown()
    if not passed:
        raise SystemExit(1)
