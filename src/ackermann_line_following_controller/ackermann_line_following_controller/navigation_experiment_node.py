"""Record repeatable, machine-readable Nav2 experiment metrics."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Mapping

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_msgs.msg import Float64, String


ACTION_NAMES = ('CLEAR', 'STOP', 'SLOWDOWN', 'APPROACH', 'LIMIT')


@dataclass(frozen=True)
class ExperimentSample:
    """One synchronized snapshot of navigation diagnostic values."""

    elapsed_s: float
    commanded_speed_mps: float
    smoothed_speed_mps: float
    applied_speed_mps: float
    measured_speed_mps: float
    cross_track_error_m: float
    distance_to_goal_m: float
    front_clearance_m: float
    nearest_clearance_m: float
    action_code: int


class ExperimentAccumulator:
    """Accumulate route metrics independently from ROS transport."""

    def __init__(self, scenario: str) -> None:
        """Initialize empty metrics for a named scenario."""
        if not scenario.strip():
            raise ValueError('scenario must not be empty')
        self.scenario = scenario
        self.started_at: float | None = None
        self.samples: list[ExperimentSample] = []
        self.distance_travelled = 0.0
        self.last_pose: tuple[float, float] | None = None
        self.action_transitions = 0
        self.last_action: int | None = None

    @property
    def active(self) -> bool:
        """Return whether the route has started collecting samples."""
        return self.started_at is not None

    def begin(self, timestamp: float) -> None:
        """Start a fresh route measurement."""
        if not math.isfinite(timestamp):
            raise ValueError('timestamp must be finite')
        self.started_at = timestamp
        self.samples.clear()
        self.distance_travelled = 0.0
        self.last_pose = None
        self.action_transitions = 0
        self.last_action = None

    def add_pose(self, x: float, y: float) -> None:
        """Integrate planar odometry distance while the route is active."""
        if not self.active or not (math.isfinite(x) and math.isfinite(y)):
            return
        pose = (x, y)
        if self.last_pose is not None:
            self.distance_travelled += math.dist(self.last_pose, pose)
        self.last_pose = pose

    def add_sample(
        self, timestamp: float, values: Mapping[str, float]
    ) -> None:
        """Append a diagnostic sample and count decision transitions."""
        if not self.active:
            return
        assert self.started_at is not None
        raw_action = values['decision/action_code']
        action = (
            int(raw_action)
            if math.isfinite(raw_action)
            and 0 <= int(raw_action) < len(ACTION_NAMES)
            else 0
        )
        if self.last_action is not None and action != self.last_action:
            self.action_transitions += 1
        self.last_action = action
        self.samples.append(
            ExperimentSample(
                elapsed_s=max(0.0, timestamp - self.started_at),
                commanded_speed_mps=values['speed/commanded'],
                smoothed_speed_mps=values['speed/smoothed'],
                applied_speed_mps=values['speed/applied'],
                measured_speed_mps=values['speed/measured'],
                cross_track_error_m=values[
                    'navigation/cross_track_error'
                ],
                distance_to_goal_m=values['navigation/distance_to_goal'],
                front_clearance_m=values['obstacle/front_clearance'],
                nearest_clearance_m=values['obstacle/nearest_clearance'],
                action_code=action,
            )
        )

    @staticmethod
    def _finite(values: list[float]) -> list[float]:
        return [value for value in values if math.isfinite(value)]

    def summary(self, outcome: str, timestamp: float) -> dict[str, object]:
        """Build aggregate acceptance metrics for the current route."""
        if not self.active:
            raise RuntimeError('experiment has not started')
        assert self.started_at is not None
        speed_errors = self._finite([
            sample.applied_speed_mps - sample.measured_speed_mps
            for sample in self.samples
        ])
        cross_track = self._finite([
            abs(sample.cross_track_error_m) for sample in self.samples
        ])
        front = self._finite([
            sample.front_clearance_m for sample in self.samples
        ])
        nearest = self._finite([
            sample.nearest_clearance_m for sample in self.samples
        ])
        measured_speed = self._finite([
            abs(sample.measured_speed_mps) for sample in self.samples
        ])
        action_samples = {
            name: sum(sample.action_code == code for sample in self.samples)
            for code, name in enumerate(ACTION_NAMES)
        }
        return {
            'scenario': self.scenario,
            'outcome': outcome,
            'success': outcome == 'SUCCEEDED',
            'duration_s': max(0.0, timestamp - self.started_at),
            'sample_count': len(self.samples),
            'distance_travelled_m': self.distance_travelled,
            'mean_abs_cross_track_error_m': (
                sum(cross_track) / len(cross_track) if cross_track else None
            ),
            'max_abs_cross_track_error_m': (
                max(cross_track) if cross_track else None
            ),
            'speed_tracking_rmse_mps': (
                math.sqrt(sum(value * value for value in speed_errors)
                          / len(speed_errors))
                if speed_errors else None
            ),
            'max_measured_speed_mps': (
                max(measured_speed) if measured_speed else None
            ),
            'minimum_front_clearance_m': min(front) if front else None,
            'minimum_nearest_clearance_m': min(nearest) if nearest else None,
            'final_goal_distance_m': (
                self.samples[-1].distance_to_goal_m
                if self.samples
                and math.isfinite(self.samples[-1].distance_to_goal_m)
                else None
            ),
            'decision_transition_count': self.action_transitions,
            'decision_sample_counts': action_samples,
        }


class NavigationExperimentNode(Node):
    """Capture diagnostic topics between route start and completion."""

    VALUE_NAMES = (
        'speed/commanded',
        'speed/smoothed',
        'speed/applied',
        'speed/measured',
        'navigation/cross_track_error',
        'navigation/distance_to_goal',
        'obstacle/front_clearance',
        'obstacle/nearest_clearance',
        'decision/action_code',
    )

    def __init__(self) -> None:
        """Connect diagnostics, odometry and task lifecycle topics."""
        super().__init__('navigation_experiment')
        self.declare_parameter('scenario', 'unnamed')
        self.declare_parameter(
            'result_file', '/tmp/ackermann_navigation_experiment.json'
        )
        self.declare_parameter('sample_rate', 10.0)
        scenario = str(self.get_parameter('scenario').value)
        self.result_file = Path(str(self.get_parameter('result_file').value))
        sample_rate = float(self.get_parameter('sample_rate').value)
        if sample_rate <= 0.0:
            raise ValueError('sample_rate must be positive')
        self.accumulator = ExperimentAccumulator(scenario)
        self.values = {name: float('nan') for name in self.VALUE_NAMES}
        self.written = False
        for name in self.VALUE_NAMES:
            self.create_subscription(
                Float64,
                f'/nav_diagnostics/{name}',
                lambda message, key=name: self._value_callback(key, message),
                10,
            )
        self.create_subscription(
            Odometry,
            '/model/ackermann_car/odometry',
            self._odom_callback,
            10,
        )
        status_qos = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            String,
            '/navigation_experiment/task_status',
            self._status_callback,
            status_qos,
        )
        self.create_timer(1.0 / sample_rate, self._sample)
        self.get_logger().info(
            f'Recording navigation experiment {scenario!r} to '
            f'{self.result_file}'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _value_callback(self, name: str, message: Float64) -> None:
        self.values[name] = float(message.data)

    def _odom_callback(self, message: Odometry) -> None:
        self.accumulator.add_pose(
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )

    def _sample(self) -> None:
        self.accumulator.add_sample(self._now(), self.values)

    def _status_callback(self, message: String) -> None:
        outcome = message.data.strip().upper()
        if outcome == 'RUNNING':
            self.accumulator.begin(self._now())
            self.written = False
            self.get_logger().info('Navigation experiment started')
        elif outcome in {'SUCCEEDED', 'FAILED', 'CANCELED'}:
            if not self.accumulator.active:
                self.accumulator.begin(self._now())
            self.finalize(outcome)

    def finalize(self, outcome: str = 'INCOMPLETE') -> None:
        """Write CSV samples and a JSON summary exactly once."""
        if self.written or not self.accumulator.active:
            return
        summary = self.accumulator.summary(outcome, self._now())
        csv_file = self.result_file.with_suffix('.csv')
        try:
            self.result_file.parent.mkdir(parents=True, exist_ok=True)
            with csv_file.open('w', encoding='utf-8', newline='') as stream:
                writer = csv.DictWriter(
                    stream, fieldnames=list(ExperimentSample.__annotations__)
                )
                writer.writeheader()
                writer.writerows(asdict(sample)
                                 for sample in self.accumulator.samples)
            self.result_file.write_text(
                json.dumps(summary, indent=2, ensure_ascii=False) + '\n',
                encoding='utf-8',
            )
        except OSError as error:
            self.get_logger().error(
                f'Unable to write experiment report: {error}'
            )
            return
        self.written = True
        self.get_logger().info(
            f'Navigation experiment {outcome}: {self.result_file}'
        )


def main(args=None) -> None:
    """Run the navigation experiment recorder."""
    rclpy.init(args=args)
    node = NavigationExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
