"""Audit synchronized RGB-D quality and acquisition-time TF without driving."""

import argparse
from collections import Counter, deque
import json
import math
from pathlib import Path
import time

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from .navigation_regression import SensorHealth


def depth_statistics(depth, encoding, near=0.2, far=8.0):
    """Normalize supported depth units; unknown encodings never imply free space."""
    if not (math.isfinite(near) and math.isfinite(far) and 0 < near < far):
        raise ValueError('Require finite 0 < near < far')
    expected = {'32FC1': np.dtype('float32'), '16UC1': np.dtype('uint16')}
    if encoding not in expected or depth.dtype != expected[encoding]:
        raise ValueError('Unsupported depth encoding or dtype')
    if depth.ndim != 2 or depth.size == 0:
        raise ValueError('Depth must be a nonempty 2D image')
    metres = depth.astype(np.float32) * (0.001 if encoding == '16UC1' else 1.0)
    valid = np.isfinite(metres) & (metres >= near) & (metres <= far)
    return {'valid_fraction': float(np.mean(valid)),
            'median_depth_m': float(np.median(metres[valid])) if valid.any() else None}


def validate_camera_pair(rgb, depth, info):
    """Require registered images, matching optical frames and valid intrinsics."""
    if not rgb.header.frame_id or len({rgb.header.frame_id, depth.header.frame_id,
                                       info.header.frame_id}) != 1:
        raise ValueError('RGB/depth/CameraInfo frames must match')
    if not (rgb.width == depth.width == info.width > 0
            and rgb.height == depth.height == info.height > 0):
        raise ValueError('Registered image dimensions must match CameraInfo')
    if (not np.isfinite(info.k).all() or info.k[0] <= 0 or info.k[4] <= 0
            or not 0 <= info.k[2] < info.width or not 0 <= info.k[5] < info.height):
        raise ValueError('Invalid camera intrinsics')
    stamps = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in (rgb, depth, info)]
    return max(stamps) - min(stamps)


def audit_summary(samples, errors, received, minimum_samples):
    """Build explicit acceptance gates, including missing and rejected data."""
    valid = [s for s in samples if s['tf_valid']]
    total = len(samples) + sum(errors.values())
    p95 = float(np.percentile([s['sync_delta_s'] for s in samples], 95)) if samples else None
    depth_fraction = float(np.median([s['valid_fraction'] for s in samples])) if samples else None
    ages = [s['frame_age_s'] for s in samples]
    age_p95 = float(np.percentile(ages, 95)) if ages else None
    maximum_received = max((s['count'] for s in received.values()), default=0)
    gates = {
        'enough_samples': len(samples) >= minimum_samples,
        'valid_pairs': bool(total) and len(samples) / total >= 0.95,
        'acquisition_tf': bool(samples) and len(valid) / len(samples) >= 0.99,
        'sync_p95_le_20ms': p95 is not None and p95 <= 0.020,
        'frame_age_p95_le_200ms': age_p95 is not None and age_p95 <= 0.2,
        'no_future_frames': bool(ages) and min(ages) >= -0.02,
        'depth_valid_median_ge_10pct': depth_fraction is not None and depth_fraction >= 0.1,
        'matching_ge_90pct': bool(maximum_received) and total / maximum_received >= 0.9,
        'all_streams_present': len(received) == 3,
        'monotonic_stamps': bool(received) and all(
            v['nonmonotonic_stamps'] == 0 for v in received.values()),
        'receipt_gaps_le_500ms': bool(received) and all(
            max(v['max_wall_gap_s'], v['last_receipt_age_s']) <= 0.5 for v in received.values()),
    }
    return {'passed': all(gates.values()), 'gates': gates, 'matched_samples': len(samples),
            'tf_valid_samples': len(valid), 'errors': dict(errors), 'sync_p95_s': p95,
            'frame_age_p95_s': age_p95,
            'median_valid_depth_fraction': depth_fraction, 'sensor_health': received}


class RgbdAudit(Node):
    """Bound synchronization and TF queues; never relabel stale input as current."""

    def __init__(self, use_sim_time=False, reliable=False):
        super().__init__('rgbd_audit', parameter_overrides=[
            Parameter('use_sim_time', value=use_sim_time)])
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.health = SensorHealth()
        self.samples = []
        self.errors = Counter()
        self.pending = deque()
        self.camera_metadata = None
        qos = QoSProfile(depth=5) if reliable else qos_profile_sensor_data
        self.inputs = [message_filters.Subscriber(self, typ, topic,
                       qos_profile=qos)
                       for typ, topic in [(Image, '/rgbd/image'), (Image, '/rgbd/depth_image'),
                                          (CameraInfo, '/rgbd/camera_info')]]
        for subscriber, name in zip(self.inputs, ('rgb', 'depth', 'camera_info')):
            subscriber.registerCallback(lambda msg, key=name: self._received(key, msg))
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            self.inputs, queue_size=15, slop=0.02)
        self.synchronizer.registerCallback(self._matched)
        self.timer = self.create_timer(0.02, self._drain)

    def _received(self, name, message):
        stamp = message.header.stamp
        self.health.observe(name, time.monotonic(), stamp.sec + stamp.nanosec * 1e-9)

    def _matched(self, rgb, depth, info):
        try:
            delta = validate_camera_pair(rgb, depth, info)
            color = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
            if color.shape != (rgb.height, rgb.width, 3):
                raise ValueError('RGB buffer dimensions do not match header')
            array = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            stats = depth_statistics(array, depth.encoding)
            stats['sync_delta_s'] = delta
            stats['frame_age_s'] = (
                self.get_clock().now().nanoseconds
                - Time.from_msg(depth.header.stamp).nanoseconds) / 1e9
            self.camera_metadata = {'width': info.width, 'height': info.height,
                                    'k': list(info.k), 'd': list(info.d),
                                    'distortion_model': info.distortion_model,
                                    'frame_id': info.header.frame_id, 'depth_encoding': depth.encoding}
            if len(self.pending) >= 15:
                self.pending.popleft()
                self.errors['tf_queue_overflow'] += 1
            self.pending.append((time.monotonic(), depth.header, stats))
        except (ValueError, TypeError, RuntimeError, CvBridgeError) as error:
            self.errors[str(error)] += 1

    def _drain(self):
        while self.pending:
            received, header, stats = self.pending[0]
            try:
                transform = self.buffer.lookup_transform('map', header.frame_id,
                                                         Time.from_msg(header.stamp))
                stats['tf_valid'] = True
                stats['tf_wait_wall_s'] = time.monotonic() - received
                stats['map_position'] = [transform.transform.translation.x,
                                         transform.transform.translation.y,
                                         transform.transform.translation.z]
            except TransformException:
                if time.monotonic() - received < 0.3:
                    break
                stats['tf_valid'] = False
            self.samples.append(stats)
            self.pending.popleft()


def main(args=None):
    """Capture a bounded audit and return failure when acceptance gates fail."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=120.0)
    parser.add_argument('--minimum-samples', type=int, default=300)
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--reliable', action='store_true',
                        help='Use only with RELIABLE sensor publishers; reduces large-frame loss')
    options = parser.parse_args(args)
    if not math.isfinite(options.seconds) or options.seconds <= 0 or options.minimum_samples <= 0:
        parser.error('seconds and minimum-samples must be positive')
    output = Path(options.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x', encoding='utf-8') as stream:
        stream.write('{}\n')
    rclpy.init(args=[])
    node = RgbdAudit(options.use_sim_time, options.reliable)
    start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < options.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
        # Unresolved frames remain failures rather than silently disappearing.
        node.errors['pending_at_end'] += len(node.pending)
        report = audit_summary(node.samples, node.errors, node.health.report(time.monotonic()),
                               options.minimum_samples)
        report.update(camera=node.camera_metadata, reliable=options.reliable,
                      wall_duration_s=time.monotonic() - start,
                      samples=node.samples)
        output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
        print(json.dumps({k: v for k, v in report.items() if k != 'samples'}), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not report['passed']:
        raise SystemExit(1)
