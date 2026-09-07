"""Measure a stationary sensor_lab run; write reproducible JSON to stdout."""

import json
import math
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan, PointCloud2


def main():
    """Collect 20 wall seconds after discovery, failing on absent streams."""
    rclpy.init()
    node = rclpy.create_node('sensor_benchmark')
    records = {}
    subscriptions = []
    topics = {
        '/scan': LaserScan,
        '/imu/data_raw': Imu,
        '/rgbd/image': Image,
        '/rgbd/depth_image': Image,
        '/rgbd/camera_info': CameraInfo,
        '/rgbd/points': PointCloud2,
    }

    def receive(topic, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row = records.setdefault(topic, {'stamps': [], 'wall': [], 'values': []})
        row['stamps'].append(stamp)
        row['wall'].append(time.monotonic())
        row['frame'] = msg.header.frame_id
        if topic == '/scan':
            finite_ranges = [
                float(value)
                for value in msg.ranges
                if math.isfinite(value) and value >= msg.range_min
            ]
            row['values'].append(
                min(finite_ranges) if finite_ranges else float('nan')
            )
            row['samples'] = len(msg.ranges)
        elif topic == '/rgbd/depth_image':
            row['encoding'] = msg.encoding
            if msg.encoding != '32FC1':
                return
            dtype = '>f4' if msg.is_bigendian else '<f4'
            pixels = np.ndarray((msg.height, msg.width), dtype=dtype,
                                buffer=msg.data, strides=(msg.step, 4))
            row['values'].append(float(pixels[msg.height // 2, msg.width // 2]))
            row['valid_fraction'] = float(np.isfinite(pixels).mean())
        elif isinstance(msg, (Image, PointCloud2)):
            row['size'] = [msg.width, msg.height]
        elif topic == '/imu/data_raw':
            row['values'].append((
                math.sqrt(
                    msg.angular_velocity.x ** 2
                    + msg.angular_velocity.y ** 2
                    + msg.angular_velocity.z ** 2
                ),
                math.sqrt(
                    msg.linear_acceleration.x ** 2
                    + msg.linear_acceleration.y ** 2
                    + msg.linear_acceleration.z ** 2
                ),
            ))

    try:
        for topic, message_type in topics.items():
            subscriptions.append(node.create_subscription(
                message_type, topic, lambda msg, topic=topic: receive(topic, msg),
                qos_profile_sensor_data))
        deadline = time.monotonic() + 15
        while len(records) < len(topics) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        records.clear()
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = {}
        passed = True
        for topic in topics:
            row = records.get(topic, {})
            stamps = row.pop('stamps', [])
            wall = row.pop('wall', [])
            values = row.pop('values', [])
            row['count'] = len(stamps)
            if len(stamps) > 1:
                row['sim_hz'] = (len(stamps) - 1) / (stamps[-1] - stamps[0])
                row['wall_hz'] = (len(wall) - 1) / (wall[-1] - wall[0])
                row['monotonic'] = bool(np.all(np.diff(stamps) > 0))
            ok = len(stamps) > 2 and row.get('monotonic', False)
            if topic in ('/scan', '/rgbd/depth_image'):
                expected = 3.0 if topic == '/scan' else 2.64
                finite = [v for v in values if math.isfinite(v)]
                row['expected_m'] = expected
                if finite:
                    row['mean_m'] = float(np.mean(finite))
                    row['std_m'] = float(np.std(finite))
                    row['absolute_error_m'] = abs(row['mean_m'] - expected)
                ok = ok and len(finite) == len(stamps) and row.get('absolute_error_m', 99) < 0.05
            elif topic == '/imu/data_raw':
                gyro_norms = [value[0] for value in values]
                acceleration_norms = [value[1] for value in values]
                if values:
                    row['gyro_norm_mean_rad_s'] = float(
                        np.mean(gyro_norms)
                    )
                    row['acceleration_norm_mean_m_s2'] = float(
                        np.mean(acceleration_norms)
                    )
                    row['gravity_absolute_error_m_s2'] = abs(
                        row['acceleration_norm_mean_m_s2'] - 9.81
                    )
                ok = (
                    ok
                    and len(values) == len(stamps)
                    and row.get('gyro_norm_mean_rad_s', 99.0) < 0.02
                    and row.get('gravity_absolute_error_m_s2', 99.0) < 0.15
                )
            row['data_passed'] = bool(ok)
            if topic == '/scan':
                row['minimum_wall_hz'] = 9.0
            elif topic == '/imu/data_raw':
                row['minimum_wall_hz'] = 80.0
            else:
                row['minimum_wall_hz'] = 12.0
            row['rate_passed'] = row.get('wall_hz', 0) >= row['minimum_wall_hz']
            ok = ok and row['rate_passed']
            row['passed'] = bool(ok)
            passed = passed and ok
            report[topic] = row
        print(json.dumps({'passed': bool(passed), 'topics': report}, indent=2))
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
