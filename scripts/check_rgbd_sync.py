"""Check synchronized RGB-D intrinsics and capture-time map TF for semantics."""

import argparse
import json
from pathlib import Path
import time

from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/indoor_rgbd_sync.json')
    parser.add_argument('--samples', type=int, default=10)
    args = parser.parse_args()
    if args.samples <= 0:
        parser.error('--samples must be positive')
    rclpy.init()
    node = rclpy.create_node(
        'rgbd_sync_check', parameter_overrides=[Parameter('use_sim_time', value=True)]
    )
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    subscribers = [
        Subscriber(node, msg_type, topic, qos_profile=qos_profile_sensor_data)
        for msg_type, topic in [
            (Image, '/rgbd/image'),
            (Image, '/rgbd/depth_image'),
            (CameraInfo, '/rgbd/camera_info'),
        ]
    ]
    synchronizer = ApproximateTimeSynchronizer(subscribers, queue_size=10, slop=0.03)
    samples = []

    def receive(rgb, depth, info):
        stamp = rclpy.time.Time.from_msg(rgb.header.stamp)
        try:
            transform = buffer.lookup_transform('map', rgb.header.frame_id, stamp)
        except TransformException:
            return
        stamps = [
            rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds
            for msg in (rgb, depth, info)
        ]
        samples.append(
            {
                'stamp_ns': stamp.nanoseconds,
                'spread_ms': (max(stamps) - min(stamps)) / 1e6,
                'rgb_size': [rgb.width, rgb.height],
                'depth_size': [depth.width, depth.height],
                'rgb_encoding': rgb.encoding,
                'depth_encoding': depth.encoding,
                'fx': info.k[0],
                'fy': info.k[4],
                'map_camera_position': [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
            }
        )

    synchronizer.registerCallback(receive)
    deadline = time.monotonic() + 30
    try:
        while (
            rclpy.ok() and len(samples) < args.samples and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()
    passed = len(samples) >= args.samples and all(
        sample['fx'] > 0
        and sample['fy'] > 0
        and sample['rgb_size'] == sample['depth_size']
        for sample in samples
    )
    result = {
        'passed': passed,
        'samples': samples,
        'limitation': 'Checks timing, dimensions and TF only; no semantic detection or depth accuracy claim.',
    }
    Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'passed': passed, 'samples': len(samples)}))
    if not passed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
