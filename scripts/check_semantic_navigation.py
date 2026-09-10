"""Send named goals and record semantic navigation terminal statuses."""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goals', nargs='+', required=True)
    parser.add_argument('--output', default='/tmp/semantic_navigation_result.json')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('semantic_navigation_check')
    events = []
    node.create_subscription(
        String,
        '/semantic_navigation/status',
        lambda message: events.append(json.loads(message.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    publisher = node.create_publisher(String, '/semantic_goal', 10)
    results = []
    try:
        deadline = time.monotonic() + 15
        while (
            not events or publisher.get_subscription_count() == 0
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not events or publisher.get_subscription_count() == 0:
            raise RuntimeError('Semantic navigation server unavailable')
        for query in args.goals:
            events.clear()
            started = time.monotonic()
            publisher.publish(String(data=query))
            terminal = None
            while time.monotonic() - started < 190:
                rclpy.spin_once(node, timeout_sec=0.1)
                terminal = next(
                    (
                        event
                        for event in events
                        if event['state']
                        in ('succeeded', 'failed', 'rejected', 'canceled', 'timeout')
                    ),
                    None,
                )
                if terminal is not None:
                    break
            if terminal is None:
                publisher.publish(String(data='__cancel__'))
            result = {
                'query': query,
                'seconds': time.monotonic() - started,
                'terminal': terminal,
                'events': list(events),
            }
            results.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        Path(args.output).write_text(
            json.dumps(results, ensure_ascii=False, indent=2) + '\n'
        )
        node.destroy_node()
        rclpy.shutdown()
    if any(
        result['terminal'] is None or result['terminal']['state'] != 'succeeded'
        for result in results
    ):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
