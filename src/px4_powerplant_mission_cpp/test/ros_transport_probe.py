#!/usr/bin/env python3
"""Bounded two-process ROS topic/service probe; never connects to a vehicle."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--child', action='store_true')
    parser.add_argument('--timeout', type=float, default=5.)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node('p0_probe_child' if args.child else 'p0_probe_parent')
    process = None
    try:
        if args.child:
            received = []
            sub = node.create_subscription(String, '/p0_probe/input', received.append, 10)

            def respond(request, response):
                response.success = True
                return response

            node.create_service(Trigger, '/p0_probe/service', respond)
            deadline = time.monotonic() + args.timeout + 1.
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            print(json.dumps({'received': len(received), 'publishers': sub.get_publisher_count()}), flush=True)
            return
        pub = node.create_publisher(String, '/p0_probe/input', 10)
        client = node.create_client(Trigger, '/p0_probe/service')
        process = subprocess.Popen([sys.executable, __file__, '--child', '--timeout', str(args.timeout)],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        future = None
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            pub.publish(String(data='p0_transport_probe'))
            rclpy.spin_once(node, timeout_sec=.02)
            if client.service_is_ready() and future is None:
                future = client.call_async(Trigger.Request())
        output, error = process.communicate(timeout=3.)
        child = json.loads(output.strip().splitlines()[-1]) if output.strip() else {}
        report = {'schema': 1, 'service_response': bool(future and future.done() and future.result().success),
                  'child': child, 'child_exit_code': process.returncode, 'child_stderr': error}
        report['passed'] = report['service_response'] and child.get('received', 0) > 0 and process.returncode == 0
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, ensure_ascii=False))
        if not report['passed']:
            raise SystemExit(1)
    finally:
        if process and process.poll() is None:
            process.kill()
            process.communicate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
