#!/usr/bin/env python3
"""Live ROS 2 raw feedback using protocol 5; only VERSION/PING/GET_HOME/READ.

Close this monitor before starting the motion bridge: serial is exclusively owned.
Raw values are not calibrated angles and are never published as JointState.
"""
import argparse
from datetime import datetime
from zoneinfo import ZoneInfo
import json
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/dual_arm_control'))
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from dual_arm_control.cm530 import SerialCM530


class Monitor(Node):
    def __init__(self, args):
        super().__init__('cm530_readonly_monitor')
        self.serial = None
        self.output = args.output
        self.sequence = 0
        self.publisher = self.create_publisher(String, '/cm530/raw_state',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status = self.create_publisher(String, '/cm530/monitor_status', 10)
        try:
            self.serial = SerialCM530(args.port, args.timeout, args.startup_listen)
            self.homes = {arm: self.serial.get_home(arm) for arm in ('arm1', 'arm2')}
        except Exception:
            if self.serial:
                self.serial.close()
            raise
        self.timer = self.create_timer(args.period, self.poll)
        self.get_logger().info('Protocol 5 connected; raw feedback on /cm530/raw_state')
        self.poll()

    def publish_record(self, record):
        payload = json.dumps(record, ensure_ascii=False)
        self.publisher.publish(String(data=payload))
        if self.output:
            temporary = self.output.with_suffix('.tmp')
            temporary.write_text(payload + '\n')
            temporary.replace(self.output)

    def poll(self):
        record = {'checked_at': datetime.now(ZoneInfo('Asia/Taipei')).isoformat(),
                  'protocol': 5, 'mode': 'read_only', 'sequence': self.sequence,
                  'port': self.serial.port.port, 'arms': {}}
        try:
            for arm in ('arm1', 'arm2'):
                raw = self.serial.read(arm)
                record['arms'][arm] = {'home_raw': self.homes[arm], 'position_raw': raw,
                                      'received_monotonic': time.monotonic()}
            record['valid'] = True
            self.publish_record(record)
            self.status.publish(String(data='read_only_connected'))
            if self.sequence % 10 == 0:
                self.get_logger().info(json.dumps(record['arms']))
            self.sequence += 1
        except Exception as exc:
            # No retries after ERR or an ambiguous transaction. Discard any
            # partial dual-arm observation instead of presenting it as valid.
            self.timer.cancel()
            self.serial.close()
            record.update(valid=False, arms={}, error=str(exc))
            self.publish_record(record)
            self.status.publish(String(data='fault: ' + str(exc)))
            self.get_logger().error('Monitoring stopped; serial closed: ' + str(exc))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--period', type=float, default=1.0)
    parser.add_argument('--timeout', type=float, default=2.0)
    parser.add_argument('--startup-listen', type=float, default=6.0)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.period) or args.period <= 0:
        parser.error('period must be positive and finite')
    rclpy.init()
    node = None
    try:
        node = Monitor(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.serial.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
