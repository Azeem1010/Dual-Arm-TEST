"""Publish explicit NOT_RUN/FAIL until a calibration and fresh image pass checks.

This monitor does not itself authorize any motion. The coordinator must recheck
the same record against the target acquisition time and current TF pose.
"""
from pathlib import Path
import hashlib
import json
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from .validity import validate


def verify_files(record, directory):
    files = record.get('evidence_files')
    if not isinstance(files, list) or not files:
        return False
    try:
        for item in files:
            path = (directory/item['path']).resolve()
            if not path.is_relative_to(directory.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
                return False
    except (OSError, KeyError, TypeError):
        return False
    return True


class CalibrationStatusNode(Node):
    def __init__(self):
        super().__init__('calibration_status')
        for name, default in [('mode','sim'),('calib_dir',''),('mount_revision',''),('pose_signature',''),('max_age_s',0.5)]:
            self.declare_parameter(name, default)
        self.images = {}
        self.subscriptions_keep = [self.create_subscription(Image, f'/{camera}_cam/image_raw', lambda msg, c=camera: self.images.__setitem__(c,msg), qos_profile_sensor_data) for camera in ('env','arm1','arm2')]
        self.pub = self.create_publisher(DiagnosticArray, '/vision/calibration_status', 10)
        self.timer = self.create_timer(.5,self.tick)

    def tick(self):
        out = DiagnosticArray()
        out.header.stamp = self.get_clock().now().to_msg()
        now = self.get_clock().now().nanoseconds/1e9
        directory = Path(self.get_parameter('calib_dir').value).expanduser()
        for camera in ('env','arm1','arm2'):
            status = DiagnosticStatus(name=f'calibration/{camera}', hardware_id=camera)
            try:
                data = yaml.safe_load((directory/f'{camera}_camera.yaml').read_text())
                if not isinstance(data, dict):
                    raise ValueError('Calibration is not a mapping')
                data['evidence_verified'] = verify_files(data, directory)
                msg = self.images.get(camera)
                if msg is None:
                    raise ValueError('No camera image')
                errors = validate(data, mode=self.get_parameter('mode').value, camera_id=camera,
                                  frame_id=msg.header.frame_id, reference_frame='world' if camera=='env' else camera+'_base_link',
                                  image_size=(msg.width,msg.height), now=now,
                                  image_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                                  max_age_s=self.get_parameter('max_age_s').value,
                                  mount_revision=self.get_parameter('mount_revision').value,
                                  pose_signature=self.get_parameter('pose_signature').value)
            except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
                errors = [str(exc)]
            status.level = DiagnosticStatus.ERROR if errors else DiagnosticStatus.OK
            status.message = 'FAIL' if errors else 'PASS'
            status.values = [KeyValue(key='reasons',value=json.dumps(errors))]
            out.status.append(status)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
