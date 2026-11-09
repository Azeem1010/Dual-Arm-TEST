"""Save actual Gazebo camera images from the live demo domain."""
import os
os.environ['ROS_DOMAIN_ID']='43'
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
node=Node('capture_revised_views')
frames={}
subscriptions=[]
for name in ('top','isometric','side'):
    subscriptions.append(node.create_subscription(Image,'/view_'+name+'/image_raw',
        lambda msg,n=name:frames.__setitem__(n,msg),qos_profile_sensor_data))
try:
    deadline=time.monotonic()+45
    while time.monotonic()<deadline and len(frames)<3:
        rclpy.spin_once(node,timeout_sec=.1)
    out=Path(__file__).resolve().parents[1]/'docs/evidence'
    for name,msg in frames.items():
        if msg.encoding not in ('rgb8','bgr8'):
            raise ValueError(msg.encoding)
        pixels=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=='rgb8':
            pixels=cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR)
        if not cv2.imwrite(str(out/f'revised_live_{name}.png'),pixels):
            raise RuntimeError('Image write failed')
        print(name,msg.width,msg.height,flush=True)
    if len(frames)!=3:
        raise RuntimeError('Not all requested camera frames arrived')
finally:
    node.destroy_node()
    rclpy.shutdown()
