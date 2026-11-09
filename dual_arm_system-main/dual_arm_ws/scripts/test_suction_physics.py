"""Actual Gazebo gravity test for attach/hold/release, no rendering required."""
import os
import json
import signal
import subprocess
import time
import uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from ament_index_python.packages import get_package_prefix

root=Path(__file__).resolve().parents[1]
evidence=root/'docs/evidence'
os.environ['GZ_PARTITION']='suction_test_'+uuid.uuid4().hex
os.environ['ROS_DOMAIN_ID']=str(50+os.getpid()%100)
plugin=Path(get_package_prefix('dual_arm_gripper'))/'lib/libdual_arm_suction.so'
world=evidence/'suction_test.sdf'
world.write_text(f'''<sdf version="1.9"><world name="suction_test"><physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics><gravity>0 0 -9.81</gravity><plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<model name="fixture"><static>true</static><link name="arm1_cad_link_4"><pose>0 0 .205 -1.5707963267948966 0 0</pose></link><link name="arm2_cad_link_4"><pose>1 0 .205 -1.5707963267948966 0 0</pose></link><plugin filename="{plugin}" name="dual_arm_gripper::SuctionSystem"><arm1_tip>0 0 0</arm1_tip><arm2_tip>0 0 0</arm2_tip></plugin></model>
<model name="cap_1"><pose>0 0 .2 0 0 0</pose><link name="body"><inertial><mass>.002</mass><inertia><ixx>0.000000067</ixx><iyy>0.000000067</iyy><izz>0.0000001</izz></inertia></inertial><collision name="body"><geometry><cylinder><radius>.01</radius><length>.01</length></cylinder></geometry></collision></link><plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher"><publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose><use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>50</update_frequency></plugin></model>
</world></sdf>''')
logs=[(evidence/'suction_gazebo.log').open('w'),(evidence/'suction_bridge.log').open('w')]
gz=subprocess.Popen(['gz','sim','-s',str(world)],stdout=logs[0],stderr=subprocess.STDOUT,start_new_session=True)
bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/gripper/arm1/command@std_msgs/msg/String]gz.msgs.StringMsg','/gripper/arm1/state@std_msgs/msg/String[gz.msgs.StringMsg','/model/cap_1/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],stdout=logs[1],stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init()
node=Node('suction_physics_test')
states=[]
heights=[]
pub=node.create_publisher(String,'/gripper/arm1/command',10)
sub=node.create_subscription(String,'/gripper/arm1/state',lambda m:states.append(m.data),10)
pose=node.create_subscription(TFMessage,'/model/cap_1/pose',lambda m:heights.extend(t.transform.translation.z for t in m.transforms),10)
def spin(seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
report={}
try:
    spin(4)
    pub.publish(String(data='1,attach,cap_1'))
    spin(1)
    command=subprocess.run(['gz','service','-s','/world/suction_test/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: false'],capture_output=True,text=True,timeout=5)
    spin(2)
    report['attach_response']=states.copy()
    report['held_height_m']=heights[-1] if heights else None
    assert '1,attached:cap_1' in states,states
    assert heights and abs(heights[-1]-.2)<.001,heights[-5:]
    pub.publish(String(data='2,release'))
    spin(1)
    report['release_response']=states.copy()
    report['released_height_m']=heights[-1] if heights else None
    assert '2,released' in states,states
    assert heights[-1]<.15,heights[-5:]
    report['status']='PASS'
except Exception as exc:
    report['status']='FAIL'
    report['error']=repr(exc)
finally:
    node.destroy_node()
    rclpy.shutdown()
    for process in (bridge,gz):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: pass
        try: os.killpg(process.pid,signal.SIGKILL)
        except ProcessLookupError: pass
    for log in logs: log.close()
    (evidence/'suction_physics.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['status']=='PASS' else 1)
