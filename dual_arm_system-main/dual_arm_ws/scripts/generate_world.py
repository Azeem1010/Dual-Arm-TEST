"""Generate the dimensioned simulation fixture (SI units)."""
from pathlib import Path
from xml.sax.saxutils import escape
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
parts = ['''<?xml version="1.0"?><sdf version="1.9"><world name="dual_arm_workspace">
<physics name="physics" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
<gravity>0 0 -9.81</gravity>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<light name="sun" type="directional"><pose>0 0 5 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><direction>0.1 0.2 -1</direction></light>
<scene><ambient>0.7 0.7 0.7 1</ambient><background>0.18 0.2 0.25 1</background></scene>
''']

def box(name, xyz, size, color, static=True):
    geom = f'<box><size>{size}</size></box>'
    parts.append(f'<model name="{name}"><static>{str(static).lower()}</static><pose>{xyz} 0 0 0</pose><link name="link"><collision name="collision"><geometry>{geom}</geometry></collision><visual name="visual"><geometry>{geom}</geometry><material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual></link></model>')

box('table', '0 0 -0.025', '0.8 0.7 0.05', '0.3 0.3 0.3 1')
# Board C|B|A, x left-to-right, y away from user. White pads fill C flush with foam.
# Static concave CAD platform collision preserves the actual eight cutouts.
# Visuals belong to the articulated robot; avoid coincident duplicate surfaces.
for index in (0,259):
    parts.append(f'''<model name="platform_collision_{index}"><static>true</static><pose>0 0.03 0 1.5707963267948966 0 0</pose><link name="body"><collision name="cad"><geometry><mesh><uri>model://dual_arm_description/meshes/cad/component_{index:03d}.stl</uri><scale>0.001 0.001 0.001</scale></mesh></geometry></collision></link></model>''')
# Separate 600 x 300 mm foam sheet on the CAD platform, 4 mm thick in simulation.
# Openings follow the measured CAD coordinates. This also covers the platform's
# front notches, so the vision specification remains a rectangular workspace.
box('foam_AB','0.1 0 0.012','0.4 0.3 0.004','1 0.15 0.55 1')
xs=[-.3,-.18,-.155,-.145,-.12,-.1]
ys=[-.15,-.0275,-.0025,.0075,.0325,.0425,.0675,.0775,.1025,.15]
for i in range(len(xs)-1):
    for j in range(len(ys)-1):
        if i in (1,3) and j in (1,3,5,7):
            continue
        box(f'foam_C_{i}_{j}',f'{(xs[i]+xs[i+1])/2} {(ys[j]+ys[j+1])/2} .012',f'{xs[i+1]-xs[i]} {ys[j+1]-ys[j]} .004','1 0.15 0.55 1')
for row in range(4):
    for col in range(2):
        box(f'slot_{row*2+col+1}_pad', f'{-0.1675+col*0.035} {-0.015+row*0.035} 0.007', '0.025 0.025 0.014', '1 1 1 1')
for i in range(8):
    parts.append(f'''<model name="cap_{i+1}"><pose>{0.14+(i%2)*0.06} {-0.1+(i//2)*0.065} 0.019 0 0 0</pose><link name="body"><inertial><mass>0.002</mass><inertia><ixx>0.000000067</ixx><iyy>0.000000067</iyy><izz>0.0000001</izz></inertia></inertial><collision name="collision"><geometry><cylinder><radius>0.01</radius><length>0.01</length></cylinder></geometry></collision><visual name="visual"><geometry><cylinder><radius>0.01</radius><length>0.01</length></cylinder></geometry><material><ambient>0 0.1 1 1</ambient><diffuse>0 0.1 1 1</diffuse></material></visual></link></model>''')
bridge = []
for name, x, height, fov in [('env', 0, 0.8, 0.9), ('arm1', 0.2, 0.5, 0.7), ('arm2', -0.2, 0.5, 0.7)]:
    # Gazebo camera +X forward; optical z forward, x right, y down.
    parts.append(f'''<model name="{name}_camera"><static>true</static><pose>{x} 0 {height} 0 1.57079632679 1.57079632679</pose><link name="link"><sensor name="camera" type="camera"><always_on>true</always_on><update_rate>15</update_rate><topic>/{name}_cam/image_raw</topic><gz_frame_id>{name}_camera_optical_frame</gz_frame_id><camera><horizontal_fov>{fov}</horizontal_fov><image><width>640</width><height>480</height><format>R8G8B8</format></image><clip><near>0.01</near><far>5</far></clip><camera_info_topic>/{name}_cam/camera_info</camera_info_topic></camera></sensor></link></model>''')
    for suffix, ros, gz in [('image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'), ('camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')]:
        bridge.append(f'- ros_topic_name: /{name}_cam/{suffix}\n  gz_topic_name: /{name}_cam/{suffix}\n  ros_type_name: {ros}\n  gz_type_name: {gz}\n  direction: GZ_TO_ROS\n')
bridge.append('- ros_topic_name: /clock\n  gz_topic_name: /clock\n  ros_type_name: rosgraph_msgs/msg/Clock\n  gz_type_name: gz.msgs.Clock\n  direction: GZ_TO_ROS\n')
bridge.append('- ros_topic_name: /gazebo/ground_truth\n  gz_topic_name: /world/dual_arm_workspace/pose/info\n  ros_type_name: tf2_msgs/msg/TFMessage\n  gz_type_name: gz.msgs.Pose_V\n  direction: GZ_TO_ROS\n')
for arm in (1,2):
    for suffix,direction in [('command','ROS_TO_GZ'),('state','GZ_TO_ROS')]:
        topic=f'/gripper/arm{arm}/{suffix}'
        bridge.append(f'- ros_topic_name: {topic}\n  gz_topic_name: {topic}\n  ros_type_name: std_msgs/msg/String\n  gz_type_name: gz.msgs.StringMsg\n  direction: {direction}\n')
for name, pose in [('isometric','0.6 -0.6 0.5 0 0.46 2.35619449'),('top','0 0 0.85 0 1.57079632679 1.57079632679'),('front','0 -0.65 0.3 0 0.326 1.57079632679'),('side','0.65 0 0.3 0 0.326 3.14159265359')]:
    parts.append(f'''<model name="view_{name}"><static>true</static><pose>{pose}</pose><link name="link"><sensor name="camera" type="camera"><always_on>true</always_on><update_rate>2</update_rate><topic>/view_{name}/image_raw</topic><camera><horizontal_fov>1.05</horizontal_fov><image><width>960</width><height>720</height><format>R8G8B8</format></image><clip><near>0.01</near><far>5</far></clip></camera></sensor></link></model>''')
    bridge.append(f'- ros_topic_name: /view_{name}/image_raw\n  gz_topic_name: /view_{name}/image_raw\n  ros_type_name: sensor_msgs/msg/Image\n  gz_type_name: gz.msgs.Image\n  direction: GZ_TO_ROS\n')
parts.append('</world></sdf>')
tree=ET.fromstring('\n'.join(parts))
for model in tree.find('world').findall('model'):
    if model.get('name','').startswith('cap_'):
        plugin=ET.SubElement(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
        for key,value in [('publish_model_pose','true'),('publish_link_pose','false'),('use_pose_vector_msg','true'),('update_frequency','30')]:
            ET.SubElement(plugin,key).text=value
        name=model.get('name')
        bridge.append(f'- ros_topic_name: /ground_truth/{name}\n  gz_topic_name: /model/{name}/pose\n  ros_type_name: tf2_msgs/msg/TFMessage\n  gz_type_name: gz.msgs.Pose_V\n  direction: GZ_TO_ROS\n')
ET.ElementTree(tree).write(ROOT / 'src/dual_arm_gazebo/worlds/dual_arm_workspace.sdf',encoding='unicode')
(ROOT / 'src/dual_arm_gazebo/config/bridge.yaml').write_text(''.join(bridge))
print('Generated physical board, eight flush white pads/caps and three fixed simulation cameras')
