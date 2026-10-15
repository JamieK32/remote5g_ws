#!/usr/bin/env python3
"""Bounded observation and tiny isolated simulator-only control check; not a race algorithm."""
import time,json,sys,math,subprocess,os,pty,signal
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,CameraInfo,JointState
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from gazebo_msgs.srv import GetEntityState
from rosidl_runtime_py.convert import message_to_ordereddict
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True);motion='--motion' in sys.argv
rclpy.init();node=rclpy.create_node('handoff_observer');last={};times={};transforms={};subscriptions=[]
def cb(topic,msg):
 last[topic]=msg;times.setdefault(topic,[]).append(time.monotonic())
 if isinstance(msg,TFMessage):
  for t in msg.transforms:transforms[t.child_frame_id]=message_to_ordereddict(t)
for topic,typ in [('/odom',Odometry),('/joint_states',JointState),('/clock',Clock),('/diagnostics',DiagnosticArray),('/camera/front/image_raw',Image),('/camera/gimbal/image_raw',Image),('/camera/front/camera_info',CameraInfo),('/camera/gimbal/camera_info',CameraInfo),('/tf',TFMessage),('/tf_static',TFMessage)]:
 qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic=='/tf_static' else qos_profile_sensor_data
 subscriptions.append(node.create_subscription(typ,topic,lambda m,t=topic:cb(t,m),qos))
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
end=time.monotonic()+65
while not all(t in last for t in ['/odom','/camera/front/image_raw','/camera/gimbal/image_raw','/clock']):
 if time.monotonic()>end:raise RuntimeError('Timed out waiting for simulator topics')
 rclpy.spin_once(node,timeout_sec=.05)
spin(6)
graph={}
for name,ns in node.get_node_names_and_namespaces():
 if name=='handoff_observer':continue
 row={}
 for key,fn in [('publishers',node.get_publisher_names_and_types_by_node),('subscriptions',node.get_subscriber_names_and_types_by_node),('services',node.get_service_names_and_types_by_node),('clients',node.get_client_names_and_types_by_node)]:
  try:row[key]=fn(name,ns)
  except Exception as e:row[key]={'error':str(e)}
 graph[(ns.rstrip('/')+'/'+name)]=row
endpoints={}
for topic,types in node.get_topic_names_and_types():
 endpoints[topic]={'types':types,'publishers':[{'node':p.node_namespace.rstrip('/')+'/'+p.node_name,'reliability':p.qos_profile.reliability.name,'durability':p.qos_profile.durability.name,'depth':p.qos_profile.depth} for p in node.get_publishers_info_by_topic(topic)]}
samples={}
for topic,msg in last.items():
 if isinstance(msg,Image):
  samples[topic]={'header':message_to_ordereddict(msg.header),'width':msg.width,'height':msg.height,'encoding':msg.encoding,'step':msg.step,'data_bytes':len(msg.data)}
  import numpy as np,cv2
  image=np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
  cv2.imwrite(str(out/(topic.split('/')[2]+'.png')),cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
 else:samples[topic]=message_to_ordereddict(msg)
frequencies={t:{'messages':len(v),'wall_hz':(len(v)-1)/(v[-1]-v[0]) if len(v)>1 else None,'wall_duration_s':v[-1]-v[0] if len(v)>1 else 0} for t,v in times.items()}
get=node.create_client(GetEntityState,'/gazebo/get_entity_state');state=None
if get.wait_for_service(timeout_sec=5):
 f=get.call_async(GetEntityState.Request(name='xtnetrc',reference_frame='world'));end=time.monotonic()+8
 while not f.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
 if f.done():state=message_to_ordereddict(f.result())
results={'status':'PASS','graph':graph,'topics':endpoints,'services':node.get_service_names_and_types(),'samples':samples,'frequencies':frequencies,'tf':transforms,'actual_vehicle_state':state,'domain':os.environ.get('ROS_DOMAIN_ID')}
if motion:
 def pose():
  p=last['/odom'].pose.pose;return [p.position.x,p.position.y,p.position.z]
 before=pose();cmd=['ros2','run','remote5g_sim','drive.py','0.10','0.15','--seconds','1'];p=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
 while p.poll() is None:spin(.05)
 assert p.returncode==0;spin(.8);after=pose();distance=math.dist(before[:2],after[:2]);assert .02<distance<.25,distance
 results['ackermann_motion']={'command':cmd,'before':before,'after':after,'distance_m':distance,'stopped_vx':last['/odom'].twist.twist.linear.x}
 # Drive the already installed keyboard executable through a real PTY.
 master,slave=pty.openpty();cmd=['ros2','run','teleop_twist_keyboard','teleop_twist_keyboard','--ros-args','-p','speed:=0.10','-p','turn:=0.05'];p=subprocess.Popen(cmd,stdin=slave,stdout=slave,stderr=slave,start_new_session=True);os.close(slave)
 try:
  spin(2);before=pose()
  for _ in range(12):os.write(master,b'i');spin(.10)
  os.write(master,b'k');spin(.8);after=pose();os.write(master,b'\x03');spin(.4)
  distance=math.dist(before[:2],after[:2]);assert .02<distance<.25,distance
  results['keyboard_motion']={'command':cmd,'keys':'12 x i at 10Hz, k, Ctrl+C','before':before,'after':after,'distance_m':distance,'stopped_vx':last['/odom'].twist.twist.linear.x}
 finally:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=4)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=3)
  os.set_blocking(master,False)
  try:log=os.read(master,100000).decode(errors='replace')
  except OSError:log='PTY closed'
  (out/'keyboard_pty.txt').write_text(log);os.close(master)
(out/'runtime.json').write_text(json.dumps(results,indent=2,ensure_ascii=False));print('PASS',list(graph),results.get('ackermann_motion',{}).get('distance_m'),results.get('keyboard_motion',{}).get('distance_m'),flush=True)
node.destroy_node();rclpy.shutdown()
