#!/usr/bin/env python3
"""Isolated real ROS/Gazebo frequency and stopping checks for the 30/50 update."""
import os,sys,time,json,signal,socket,subprocess,math
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from ackermann_msgs.msg import AckermannDriveStamped
from rcl_interfaces.srv import GetParameters
ROOT=Path(__file__).resolve().parents[3];OUT=ROOT/'reports/frequency_30fps_50hz';OUT.mkdir(parents=True,exist_ok=True)
PORT=11407;env=os.environ.copy();env.update(ROS_DOMAIN_ID='97',ROS_LOCALHOST_ONLY='1',GAZEBO_MASTER_URI=f'http://127.0.0.1:{PORT}',ROS_LOG_DIR=str(OUT/'ros_logs'))
os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','GAZEBO_MASTER_URI','ROS_LOG_DIR')})
def cleanup(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
class Observer:
 def __init__(self):
  self.n=rclpy.create_node('frequency_acceptance');self.rows={};self.last={};self.clock=0;self.subs=[]
  for topic,typ in [('/odom',Odometry),('/joint_states',JointState),('/camera/front/image_raw',Image),('/camera/front/camera_info',CameraInfo),('/camera/gimbal/image_raw',Image),('/camera/gimbal/camera_info',CameraInfo),('/ackermann_cmd',AckermannDriveStamped),('/tf',TFMessage),('/clock',Clock)]:
   qos=qos_profile_sensor_data if topic=='/clock' else QoSProfile(depth=1 if typ is Image else 200,reliability=ReliabilityPolicy.RELIABLE)
   self.subs.append(self.n.create_subscription(typ,topic,lambda m,t=topic:self.cb(t,m),qos))
 def cb(self,t,m):
  self.last[t]=m;wall=time.monotonic()
  if t=='/clock':self.clock=m.clock.sec+m.clock.nanosec*1e-9;return
  if t=='/tf':
   for v in m.transforms:
    stamp=v.header.stamp.sec+v.header.stamp.nanosec*1e-9;self.rows.setdefault('tf:'+v.child_frame_id,[]).append((wall,stamp))
  else:
   stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;self.rows.setdefault(t,[]).append((wall,stamp))
 def spin(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(self.n,timeout_sec=.01)
 def ready(self):
  end=time.monotonic()+60
  while not all(t in self.last for t in ('/odom','/joint_states','/camera/front/image_raw','/camera/gimbal/image_raw','/camera/front/camera_info')):
   assert time.monotonic()<end,'Missing topics';rclpy.spin_once(self.n,timeout_sec=.02)
 def parameter(self,names):
  c=self.n.create_client(GetParameters,'/robot_state_publisher/get_parameters');assert c.wait_for_service(timeout_sec=8);f=c.call_async(GetParameters.Request(names=names));end=time.monotonic()+8
  while not f.done():assert time.monotonic()<end;rclpy.spin_once(self.n,timeout_sec=.01)
  return f.result().values
 def stats(self,t,start):
  a=[(w,s) for w,s in self.rows.get(t,[]) if w>=start];unique=[]
  for w,s in a:
   if not unique or s>unique[-1][1]:unique.append((w,s))
  assert len(unique)>3,(t,len(unique))
  return {'samples':len(unique),'wall_hz':(len(unique)-1)/(unique[-1][0]-unique[0][0]),'simulation_hz':(len(unique)-1)/(unique[-1][1]-unique[0][1])}
 def drive(self,interrupt=False):
  before=self.last['/odom'].pose.pose.position;start=time.monotonic();f=(OUT/('drive_interrupt.log' if interrupt else 'drive_normal.log')).open('w');p=subprocess.Popen(['ros2','run','remote5g_sim','drive.py','0.10','0.10','--seconds','30' if interrupt else '4'],env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  try:
   end=time.monotonic()+12
   if interrupt:
    while not any(w>start for w,s in self.rows.get('/ackermann_cmd',[])):
     assert time.monotonic()<end;rclpy.spin_once(self.n,timeout_sec=.01)
    self.spin(.8);os.killpg(p.pid,signal.SIGINT)
   while p.poll() is None:assert time.monotonic()<end;rclpy.spin_once(self.n,timeout_sec=.01)
   assert p.returncode==0,p.returncode;self.spin(1)
   speed=self.last['/odom'].twist.twist.linear.x;assert abs(speed)<.02,speed
   assert self.last['/ackermann_cmd'].drive.speed==0
   after=self.last['/odom'].pose.pose.position;displacement=math.hypot(after.x-before.x,after.y-before.y)
   assert displacement>.02,displacement
   rows=[(w,s) for w,s in self.rows['/ackermann_cmd'] if w>start];rate=(len(rows)-1)/(rows[-1][0]-rows[0][0]);assert 43<rate<57,rate
   return {'status':'PASS','command_wall_hz':rate,'messages':len(rows),'stopped_vx_mps':speed,'displacement_m':displacement,'process_exit_code':p.returncode,'interrupted':interrupt}
  finally:cleanup(p);f.close()
with socket.socket() as sock:
 assert sock.connect_ex(('127.0.0.1',PORT))!=0,'Test port occupied'
rclpy.init();results={}
try:
 for scene in ('flat','competition'):
  cmd=['ros2','launch','remote5g_sim','sim.launch.py','scene:=flat','gui:=false',f'port:={PORT}'] if scene=='flat' else ['ros2','launch','remote5g_competition','competition.launch.py','gui:=false',f'port:={PORT}',f'output:={OUT}/competition_scene']
  log=(OUT/f'{scene}_launch.log').open('w');p=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);obs=None
  try:
   obs=Observer();obs.ready();obs.spin(1);start=time.monotonic();obs.spin(8)
   topics=['/odom','/joint_states','/camera/front/image_raw','/camera/front/camera_info','/camera/gimbal/image_raw','/camera/gimbal/camera_info','tf:base_footprint','tf:front_left_knuckle']
   rates={t:obs.stats(t,start) for t in topics};(OUT/f'{scene}_measurements.json').write_text(json.dumps(rates,indent=2));params=obs.parameter(['publish_frequency','robot_description']);assert params[0].double_value==50.0
   assert '<control_period>0.02</control_period>' in params[1].string_value and '<update_rate>30.0</update_rate>' in params[1].string_value
   for t in ('/odom','/joint_states','tf:base_footprint'):assert 44<rates[t]['simulation_hz']<=51,(t,rates[t])
   assert 28<rates['/camera/front/camera_info']['simulation_hz']<31
   assert 14<rates['/camera/gimbal/camera_info']['simulation_hz']<16
   front=obs.last['/camera/front/image_raw'];assert (front.width,front.height)==((640,480) if scene=='flat' else (1280,720))
   results[scene]={'status':'PASS','rates':rates,'robot_state_publisher_max_hz':50.0,'front_resolution':[front.width,front.height],'frame_id':front.header.frame_id,'rates_basis':'wall arrival and unique simulation stamps are reported separately; no hard real-time guarantee'}
   if scene=='flat':results[scene]['normal_stop']=obs.drive();results[scene]['ctrl_c_stop']=obs.drive(True)
   (OUT/'results.json').write_text(json.dumps(results,indent=2));print(scene,json.dumps(results[scene]),flush=True)
  finally:
   if obs:obs.n.destroy_node()
   cleanup(p);log.close()
  time.sleep(1)
finally:rclpy.shutdown()
print('30fps/50Hz checks complete; existing user simulation untouched.',flush=True)
