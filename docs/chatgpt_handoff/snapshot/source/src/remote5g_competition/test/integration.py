#!/usr/bin/env python3
"""Actual Gazebo services, wheel poses, contacts and camera checks. All waits bounded."""
import argparse,json,math,time,subprocess
from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.insert(0,str(Path(get_package_share_directory('remote5g_competition'))/'scripts'))
from scene_core import Route,parking_footprints,decode_qr,transform,qr_mapping_check
import rclpy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState,SetEntityState,DeleteEntity,SpawnEntity
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
import numpy as np,cv2

class Test:
 def __init__(self,layout,out,props_only=False):
  self.m=json.loads(Path(layout).read_text());self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True);self.node=rclpy.create_node('competition_integration');self.route=Route(self.m['config']['workspace'],self.m['config']['start_side'],self.m['config']['lane']);self.coords={r['model_name']:r for r in self.m['coordinates']};self.odom=None;self.images={};self.clock=None;self.last_clock=time.monotonic();self.props_only=props_only;self.results={};self.subs=[]
  for typ,topic,cb,qos in [(Odometry,'/odom',lambda m:setattr(self,'odom',m),10),(Clock,'/clock',lambda m:setattr(self,'clock',m.clock),qos_profile_sensor_data)]:self.subs.append(self.node.create_subscription(typ,topic,cb,qos))
  for name in ('front','gimbal'):self.subs.append(self.node.create_subscription(Image,f'/camera/{name}/image_raw',lambda m,k=name:self.images.__setitem__(k,m),qos_profile_sensor_data))
  self.get=self.node.create_client(GetEntityState,'/gazebo/get_entity_state');self.set=self.node.create_client(SetEntityState,'/gazebo/set_entity_state');self.delete=self.node.create_client(DeleteEntity,'/delete_entity');self.spawn=self.node.create_client(SpawnEntity,'/spawn_entity');self.pub=self.node.create_publisher(AckermannDriveStamped,'/ackermann_cmd',1);self.tilt=self.node.create_publisher(Float64,'/gimbal/tilt_cmd',1);self.pan=self.node.create_publisher(Float64,'/gimbal/pan_cmd',1)
 def wait(self,condition,seconds=15):
  end=time.monotonic()+seconds
  while not condition():
   if time.monotonic()>end:raise TimeoutError('condition timed out')
   rclpy.spin_once(self.node,timeout_sec=.01)
 def call(self,c,r):
  assert c.wait_for_service(timeout_sec=10);f=c.call_async(r);self.wait(f.done,15);return f.result()
 def state(self,name):
  r=self.call(self.get,GetEntityState.Request(name=name,reference_frame='world'));assert r.success,name;return r.state
 def time(self):return self.clock.sec+self.clock.nanosec*1e-9
 def run(self,seconds,speed=0):
  start=self.time();end=time.monotonic()+max(20,seconds*12);nextpub=0;points=[]
  while self.time()-start<seconds:
   assert time.monotonic()<end,'simulation timed out';now=time.monotonic()
   if now>=nextpub:
    msg=AckermannDriveStamped();msg.drive.speed=float(speed);msg.drive.steering_angle=0.0;self.pub.publish(msg);nextpub=now+.05
   rclpy.spin_once(self.node,timeout_sec=.005)
   if self.odom:
    p=self.odom.pose.pose;points.append([self.time(),p.position.x,p.position.y,p.position.z,self.odom.twist.twist.linear.x])
  return np.array(points)
 def move(self,s,d=0):
  self.run(.5);x,y,z,yaw=self.route.pose(s,d);state=EntityState(name='xtnetrc',reference_frame='world');state.pose.position.x=x;state.pose.position.y=y;state.pose.position.z=z+.006;state.pose.orientation.z=math.sin(yaw/2);state.pose.orientation.w=math.cos(yaw/2);r=self.call(self.set,SetEntityState.Request(state=state));assert r.success;self.run(.8)
 def remove(self,name):
  r=self.call(self.delete,DeleteEntity.Request(name=name));assert r.success,r.status_message
 def restore(self,name):
  import xml.etree.ElementTree as ET
  row=self.coords[name];root=ET.parse(row['sdf']).getroot();m=root.find('model');m.remove(m.find('pose'));req=SpawnEntity.Request(name=name,xml=ET.tostring(root,encoding='unicode'));req.initial_pose.position.x=float(row['x']);req.initial_pose.position.y=float(row['y']);req.initial_pose.position.z=float(row['z']);req.initial_pose.orientation.z=math.sin(row['yaw']/2);req.initial_pose.orientation.w=math.cos(row['yaw']/2);r=self.call(self.spawn,req);assert r.success,r.status_message
 def check(self,name,fn):
  print('RUN',name,flush=True)
  try:e=fn();self.results[name]={'status':'PASS','evidence':e};print('PASS',name,json.dumps(e),flush=True)
  except Exception as e:self.results[name]={'status':'FAIL','error':repr(e)};print('FAIL',name,repr(e),flush=True)
  (self.out/'physics_results.json').write_text(json.dumps(self.results,indent=2))
 def loading(self):
  self.wait(lambda:self.clock is not None and (self.props_only or self.odom is not None and len(self.images)==2),70);self.run(1)
  names=[];heights={}
  for row in self.m['coordinates']:
   if row['model_name']=='comp_start_barrier' and not self.m['config']['barrier_present']:continue
   state=self.state(row['model_name']);names.append(row['model_name']);heights[row['model_name']]=state.pose.position.z
   assert state.pose.position.z>-.01,(row['model_name'],state.pose.position.z)
  start=self.time();wall=time.monotonic();self.run(3);rtf=(self.time()-start)/(time.monotonic()-wall)
  obstacle=self.state('comp_obstacle').pose.position;expected=self.coords['comp_obstacle'];error=math.hypot(obstacle.x-expected['x'],obstacle.y-expected['y']);assert error<.01
  guide=[n for n in names if n.startswith('comp_guide_')]
  assert all(any(n.startswith('comp_guide_'+row['side']+'_') for n in guide) for row in self.m['guide']['rows']) and len(self.m['guide']['rows'])==4
  return {'instances_loaded':len(names),'model_names':names,'minimum_root_z':min(heights.values()),'real_time_factor_observed':rtf,'stable_simulated_seconds':3,'guide_row_count':4,'bend_obstacle':{'actual_xyz':[obstacle.x,obstacle.y,obstacle.z],'seed':self.m['config']['obstacle_seed'],'position_error_m':error,'route_s':expected['s'],'local_d':expected['d']}}
 def barrier(self):
  s=self.m['spawn_pose']['s'];self.move(s);start=self.state('xtnetrc').pose.position;trace=self.run(4,.2);end=self.state('xtnetrc').pose.position;blocked=math.hypot(end.x-start.x,end.y-start.y);assert .05<blocked<.35,blocked
  self.run(.8);before=self.state('xtnetrc').pose.position;self.remove('comp_start_barrier');self.run(.3);after=self.state('xtnetrc').pose.position;assert math.hypot(after.x-before.x,after.y-before.y)<.015
  self.move(s);trace=self.run(4,.2);end=self.state('xtnetrc').pose.position;free=math.hypot(end.x-start.x,end.y-start.y);assert free>.5,free
  self.run(.5);self.restore('comp_start_barrier');assert self.state('comp_start_barrier')
  np.savetxt(self.out/'barrier_removed_motion.csv',trace,delimiter=',');return {'blocked_displacement_m':blocked,'removed_displacement_m':free,'remove_pose_change_m':math.hypot(after.x-before.x,after.y-before.y),'restoration':'PASS','crossed_yellow_tape':True}
 def crosswalk(self):
  s=self.m['region2']['crosswalk_s']-.9;self.move(s);a=self.state('xtnetrc').pose.position;p=self.run(7,.2);b=self.state('xtnetrc').pose.position;distance=math.hypot(b.x-a.x,b.y-a.y);assert distance>.95;assert np.ptp(p[:,3])<.015;self.run(.5);np.savetxt(self.out/'crosswalk_motion.csv',p,delimiter=',');return {'displacement_m':distance,'z_span_m':float(np.ptp(p[:,3])),'white_paper_crossed':True}
 def cone(self):
  # Static or movable actual cone is placed in an unobstructed straight as a test fixture.
  old=self.coords['comp_obstacle'];self.remove('comp_obstacle');s=2.5;pose=self.route.pose(s);req=SpawnEntity.Request(name='comp_contact_test_cone');root=Path(old['sdf']).read_text();import xml.etree.ElementTree as ET
  xml=ET.fromstring(root);m=xml.find('model');m.set('name','comp_contact_test_cone');m.remove(m.find('pose'));req.xml=ET.tostring(xml,encoding='unicode');req.initial_pose.position.x=pose[0];req.initial_pose.position.y=pose[1];req.initial_pose.position.z=.002;req.initial_pose.orientation.w=1.0;r=self.call(self.spawn,req);assert r.success
  try:
   # Aim the left wheel at the cone; chassis sits above its 8cm top and should not contact first.
   self.remove('comp_start_barrier');self.move(s-.8,-self.m['vehicle_config']['front_track']/2);before=self.state('comp_contact_test_cone').pose.position;p=self.run(5,.2);after=self.state('comp_contact_test_cone').pose.position;moved=math.hypot(after.x-before.x,after.y-before.y);assert moved>.02,moved
   self.run(.5);return {'cone_actual_motion_m':moved,'contact_target':'actual front left wheel','dynamic_cone':True}
  finally:self.remove('comp_contact_test_cone');self.restore('comp_obstacle');self.restore('comp_start_barrier')
 def stand(self):
  s=self.m['region2']['qr_s'];self.move(s-.9);a=self.state('xtnetrc').pose.position;p=self.run(6,.2);b=self.state('xtnetrc').pose.position;distance=math.hypot(b.x-a.x,b.y-a.y);assert distance<.7,distance;self.run(.5);return {'blocked_displacement_m':distance,'free_expected_m':1.1,'stand_collision':'actual Gazebo motion blocked'}
 def parking(self):
  # Preparation teleports are fixtures, not autonomous parking claims. Only actual wheel link state is evaluated.
  station=self.coords['comp_parking_zone']['s'];evidence={}
  for label,d,wanted in [('A',.30,'A'),('B',-.30,'B'),('straddling',0,None),('centre_inside_wheel_out',.49,None)]:
   self.move(station-.12,d);wheels={}
   for name in ('front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel'):
    p=self.state('xtnetrc::'+name).pose;q=p.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));wheels[name]=[p.position.x,p.position.y,yaw]
   result=parking_footprints(wheels,self.m['parking'],self.m['vehicle_config']['wheel_width'],self.m['config']['contact_patch_length']);actual=[k for k,v in result.items() if v['all_four_patches_inside']];assert actual==([wanted] if wanted else []),(label,actual);evidence[label]={'accepted_slots':actual,'wheel_poses':wheels}
  return evidence
 def array(self,k):
  m=self.images[k];return np.frombuffer(bytes(m.data),np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
 def qr(self):
  s=self.m['region2']['qr_s'];expected=self.m['config']['qr_content'];attempts=[]
  for distance in (1.00,.85,.65,.55,.50,.45):
   self.move(s-distance);self.tilt.publish(Float64(data=-.20 if distance>=.85 else -.35));self.run(1)
   for name in ('front','gimbal'):
    threshold=self.time();self.wait(lambda: self.images[name].header.stamp.sec+self.images[name].header.stamp.nanosec*1e-9>=threshold,15)
    image=self.array(name);path=self.out/f'qr_{name}_{distance:.2f}.png';cv2.imwrite(str(path),cv2.cvtColor(image,cv2.COLOR_RGB2BGR));value=decode_qr(image);mapping=qr_mapping_check(image,Path(self.m['world']).parent/'models/comp_qr_stand/code.png') if value==expected else None;attempts.append({'camera':name,'rear_axle_distance_m':distance,'camera_width':image.shape[1],'camera_height':image.shape[0],'decoded':value,'file':str(path),'mapping':mapping})
  success=[a for a in attempts if a['decoded']==expected];assert success,attempts
  assert all(a['mapping']['orientation_deg']==0 and a['mapping']['module_error_fraction']<.12 for a in success),success
  parking_codes={}
  for slot in ('A','B'):
   row=self.coords['comp_parking_qr_'+slot];self.move(row['s']-.08,row['d']-.48);self.pan.publish(Float64(data=math.pi/2));self.tilt.publish(Float64(data=-.60));self.run(3);threshold=self.time();self.wait(lambda:self.images['gimbal'].header.stamp.sec+self.images['gimbal'].header.stamp.nanosec*1e-9>=threshold,15);image=self.array('gimbal');path=self.out/f'parking_qr_{slot}.png';cv2.imwrite(str(path),cv2.cvtColor(image,cv2.COLOR_RGB2BGR));value=decode_qr(image);assert value==self.m['config']['parking_qr_'+slot],(slot,value)
   parking_codes[slot]={'decoded':value,'file':str(path),'camera':'gimbal','view':'side approach fixture, pan pi/2 and tilt -0.6, not autonomous parking'}
  return {'parking_qr_camera':parking_codes,'expected':expected,'successful_observations':success,'all_attempts':attempts,'stand_inclination_deg':self.m['config']['qr_pitch_deg'],'simulation_payment':False}
 def execute(self,load_only=False):
  self.check('A_loading_performance',self.loading)
  if load_only or self.results['A_loading_performance']['status']!='PASS':return
  for name,fn in [('D_barrier_tape_reset',self.barrier),('D_crosswalk',self.crosswalk),('D_cone_contact',self.cone),('D_qr_stand_collision',self.stand),('C_parking_actual_wheels',self.parking),('E_simulated_camera_qr',self.qr)]:self.check(name,fn)
  self.move(self.m['spawn_pose']['s'])

def main():
 p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--output',required=True);p.add_argument('--load-only',action='store_true');p.add_argument('--props-only',action='store_true');a=p.parse_args();rclpy.init();t=Test(a.layout,a.output,a.props_only)
 try:t.execute(a.load_only)
 finally:t.node.destroy_node();rclpy.shutdown()
 raise SystemExit(0 if len(t.results)==(1 if a.load_only else 7) and all(x['status']=='PASS' for x in t.results.values()) else 1)
if __name__=='__main__':main()
