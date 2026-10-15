#!/usr/bin/env python3
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from gazebo_msgs.srv import SpawnEntity,DeleteEntity,GetEntityState

def call(node,client,request):
 if not client.wait_for_service(timeout_sec=10):raise TimeoutError('Gazebo service unavailable')
 f=client.call_async(request);rclpy.spin_until_future_complete(node,f,timeout_sec=10)
 if not f.done():raise TimeoutError('Gazebo service timeout')
 return f.result()

def main():
 p=argparse.ArgumentParser();p.add_argument('action',choices=['remove','restore','reset']);p.add_argument('--layout',required=True);p.add_argument('--name',default='comp_start_barrier');a=p.parse_args();m=json.loads(Path(a.layout).read_text());rows=m['coordinates'] if a.name=='all' else [r for r in m['coordinates'] if r['model_name']==a.name]
 if not rows:raise SystemExit('Unknown competition model name')
 rclpy.init();n=rclpy.create_node('competition_prop_manager');get=n.create_client(GetEntityState,'/gazebo/get_entity_state');delete=n.create_client(DeleteEntity,'/delete_entity');spawn=n.create_client(SpawnEntity,'/spawn_entity')
 try:
  for row in rows:
   name=row['model_name'];assert name.startswith('comp_');exists=call(n,get,GetEntityState.Request(name=name,reference_frame='world')).success
   if exists and a.action in ('remove','reset'):
    result=call(n,delete,DeleteEntity.Request(name=name));assert result.success;print('Removed',name)
   if a.action=='reset' or a.action=='restore' and not exists:
    root=ET.parse(row['sdf']).getroot();model=root.find('model');pose=model.find('pose')
    if pose is not None:model.remove(pose)
    req=SpawnEntity.Request(name=name,xml=ET.tostring(root,encoding='unicode'));req.initial_pose.position.x=float(row['x']);req.initial_pose.position.y=float(row['y']);req.initial_pose.position.z=float(row['z'])
    import math
    req.initial_pose.orientation.z=math.sin(row['yaw']/2);req.initial_pose.orientation.w=math.cos(row['yaw']/2)
    result=call(n,spawn,req);assert result.success,result.status_message;print('Restored',name)
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
