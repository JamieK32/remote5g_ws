#!/usr/bin/env python3
import argparse,json,math
from pathlib import Path
import rclpy
from gazebo_msgs.srv import GetEntityState
from scene_core import parking_footprints
from manage_props import call

def main():
 p=argparse.ArgumentParser();p.add_argument('--layout',required=True);p.add_argument('--vehicle',default='xtnetrc');a=p.parse_args();m=json.loads(Path(a.layout).read_text());rclpy.init();n=rclpy.create_node('competition_parking_check');client=n.create_client(GetEntityState,'/gazebo/get_entity_state');wheels={}
 try:
  for name in ('front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel'):
   r=call(n,client,GetEntityState.Request(name=a.vehicle+'::'+name,reference_frame='world'));assert r.success,name;p=r.state.pose;q=p.orientation
   wheels[name]=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
  result=parking_footprints(wheels,m['parking'],m['vehicle_config']['wheel_width'],m['config']['contact_patch_length']);print(json.dumps(result,indent=2))
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
