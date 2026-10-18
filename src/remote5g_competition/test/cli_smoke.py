#!/usr/bin/env python3
import sys,json,signal,subprocess,os,time,socket
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
sys.path.insert(0,str(Path(get_package_share_directory('remote5g_competition'))/'scripts'))
import rclpy
from gazebo_msgs.srv import GetEntityState
from manage_props import call
out=(Path.home()/'.ros/remote5g/reports/competition/commands_final');out.mkdir(parents=True,exist_ok=True);env=os.environ.copy();env['ROS_DOMAIN_ID']='86'
with socket.socket() as sock:
 sock.settimeout(.5)
 if sock.connect_ex(('127.0.0.1',11386))==0:raise SystemExit('CLI test port 11386 occupied; refusing to touch an existing scene')
cmd=['ros2','launch','remote5g_competition','competition.launch.py','gui:=false','port:=11386','track_color:=blue','start_side:=right',f'output:={out}/layout','vehicle_x:=0','vehicle_y:=-39.525','vehicle_z:=0.006','vehicle_yaw:=0']
log=(out/'gazebo.log').open('w');proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 os.environ['ROS_DOMAIN_ID']='86';rclpy.init();node=rclpy.create_node('competition_cli_verify');get=node.create_client(GetEntityState,'/gazebo/get_entity_state');assert get.wait_for_service(timeout_sec=30)
 end=time.monotonic()+40
 while True:
  response=call(node,get,GetEntityState.Request(name='xtnetrc',reference_frame='world'))
  if response.success:break
  assert time.monotonic()<end;time.sleep(.1)
 start=response.state.pose.position;assert abs(start.x)<.03 and abs(start.y+39.525)<.03
 commands=[['ros2','run','remote5g_competition','manage_props.py','remove','--layout',str(out/'layout/layout.json'),'--name','comp_start_barrier'],['ros2','run','remote5g_competition','manage_props.py','restore','--layout',str(out/'layout/layout.json'),'--name','comp_start_barrier'],['ros2','run','remote5g_competition','manage_props.py','reset','--layout',str(out/'layout/layout.json'),'--name','comp_guide_inner_000'],['ros2','run','remote5g_competition','parking_check.py','--layout',str(out/'layout/layout.json')],['ros2','run','gazebo_ros','spawn_entity.py','-entity','comp_demo_cone','-file',str(Path(get_package_share_directory('remote5g_competition'))/'models/comp_cone_red/model.sdf'),'-x','0','-y','0','-z','0.001','-timeout','15']]
 with (out/'commands.log').open('w') as f:
  for cmd in commands:
   f.write('COMMAND '+' '.join(cmd)+'\n');r=subprocess.Popen(cmd,env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
   try:stdout,stderr=r.communicate(timeout=25)
   finally:
    if r.poll() is None:
     os.killpg(r.pid,signal.SIGINT)
     try:r.wait(timeout=3)
     except subprocess.TimeoutExpired:os.killpg(r.pid,signal.SIGTERM);r.wait(timeout=3)
   f.write(stdout+stderr+'\n');assert r.returncode==0,(cmd,stderr)
 endpose=call(node,get,GetEntityState.Request(name='xtnetrc',reference_frame='world')).state.pose.position;assert abs(endpose.x-start.x)<.01 and abs(endpose.y-start.y)<.01
 assert call(node,get,GetEntityState.Request(name='comp_demo_cone',reference_frame='world')).success
 (out/'result.json').write_text(json.dumps({'status':'PASS','spawn_override_actual_pose':[start.x,start.y,start.z],'management_vehicle_pose_delta':[endpose.x-start.x,endpose.y-start.y],'standalone_cone_spawn':'PASS','commands_executed':len(commands)},indent=2))
 node.destroy_node();rclpy.shutdown();print('Management, parking-check CLI, standalone spawn and manual spawn override PASS')
finally:
 os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=8)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
 log.close()
