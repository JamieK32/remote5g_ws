#!/usr/bin/env python3
import os,sys,signal,time,socket,subprocess,json
from pathlib import Path
out=Path(__file__).resolve().parent;root=out.parents[2];env=os.environ.copy();env.update(ROS_DOMAIN_ID='95',ROS_LOCALHOST_ONLY='1',GAZEBO_MASTER_URI='http://127.0.0.1:11405',ROS_LOG_DIR=str(out/'runtime/ros_logs'),ROS_HOME=str(out/'runtime/ros_home'),GAZEBO_LOG_PATH=str(out/'runtime/gazebo_logs'),XDG_CONFIG_HOME=str(out/'runtime/xdg_config'))
index={}
def clean(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=4)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
def call(name,args,timeout=15,extra=None):
 target=out/'evidence'/f'{name}.txt';e=env.copy();e.update(extra or {})
 with target.open('w') as f:
  f.write('COMMAND: '+repr(args)+'\n');f.flush();p=subprocess.Popen(args,cwd=root,env=e,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
  try:code=p.wait(timeout=timeout)
  except subprocess.TimeoutExpired:code=124;clean(p)
  f.write('\nEXIT_CODE: '+str(code)+'\n')
 index[name]={'exit_code':code,'command':args,'file':str(target.relative_to(out))};print(name,code,flush=True);return code
with socket.socket() as s:
 if s.connect_ex(('127.0.0.1',11405))==0:raise SystemExit('Audit port is occupied; refusing to start')
for scene in ('competition','flat'):
 folder=out/'runtime'/scene;folder.mkdir(parents=True,exist_ok=True)
 args=['ros2','launch','remote5g_competition','competition.launch.py','gui:=true','port:=11405',f'output:={folder}/scene',f'capture_dir:={folder}/screenshots'] if scene=='competition' else ['ros2','launch','remote5g_sim','sim.launch.py','gui:=false','scene:=flat','port:=11405']
 log=(folder/'launch.log').open('w');p=subprocess.Popen(args,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  probe=[sys.executable,str(out/'probe_runtime.py'),str(folder)]+(['--motion'] if scene=='flat' else [])
  code=call(scene+'_probe',probe,110);assert code==0,'runtime probe failed: '+scene
  if scene=='competition':
   for name,args in {
    'node_list':['ros2','node','list','--no-daemon'],
    'topic_list':['ros2','topic','list','-t','--no-daemon'],
    'service_list':['ros2','service','list','-t'],
    'action_list':['ros2','action','list','-t'],
    'odom_echo':['ros2','topic','echo','/odom','--once'],
    'clock_echo':['ros2','topic','echo','/clock','--qos-reliability','best_effort','--once'],
    'image_qos':['ros2','topic','info','/camera/front/image_raw','--verbose'],
    'controller_info':['ros2','node','info','/xtnetrc_control'],
    'param_controller':['ros2','param','dump','/xtnetrc_control'],
    'param_rsp':['ros2','param','dump','/robot_state_publisher'],
    'interface_ackermann':['ros2','interface','show','ackermann_msgs/msg/AckermannDriveStamped'],
    'interface_twist':['ros2','interface','show','geometry_msgs/msg/Twist'],
    'gimbal_center':['ros2','service','call','/gimbal/center','std_srvs/srv/Trigger','{}'],
   }.items():call(name,args)
   call('tf_echo',['ros2','run','tf2_ros','tf2_echo','odom','front_camera_optical_frame'],5)
   call('odom_hz',['ros2','topic','hz','/odom'],6)
   bag=folder/'sample_bag';recordlog=(folder/'bag_record.log').open('w');b=subprocess.Popen(['ros2','bag','record','-o',str(bag),'--use-sim-time','/odom','/joint_states','/tf','/tf_static','/clock'],cwd=root,env=env,stdout=recordlog,stderr=subprocess.STDOUT,start_new_session=True);time.sleep(6);clean(b);recordlog.close();call('bag_info',['ros2','bag','info',str(bag)])
   # Startup checks only: do not claim that a human configured all RViz displays.
   for name,args in [('rviz_start',['rviz2','-f','odom','--ros-args','-p','use_sim_time:=true']),('rqt_image_start',['ros2','run','rqt_image_view','rqt_image_view','/camera/front/image_raw'])]:
    call(name,args,6)
 finally:clean(p);log.close()
 time.sleep(1)
# Playback is isolated from every live controller and publishes its own clock.
bag=out/'runtime/competition/sample_bag';call('bag_play',['ros2','bag','play',str(bag),'--topics','/odom','/joint_states','/tf','/tf_static','--clock','20'],20,{'ROS_DOMAIN_ID':'96'})
call('daemon_stop',['ros2','daemon','stop'])
(out/'runtime_checks.json').write_text(json.dumps(index,indent=2));print('Runtime checks complete. All owned launch groups stopped.',flush=True)
