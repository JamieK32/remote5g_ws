import os
import sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory,get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,SetEnvironmentVariable,ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def setup(context):
 package=Path(get_package_share_directory('remote5g_competition'));sys.path.insert(0,str(package/'scripts'))
 from scene_core import load_config,generate
 def get(k):return LaunchConfiguration(k).perform(context)
 cfg=load_config(get('config'),{'workspace':get('workspace')} if get('workspace') else None)
 for k in ('start_side','track_color','lane_change','parking_target','obstacle_mode'):
  if get(k):cfg[k]=get(k)
 for k in ('obstacle_seed','sign_seed'):
  if get(k):cfg[k]=int(get(k))
 cfg.setdefault('vehicle_spawn',{})
 for k in ('x','y','z','yaw'):
  if get('vehicle_'+k):cfg['vehicle_spawn'][k]=float(get('vehicle_'+k))
 cfg['barrier_present']=get('barrier_present').lower()=='true';cfg['cone_static']=get('cone_static').lower()=='true'
 m=generate(cfg,package,get('output'))
 from validate_layout import validate
 validate(Path(get('output'))/'layout.json')
 root=Path(get('output')).resolve();vehicle_share=Path(get_package_share_directory('remote5g_sim'))
 env=[SetEnvironmentVariable('GAZEBO_MASTER_URI','http://127.0.0.1:'+get('port')),SetEnvironmentVariable('GAZEBO_IP','127.0.0.1'),SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI',''),SetEnvironmentVariable('GAZEBO_MODEL_PATH',str(root/'models')+':'+str(Path(cfg['workspace'])/'simulation/models')+':'+str(package/'models')+':'+os.environ.get('GAZEBO_MODEL_PATH','')),SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',get_package_prefix('remote5g_sim')+'/lib:'+get_package_prefix('remote5g_competition')+'/lib:/opt/ros/humble/lib:'+os.environ.get('GAZEBO_PLUGIN_PATH',''))]
 nodes=[ExecuteProcess(cmd=['gzserver','--verbose',m['world'],'-s','libgazebo_ros_init.so','-s','libgazebo_ros_factory.so'],output='screen')]
 if get('spawn_vehicle').lower()=='true':
  robot=xacro.process_file(str(vehicle_share/'urdf/car.xacro'),mappings={'front_width':get('camera_width'),'front_height':get('camera_height'),'gimbal_width':get('camera_width'),'gimbal_height':get('camera_height')}).toxml()
  pose=m['spawn_pose'];nodes.extend([Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':robot,'use_sim_time':True,'publish_frequency':50.0}],output='screen'),Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-entity','xtnetrc','-topic','robot_description','-x',str(pose['x']),'-y',str(pose['y']),'-z',str(pose['z']),'-Y',str(pose['yaw']),'-timeout','60'],output='screen')])
 if get('gui').lower()=='true':
  client=['gzclient']
  if get('capture_dir'):
   dest=Path(get('capture_dir')).resolve();dest.mkdir(parents=True,exist_ok=True);env.extend([SetEnvironmentVariable('COMPETITION_CAPTURE_DIR',str(dest)),SetEnvironmentVariable('COMPETITION_LAYOUT',str(root/'layout.json'))]);client.extend(['--gui-client-plugin',get_package_prefix('remote5g_competition')+'/lib/libcompetition_capture.so'])
  nodes.append(ExecuteProcess(cmd=client,output='screen'))
 return env+nodes

def generate_launch_description():
 package=Path(get_package_share_directory('remote5g_competition'))
 args={'config':str(package/'config/competition.yaml'),'workspace':'','output':str(Path.home()/'.ros/remote5g/reports/competition/current'),'start_side':'','track_color':'','lane_change':'','parking_target':'','obstacle_mode':'','obstacle_seed':'','sign_seed':'','barrier_present':'true','cone_static':'false','spawn_vehicle':'true','gui':'true','port':'11385','camera_width':'1280','camera_height':'720','capture_dir':'','vehicle_x':'','vehicle_y':'','vehicle_z':'','vehicle_yaw':''}
 return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in args.items()]+[OpaqueFunction(function=setup)])
