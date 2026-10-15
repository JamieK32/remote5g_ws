#!/usr/bin/env python3
from pathlib import Path
import subprocess,json,os
out=Path(__file__).resolve().parent;root=out.parents[2]
commands={
 'system':'cat /etc/os-release; uname -smr; ps -p $$ -o comm=; test -f /.dockerenv && echo docker_marker_present; true',
 'versions':'python3 --version; cmake --version; g++ --version; gazebo --version; colcon version-check --help',
 'ros_tools':'command -v ros2 rviz2 rqt colcon catkin_make docker; ros2 doctor --report',
 'packages':'colcon list; ros2 pkg executables remote5g_sim; ros2 pkg executables remote5g_competition; ros2 pkg prefix remote5g_sim; ros2 pkg prefix remote5g_competition',
 'installed_tools':"dpkg-query -W -f='${Package} ${Version} ${db:Status-Status}\\n' 'ros-humble-rviz2' 'ros-humble-rqt*' 'ros-humble-rosbag2*' 'ros-humble-teleop*' 'python3-opencv' 'gazebo' 'libgazebo-dev'",
 'tool_help':'ros2 bag record --help; ros2 bag play --help; ros2 bag info --help; ros2 topic echo --help; ros2 topic hz --help; ros2 node list --help; ros2 launch remote5g_sim sim.launch.py --show-args; ros2 launch remote5g_competition competition.launch.py --show-args',
 'rqt_plugins':'rqt --list-plugins; ros2 pkg executables teleop_twist_keyboard; ros2 pkg executables rqt_image_view; ros2 pkg executables rqt_reconfigure; rviz2 --help',
 'python_modules':"python3 -c \"import sys,cv2,numpy,yaml,rclpy; print('python',sys.executable); print('cv2',cv2.__version__,cv2.__file__); print('numpy',numpy.__version__); print('yaml',yaml.__version__); print('rclpy',rclpy.__file__)\"",
 'git':'git status --short; git rev-parse --show-toplevel',
 'isolated_dependency_cache':"python3 -c \"import importlib.util;print({x:bool(importlib.util.find_spec(x)) for x in ['qrcode','zxingcpp','fitz']})\"; test -d /home/jamiek/.cache/remote5g_scene_libs && ls /home/jamiek/.cache/remote5g_scene_libs | head -25",
}
env=os.environ.copy();env.update(ROS_DOMAIN_ID='95',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'runtime/ros_logs'),ROS_HOME=str(out/'runtime/ros_home'),QT_QPA_PLATFORM='offscreen')
index={}
for name,cmd in commands.items():
 full='source /opt/ros/humble/setup.bash\nsource '+str(root/'install/setup.bash')+'\n'+cmd
 try:
  r=subprocess.run(['bash','-c',full],cwd=root,env=env,text=True,capture_output=True,timeout=35);code=r.returncode;body=r.stdout+r.stderr
 except subprocess.TimeoutExpired as e:code=124;body=(e.stdout or b'').decode(errors='replace') if isinstance(e.stdout,bytes) else (e.stdout or '');body+='\nTIMEOUT after 35s'
 (out/'evidence'/f'{name}.txt').write_text('COMMAND:\n'+cmd+'\n\nEXIT_CODE: '+str(code)+'\n\nOUTPUT:\n'+body)
 index[name]={'exit_code':code,'file':f'evidence/{name}.txt'};print(name,code,flush=True)
allow=['ROS_DISTRO','ROS_VERSION','ROS_DOMAIN_ID','RMW_IMPLEMENTATION','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH','PYTHONPATH','DISPLAY','WAYLAND_DISPLAY','VIRTUAL_ENV','CONDA_PREFIX','SHELL']
cmd='source /opt/ros/humble/setup.bash\nsource '+str(root/'install/setup.bash')+'\npython3 -c '+__import__('shlex').quote('import os,json;print(json.dumps({k:os.environ.get(k) for k in '+repr(allow)+'},indent=2))')
r=subprocess.run(['bash','-c',cmd],env=env,cwd=root,capture_output=True,text=True);(out/'environment_after_source.json').write_text(r.stdout)
(out/'environment_checks.json').write_text(json.dumps(index,indent=2))
