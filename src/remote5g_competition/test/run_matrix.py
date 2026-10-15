#!/usr/bin/env python3
"""Owned process groups, unique ROS/Gazebo instance, bounded execution and real results."""
import argparse,json,os,signal,socket,subprocess,time,sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
sys.path.insert(0,str(Path(get_package_share_directory('remote5g_competition'))/'scripts'))
from scene_core import generate,load_config
from validate_layout import validate

def cleanup(proc):
 if proc:
  try:os.killpg(proc.pid,signal.SIGINT)
  except ProcessLookupError:return
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:
   os.killpg(proc.pid,signal.SIGTERM)
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',default='/home/jamiek/remote5g_ws/reports/competition/final');p.add_argument('--gui',action='store_true');p.add_argument('--single',choices=['left_red','right_blue']);a=p.parse_args();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=True)
 package=Path(get_package_share_directory('remote5g_competition'));base=load_config(package/'config/competition.yaml');env=os.environ.copy();env['ROS_DOMAIN_ID']=env.get('COMPETITION_TEST_DOMAIN','85');port=int(env.get('COMPETITION_TEST_PORT','11385'));env['GAZEBO_MASTER_URI']=f'http://127.0.0.1:{port}';summary={}
 with socket.socket() as sock:
  sock.settimeout(.5)
  if sock.connect_ex(('127.0.0.1',port))==0:raise SystemExit('Test Gazebo port occupied; choose COMPETITION_TEST_PORT')
 cases=[('left_red','left','red','left',True),('left_blue','left','blue','right',False),('right_red','right','red','left',False),('right_blue','right','blue','right',True),('props_only','right','blue','left',False)]
 if a.single:cases=[c for c in cases if c[0]==a.single]
 try:
  for name,side,color,change,full in cases:
   folder=out/name;folder.mkdir(parents=True,exist_ok=True);cfg=base.copy();cfg.update(start_side=side,track_color=color,lane_change=change)
   if name=='props_only':cfg.update(cone_static=True,barrier_present=False)
   layout=generate(cfg,package,folder);geometry=validate(folder/'layout.json');(folder/'geometry.json').write_text(json.dumps(geometry,indent=2))
   # Assert deterministic seeded output and alternative seed reproducibly changes only the obstacle.
   again=generate(cfg,package,folder);assert again['obstacle']==layout['obstacle'] and again['coordinates']==layout['coordinates']
   (folder/'input.yaml').write_text(__import__('yaml').safe_dump(cfg))
   cmd=['ros2','launch','remote5g_competition','competition.launch.py',f'config:={folder}/input.yaml',f'output:={folder}',f'port:={port}','gui:=true' if a.gui and name==cases[0][0] else 'gui:=false',f'cone_static:={str(cfg["cone_static"]).lower()}',f'barrier_present:={str(cfg["barrier_present"]).lower()}',f'spawn_vehicle:={str(name!="props_only").lower()}']
   if a.gui and name==cases[0][0]:cmd.append(f'capture_dir:={folder}/screenshots')
   print('START',name,flush=True);started=time.monotonic();proc=None
   try:
    with (folder/'gazebo.log').open('w') as log:
     proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
     argv=['ros2','run','remote5g_competition','integration.py','--layout',str(folder/'layout.json'),'--output',str(folder)]
     if not full:argv.append('--load-only')
     if name=='props_only':argv.append('--props-only')
     with (folder/'integration.log').open('w') as logs:
      result=subprocess.Popen(argv,env=env,stdout=logs,stderr=subprocess.STDOUT,start_new_session=True)
      try:result.wait(timeout=270)
      finally:cleanup(result)
     runtime=time.monotonic()-started;results=json.loads((folder/'physics_results.json').read_text());summary[name]={'status':'PASS' if result.returncode==0 else 'FAIL','process_exit_code':result.returncode,'elapsed_wall_seconds':runtime,'results':results,'geometry':geometry}
     if a.gui and name==cases[0][0]:
      end=time.monotonic()+35
      while len(list((folder/'screenshots').glob('*.png')))<7 and time.monotonic()<end:time.sleep(.2)
      summary[name]['gui_screenshot_count']=len(list((folder/'screenshots').glob('*.png')))
     print(name,summary[name]['status'],f'{runtime:.2f}s',flush=True)
   finally:cleanup(proc)
   (out/'matrix_results.json').write_text(json.dumps(summary,indent=2))
 finally:(out/'matrix_results.json').write_text(json.dumps(summary,indent=2))
 raise SystemExit(0 if summary and all(v['status']=='PASS' for v in summary.values()) else 1)
if __name__=='__main__':main()
