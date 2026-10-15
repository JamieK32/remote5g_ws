#!/usr/bin/env python3
"""Isolated additional checks for track spawning, configurable images and documented CLI."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import rclpy
from regression import Regression


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--scene',choices=['flat','red','blue'],default='flat')
    parser.add_argument('--output',required=True); parser.add_argument('--gui',action='store_true'); args=parser.parse_args()
    out=Path(args.output).resolve(); out.mkdir(parents=True,exist_ok=True)
    port=int(os.environ.get('XTNETRC_TEST_PORT','11376'))
    with socket.socket() as s:
        s.settimeout(1)
        if s.connect_ex(('127.0.0.1',port))==0: raise SystemExit('Port occupied')
    os.environ['ROS_DOMAIN_ID']=os.environ.get('XTNETRC_TEST_DOMAIN','76')
    os.environ['GAZEBO_MASTER_URI']=f'http://127.0.0.1:{port}'; os.environ['GAZEBO_IP']='127.0.0.1'
    os.environ['GAZEBO_MODEL_DATABASE_URI']=''
    log=(out/'gazebo.log').open('w'); gui=None; test=None
    cmd=['ros2','launch','remote5g_sim','sim.launch.py',f'gui:={str(args.gui).lower()}',f'port:={port}',f'scene:={args.scene}','front_width:=1280','front_height:=720','gimbal_width:=1280','gimbal_height:=720']
    if args.gui: cmd.append(f'capture_dir:={out}')
    print('LAUNCH', ' '.join(cmd), flush=True)
    proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def invoke(cmd):
        print('COMMAND', ' '.join(cmd),flush=True)
        result=subprocess.run(cmd,timeout=20,capture_output=True,text=True)
        print(result.stdout,result.stderr,flush=True); assert result.returncode==0
    def cleanup(p):
        if p:
            try: os.killpg(p.pid,signal.SIGINT)
            except ProcessLookupError: return
            try: p.wait(timeout=7)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid,signal.SIGTERM); p.wait(timeout=5)
    try:
        rclpy.init(); test=Regression(out)
        def ready():
            if proc.poll() is not None: raise RuntimeError('Gazebo launch exited; inspect gazebo.log')
            return test.o is not None and len(test.images)==2 and len(test.info)==2
        test.wait(ready,70)
        test.run(2)
        x0=test.snapshot()
        for name,m in test.images.items():
            c=test.info[name]; assert (m.width,m.height)==(1280,720) and c.width==1280 and c.k[0]>0
        if args.scene=='flat':
            for speed,steer in [('0.2','0'),('-0.2','0'),('0.2','0.25'),('0.2','-0.25'),('0','0')]:
                invoke(['ros2','run','remote5g_sim','drive.py',speed,steer,'--seconds','1'])
                test.run(0.5)
            invoke(['ros2','topic','pub','--once','/gimbal/pan_cmd','std_msgs/msg/Float64','{data: 0.6}'])
            invoke(['ros2','topic','pub','--once','/gimbal/tilt_cmd','std_msgs/msg/Float64','{data: 0.3}'])
            test.run(1)
            invoke(['ros2','service','call','/gimbal/center','std_srvs/srv/Trigger','{}'])
            invoke(['ros2','run','remote5g_sim','inspect_cameras.py','--output',str(out)])
            test.run(3)
        else:
            trace=test.run(3,0.2,0); test.traces['track_forward']=trace; test.save()
            assert test.o.pose.pose.position.x-x0[1]>0.3
            assert abs(test.o.pose.pose.position.y+37.2)<0.1 and abs(test.o.pose.pose.position.z)<0.02
            invoke(['ros2','run','remote5g_sim','inspect_cameras.py','--output',str(out)])
            test.settled()
        evidence={'status':'PASS','scene':args.scene,'resolution':[1280,720],'pose_start':x0,'pose_end':test.snapshot(),'joint_angles':test.joints(),'images':test.count,'gui_viewport_files':[str(p) for p in out.glob('gui_view_*.png')]}
        (out/'smoke_result.json').write_text(json.dumps(evidence,indent=2)); print(json.dumps(evidence),flush=True)
    finally:
        if test:
            test.command(0,0); test.n.destroy_node(); rclpy.shutdown()
        cleanup(gui); cleanup(proc); log.close()

if __name__=='__main__': main()
