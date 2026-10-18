#!/usr/bin/env python3
"""Bounded regression using actual Gazebo dynamics and sensor messages.
The outer runner owns and cleans up its process group; this node never kills processes.
"""
import argparse
import json
import math
from pathlib import Path
import time
import traceback
import numpy as np
import cv2
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def wrap(v): return math.atan2(math.sin(v),math.cos(v))

def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9

class Regression:
    def __init__(self,out):
        self.out=out; out.mkdir(parents=True,exist_ok=True)
        self.n=rclpy.create_node('xtnetrc_regression'); self.subs=[]
        self.o=None; self.j=None; self.diag=None; self.models=None; self.tf={}; self.static={}
        self.images={}; self.info={}; self.count={'front':0,'gimbal':0}; self.traces={}; self.results={}
        for typ,topic,attr in [(Odometry,'/odom','o'),(JointState,'/joint_states','j'),(DiagnosticArray,'/diagnostics','diag'),(ModelStates,'/gazebo/model_states','models')]:
            self.subs.append(self.n.create_subscription(typ,topic,lambda m,a=attr:setattr(self,a,m),10))
        self.subs.append(self.n.create_subscription(TFMessage,'/tf',lambda m:self.tf.update({x.child_frame_id:x for x in m.transforms}),100))
        self.subs.append(self.n.create_subscription(TFMessage,'/tf_static',lambda m:self.static.update({x.child_frame_id:x for x in m.transforms}),QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for name in ('front','gimbal'):
            self.subs.append(self.n.create_subscription(Image,f'/camera/{name}/image_raw',lambda m,k=name:self.image(k,m),qos_profile_sensor_data))
            self.subs.append(self.n.create_subscription(CameraInfo,f'/camera/{name}/camera_info',lambda m,k=name:self.info.__setitem__(k,m),qos_profile_sensor_data))
        self.ack=self.n.create_publisher(AckermannDriveStamped,'/ackermann_cmd',1)
        self.tw=self.n.create_publisher(Twist,'/cmd_vel',1)
        self.pan=self.n.create_publisher(Float64,'/gimbal/pan_cmd',1)
        self.tilt=self.n.create_publisher(Float64,'/gimbal/tilt_cmd',1)
        self.get=self.n.create_client(GetEntityState,'/gazebo/get_entity_state')
        self.spawn=self.n.create_client(SpawnEntity,'/spawn_entity')
        self.delete=self.n.create_client(DeleteEntity,'/delete_entity')
        self.center=self.n.create_client(Trigger,'/gimbal/center')
    def image(self,k,m): self.images[k]=m; self.count[k]+=1
    def wait(self,fn,seconds=30):
        end=time.monotonic()+seconds
        while not fn():
            if time.monotonic()>end: raise TimeoutError('condition timed out')
            rclpy.spin_once(self.n,timeout_sec=0.01)
    def call(self,client,req):
        if not client.wait_for_service(timeout_sec=5): raise TimeoutError('service missing')
        future=client.call_async(req); self.wait(future.done,10); return future.result()
    def snapshot(self):
        p=self.o.pose.pose
        return [stamp(self.o),p.position.x,p.position.y,p.position.z,yaw(p.orientation),self.o.twist.twist.linear.x,self.o.twist.twist.angular.z]
    def joints(self): return dict(zip(self.j.name,self.j.position))
    def command(self,v,a):
        m=AckermannDriveStamped(); m.drive.speed=float(v); m.drive.steering_angle=float(a); self.ack.publish(m)
    def run(self,duration,v=0,a=0,publish=True,twist=None):
        start=stamp(self.o); deadline=time.monotonic()+max(30,duration*10); next_pub=0; points=[]; last=-1
        while stamp(self.o)-start<duration:
            if time.monotonic()>deadline: raise TimeoutError('simulation stopped or too slow')
            now=time.monotonic()
            if publish and now>=next_pub:
                if twist is None: self.command(v,a)
                else:
                    m=Twist(); m.linear.x=float(twist[0]); m.angular.z=float(twist[1]); self.tw.publish(m)
                next_pub=now+0.05
            rclpy.spin_once(self.n,timeout_sec=0.005)
            if stamp(self.o)!=last: points.append(self.snapshot()); last=stamp(self.o)
        return np.array(points)
    def check(self,key,fn):
        print('RUN',key,flush=True)
        try:
            evidence=fn(); self.results[key]={'status':'PASS','evidence':evidence}
            print('PASS',key,json.dumps(evidence,ensure_ascii=False),flush=True)
        except Exception as e:
            self.results[key]={'status':'FAIL','error':str(e),'traceback':traceback.format_exc()}
            print('FAIL',key,str(e),flush=True)
            if self.o: self.command(0,0)
        self.save()
    def save(self):
        (self.out/'results.json').write_text(json.dumps(self.results,indent=2,ensure_ascii=False))
        for key,points in self.traces.items(): np.savetxt(self.out/f'{key}.csv',points,delimiter=',',header='sim_time,x,y,z,yaw,vx,yaw_rate',comments='')
    def settled(self): return self.run(1,0,0)
    def startup(self):
        self.wait(lambda:self.o is not None and self.j is not None and len(self.images)==2 and len(self.info)==2,60)
        self.run(2)
        req=GetEntityState.Request(); req.name='xtnetrc'; req.reference_frame='world'
        state=self.call(self.get,req); assert state.success
        j=self.joints(); assert len(j)==8
        assert abs(self.o.pose.pose.position.z)<0.02
        assert abs(yaw(self.o.pose.pose.orientation))<0.05
        # Check real base_link and each wheel link through Gazebo state service.
        heights={}
        for name in ('base_link','front_left_wheel','front_right_wheel','rear_left_wheel','rear_right_wheel'):
            req.name='xtnetrc::'+name; r=self.call(self.get,req); assert r.success,name
            heights[name]=r.state.pose.position.z
        for k,v in heights.items(): assert 0.045<v<0.065,(k,v)
        # Compare Gazebo model root and plugin odom, using nearly simultaneous samples.
        req.name='xtnetrc'; r=self.call(self.get,req)
        error=math.hypot(r.state.pose.position.x-self.o.pose.pose.position.x,r.state.pose.position.y-self.o.pose.pose.position.y)
        assert error<0.015,error
        self.wait(lambda:'base_footprint' in self.tf and 'front_camera_optical_frame' in self.static and 'gimbal_camera_optical_frame' in self.static,5)
        return {'actual_link_heights_m':heights,'gazebo_odom_position_error_m':error,'joint_count':len(j),'tf_static':sorted(self.static),'tf':sorted(self.tf)}
    def rest(self):
        p=self.run(3); self.traces['rest']=p
        drift=np.linalg.norm(p[-1,1:3]-p[0,1:3]); zspan=np.ptp(p[:,3]); assert drift<0.02 and zspan<0.01
        return {'drift_m':float(drift),'z_span_m':float(zspan),'duration_s':float(p[-1,0]-p[0,0])}
    def straight(self,reverse=False):
        self.settled(); start=self.snapshot(); points=self.run(4,-0.25 if reverse else 0.25,0); key='reverse' if reverse else 'forward'; self.traces[key]=points
        delta=points[-1,1:3]-np.array(start[1:3]); longitudinal=delta@[math.cos(start[4]),math.sin(start[4])]
        assert longitudinal< -0.6 if reverse else longitudinal>0.6
        j=self.joints(); assert abs(j['front_left_steering_joint'])<0.02 and abs(j['front_right_steering_joint'])<0.02
        assert all(x< -1 if reverse else x>1 for name,x in zip(self.j.name,self.j.velocity) if name.endswith('wheel_joint'))
        final=self.run(1.5); assert abs(self.o.twist.twist.linear.x)<0.03
        return {'displacement_local_x_m':float(longitudinal),'duration_sim_s':float(points[-1,0]-start[0]),'mean_vx_mps':float(np.mean(points[-40:,5])),'stop_vx_mps':self.o.twist.twist.linear.x}
    def turn(self,sign):
        self.settled(); a=sign*0.25; points=self.run(8,0.25,a); key='left' if sign>0 else 'right'; self.traces[key]=points
        j=self.joints(); l=j['front_left_steering_joint']; r=j['front_right_steering_joint']
        t=math.tan(a); theoretical=[math.atan2(t,1-0.24*t/(2*0.23861)),math.atan2(t,1+0.24*t/(2*0.23861))]
        assert l*sign>0 and r*sign>0
        assert abs(l)>abs(r) if sign>0 else abs(r)>abs(l)
        assert max(abs(l-theoretical[0]),abs(r-theoretical[1]))<0.035
        q=points[points[:,0]>points[0,0]+1.5]; x=q[:,1]; y=q[:,2]
        # Circle fit to the measured rear axle trajectory after acceleration/steering ramps.
        coeff=np.linalg.lstsq(np.c_[2*x,2*y,np.ones(len(x))],x*x+y*y,rcond=None)[0]
        radius=math.sqrt(coeff[2]+coeff[0]**2+coeff[1]**2)
        theory=0.23861/abs(t); error=abs(radius-theory)/theory
        dyaw=wrap(q[-1,4]-q[0,4]); assert dyaw*sign>0.2
        assert error<0.20,(radius,theory,error)
        rms=np.sqrt(np.mean((np.hypot(x-coeff[0],y-coeff[1])-radius)**2)); assert rms<0.03
        torques=dict(zip(self.j.name,self.j.effort)); assert abs(torques['rear_left_wheel_joint']-torques['rear_right_wheel_joint'])<1e-10
        self.settled()
        return {'steering_actual_rad':[l,r],'steering_theory_rad':theoretical,'theoretical_curvature_per_m':sign/theory,'theoretical_radius_m':theory,'actual_radius_m':radius,'relative_radius_error':error,'circle_fit_rms_m':float(rms),'yaw_change_rad':dyaw,'identical_rear_torque':True}
    def spin_reject(self):
        self.settled(); start=self.snapshot(); p=self.run(2,twist=(0,1)); self.traces['spin_reject']=p
        drift=np.linalg.norm(p[-1,1:3]-np.array(start[1:3])); dyaw=wrap(p[-1,4]-start[4]); assert drift<0.02 and abs(dyaw)<0.03
        assert self.diag.status[0].message=='unrealizable_twist'
        return {'displacement_m':float(drift),'yaw_change_rad':dyaw,'diagnostic':self.diag.status[0].message}
    def cameras(self):
        before=self.count.copy(); self.run(2); evidence={}
        for k,m in self.images.items():
            assert self.count[k]-before[k]>5
            c=self.info[k]; assert (m.width,m.height)==(640,480) and c.width==m.width and c.height==m.height
            assert m.header.frame_id==c.header.frame_id==f'{k}_camera_optical_frame'
            assert c.k[0]>0 and c.k[4]>0 and len(c.d)==5 and all(x==0 for x in c.d)
            im=self.array(k); assert im.std()>2; cv2.imwrite(str(self.out/f'{k}.png'),cv2.cvtColor(im,cv2.COLOR_RGB2BGR))
            evidence[k]={'width':m.width,'height':m.height,'encoding':m.encoding,'frames_in_2_sim_seconds':self.count[k]-before[k],'frame':m.header.frame_id,'K':list(c.k),'pixel_std':float(im.std())}
        assert self.info['front'].k[0]!=self.info['gimbal'].k[0]
        return evidence
    def array(self,k):
        m=self.images[k]; return np.frombuffer(bytes(m.data),np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
    def gimbal(self):
        self.settled(); self.call(self.center,Trigger.Request()); self.run(1)
        baseline=self.array('gimbal'); fixed=self.array('front'); evidence={}
        for pan,tilt,key in [(0.6,0,'pan_left'),(-0.6,0,'pan_right'),(0,0.45,'tilt_up'),(0,-0.4,'tilt_down'),(8,8,'upper_limit'),(-8,-8,'lower_limit')]:
            self.pan.publish(Float64(data=float(pan))); self.tilt.publish(Float64(data=float(tilt))); self.run(4)
            j=self.joints(); ep=max(-math.pi/2,min(math.pi/2,pan)); et=max(-0.6,min(0.8,tilt))
            assert abs(j['gimbal_pan_joint']-ep)<0.05 and abs(j['gimbal_tilt_joint']-et)<0.05
            assert abs(j['front_left_steering_joint'])<0.02 and abs(j['front_right_steering_joint'])<0.02
            im=self.array('gimbal'); diff=float(np.mean(np.abs(im.astype(float)-baseline)))
            frontdiff=float(np.mean(np.abs(self.array('front').astype(float)-fixed)))
            assert diff>3 and frontdiff<3,(key,diff,frontdiff)
            cv2.imwrite(str(self.out/f'gimbal_{key}.png'),cv2.cvtColor(im,cv2.COLOR_RGB2BGR))
            evidence[key]={'actual_rad':[j['gimbal_pan_joint'],j['gimbal_tilt_joint']],'gimbal_image_change':diff,'fixed_image_change':frontdiff}
        self.call(self.center,Trigger.Request()); self.run(3)
        j=self.joints(); assert abs(j['gimbal_pan_joint'])<0.03 and abs(j['gimbal_tilt_joint'])<0.03
        evidence['center_rad']=[j['gimbal_pan_joint'],j['gimbal_tilt_joint']]
        return evidence
    def safety(self):
        self.run(2,0.25,0); start=time.monotonic(); p=self.run(2,publish=False); self.traces['timeout']=p
        assert self.diag.status[0].message=='command_timeout' and abs(self.o.twist.twist.linear.x)<0.03
        stable=self.run(1,publish=False); assert abs(self.o.twist.twist.linear.x)<0.03
        self.command(float('nan'),0); self.run(0.3,publish=False); assert self.diag.status[0].message=='invalid_command'
        self.command(0,float('inf')); self.run(0.3,publish=False); assert self.diag.status[0].message=='invalid_command'
        assert abs(self.o.twist.twist.linear.x)<0.03
        self.run(1,0.2,0); assert self.o.twist.twist.linear.x>0.1
        limited=self.run(2,9,9); values={x.key:float(x.value) for x in self.diag.status[0].values}
        assert abs(values['requested_angle'])<=math.radians(25)+1e-6 and abs(values['requested_speed'])<=0.6
        assert max(abs(x) for k,x in self.joints().items() if k.endswith('steering_joint'))<=0.6+0.01
        self.settled()
        return {'watchdog_state':'command_timeout','stop_vx_mps':float(p[-1,5]),'old_command_did_not_resume':True,'NaN_Inf_rejected':True,'new_valid_command_resumed':True,'clamped_requested_speed':values['requested_speed'],'clamped_requested_angle':values['requested_angle']}
    def twist_reverse(self):
        self.settled(); start=self.snapshot(); p=self.run(3,twist=(-0.2,0.2)); self.traces['twist_reverse']=p
        dyaw=wrap(p[-1,4]-start[4]); j=self.joints()
        assert dyaw>0.15 and j['front_right_steering_joint']<j['front_left_steering_joint']<0
        self.settled(); return {'yaw_change_rad':dyaw,'steering_rad':[j['front_left_steering_joint'],j['front_right_steering_joint']]}
    def collision(self):
        self.settled(); start=self.snapshot(); h=start[4]; x=start[1]+0.65*math.cos(h); y=start[2]+0.65*math.sin(h)
        req=SpawnEntity.Request(); req.name='regression_wall'
        req.xml='<sdf version="1.6"><model name="regression_wall"><static>true</static><link name="wall"><collision name="wall"><geometry><box><size>0.1 2 0.8</size></box></geometry></collision><visual name="wall"><geometry><box><size>0.1 2 0.8</size></box></geometry></visual></link></model></sdf>'
        req.initial_pose.position.x=x; req.initial_pose.position.y=y; req.initial_pose.position.z=0.4
        req.initial_pose.orientation.z=math.sin(h/2); req.initial_pose.orientation.w=math.cos(h/2)
        r=self.call(self.spawn,req); assert r.success
        try:
            p=self.run(6,0.2,0); self.traces['collision']=p
            delta=p[-1,1:3]-np.array(start[1:3]); distance=delta@[math.cos(h),math.sin(h)]
            assert 0.1<distance<0.4,(distance,p[-1].tolist())
            assert abs(np.mean(p[-20:,5]))<0.04
            assert np.ptp(p[:,3])<0.05
            self.settled()
            return {'forward_displacement_m':float(distance),'free_motion_expected_m':1.1,'final_vx_mps':float(np.mean(p[-20:,5])),'max_vertical_excursion_m':float(np.ptp(p[:,3]))}
        finally:
            self.call(self.delete,DeleteEntity.Request(name='regression_wall'))
    def execute(self):
        self.check('B_spawn_and_TF',self.startup)
        if self.o is None: return
        for key,fn in [('B_rest',self.rest),('C_forward',lambda:self.straight(False)),('C_reverse',lambda:self.straight(True)),('D_left',lambda:self.turn(1)),('E_right',lambda:self.turn(-1)),('F_spin_rejection',self.spin_reject),('G_cameras',self.cameras),('H_gimbal',self.gimbal),('J_safety',self.safety),('Twist_reverse',self.twist_reverse),('I_collision',self.collision)]: self.check(key,fn)
        self.command(0,0)


def main():
    p=argparse.ArgumentParser(); p.add_argument('--output',default=str(Path.home()/'.ros/remote5g/reports/xtnetrc/latest')); a=p.parse_args()
    rclpy.init(); test=Regression(Path(a.output))
    try: test.execute()
    finally: test.save(); test.n.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if len(test.results)==12 and all(x['status']=='PASS' for x in test.results.values()) else 1)

if __name__=='__main__': main()
