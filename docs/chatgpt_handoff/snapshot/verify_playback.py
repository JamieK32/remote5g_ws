#!/usr/bin/env python3
import os,time,json,subprocess,signal
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.parameter import Parameter
out=Path(__file__).resolve().parent;os.environ['ROS_DOMAIN_ID']='96';os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']=str(out/'runtime/ros_logs')
rclpy.init();n=rclpy.create_node('handoff_playback_observer',parameter_overrides=[Parameter('use_sim_time',value=True)]);received={'odom':0,'clock':0};stamps={}
def odom(m):received['odom']+=1;stamps['last_odom_seconds']=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
def clock(m):received['clock']+=1;stamps['last_clock_seconds']=m.clock.sec+m.clock.nanosec*1e-9
subs=[n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data),n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)]
log=(out/'evidence/bag_play_observed.log').open('w');p=subprocess.Popen(['ros2','bag','play',str(out/'runtime/competition/sample_bag'),'--topics','/odom','/joint_states','/tf','/tf_static','--clock','20'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 end=time.monotonic()+15
 while p.poll() is None and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
 assert p.poll()==0 and all(received.values()),received
 result={'status':'PASS','isolated_ros_domain':96,'received_counts':received,'samples':stamps,'observer_use_sim_time':n.get_parameter('use_sim_time').value,'observer_clock_seconds':n.get_clock().now().nanoseconds*1e-9}
 (out/'playback_observation.json').write_text(json.dumps(result,indent=2));print(result)
finally:
 if p.poll() is None:os.killpg(p.pid,signal.SIGINT);p.wait(timeout=5)
 n.destroy_node();rclpy.shutdown();log.close()
