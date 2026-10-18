#!/usr/bin/env python3
import argparse
from pathlib import Path
import time
import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output', default=str(Path.home()/'.ros/remote5g/reports/xtnetrc/cameras'))
    parser.add_argument('--timeout', type=float, default=15)
    args=parser.parse_args(); out=Path(args.output); out.mkdir(parents=True,exist_ok=True)
    rclpy.init(); n=rclpy.create_node('xtnetrc_camera_check'); images={}; infos={}; subscriptions=[]
    for name in ('front','gimbal'):
        subscriptions.append(n.create_subscription(Image,f'/camera/{name}/image_raw',lambda m,k=name:images.__setitem__(k,m),qos_profile_sensor_data))
        subscriptions.append(n.create_subscription(CameraInfo,f'/camera/{name}/camera_info',lambda m,k=name:infos.__setitem__(k,m),qos_profile_sensor_data))
    end=time.monotonic()+args.timeout
    while (len(images)<2 or len(infos)<2) and time.monotonic()<end: rclpy.spin_once(n,timeout_sec=0.1)
    ok=len(images)==2 and len(infos)==2
    for name,m in images.items():
        data=np.frombuffer(bytes(m.data),dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='rgb8': data=cv2.cvtColor(data,cv2.COLOR_RGB2BGR)
        info=infos.get(name); valid=info is not None and info.k[0]>0 and info.width==m.width and info.header.frame_id==m.header.frame_id and data.std()>1
        ok=ok and valid
        cv2.imwrite(str(out/f'{name}.png'),data)
        print(f'{name}: {m.width}x{m.height}, {m.encoding}, frame={m.header.frame_id}, CameraInfo={valid}')
    n.destroy_node(); rclpy.shutdown(); raise SystemExit(0 if ok else 1)

if __name__=='__main__': main()
