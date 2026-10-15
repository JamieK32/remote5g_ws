#!/usr/bin/env python3
"""Finite-duration 20 Hz simulator command source; never accesses hardware."""
import argparse
import math
import time
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('speed', type=float)
    parser.add_argument('steering', type=float)
    parser.add_argument('--seconds', type=float, default=3.0)
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.speed, args.steering, args.seconds)) or not 0 < args.seconds <= 120:
        parser.error('finite values and duration 0 < seconds <= 120 required')
    rclpy.init()
    node = rclpy.create_node('xtnetrc_command_source')
    pub = node.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 1)
    end = time.monotonic()+10
    while pub.get_subscription_count()==0 and time.monotonic()<end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if pub.get_subscription_count()==0:
        node.destroy_node(); rclpy.shutdown(); raise SystemExit('No simulator subscriber within 10 seconds')
    msg=AckermannDriveStamped(); msg.drive.speed=args.speed; msg.drive.steering_angle=args.steering
    try:
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            msg.header.stamp=node.get_clock().now().to_msg(); pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        msg.drive.speed=0.0; msg.drive.steering_angle=0.0
        for _ in range(5): pub.publish(msg); rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
