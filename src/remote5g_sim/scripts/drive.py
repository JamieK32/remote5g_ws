#!/usr/bin/env python3
"""Finite-duration 50 Hz wall-time simulator command source; no hardware access."""
import argparse
import math
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped

COMMAND_RATE_HZ = 50.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('speed', type=float)
    parser.add_argument('steering', type=float)
    parser.add_argument('--seconds', type=float, default=3.0)
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.speed, args.steering, args.seconds)) or not 0 < args.seconds <= 120:
        parser.error('finite values and duration 0 < seconds <= 120 required')
    # Keep the context alive on Ctrl+C so finally can send explicit zero commands.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('xtnetrc_command_source')
    pub = node.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 1)
    end = time.monotonic()+10
    while pub.get_subscription_count()==0 and time.monotonic()<end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if pub.get_subscription_count()==0:
        node.destroy_node(); rclpy.shutdown(); raise SystemExit('No simulator subscriber within 10 seconds')
    msg=AckermannDriveStamped(); msg.drive.speed=args.speed; msg.drive.steering_angle=args.steering
    def publish_command():
        msg.header.stamp=node.get_clock().now().to_msg()
        pub.publish(msg)
    # The node uses wall time by default, matching the bounded duration and lease.
    timer=node.create_timer(1.0/COMMAND_RATE_HZ,publish_command)
    try:
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        timer.cancel()
        msg.drive.speed=0.0; msg.drive.steering_angle=0.0
        for _ in range(5): publish_command(); rclpy.spin_once(node, timeout_sec=1.0/COMMAND_RATE_HZ)
        node.destroy_timer(timer)
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
