"""Shutdown transport helper; regular commands are published by the node timer."""
import time

import rclpy


def stop_vehicle(node, duration=0.2):
    deadline = time.monotonic()+duration
    while rclpy.ok():
        node.command(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() >= deadline:
            break
