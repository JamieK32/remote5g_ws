#!/usr/bin/env python3
"""Bounded Gazebo drive check, with pose rollback and ground-truth measurements.

The lane controller receives images, calibration and odometry only. The layout
file is used here solely to measure cross-track error, never to steer the car.
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

from b1_lane_follow import LaneFollower, parse_args, stop_vehicle
from gazebo_start_pose import GazeboStartPose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--distance', type=float, default=25.0)
    parser.add_argument('--speed', type=float, default=0.7)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--output', default=str(Path.home()/'.ros/remote5g/reports/curve_follow_v2/drive'))
    parser.add_argument('--layout', default=str(Path.home()/'.ros/remote5g/reports/competition/current/layout.json'))
    options = parser.parse_args()
    layout = json.loads(Path(options.layout).read_text())
    assert layout['config']['start_side'] == 'left', 'This measurement fixture is for the left start'
    radius, half = layout['route']['radius'], layout['route']['half_straight']
    output = Path(options.output)
    output.mkdir(parents=True, exist_ok=True)
    args = parse_args(['--drive', '--speed', str(options.speed), '--continuous', '--filter-straight'])
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LaneFollower(args)
    node.timer.cancel()
    returner = GazeboStartPose(node)
    samples = []
    traveled = 0.0
    previous = None
    next_image = 0.0
    error = None
    started = time.monotonic()
    try:
        returner.capture()
        node.good_frames = 0
        node.timer.reset()
        while time.monotonic()-started < options.timeout:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.world_pose is None:
                continue
            x, y, yaw = node.world_pose
            if previous is not None:
                traveled += math.hypot(x-previous[0], y-previous[1])
            previous = (x, y)
            cross_track = abs(math.hypot(x+half, y)-radius) if x < -half else abs(y+radius)
            samples.append([time.monotonic()-started, traveled, x, y, yaw, cross_track,
                            node.steering, node.error if node.error is not None else float('nan'),
                            node.good_frames])
            if node.debug is not None and traveled >= next_image:
                cv2.imwrite(str(output/f'frame_{round(traveled):03d}m.png'), node.debug)
                if node.last_bgr is not None:
                    cv2.imwrite(str(output/f'raw_{round(traveled):03d}m.png'), node.last_bgr)
                    metadata = {'pose':node.world_pose, 'camera_k':node.ground_tracker.k.tolist(),
                                'camera_origin':node.ground_tracker.origin.tolist(),
                                'camera_rotation':node.ground_tracker.rotation.tolist(),
                                'boundaries':[{'coefficients':b.coefficients.tolist(), 'support':b.support,
                                               'start':b.start, 'end':b.end} for b in node.last_boundaries],
                                'selected':node.last_estimate.center.tolist() if node.last_estimate else None}
                    (output/f'frame_{round(traveled):03d}m.json').write_text(json.dumps(metadata,indent=2))
                next_image += 1.0
            if cross_track > 0.45:
                raise RuntimeError(f'Cross-track error exceeded 0.45m: {cross_track:.3f}')
            if traveled >= options.distance:
                break
        if traveled < options.distance:
            raise RuntimeError(f'Distance not reached before timeout: {traveled:.2f}m')
    except Exception as exc:
        error = str(exc)
    finally:
        node.done = True
        node.timer.cancel()
        try:
            stop_vehicle(node, 0.6)
            if returner.initial_pose is not None:
                returner.restore()
            stop_vehicle(node, 0.3)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    data = np.asarray(samples)
    report = {'status':'PASS' if error is None else 'FAIL', 'error':error,
              'target_speed_mps':options.speed, 'traveled_m':traveled,
              'wall_seconds':time.monotonic()-started,
              'max_cross_track_m':float(data[:,5].max()) if len(data) else None,
              'rms_cross_track_m':float(np.sqrt(np.mean(data[:,5]**2))) if len(data) else None,
              'yaw_change_deg':float(math.degrees(math.atan2(math.sin(data[-1,4]-data[0,4]),
                                                            math.cos(data[-1,4]-data[0,4])))) if len(data) else None}
    np.savetxt(output/'trace.csv', data, delimiter=',',
               header='wall_seconds,traveled_m,x,y,yaw,cross_track_m,steering_rad,target_y_m,good_frames')
    (output/'result.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    raise SystemExit(0 if error is None else 1)


if __name__ == '__main__':
    main()
