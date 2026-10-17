#!/usr/bin/env python3
"""ROS2 Humble/Gazebo Classic: safe starter white-line following on blue/red track.

Preview (no vehicle commands): python3 learning/b1_lane_follow.py --show
Filter straight markings:    python3 learning/b1_lane_follow.py --filter-straight --show
Drive (low speed, bounded):   python3 learning/b1_lane_follow.py --drive --show
Continuous drive:            python3 learning/b1_lane_follow.py --drive --speed 0.7 --continuous --filter-straight --show
"""
import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo

from gazebo_start_pose import GazeboStartPose, SimulationResetError
from ground_lane_tracker import GroundLaneTracker


# These rows are deliberately high in the image: the fixed camera points down.
# Close to the bottom of the image, a 1.22-m-wide lane may exceed the FOV.
SCAN_ROWS = (0.22, 0.28, 0.34)


def white_line_pairs(bgr):
    """Return normalized lane-center error plus a drawable debug frame.

    Positive error means the lane center appears to the RIGHT of image center.
    Both left and right white markings are required; no single-line guessing.
    """
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # White = bright and low saturation; adjust after checking actual frames.
    mask = cv2.inRange(hsv, (0, 0, 150), (179, 75, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    debug = bgr.copy()
    cv2.line(debug, (w // 2, 0), (w // 2, h - 1), (0, 220, 255), 2)
    matches = []

    for row_frac in SCAN_ROWS:
        y = round(h * row_frac)
        band = mask[max(0, y - 5):min(h, y + 6)]
        # Fraction of white pixels per column, smoothed to tolerate thin lines.
        profile = np.convolve((band > 0).mean(axis=0),
                              np.ones(7) / 7.0, mode='same')
        flags = profile > 0.12
        edges = np.diff(np.r_[False, flags, False].astype(np.int8))
        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1)
        centers = [(a + b) / 2 for a, b in zip(starts, ends)
                   if 2 <= b - a <= w * 0.11]
        left = [x for x in centers if 0.035 * w < x < 0.48 * w]
        right = [x for x in centers if 0.52 * w < x < 0.965 * w]
        pairs = [(l, r) for l in left for r in right
                 if 0.28 * w < r - l < 0.93 * w]
        cv2.line(debug, (0, y), (w - 1, y), (100, 100, 100), 1)
        if not pairs:
            continue
        # Prefer a lane roughly centered in the view, and avoid implausible width.
        l, r = min(pairs, key=lambda p:
                   abs((p[0] + p[1]) / 2 - w / 2) / w
                   + 0.15 * abs((p[1] - p[0]) / w - 0.65))
        cx = (l + r) / 2
        matches.append((row_frac, cx))
        cv2.circle(debug, (round(l), y), 6, (255, 100, 0), -1)
        cv2.circle(debug, (round(r), y), 6, (255, 100, 0), -1)
        cv2.circle(debug, (round(cx), y), 6, (0, 255, 0), -1)

    # Require at least two scan rows to reduce false detections of random marks.
    if len(matches) < 2:
        error = None
        status = 'NO TWO-LINE MATCH'
    else:
        # Farther scan rows get a little more weight for looking ahead.
        weights = np.array([1.5 if y <= 0.28 else 1.0
                            for y, _ in matches], dtype=np.float64)
        lane_x = float(np.average([x for _, x in matches], weights=weights))
        error = (lane_x - w / 2) / (w / 2)
        status = f'error={error:+.3f}'
    cv2.putText(debug, status, (12, h - 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.85, (0, 255, 0) if error is not None else (0, 0, 255), 2)
    # A right-hand panel makes it easy to inspect the actual white threshold.
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(mask_bgr, 'WHITE MASK', (12, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return error, cv2.hconcat([debug, mask_bgr])


class LaneFollower(Node):
    def __init__(self, args):
        super().__init__('b1_vision_lane_follower')
        self.args = args
        self.ground_tracker = None
        self.world_pose = None
        self.lane_summary = ''
        self.last_bgr = None
        self.last_boundaries = []
        self.last_estimate = None
        if args.filter_straight:
            config = Path(__file__).resolve().parents[1] / 'src/remote5g_sim/config/vehicle.yaml'
            p = yaml.safe_load(config.read_text())
            self.ground_tracker = GroundLaneTracker(
                wheelbase=p['wheelbase'], camera_x=p['front_x'],
                camera_height=p['wheel_radius']+p['front_z'], camera_pitch=p['front_pitch'],
                lane_width=args.lane_width, lookahead=args.lookahead)
            self.info_sub = self.create_subscription(
                CameraInfo, '/camera/front/camera_info', self.on_camera_info, qos_profile_sensor_data)
        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 1)
        self.image_sub = self.create_subscription(
            Image, '/camera/front/image_raw', self.on_image, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)
        self.timer = self.create_timer(0.02, self.on_control)  # 50-Hz command timer
        self.error = None
        self.steering = 0.0
        self.good_frames = 0
        self.image_at = 0.0
        self.odom_at = 0.0
        self.position = None
        self.start_position = None
        self.started_at = None
        self.wait_started = time.monotonic()
        self.lost_since = None
        self.done = False
        self.last_log = 0.0
        self.debug = None

    def on_image(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width * 3:
            self.get_logger().error(f'Unsupported image encoding/step: {msg.encoding}')
            self.error = None
            return
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self.last_bgr = img
            if self.ground_tracker is not None:
                mask = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (0, 0, 150), (179, 75, 255))
                estimate, birdseye, boundaries, _ = self.ground_tracker.detect(mask, self.world_pose)
                self.last_estimate, self.last_boundaries = estimate, boundaries
                debug = self.ground_tracker.draw(img, estimate, birdseye, boundaries)
                error = estimate.target[1] if estimate is not None else None
                if estimate is not None:
                    self.steering = float(np.clip(estimate.steering, -self.args.max_steer, self.args.max_steer))
                    mode = 'curve' if abs(estimate.center[0]) > 0.0045 else 'straight'
                    self.lane_summary = f', lane={mode}, width={estimate.width:.2f}m, target_y={error:+.3f}m'
            else:
                error, debug = white_line_pairs(img)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(f'Image decode/processing failed: {exc}')
            self.error = None
            return
        self.image_at = time.monotonic()
        self.error = error
        self.debug = debug
        if error is None:
            self.good_frames = 0
        else:
            self.good_frames += 1
            # error > 0 means lane center on right -> steer right (negative).
            if self.ground_tracker is None:
                angle = -self.args.kp * error
                self.steering = float(np.clip(angle, -self.args.max_steer, self.args.max_steer))

    def on_camera_info(self, msg):
        try:
            self.ground_tracker.set_camera(msg.k)
        except ValueError as exc:
            self.get_logger().error(str(exc))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.position = (p.x, p.y)
        self.odom_at = time.monotonic()
        if self.ground_tracker is not None:
            q = msg.pose.pose.orientation
            roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
            pitch = math.asin(float(np.clip(2*(q.w*q.y-q.z*q.x), -1, 1)))
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.world_pose = (p.x, p.y, yaw)
            self.ground_tracker.set_attitude(roll, pitch, p.z)

    def command(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = speed
        msg.drive.steering_angle = steering
        self.pub.publish(msg)

    def finish(self, reason):
        self.get_logger().info(reason)
        self.done = True
        self.command(0.0, 0.0)

    def on_control(self):
        if not self.args.drive or self.done:
            return
        now = time.monotonic()
        image_ok = now - self.image_at < 0.6
        odom_ok = self.position is not None and now - self.odom_at < 1.0
        detected = self.error is not None and self.good_frames >= 3

        if not image_ok or not odom_ok or not detected:
            self.command(0.0, 0.0)
            if self.args.continuous:
                if self.lost_since is None:
                    self.get_logger().warning(
                        'Stopped: waiting for valid lane, camera and odom; '
                        'will resume when detection recovers')
                    self.lost_since = now
                if not image_ok or not odom_ok:
                    self.good_frames = 0
                return
            if self.started_at is None:
                if now - self.wait_started > 12.0:
                    self.finish('Failed to acquire lane/odom within 12 seconds')
            else:
                if self.lost_since is None:
                    self.lost_since = now
                if now - self.lost_since >= 0.5:
                    self.finish('Lane or camera/odom lost: safe stop')
            return
        recovered = self.lost_since is not None
        self.lost_since = None
        if self.started_at is None:
            self.started_at = now
            self.start_position = self.position
            self.get_logger().info(
                f'Lane acquired; beginning drive at {self.args.speed:.2f} m/s')
        elif self.args.continuous and recovered:
            self.get_logger().info('Lane and sensors recovered; resuming drive')

        distance = math.hypot(self.position[0] - self.start_position[0],
                              self.position[1] - self.start_position[1])
        if not self.args.continuous and distance >= self.args.max_distance:
            self.finish(f'Distance limit reached: {distance:.2f} m')
            return
        if not self.args.continuous and now - self.started_at >= self.args.seconds:
            self.finish('Time limit reached')
            return
        self.command(self.args.speed, self.steering)
        if now - self.last_log > 1.0:
            self.get_logger().info(
                f'distance={distance:.2f}m, error={self.error:+.3f}, '
                f'steer={math.degrees(self.steering):+.2f}deg{self.lane_summary}')
            self.last_log = now


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Preview or follow two white track borders')
    parser.add_argument('--drive', action='store_true', help='enable actual vehicle commands')
    parser.add_argument('--show', action='store_true', help='OpenCV debug window; q to exit')
    parser.add_argument('--filter-straight', action='store_true',
                        help='use calibrated ground-lane tracking; prefer curves over crossing straight markings')
    parser.add_argument('--lookahead', type=float, default=2.5,
                        help='ground-lane pure pursuit lookahead in metres (default: 2.5)')
    parser.add_argument('--lane-width', type=float, default=1.22,
                        help='expected ground lane width in metres (default: 1.22)')
    parser.add_argument('--continuous', action='store_true',
                        help='disable time/distance limits; stop and wait on detection loss, '
                             'resume when valid; Ctrl+C to exit')
    parser.add_argument('--reset-on-exit', action='store_true',
                        help='Gazebo only: save starting world pose and reset vehicle on exit; requires --drive')
    parser.add_argument('--vehicle-entity', default='xtnetrc',
                        help='Gazebo vehicle model name for --reset-on-exit (default: xtnetrc)')
    parser.add_argument('--save-dir', default='', help='save periodic debug screenshots')
    parser.add_argument('--speed', type=float, default=0.10, help='target speed in m/s, 0 < speed <= 0.7')
    parser.add_argument('--kp', type=float, default=0.35, help='pixel-mode gain; used without --filter-straight')
    parser.add_argument('--max-steer', type=float, default=0.22)
    parser.add_argument('--seconds', type=float, default=15.0, help='time limit; ignored with --continuous')
    parser.add_argument('--max-distance', type=float, default=2.0, help='displacement limit; ignored with --continuous')
    args = parser.parse_args(argv)
    if args.reset_on_exit and not args.drive:
        parser.error('--reset-on-exit requires --drive; preview never resets the vehicle')
    if not args.vehicle_entity.strip():
        parser.error('--vehicle-entity must not be empty')
    if not (0.0 < args.speed <= 0.70 and
            0.0 < args.kp <= 2.0 and
            0.0 < args.max_steer <= 0.40 and
            0.8 <= args.lookahead <= 6.0 and
            0.5 <= args.lane_width <= 3.0 and
            0.0 < args.seconds <= 160.0 and
            0.0 < args.max_distance <= 180.0):
        parser.error('Require 0 < speed <= 0.7, 0 < kp <= 2, 0 < max-steer <= 0.4, '
                     '0.8 <= lookahead <= 6, 0.5 <= lane-width <= 3, '
                     '0 < seconds <= 160 and 0 < max-distance <= 180')
    return args


def stop_vehicle(node, duration=0.2):
    """Keep commanding zero for wall time, even if callbacks return immediately."""
    deadline = time.monotonic() + duration
    while rclpy.ok():
        node.command(0.0, 0.0)
        rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic() >= deadline:
            break


def main():
    args = parse_args()
    savedir = Path(args.save_dir) if args.save_dir else None
    if savedir:
        savedir.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LaneFollower(args)
    last_save = 0.0
    image_number = 0
    returner = None
    exit_code = 0
    try:
        if args.reset_on_exit:
            # Service calls spin this node. Disable control before spinning so
            # even a valid camera/odom cannot move it before capture completes.
            node.timer.cancel()
            returner = GazeboStartPose(node, args.vehicle_entity)
            returner.capture()
            node.good_frames = 0
            node.wait_started = time.monotonic()
            node.timer.reset()
        mode = 'DRIVE' if args.drive else 'PREVIEW ONLY (NO VEHICLE COMMANDS)'
        node.get_logger().info(f'{mode}: waiting for /camera/front/image_raw')
        if args.drive:
            node.get_logger().info(f'Target speed: {args.speed:.2f} m/s')
            if args.continuous:
                node.get_logger().info(
                    'Continuous mode: no time/distance limit; stop and wait on '
                    'lane/sensor loss, resume on recovery; Ctrl+C or q to exit')
        if args.filter_straight:
            node.get_logger().info(
                'Ground lane tracker enabled: GREEN=selected lane borders, CYAN=center, '
                'RED=straight candidates; waiting for front camera calibration')
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.debug is not None:
                if args.show:
                    cv2.imshow('B1 lane detection / white mask', node.debug)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                if savedir and time.monotonic() - last_save > 2.0:
                    cv2.imwrite(str(savedir / f'lane_{image_number:03d}.png'), node.debug)
                    image_number += 1
                    last_save = time.monotonic()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except SimulationResetError as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.done = True
        node.timer.cancel()
        try:
            # Control stays disabled throughout stop, reset and verification.
            if args.drive and rclpy.ok():
                stop_vehicle(node, 0.6 if returner is not None else 0.2)
                if returner is not None and returner.initial_pose is not None:
                    try:
                        returner.restore()
                    except SimulationResetError as exc:
                        node.get_logger().error(f'Return to start failed: {exc}')
                        exit_code = 1
                    stop_vehicle(node, 0.3)
        finally:
            if args.show:
                cv2.destroyAllWindows()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
