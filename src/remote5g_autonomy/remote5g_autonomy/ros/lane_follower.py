"""ROS adapter: messages in, pure perception/control calls, Ackermann commands out."""
import math
import time

import cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image

from remote5g_autonomy.control import FollowController, steering_for_observation
from remote5g_autonomy.perception.pipeline import LanePerception
from remote5g_autonomy.visualization.render import render_ground, render_pixel
from .images import decode_bgr
from .resources import load_vehicle_geometry


class LaneFollower(Node):
    def __init__(self, options):
        super().__init__('lane_follower')
        self.options = options
        self.geometry = load_vehicle_geometry(options.vehicle_config)
        self.controller = FollowController(options, time.monotonic())
        self.perception = LanePerception(options, self.geometry)
        self.last_bgr = self.last_observation = self.debug = None
        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 1)
        self.image_sub = self.create_subscription(Image, '/camera/front/image_raw',
                                                   self.on_image, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        if options.filter_straight:
            self.info_sub = self.create_subscription(CameraInfo, '/camera/front/camera_info',
                                                      self.on_camera_info, qos_profile_sensor_data)
        self.timer = self.create_timer(options.control_period, self.on_control)
        if options.drive and options.speed > self.geometry.max_speed:
            self.get_logger().warning(
                f'Requested {options.speed:.2f} m/s exceeds simulator max_speed '
                f'{self.geometry.max_speed:.2f} m/s; the vehicle plugin will limit it')

    def on_image(self, msg):
        try:
            bgr = decode_bgr(msg)
            observation = self.perception.process(bgr)
            steering = steering_for_observation(observation, self.options, self.geometry.wheelbase)
            if self.perception.tracker is not None:
                debug = render_ground(self.perception.tracker, bgr, observation.estimate,
                                      observation.birdseye, observation.boundaries, steering)
            else:
                debug = render_pixel(bgr, observation.pixel)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(f'Image decode/processing failed: {exc}')
            self.controller.invalidate_image()
            return
        self.last_bgr, self.last_observation, self.debug = bgr, observation, debug
        self.controller.observe(observation.error, steering, time.monotonic())

    def on_camera_info(self, msg):
        try:
            self.perception.camera_info(msg.k)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self.controller.invalidate_image()

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y))
        pitch = math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x))))
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.controller.odometry((p.x, p.y), time.monotonic())
        self.perception.odometry((p.x, p.y, p.z), (roll, pitch, yaw))

    def command(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed, msg.drive.steering_angle = float(speed), float(steering)
        self.pub.publish(msg)

    def on_control(self):
        decision = self.controller.step(time.monotonic())
        if decision.publish:
            self.command(decision.speed, decision.steering)
        for level, text in decision.events:
            observation = self.last_observation
            if text.startswith('distance=') and observation is not None and observation.estimate is not None:
                estimate = observation.estimate
                mode = 'curve' if abs(estimate.center[0]) > 0.0045 else 'straight'
                text += f', lane={mode}, width={estimate.width:.2f}m, target_y={observation.error:+.3f}m'
            # Humble caches severity per call site; keep each level on its own line.
            if level == 'warning':
                self.get_logger().warning(text)
            else:
                self.get_logger().info(text)
