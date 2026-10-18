"""Exercise control transitions with Humble's real logger, without ROS transport."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import rclpy.logging

from remote5g_autonomy.control import FollowController
from remote5g_autonomy.options import FollowOptions
from remote5g_autonomy.ros.lane_follower import LaneFollower


class ControlLoggingTests(unittest.TestCase):
    def test_wait_drive_loss_and_recovery_can_change_log_severity(self):
        rclpy.logging.initialize()
        logger = rclpy.logging.get_logger('lane_follower_logging_test')
        options = FollowOptions(drive=True, speed=0.7, continuous=True).validate()
        controller = FollowController(options, 0.0)
        # No node or publisher is created: commands are captured locally.
        node = SimpleNamespace(controller=controller, last_observation=None,
                               command=Mock(), get_logger=lambda: logger)

        with patch('remote5g_autonomy.ros.lane_follower.time.monotonic') as clock:
            clock.return_value = 0.0
            LaneFollower.on_control(node)
            for now in (0.1, 0.3):
                controller.odometry((0.0, 0.0), now)
                for _ in range(options.required_good_frames):
                    controller.observe(0.1, -0.02, now)
                clock.return_value = now
                LaneFollower.on_control(node)
                if now == 0.1:
                    controller.invalidate_image()
                    clock.return_value = 0.2
                    LaneFollower.on_control(node)

        self.assertEqual([call.args[0] for call in node.command.call_args_list],
                         [0.0, 0.7, 0.0, 0.7])
        self.assertFalse(controller.done)
