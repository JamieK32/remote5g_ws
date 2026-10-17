"""Control/CLI checks with mocked commands; no ROS nodes or vehicle traffic.

Source ROS and workspace setup, then run:
python3 -m unittest discover -s learning -p test_b1_lane_follow.py -v
"""
import contextlib
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import b1_lane_follow as lane


class ContinuousDriveTests(unittest.TestCase):
    def follower(self, continuous=True):
        args = lane.parse_args(['--drive', '--speed', '0.7'] +
                               (['--continuous'] if continuous else []))
        return SimpleNamespace(
            args=args, done=False, image_at=99.9, odom_at=99.9,
            position=(100.0, 0.0), start_position=(0.0, 0.0),
            error=0.1, steering=-0.035, good_frames=3,
            started_at=1.0, wait_started=0.0, lost_since=None,
            last_log=100.0, command=Mock(), finish=Mock(), get_logger=Mock())

    def tick(self, follower):
        with patch.object(lane.time, 'monotonic', return_value=100.0):
            lane.LaneFollower.on_control(follower)

    def test_requested_command_parses(self):
        args = lane.parse_args(['--drive', '--speed', '0.7', '--continuous',
                                '--filter-straight', '--show'])
        self.assertEqual(args.speed, 0.7)
        self.assertTrue(args.drive and args.continuous and args.filter_straight and args.show)

    def test_invalid_speeds_rejected(self):
        for value in ('0', '-0.1', '0.71', 'nan', 'inf'):
            with self.subTest(speed=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as exc:
                    lane.parse_args(['--speed', value])
                self.assertEqual(exc.exception.code, 2)

    def test_continuous_ignores_both_limits(self):
        follower = self.follower()
        self.tick(follower)
        follower.command.assert_called_once_with(0.7, follower.steering)
        follower.finish.assert_not_called()

    def test_bounded_mode_retains_distance_and_time_limits(self):
        follower = self.follower(continuous=False)
        self.tick(follower)
        self.assertIn('Distance limit', follower.finish.call_args.args[0])
        follower = self.follower(continuous=False)
        follower.position = (0.1, 0.0)
        self.tick(follower)
        follower.finish.assert_called_once_with('Time limit reached')

    def test_start_waits_indefinitely_and_stays_stopped(self):
        follower = self.follower()
        follower.started_at = None
        follower.image_at = follower.odom_at = 0.0
        follower.position = None
        self.tick(follower)
        follower.command.assert_called_once_with(0.0, 0.0)
        follower.finish.assert_not_called()
        self.assertFalse(follower.done)
        self.assertEqual(follower.good_frames, 0)

    def test_lane_loss_stops_then_recovers(self):
        follower = self.follower()
        follower.error = None
        follower.good_frames = 0
        self.tick(follower)
        follower.command.assert_called_with(0.0, 0.0)
        follower.finish.assert_not_called()
        follower.error = 0.1
        follower.good_frames = 2
        self.tick(follower)
        follower.command.assert_called_with(0.0, 0.0)
        follower.good_frames = 3
        self.tick(follower)
        follower.command.assert_called_with(0.7, follower.steering)
        self.assertIsNone(follower.lost_since)

    def test_stale_sensor_stops_and_requires_new_frames(self):
        for field in ('image_at', 'odom_at'):
            with self.subTest(sensor=field):
                follower = self.follower()
                setattr(follower, field, 0.0)
                self.tick(follower)
                follower.command.assert_called_once_with(0.0, 0.0)
                follower.finish.assert_not_called()
                self.assertEqual(follower.good_frames, 0)

    def test_ctrl_c_sends_zero_commands_before_shutdown(self):
        node = Mock(done=False, debug=None)
        argv = ['b1_lane_follow.py', '--drive', '--speed', '0.7', '--continuous',
                '--filter-straight', '--show']
        with patch('sys.argv', argv), \
             patch.object(lane, 'LaneFollower', return_value=node), \
             patch.object(lane.rclpy, 'init'), \
             patch.object(lane.rclpy, 'ok', return_value=True), \
             patch.object(lane.rclpy, 'spin_once', side_effect=KeyboardInterrupt), \
             patch.object(lane, 'stop_vehicle', side_effect=lambda n, duration: n.command(0.0, 0.0)) as stop, \
             patch.object(lane.rclpy, 'shutdown') as shutdown, \
             patch.object(lane.cv2, 'destroyAllWindows'):
            lane.main()
        stop.assert_called_once_with(node, 0.2)
        self.assertEqual(node.command.call_count, 1)
        for call in node.command.call_args_list:
            self.assertEqual(call.args, (0.0, 0.0))
        node.timer.cancel.assert_called_once()
        self.assertTrue(node.done)
        node.destroy_node.assert_called_once()
        shutdown.assert_called_once()

    def test_stop_commands_last_for_wall_time(self):
        node = Mock()
        with patch.object(lane.rclpy, 'ok', return_value=True), \
             patch.object(lane.rclpy, 'spin_once'), \
             patch.object(lane.time, 'monotonic', side_effect=[0.0, 0.01, 0.1, 0.2]):
            lane.stop_vehicle(node, 0.2)
        self.assertEqual(node.command.call_count, 3)
        for call in node.command.call_args_list:
            self.assertEqual(call.args, (0.0, 0.0))

    def test_reset_requires_drive(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            lane.parse_args(['--reset-on-exit'])

    def reset_main(self, capture_error=False, restore_error=False):
        node = Mock(done=False, debug=None)
        events = []
        node.timer.cancel.side_effect = lambda: events.append('timer_cancel')
        node.timer.reset.side_effect = lambda: events.append('timer_start')
        returner = Mock(initial_pose=None if capture_error else object())

        def capture():
            events.append('capture')
            if capture_error:
                raise lane.SimulationResetError('capture failed')

        def restore():
            self.assertTrue(node.done)
            events.append('restore')
            if restore_error:
                raise lane.SimulationResetError('restore failed')

        def interrupt(*args, **kwargs):
            events.append('interrupt')
            raise KeyboardInterrupt

        returner.capture.side_effect = capture
        returner.restore.side_effect = restore
        argv = ['b1_lane_follow.py', '--drive', '--reset-on-exit']
        with patch('sys.argv', argv), \
             patch.object(lane, 'LaneFollower', return_value=node), \
             patch.object(lane, 'GazeboStartPose', return_value=returner), \
             patch.object(lane.rclpy, 'init'), \
             patch.object(lane.rclpy, 'ok', return_value=True), \
             patch.object(lane.rclpy, 'spin_once', side_effect=interrupt), \
             patch.object(lane, 'stop_vehicle', side_effect=lambda n, d: events.append('stop')) as stop, \
             patch.object(lane.rclpy, 'shutdown') as shutdown:
            if capture_error or restore_error:
                with self.assertRaises(SystemExit) as exc:
                    lane.main()
                self.assertEqual(exc.exception.code, 1)
            else:
                lane.main()
        node.destroy_node.assert_called_once()
        shutdown.assert_called_once()
        return node, returner, events, stop

    def test_capture_before_control_and_stop_before_restore(self):
        _, returner, events, _ = self.reset_main()
        self.assertEqual(events, ['timer_cancel', 'capture', 'timer_start', 'interrupt',
                                  'timer_cancel', 'stop', 'restore', 'stop'])
        returner.restore.assert_called_once()

    def test_capture_failure_never_enables_control_or_restores(self):
        node, returner, events, _ = self.reset_main(capture_error=True)
        node.timer.reset.assert_not_called()
        returner.restore.assert_not_called()
        self.assertNotIn('interrupt', events)

    def test_restore_failure_still_stops_and_cleans_up(self):
        _, _, events, stop = self.reset_main(restore_error=True)
        self.assertEqual(stop.call_count, 2)
        self.assertEqual(events[-2:], ['restore', 'stop'])


if __name__ == '__main__':
    unittest.main()
