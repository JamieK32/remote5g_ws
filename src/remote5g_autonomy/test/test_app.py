"""Lifecycle checks use mocked ROS transport, GUI and reset clients."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import remote5g_autonomy.app as app
import remote5g_autonomy.ros.commands as commands
from remote5g_autonomy.options import FollowOptions


class AppTests(unittest.TestCase):
    def exercise(self, reset=False, capture_error=False, restore_error=False, drive=True, quit_window=False):
        controller = SimpleNamespace(done=False, arm_after_snapshot=Mock())
        node = Mock(controller=controller, debug=None)
        events = []
        node.timer.cancel.side_effect = lambda: events.append('cancel')
        node.timer.reset.side_effect = lambda: events.append('start')
        returner = Mock(initial_pose=None if capture_error else object())
        display = Mock()
        display.update.return_value = quit_window

        def capture():
            events.append('capture')
            if capture_error:
                raise app.SimulationResetError('capture failed')

        def restore():
            self.assertTrue(controller.done)
            events.append('restore')
            if restore_error:
                raise app.SimulationResetError('restore failed')

        returner.capture.side_effect = capture
        returner.restore.side_effect = restore
        options = FollowOptions(drive=drive, reset_on_exit=reset)
        with patch.object(app, 'LaneFollower', return_value=node), \
             patch.object(app, 'DebugDisplay', return_value=display), \
             patch.object(app, 'create_returner', return_value=returner) as factory, \
             patch.object(app.rclpy, 'init'), patch.object(app.rclpy, 'ok', return_value=True), \
             patch.object(app.rclpy, 'spin_once', side_effect=None if quit_window else KeyboardInterrupt), \
             patch.object(app, 'stop_vehicle', side_effect=lambda n, d: events.append('stop')) as stop, \
             patch.object(app.rclpy, 'shutdown') as shutdown:
            code = app.run(options)
        node.destroy_node.assert_called_once()
        display.close.assert_called_once()
        shutdown.assert_called_once()
        self.assertTrue(controller.done)
        return code, node, returner, events, stop, factory

    def test_ctrl_c_stops_before_cleanup(self):
        code, _, _, events, stop, factory = self.exercise()
        self.assertEqual(code, 0)
        self.assertEqual(events, ['cancel', 'stop'])
        self.assertEqual(stop.call_args.args[1], 0.2)
        factory.assert_not_called()

    def test_snapshot_precedes_control_and_return_follows_stop(self):
        code, _, _, events, _, _ = self.exercise(reset=True)
        self.assertEqual(code, 0)
        self.assertEqual(events, ['cancel', 'capture', 'start', 'cancel', 'stop', 'restore', 'stop'])

    def test_snapshot_failure_never_starts_control(self):
        code, node, returner, _, _, _ = self.exercise(reset=True, capture_error=True)
        self.assertEqual(code, 1)
        node.timer.reset.assert_not_called()
        returner.restore.assert_not_called()

    def test_reset_failure_is_reported_and_cleanup_still_runs(self):
        code, _, _, _, stop, _ = self.exercise(reset=True, restore_error=True)
        self.assertEqual(code, 1)
        self.assertEqual(stop.call_count, 2)

    def test_preview_does_not_publish_or_import_reset_client(self):
        _, _, _, _, stop, factory = self.exercise(drive=False)
        stop.assert_not_called()
        factory.assert_not_called()

    def test_window_quit_uses_the_same_stop_path(self):
        self.assertEqual(self.exercise(quit_window=True)[0], 0)

    def test_stop_duration_is_wall_time_not_number_of_callbacks(self):
        node = Mock()
        with patch.object(commands.rclpy, 'ok', return_value=True), \
             patch.object(commands.rclpy, 'spin_once'), \
             patch.object(commands.time, 'monotonic', side_effect=[0.0, 0.01, 0.1, 0.2]):
            commands.stop_vehicle(node, 0.2)
        self.assertEqual(node.command.call_count, 3)
        for call in node.command.call_args_list:
            self.assertEqual(call.args, (0.0, 0.0))
