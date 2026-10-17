"""Gazebo reset service contract checks; service clients are mocked."""
from copy import deepcopy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from gazebo_msgs.msg import EntityState
import gazebo_start_pose as reset


class GazeboStartPoseTests(unittest.TestCase):
    def setUp(self):
        self.state = EntityState(name='xtnetrc', reference_frame='world')
        p, q = self.state.pose.position, self.state.pose.orientation
        p.x, p.y, p.z = 3.4, -1.2, 0.08
        q.x, q.y, q.z, q.w = 0.1, 0.2, 0.3, math.sqrt(0.86)
        self.state.twist.linear.x = 2.0
        self.response = SimpleNamespace(success=True, state=self.state)
        self.get_client, self.set_client = Mock(), Mock()
        for client in (self.get_client, self.set_client):
            client.wait_for_service.return_value = True
            client.call_async.return_value.done.return_value = True
        self.get_client.call_async.return_value.result.return_value = self.response
        self.set_client.call_async.return_value.result.return_value = SimpleNamespace(success=True)
        self.node = Mock()
        self.node.create_client.side_effect = [self.get_client, self.set_client]
        self.returner = reset.GazeboStartPose(self.node)

    def test_full_world_pose_restored_with_zero_velocity(self):
        self.returner.capture()
        expected = deepcopy(self.state.pose)
        # A response object changing later must not change the saved start pose.
        self.state.pose.position.x += 5
        self.assertEqual(self.returner.initial_pose, expected)
        self.response.state = EntityState(pose=deepcopy(expected))
        self.returner.restore()
        request = self.set_client.call_async.call_args.args[0]
        self.assertEqual(request.state.name, 'xtnetrc')
        self.assertEqual(request.state.reference_frame, 'world')
        self.assertEqual(request.state.pose, expected)
        self.assertEqual(request.state.twist, EntityState().twist)
        for call in self.get_client.call_async.call_args_list:
            self.assertEqual(call.args[0].reference_frame, 'world')

    def test_no_snapshot_never_teleports_to_default_origin(self):
        with self.assertRaises(reset.SimulationResetError):
            self.returner.restore()
        self.set_client.call_async.assert_not_called()

    def test_missing_return_service_prevents_capture(self):
        self.set_client.wait_for_service.return_value = False
        with self.assertRaises(reset.SimulationResetError):
            self.returner.capture()
        self.get_client.call_async.assert_not_called()
        self.assertIsNone(self.returner.initial_pose)

    def test_missing_entity_or_invalid_pose_rejected(self):
        self.response.success = False
        with self.assertRaises(reset.SimulationResetError):
            self.returner.capture()
        self.response.success = True
        self.state.pose.orientation.w = 0.0
        with self.assertRaises(reset.SimulationResetError):
            self.returner.capture()
        self.assertIsNone(self.returner.initial_pose)

    def test_server_rejecting_restore_reports_failure(self):
        self.returner.capture()
        self.set_client.call_async.return_value.result.return_value.success = False
        with self.assertRaises(reset.SimulationResetError):
            self.returner.restore()

    def test_accepted_but_wrong_pose_reports_failure(self):
        self.returner.capture()
        self.state.pose.position.x += 1.0
        with self.assertRaises(reset.SimulationResetError):
            self.returner.restore()

    def test_quaternion_sign_does_not_change_orientation(self):
        self.returner.capture()
        q = self.state.pose.orientation
        q.x, q.y, q.z, q.w = -q.x, -q.y, -q.z, -q.w
        self.returner.restore()

    def test_request_timeout_is_bounded(self):
        future = self.get_client.call_async.return_value
        future.done.return_value = False
        with patch.object(reset.rclpy, 'ok', return_value=True), \
             patch.object(reset.time, 'monotonic', side_effect=[0.0, 5.1]):
            with self.assertRaises(reset.SimulationResetError):
                self.returner.capture()
        future.cancel.assert_called_once()


if __name__ == '__main__':
    unittest.main()
