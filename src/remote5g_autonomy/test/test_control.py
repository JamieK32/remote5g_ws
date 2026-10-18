"""Core policy tests run without ROS, Gazebo, image callbacks or a clock mock."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest

from remote5g_autonomy.control import FollowController, pure_pursuit_steering
from remote5g_autonomy.options import FollowOptions, parse_args


class FollowControlTests(unittest.TestCase):
    def controller(self, continuous=True):
        options = FollowOptions(drive=True, speed=0.7, continuous=continuous).validate()
        controller = FollowController(options, 0)
        controller.odometry((100, 0), 99.9)
        for _ in range(3):
            controller.observe(0.1, -0.035, 99.9)
        controller.started_at, controller.start_position, controller.last_log = 1.0, (0, 0), 100.0
        return controller

    def test_continuous_ignores_time_and_distance(self):
        decision = self.controller().step(100)
        self.assertTrue(decision.publish)
        self.assertEqual((decision.speed, decision.steering), (0.7, -0.035))

    def test_default_bounded_mode_keeps_both_limits(self):
        for position, expected in [((100, 0), 'Distance limit'), ((0.1, 0), 'Time limit')]:
            with self.subTest(position=position):
                controller = self.controller(False)
                controller.position = position
                decision = controller.step(100)
                self.assertTrue(controller.done)
                self.assertEqual(decision.speed, 0)
                self.assertIn(expected, decision.events[-1][1])

    def test_preview_never_requests_publication(self):
        controller = FollowController(FollowOptions(), 0)
        self.assertFalse(controller.step(100).publish)

    def test_start_waits_indefinitely_only_in_continuous_mode(self):
        for continuous in (False, True):
            with self.subTest(continuous=continuous):
                controller = FollowController(FollowOptions(drive=True, continuous=continuous), 0)
                decision = controller.step(100)
                self.assertEqual(decision.speed, 0)
                self.assertEqual(controller.done, not continuous)

    def test_lane_loss_requires_three_new_good_frames(self):
        controller = self.controller()
        controller.observe(None, 0, 100)
        self.assertEqual(controller.step(100).speed, 0)
        for count in range(1, 4):
            controller.observe(0.1, -0.02, 100)
            decision = controller.step(100)
            self.assertEqual(decision.speed, 0.7 if count == 3 else 0)

    def test_stale_camera_or_odom_stops_and_resets_confirmation(self):
        for field in ('image_at', 'odom_at'):
            with self.subTest(field=field):
                controller = self.controller()
                setattr(controller, field, 0)
                self.assertEqual(controller.step(100).speed, 0)
                self.assertFalse(controller.done)
                self.assertEqual(controller.good_frames, 0)

    def test_bounded_loss_finishes_after_timeout(self):
        controller = self.controller(False)
        controller.position = (0.1, 0)
        controller.observe(None, 0, 5)
        controller.step(5)
        controller.step(5.6)
        self.assertTrue(controller.done)

    def test_nonfinite_observation_is_invalid(self):
        controller = self.controller()
        controller.observe(float('nan'), 0, 100)
        self.assertEqual(controller.good_frames, 0)
        self.assertEqual(controller.step(100).speed, 0)

    def test_pure_pursuit_geometry_and_invalid_target(self):
        self.assertGreater(pure_pursuit_steering((2.5, 0.1), 0.23861), 0)
        self.assertLess(pure_pursuit_steering((2.5, -0.1), 0.23861), 0)
        with self.assertRaises(ValueError):
            pure_pursuit_steering((0, 0), 0.23861)


class ConfigurationTests(unittest.TestCase):
    def test_existing_flags_and_user_speed_limit_are_preserved(self):
        options = parse_args(['--drive', '--speed', '2.1', '--continuous', '--filter-straight',
                              '--show', '--reset-on-exit'])
        self.assertEqual(options.speed, 2.1)
        self.assertTrue(options.drive and options.continuous and options.filter_straight)

    def test_invalid_values_are_rejected(self):
        for value in ('0', '-0.1', '2.11', 'nan', 'inf'):
            with self.subTest(value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args(['--speed', value])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(['--reset-on-exit'])

    def test_yaml_cli_precedence_and_relative_vehicle_path(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'follow.yaml'
            path.write_text('speed: 0.4\nshow: true\nvehicle_config: vehicle.yaml\n')
            options = parse_args(['--config', str(path), '--speed', '0.7', '--no-show'])
            self.assertEqual(options.speed, 0.7)
            self.assertFalse(options.show)
            self.assertEqual(options.vehicle_config, str(Path(directory)/'vehicle.yaml'))

    def test_bad_config_does_not_silently_ignore_keys_or_types(self):
        for content in ('typo_speed: 0.7\n', 'show: "false"\n', 'required_good_frames: 1.5\n'):
            with self.subTest(content=content), tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'bad.yaml'
                path.write_text(content)
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                    parse_args(['--config', str(path)])

    def test_cli_vehicle_path_is_relative_to_current_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'follow.yaml'
            path.write_text('vehicle_config: configured.yaml\n')
            options = parse_args(['--config', str(path), '--vehicle-config', 'cli_vehicle.yaml'])
            self.assertEqual(options.vehicle_config, str(Path.cwd()/'cli_vehicle.yaml'))
