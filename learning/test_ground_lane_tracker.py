"""Lane-selection geometry regressions; no ROS/Gazebo needed."""
import math
import unittest

import numpy as np

from ground_lane_tracker import Boundary, GroundLaneTracker


class GroundLaneTrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = GroundLaneTracker()
        self.k = [762.7249, 0, 640.5, 0, 762.7249, 360.5, 0, 0, 1]

    def markings(self, center, width=1.22, crossing=False):
        rng = np.random.default_rng(42)
        x = np.arange(0.75, 10.0, 0.1)
        curves = []
        for offset in (-width/2, width/2):
            y = np.polyval(center, x) + offset + rng.normal(0, 0.003, len(x))
            curves.append(np.column_stack([x, y]))
        if crossing:
            for offset in (-0.61, 0.61, -1.83, 1.83):
                curves.append(np.column_stack([x, offset+rng.normal(0, 0.003, len(x))]))
        return np.vstack(curves)

    def test_arc_pair_beats_crossing_straight_pair(self):
        expected = np.array([-0.0125, 0.045, -0.03])
        points = self.markings(expected, crossing=True)
        estimate = self.tracker.choose(self.tracker.boundaries(points))
        self.assertIsNotNone(estimate)
        np.testing.assert_allclose(np.polyval(estimate.center, [1, 3, 6, 9]),
                                   np.polyval(expected, [1, 3, 6, 9]), atol=0.04)
        self.assertAlmostEqual(estimate.width, 1.22, delta=0.06)

    def test_left_and_right_curves_have_correct_steering_sign(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                tracker = GroundLaneTracker()
                expected = np.array([direction*0.013, 0.0, 0.0])
                estimate = tracker.choose(tracker.boundaries(self.markings(expected)))
                self.assertIsNotNone(estimate)
                self.assertGreater(direction*estimate.steering, 0)
                self.assertLess(abs(estimate.steering), 0.05)

    def test_ordinary_straight_lane_is_retained(self):
        expected = np.array([0.0, 0.02, 0.0])
        estimate = self.tracker.choose(self.tracker.boundaries(self.markings(expected)))
        self.assertIsNotNone(estimate)
        self.assertAlmostEqual(estimate.target[1], 0.05, delta=0.02)
        self.assertGreater(estimate.steering, 0)

    def test_wrong_width_and_mixed_boundaries_are_rejected(self):
        for left_coeff, right_coeff in [([0, 0, 0.2], [0, 0, -0.2]),
                                        ([-0.02, 0, 0.61], [0, 0, -0.61])]:
            with self.subTest(left=left_coeff, right=right_coeff):
                left = Boundary(np.array(left_coeff), 80, 0.8, 9.0, 0.003)
                right = Boundary(np.array(right_coeff), 80, 0.8, 9.0, 0.003)
                self.assertIsNone(self.tracker.choose([left, right]))

    def test_previous_lane_is_propagated_by_ego_motion(self):
        self.tracker.previous = np.array([-0.013, 0.02, -0.04])
        self.tracker.previous_pose = (0.0, 0.0, 0.0)
        predicted = self.tracker.predicted_center((0.3, 0.0, 0.0))
        x = np.array([1, 3, 6, 9])
        np.testing.assert_allclose(np.polyval(predicted, x),
                                   np.polyval(self.tracker.previous, x+0.3), atol=1e-8)
        self.assertIsNone(self.tracker.predicted_center((3.0, 0.0, 0.0)))

    def test_previous_curve_survives_competing_parallel_straights(self):
        expected = np.array([-0.013, -0.01, -0.05])
        self.tracker.previous, self.tracker.previous_pose = expected.copy(), (0.0, 0.0, 0.0)
        points = self.markings(expected, crossing=True)
        pose = (0.0, 0.0, 0.0)
        boundaries = self.tracker.boundaries(points, self.tracker.predicted_center(pose))
        estimate = self.tracker.choose(boundaries, pose)
        self.assertIsNotNone(estimate)
        self.assertLess(estimate.center[0], -0.009)
        self.assertLess(estimate.steering, 0)

    def test_missing_calibration_and_blank_images_do_not_guess(self):
        mask = np.zeros((720, 1280), np.uint8)
        self.assertIsNone(self.tracker.detect(mask)[0])
        self.tracker.set_camera(self.k)
        self.tracker.previous, self.tracker.previous_pose = np.array([-0.013, 0, 0]), (0, 0, 0)
        self.assertIsNone(self.tracker.detect(mask, (0, 0, 0))[0])

    def test_optical_axis_and_left_right_projection(self):
        self.tracker.set_camera(self.k)
        distance = 0.3 + 0.235315/math.tan(0.45)
        u, v = self.tracker.project([[distance, 0, 0], [distance, 0.1, 0]])
        self.assertAlmostEqual(float(u[0]), 640.5, places=3)
        self.assertAlmostEqual(float(v[0]), 360.5, places=3)
        self.assertLess(float(u[1]), float(u[0]))


if __name__ == '__main__':
    unittest.main()
