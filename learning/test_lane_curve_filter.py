"""Offline geometry checks: python3 -m unittest discover -s learning -p 'test_lane_curve_filter.py'."""
import unittest

import cv2
import numpy as np

from lane_curve_filter import filter_straight_markings


class StraightFilterTests(unittest.TestCase):
    def setUp(self):
        self.straight = np.zeros((360, 640), np.uint8)
        self.curves = np.zeros_like(self.straight)
        # Perspective-like crossing stripes, interrupted by a vehicle/body mask.
        cv2.line(self.straight, (80, 345), (440, 10), 255, 8)
        cv2.line(self.straight, (560, 345), (200, 10), 255, 8)
        self.straight[190:245] = 0
        ys = np.arange(10, 346)
        for base in (110, 350):
            xs = base + 0.0018 * (ys - 180) ** 2
            points = np.column_stack([xs, ys]).astype(np.int32)
            cv2.polylines(self.curves, [points], False, 255, 6)

    def test_crossing_straights_removed_while_arcs_survive(self):
        original = self.straight | self.curves
        kept, removed, count = filter_straight_markings(original)
        self.assertGreaterEqual(count, 2)
        straight_rejection = np.count_nonzero(removed & self.straight) / np.count_nonzero(self.straight)
        arc_retention = np.count_nonzero(kept & self.curves) / np.count_nonzero(self.curves)
        self.assertGreater(straight_rejection, 0.95)
        # Shared crossing pixels are inherently ambiguous and may be removed.
        self.assertGreater(arc_retention, 0.85)
        np.testing.assert_array_equal(kept | removed, original)
        self.assertEqual(np.count_nonzero(kept & removed), 0)

    def test_arcs_alone_are_preserved(self):
        kept, _, count = filter_straight_markings(self.curves)
        self.assertEqual(count, 0)
        np.testing.assert_array_equal(kept, self.curves)

    def test_horizontal_and_vertical_lines(self):
        for start, end in [((10, 180), (630, 180)), ((320, 10), (320, 350))]:
            with self.subTest(start=start, end=end):
                mask = np.zeros_like(self.straight)
                cv2.line(mask, start, end, 255, 7)
                _, removed, count = filter_straight_markings(mask)
                self.assertGreaterEqual(count, 1)
                self.assertGreater(np.count_nonzero(removed) / np.count_nonzero(mask), 0.95)

    def test_short_unresolved_marking_is_preserved(self):
        mask = np.zeros_like(self.straight)
        cv2.line(mask, (250, 160), (360, 195), 255, 5)
        kept, _, count = filter_straight_markings(mask)
        self.assertEqual(count, 0)
        np.testing.assert_array_equal(kept, mask)

    def test_empty_and_tiny_images(self):
        for shape in [(360, 640), (8, 8), (1, 1)]:
            with self.subTest(shape=shape):
                mask = np.zeros(shape, np.uint8)
                kept, removed, count = filter_straight_markings(mask)
                self.assertEqual(count, 0)
                self.assertFalse(kept.any() or removed.any())


if __name__ == '__main__':
    unittest.main()
