from types import SimpleNamespace
import unittest

import numpy as np

from remote5g_autonomy.ros.images import decode_bgr


class ImageConversionTests(unittest.TestCase):
    def test_rgb_conversion_and_row_padding(self):
        msg = SimpleNamespace(encoding='rgb8', width=1, height=2, step=4,
                              data=bytes([255, 0, 0, 99, 0, 255, 0, 99]))
        image = decode_bgr(msg)
        np.testing.assert_array_equal(image, [[[0, 0, 255]], [[0, 255, 0]]])

    def test_bgr_is_not_swapped(self):
        msg = SimpleNamespace(encoding='bgr8', width=1, height=1, step=3, data=bytes([1, 2, 3]))
        np.testing.assert_array_equal(decode_bgr(msg), [[[1, 2, 3]]])

    def test_unsupported_and_truncated_frames_are_rejected(self):
        for encoding, step, data in [('mono8', 3, bytes(3)), ('rgb8', 2, bytes(2)), ('rgb8', 3, bytes(2))]:
            with self.subTest(encoding=encoding, step=step):
                with self.assertRaises(ValueError):
                    decode_bgr(SimpleNamespace(encoding=encoding, width=1, height=1, step=step, data=data))
