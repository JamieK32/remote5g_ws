"""White-pixel segmentation and the original three-row pixel detector."""
from dataclasses import dataclass

import cv2
import numpy as np


SCAN_ROWS = (0.22, 0.28, 0.34)


def white_mask(bgr, close=False):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 150), (179, 75, 255))
    if close:
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    return mask


@dataclass
class PixelDetection:
    error: float | None
    mask: np.ndarray
    matches: list


def detect_pixel_lane(bgr):
    h, w = bgr.shape[:2]
    mask = white_mask(bgr, close=True)
    matches = []
    for row in SCAN_ROWS:
        y = round(h*row)
        band = mask[max(0, y-5):min(h, y+6)]
        profile = np.convolve((band > 0).mean(axis=0), np.ones(7)/7, mode='same')
        flags = profile > 0.12
        edges = np.diff(np.r_[False, flags, False].astype(np.int8))
        starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
        centers = [(a+b)/2 for a, b in zip(starts, ends) if 2 <= b-a <= w*0.11]
        left = [x for x in centers if 0.035*w < x < 0.48*w]
        right = [x for x in centers if 0.52*w < x < 0.965*w]
        pairs = [(l, r) for l in left for r in right if 0.28*w < r-l < 0.93*w]
        if pairs:
            l, r = min(pairs, key=lambda p: abs((p[0]+p[1])/2-w/2)/w
                       + 0.15*abs((p[1]-p[0])/w-0.65))
            matches.append((row, l, r))
    error = None
    if len(matches) >= 2:
        weights = [1.5 if row <= 0.28 else 1.0 for row, _, _ in matches]
        center = np.average([(l+r)/2 for _, l, r in matches], weights=weights)
        error = float((center-w/2)/(w/2))
    return PixelDetection(error, mask, matches)
