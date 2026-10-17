"""Conservative removal of long, straight white markings from a binary image.

This is an image-space shape filter, not a route selector. Ambiguous/short
markings are retained. At crossings, pixels shared by a line and an arc cannot
be separated, so some small gaps in the surviving arcs are expected.
"""
import cv2
import numpy as np


def _row_runs(mask, stride):
    points, widths = [], []
    h, w = mask.shape
    for y in range(stride, h - stride, stride):
        edges = np.diff(np.r_[False, mask[y] > 0, False].astype(np.int8))
        starts = np.flatnonzero(edges == 1)
        ends = np.flatnonzero(edges == -1)
        for start, end in zip(starts, ends):
            width = end - start
            if 1 <= width <= 0.18 * w:
                points.append((y, (start + end - 1) * 0.5))
                widths.append(width)
    return np.asarray(points, dtype=float).reshape(-1, 2), np.asarray(widths)


def _straight_tracks(mask):
    """Find x=a*y+b tracks; run again on transpose for horizontal markings."""
    h, w = mask.shape
    stride = max(2, round(h / 110))
    points, widths = _row_runs(mask, stride)
    if len(points) < 20:
        return []
    rng = np.random.default_rng(7)
    pairs = rng.integers(len(points), size=(700, 2))
    p, q = points[pairs[:, 0]], points[pairs[:, 1]]
    separated = np.abs(q[:, 0] - p[:, 0]) > 0.45 * h
    p, q = p[separated], q[separated]
    if not len(p):
        return []
    slopes = (q[:, 1] - p[:, 1]) / (q[:, 0] - p[:, 0])
    intercepts = p[:, 1] - slopes * p[:, 0]
    tolerance = max(1.1, 0.002 * w)
    residuals = np.abs(points[None, :, 1] -
                       slopes[:, None] * points[None, :, 0] - intercepts[:, None])
    scores = (residuals < tolerance).sum(axis=1)
    tracks = []
    used = np.zeros(len(points), dtype=bool)
    for idx in np.argsort(scores)[::-1]:
        inliers = (residuals[idx] < tolerance) & ~used
        if inliers.sum() < max(16, 0.35 * h / stride):
            continue
        ys, xs = points[inliers].T
        # Work in normalized row coordinates for well-conditioned polynomial fits.
        t = ys / h
        line = np.polyfit(t, xs, 1)
        prediction = np.polyval(line, points[:, 0] / h)
        inliers = (np.abs(points[:, 1] - prediction) < tolerance) & ~used
        ys, xs = points[inliers].T
        if len(ys) < 16 or np.ptp(ys) < 0.5 * h:
            continue
        span = np.ptp(ys)
        if np.hypot(span, line[0] * span / h) < 0.45 * np.hypot(h, w):
            continue
        # Include the immediate neighbourhood when testing for curvature. Fitting
        # only the line's tight inliers could flatten a shallow arc artificially.
        near = (np.abs(points[:, 1] - prediction) < 3 * tolerance) & ~used
        near &= (points[:, 0] >= ys.min()) & (points[:, 0] <= ys.max())
        ty = points[near, 0] / h
        curve = np.polyfit(ty, points[near, 1], 2)
        bow = abs(curve[0]) * (span / h) ** 2 / 4
        if bow > max(0.9, 0.0018 * w):
            continue
        # A perspective-projected stripe's width varies approximately linearly.
        # Exclude fat merged crossing blobs from the stripe-width estimate.
        stripe_widths = widths[inliers]
        typical = np.median(stripe_widths)
        narrow = stripe_widths < max(typical * 2.0, typical + 3)
        if narrow.sum() < 12:
            continue
        line = np.polyfit(ys[narrow] / h, xs[narrow], 1)
        thickness = np.polyfit(ys[narrow] / h, stripe_widths[narrow], 1)
        tracks.append((line, thickness, float(ys.min()), float(ys.max())))
        used |= inliers
        if len(tracks) >= 12:
            break
    return tracks


def filter_straight_markings(mask):
    """Return (remaining_mask, removed_mask, number_of_straight_tracks).

    Inputs/outputs are single-channel uint8 masks (0/255). Scale is capped for
    predictable fitting cost; removal is intersected with the original pixels.
    """
    h, w = mask.shape
    scale = min(1.0, 640.0 / max(h, w))
    size = (max(1, round(w * scale)), max(1, round(h * scale)))
    small = cv2.resize(mask, size, interpolation=cv2.INTER_NEAREST)
    removal = np.zeros_like(small)
    count = 0
    for transpose in (False, True):
        work = small.T if transpose else small
        target = removal.T if transpose else removal
        tracks = _straight_tracks(work)
        count += len(tracks)
        height, width = work.shape
        for line, thickness, start, end in tracks:
            for y in range(max(0, int(start) - 2), min(height, int(end) + 3)):
                center = np.polyval(line, y / height)
                half = max(1.0, np.polyval(thickness, y / height) / 2) + 1.5
                left, right = max(0, int(np.floor(center - half))), min(width, int(np.ceil(center + half)) + 1)
                if right > left:
                    target[y, left:right] = 255
    removal = cv2.resize(removal, (w, h), interpolation=cv2.INTER_NEAREST)
    removed = cv2.bitwise_and(mask, removal)
    return cv2.bitwise_and(mask, cv2.bitwise_not(removal)), removed, count
