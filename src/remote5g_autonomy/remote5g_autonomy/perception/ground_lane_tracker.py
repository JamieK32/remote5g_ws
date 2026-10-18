"""Calibrated ground-plane lane fitting; no track map input.

Coordinates are metres: x forward from the rear axle, y to the vehicle's left.
Odometry is used only for attitude compensation and inter-frame continuity.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass
class Boundary:
    coefficients: np.ndarray
    support: int
    start: float
    end: float
    residual: float

    def at(self, x):
        return np.polyval(self.coefficients, x)


@dataclass
class LaneEstimate:
    center: np.ndarray
    left: Boundary
    right: Boundary
    target: tuple
    width: float


def rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class GroundLaneTracker:
    def __init__(self, camera_x=0.3, camera_height=0.235315,
                 camera_pitch=0.45, lane_width=1.22, lookahead=2.5):
        self.camera_origin = np.array([camera_x, 0.0, camera_height])
        self.camera_pitch = camera_pitch
        self.lane_width = lane_width
        self.lookahead = lookahead
        self.k = None
        self.previous = None
        self.previous_pose = None
        self.xs = np.arange(0.65, 10.01, 0.05)
        self.ys = np.arange(4.0, -4.001, -0.025)
        gx, gy = np.meshgrid(self.xs, self.ys, indexing='ij')
        self.ground = np.stack([gx, gy, np.zeros_like(gx)], axis=-1)
        self.rotation = rotation_y(camera_pitch)
        self.origin = self.camera_origin.copy()

    def set_camera(self, k):
        matrix = np.asarray(k, dtype=float).reshape(3, 3)
        if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
            raise ValueError('Invalid camera intrinsics')
        self.k = matrix

    def set_attitude(self, roll=0.0, pitch=0.0, height=0.0):
        c, s = math.cos(roll), math.sin(roll)
        body = rotation_y(pitch) @ np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        self.rotation = body @ rotation_y(self.camera_pitch)
        self.origin = body @ self.camera_origin + np.array([0.0, 0.0, height])

    def project(self, ground):
        """Ground coordinates to pixels, including camera mounting/vehicle tilt."""
        camera = (np.asarray(ground) - self.origin) @ self.rotation
        depth = camera[..., 0]
        valid = depth > 0.05
        depth = np.where(valid, depth, 1.0)
        u = self.k[0, 2] - self.k[0, 0] * camera[..., 1] / depth
        v = self.k[1, 2] - self.k[1, 1] * camera[..., 2] / depth
        return np.where(valid, u, -1).astype(np.float32), np.where(valid, v, -1).astype(np.float32)

    def birdseye(self, mask):
        u, v = self.project(self.ground)
        return cv2.remap(mask, u, v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    def points(self, birdseye):
        points = []
        for row in range(0, len(self.xs), 2):
            # Broad crossing blobs are excluded instead of becoming lane centers.
            profile = (birdseye[max(0, row-1):row+2] > 100).mean(axis=0) > 0.45
            edges = np.diff(np.r_[False, profile, False].astype(np.int8))
            starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
            for start, end in zip(starts, ends):
                if 1 <= end-start <= 9:
                    y = np.interp((start+end-1)/2, np.arange(len(self.ys)), self.ys)
                    points.append((self.xs[row], y))
        return np.asarray(points, dtype=float).reshape(-1, 2)

    def boundaries(self, points, prediction=None):
        if len(points) < 35:
            return []
        rng = np.random.default_rng(31)
        triples = rng.integers(len(points), size=(2400, 3))
        sample = points[triples]
        ordered = np.sort(sample[:, :, 0], axis=1)
        valid = (np.diff(ordered, axis=1).min(axis=1) > 1.3) & (np.ptp(ordered, axis=1) > 4.5)
        sample = sample[valid]
        if not len(sample):
            return []
        design = np.stack([sample[:, :, 0]**2, sample[:, :, 0], np.ones(sample.shape[:2])], axis=-1)
        coefficients = np.linalg.solve(design, sample[:, :, 1, None])[:, :, 0]
        valid = (np.abs(coefficients[:, 0]) < 0.12) & (np.abs(coefficients[:, 1]) < 1.0)
        coefficients = coefficients[valid]
        if prediction is not None:
            seeds = []
            for sign in (-1, 1):
                seed = prediction.copy()
                seed[2] += sign*self.lane_width/2
                seeds.append(seed)
            coefficients = np.vstack([seeds, coefficients])
        if not len(coefficients):
            return []
        design_all = np.stack([points[:, 0]**2, points[:, 0], np.ones(len(points))], axis=0)
        residuals = np.abs(coefficients @ design_all - points[:, 1])
        tolerance = 0.025 + 0.006 * points[:, 0]
        scores = (residuals < tolerance).sum(axis=1).astype(float)
        if prediction is not None:
            scores[:2] += 15
        results = []
        for idx in np.argsort(scores)[::-1]:
            keep = residuals[idx] < tolerance
            if keep.sum() < 28:
                continue
            for _ in range(2):
                if keep.sum() < 20:
                    break
                p = points[keep]
                # Weight close measurements more; far pixels cover more ground.
                coefficient = np.polyfit(p[:, 0], p[:, 1], 2, w=1/(0.5+p[:, 0]))
                error = np.abs(np.polyval(coefficient, points[:, 0]) - points[:, 1])
                keep = error < tolerance
            p = points[keep]
            if len(p) < 28 or np.ptp(p[:, 0]) < 4.0 or p[:, 0].min() > 2.2:
                continue
            coverage = len(np.unique(p[:, 0])) / (np.ptp(p[:, 0])/0.1+1)
            if coverage < 0.60:
                continue
            if abs(coefficient[0]) > 0.12 or abs(coefficient[1]) > 1.0:
                continue
            # Intersecting straight/curved markings share near-field pixels.
            # They may support both hypotheses; consuming them greedily would
            # erase the arc before it can be paired with its other boundary.
            if any(np.mean(np.abs(np.polyval(coefficient, [1, 3, 6, 9]) -
                                  b.at([1, 3, 6, 9]))) < 0.08 for b in results):
                continue
            results.append(Boundary(coefficient, len(p), p[:, 0].min(), p[:, 0].max(),
                                    float(np.median(error[keep]))))
            if len(results) >= 16:
                break
        return results

    def predicted_center(self, pose):
        if self.previous is None or pose is None or self.previous_pose is None:
            return None
        px, py, yaw = self.previous_pose
        nx, ny, next_yaw = pose
        sample_x = np.linspace(0.4, 10.0, 40)
        sample_y = np.polyval(self.previous, sample_x)
        world_x = px + math.cos(yaw)*sample_x - math.sin(yaw)*sample_y
        world_y = py + math.sin(yaw)*sample_x + math.cos(yaw)*sample_y
        dx, dy = world_x-nx, world_y-ny
        local_x = math.cos(next_yaw)*dx + math.sin(next_yaw)*dy
        local_y = -math.sin(next_yaw)*dx + math.cos(next_yaw)*dy
        if math.hypot(nx-px, ny-py) > 1.5:
            return None
        return np.polyfit(local_x, local_y, 2)

    def choose(self, boundaries, pose=None):
        prediction = self.predicted_center(pose)
        pairs = []
        for left in boundaries:
            for right in boundaries:
                start, end = max(left.start, right.start), min(left.end, right.end, 8.0)
                if end-start < 3.5:
                    continue
                check_x = np.linspace(max(1.0, start), end, 12)
                widths = left.at(check_x) - right.at(check_x)
                if np.min(widths) < self.lane_width*0.73 or np.max(widths) > self.lane_width*1.28:
                    continue
                if abs(left.coefficients[0]-right.coefficients[0]) > 0.009:
                    continue
                center = (left.coefficients+right.coefficients)/2
                near = abs(np.polyval(center, 0.8))
                if near > self.lane_width*0.65:
                    continue
                curved = abs(center[0]) > 0.0045
                score = near*2.0 + np.mean(np.abs(widths-self.lane_width))*3.0
                score += 2*(left.residual+right.residual)
                score -= 0.002*(left.support+right.support)
                # At crossings, an established curved lane beats a parallel
                # straight pair. Ordinary straight lanes remain usable elsewhere.
                score -= 0.35 if curved else 0.0
                if prediction is not None:
                    score += 3*np.mean(np.abs(np.polyval(center, [1, 3, 6, 9]) -
                                              np.polyval(prediction, [1, 3, 6, 9])))
                pairs.append((score, center, left, right, float(np.mean(widths))))
        if not pairs:
            return None
        _, center, left, right, width = min(pairs, key=lambda p:p[0])
        if prediction is not None:
            center = 0.65*center + 0.35*prediction
        x = min(self.lookahead, left.end, right.end)
        y = float(np.polyval(center, x))
        self.previous, self.previous_pose = center.copy(), pose
        return LaneEstimate(center, left, right, (x,y), width)

    def detect(self, mask, pose=None):
        if self.k is None:
            return None, None, [], np.empty((0, 2))
        birdseye = self.birdseye(mask)
        points = self.points(birdseye)
        boundaries = self.boundaries(points, self.predicted_center(pose))
        return self.choose(boundaries, pose), birdseye, boundaries, points

