"""Image-to-lane observations, independent of ROS messages, commands and GUI."""
from dataclasses import dataclass

import numpy as np

from .ground_lane_tracker import Boundary, GroundLaneTracker, LaneEstimate
from .white_lines import PixelDetection, detect_pixel_lane, white_mask


@dataclass
class LaneObservation:
    error: float | None = None
    target: tuple[float, float] | None = None
    estimate: LaneEstimate | None = None
    birdseye: np.ndarray | None = None
    boundaries: list[Boundary] | None = None
    pixel: PixelDetection | None = None


class LanePerception:
    def __init__(self, options, geometry):
        self.pose = None
        self.tracker = None
        if options.filter_straight:
            self.tracker = GroundLaneTracker(
                camera_x=geometry.camera_x, camera_height=geometry.camera_height,
                camera_pitch=geometry.camera_pitch, lane_width=options.lane_width,
                lookahead=options.lookahead)

    def camera_info(self, k):
        if self.tracker is not None:
            self.tracker.set_camera(k)

    def odometry(self, position, rpy):
        self.pose = (position[0], position[1], rpy[2])
        if self.tracker is not None:
            self.tracker.set_attitude(rpy[0], rpy[1], position[2])

    def process(self, bgr):
        if self.tracker is None:
            pixel = detect_pixel_lane(bgr)
            return LaneObservation(error=pixel.error, pixel=pixel)
        estimate, birdseye, boundaries, _ = self.tracker.detect(white_mask(bgr), self.pose)
        return LaneObservation(error=estimate.target[1] if estimate else None,
                               target=estimate.target if estimate else None,
                               estimate=estimate, birdseye=birdseye, boundaries=boundaries)
