"""Rendering only; detectors and control never open windows."""
import math

import cv2
import numpy as np

from remote5g_autonomy.perception.white_lines import SCAN_ROWS


def render_pixel(bgr, detection):
    debug = bgr.copy()
    h, w = bgr.shape[:2]
    cv2.line(debug, (w//2, 0), (w//2, h-1), (0, 220, 255), 2)
    for row in SCAN_ROWS:
        y = round(h*row)
        cv2.line(debug, (0, y), (w-1, y), (100, 100, 100), 1)
    for row, left, right in detection.matches:
        y = round(h*row)
        for x, color in ((left, (255, 100, 0)), (right, (255, 100, 0)),
                         ((left+right)/2, (0, 255, 0))):
            cv2.circle(debug, (round(x), y), 6, color, -1)
    error = detection.error
    status = 'NO TWO-LINE MATCH' if error is None else f'error={error:+.3f}'
    cv2.putText(debug, status, (12, h-24), cv2.FONT_HERSHEY_SIMPLEX,
                0.85, (0, 255, 0) if error is not None else (0, 0, 255), 2)
    mask = cv2.cvtColor(detection.mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(mask, 'WHITE MASK', (12, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return cv2.hconcat([debug, mask])


def render_ground(tracker, bgr, estimate, birdseye, boundaries, steering):
    debug = bgr.copy()
    h, w = bgr.shape[:2]
    if birdseye is None:
        cv2.putText(debug, 'WAITING FOR /camera/front/camera_info', (12, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return debug
    ground = cv2.cvtColor(birdseye, cv2.COLOR_GRAY2BGR)

    def draw_model(coefficient, start, end, color, thickness):
        x = np.linspace(start, end, 160)
        y = np.polyval(coefficient, x)
        u, v = tracker.project(np.column_stack([x, y, np.zeros_like(x)]))
        pixels = np.column_stack([u, v]).astype(np.int32)
        cv2.polylines(debug, [pixels], False, color, thickness)
        pixels = np.column_stack([(4-y)/0.025, (x-0.65)/0.05]).astype(np.int32)
        cv2.polylines(ground, [pixels], False, color, thickness)

    # Only reliable straight hypotheses are displayed as rejected lines.
    for b in boundaries:
        if abs(b.coefficients[0]) < 0.003 and b.support >= 50:
            draw_model(b.coefficients, b.start, b.end, (0, 0, 255), 2)
    if estimate is not None:
        for b in (estimate.left, estimate.right):
            draw_model(b.coefficients, b.start, b.end, (0, 255, 0), 3)
        draw_model(estimate.center, 0.65, min(estimate.left.end, estimate.right.end),
                   (255, 255, 0), 2)
        x, y = estimate.target
        u, v = tracker.project(np.array([[x, y, 0.0]]))
        cv2.circle(debug, (round(float(u[0])), round(float(v[0]))), 8, (0, 255, 255), -1)
        mode = 'CURVE' if abs(estimate.center[0]) > 0.0045 else 'STRAIGHT'
        status = (f'{mode}  width={estimate.width:.2f}m  '
                  f'target=({x:.1f},{y:+.2f})m  steer={math.degrees(steering):+.2f}deg')
    else:
        status = 'NO CONSISTENT GROUND LANE PAIR - STOP'
    cv2.putText(debug, status, (12, h-22), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 255, 0) if estimate else (0, 0, 255), 2)
    cv2.putText(debug, 'GREEN: SELECTED BORDERS / CYAN: CENTER / RED: STRAIGHT CANDIDATES',
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2)
    panel = cv2.flip(ground, 0)
    panel = cv2.resize(panel, (round(h*8/9.35), h), interpolation=cv2.INTER_NEAREST)
    cv2.putText(panel, 'GROUND VIEW / FAR: 10m', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    cv2.putText(panel, 'NEAR: 0.65m', (10, h-20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
    return cv2.hconcat([debug, panel])
