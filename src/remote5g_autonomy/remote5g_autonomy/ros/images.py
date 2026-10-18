"""Convert a ROS Image-compatible object to BGR, including padded rows."""
import cv2
import numpy as np


def decode_bgr(msg):
    if msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width*3:
        raise ValueError(f'Unsupported image encoding/step: {msg.encoding}')
    if msg.width <= 0 or msg.height <= 0:
        raise ValueError('Image dimensions must be positive')
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = image[:, :msg.width*3].reshape(msg.height, msg.width, 3)
    return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else image
