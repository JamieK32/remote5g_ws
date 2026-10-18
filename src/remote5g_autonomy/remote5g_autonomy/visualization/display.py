"""Optional OpenCV window and periodic debug-image saving."""
from pathlib import Path
import time

import cv2


class DebugDisplay:
    def __init__(self, show=False, save_dir=''):
        self.show = show
        self.directory = Path(save_dir).expanduser() if save_dir else None
        if self.directory:
            self.directory.mkdir(parents=True, exist_ok=True)
        self.last_save = 0.0
        self.image_number = 0

    def update(self, image):
        if image is None:
            return False
        if self.show:
            cv2.imshow('Lane detection / ground view', image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return True
        if self.directory and time.monotonic()-self.last_save > 2.0:
            destination = self.directory/f'lane_{self.image_number:03d}.png'
            if not cv2.imwrite(str(destination), image):
                raise OSError(f'Failed to save debug image: {destination}')
            self.last_save = time.monotonic()
            self.image_number += 1
        return False

    def close(self):
        if self.show:
            cv2.destroyAllWindows()
