"""Coalesce terminal/launch exit signals without interrupting cleanup."""
import signal


class ExitSignals:
    def __init__(self):
        self.requested = False
        self.previous = {}

    def _request(self, signum, frame):
        self.requested = True

    def __enter__(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            self.previous[signum] = signal.signal(signum, self._request)
        return self

    def __exit__(self, exc_type, exc, traceback):
        for signum, handler in self.previous.items():
            signal.signal(signum, handler)
