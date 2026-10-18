import os
import signal
import unittest

from remote5g_autonomy.exit_signals import ExitSignals


class ExitSignalTests(unittest.TestCase):
    def test_repeated_interrupts_are_coalesced_and_handlers_restored(self):
        previous_int = signal.getsignal(signal.SIGINT)
        previous_term = signal.getsignal(signal.SIGTERM)
        with ExitSignals() as requested:
            self.assertFalse(requested.requested)
            os.kill(os.getpid(), signal.SIGINT)
            os.kill(os.getpid(), signal.SIGINT)
            self.assertTrue(requested.requested)
            # More signals during cleanup keep the request latched.
            os.kill(os.getpid(), signal.SIGTERM)
            self.assertTrue(requested.requested)
        self.assertEqual(signal.getsignal(signal.SIGINT), previous_int)
        self.assertEqual(signal.getsignal(signal.SIGTERM), previous_term)
