# -*- coding: utf-8 -*-
import os
import sys
import unittest

@unittest.skipIf(sys.version_info[0] < 3, 'Pi program requires Python 3')
class HumanoidTests(unittest.TestCase):
    def setUp(self):
        from pathlib import Path
        self.root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(self.root / 'humanoid/scripts'))
        import competition
        self.module = competition

    def test_source_and_candidate_actions_have_valid_frames(self):
        for path in (self.root / 'humanoid/ActionGroup').rglob('*.d6a'):
            self.assertTrue(self.module.action_frames(path))

    def test_legacy_backend_never_imports_controller(self):
        import threading
        from unittest.mock import Mock, patch
        config = {'actions': {}, 'obstacle_guard': {}, 'hardware_backend': 'legacy'}
        board = Mock()
        with patch.object(self.module, 'validate_route', return_value=[]), \
                patch.object(self.module.importlib, 'import_module', return_value=board) as load:
            motion = self.module.Motion(config, threading.Event(), True)
        load.assert_called_once_with('hiwonder.Board')
        self.assertIs(motion.old_board, board)
        self.assertIsNone(motion.controller)

    def test_stop_sends_no_additional_servo_commands(self):
        import threading
        motion = self.module.Motion.__new__(self.module.Motion)
        motion.config = {'actions': {'forward': 'f'}}
        motion.stop = threading.Event()
        motion.stop.set()
        with self.assertRaises(self.module.Stopped):
            motion.action('forward')

    def test_carrying_keeps_grasp_arm_pulses(self):
        import threading
        class Controller(object):
            def __init__(self):
                self.sent = []
            def set_bus_servo_pulse(self, servo, pulse, duration):
                self.sent.append((servo, pulse))
        motion = self.module.Motion.__new__(self.module.Motion)
        motion.config = {'actions': {'forward': 'f'}, 'hold_sensor_file': ''}
        motion.stop = threading.Event()
        motion.execute = True
        motion.controller = Controller()
        motion.frames = {'f': [(1, 20) + (500,) * 18]}
        motion.hold_values = {6: 574, 7: 921, 8: 477, 14: 392, 15: 79, 16: 596}
        motion.action('forward')
        actual = dict(motion.controller.sent)
        self.assertEqual(len(actual), 16)
        for servo, pulse in motion.hold_values.items():
            self.assertEqual(actual[servo], pulse)
        self.assertEqual(actual[1], 500)

    def test_drop_sensor_missing_or_dropped_blocks(self):
        import tempfile
        import time
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'hold.txt'
            config = {'hold_sensor_file': str(path), 'hold_sensor_timeout': 1}
            with self.assertRaises(RuntimeError):
                self.module.check_hold(config)
            path.write_text('dropped')
            with self.assertRaises(RuntimeError):
                self.module.check_hold(config)
            path.write_text('held')
            self.module.check_hold(config)
            os.utime(str(path), (time.time() - 2, time.time() - 2))
            with self.assertRaises(RuntimeError):
                self.module.check_hold(config)


if __name__ == '__main__':
    unittest.main()
