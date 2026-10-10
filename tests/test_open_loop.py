import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Pi entry requires Python 3')
class OpenLoopTests(unittest.TestCase):
    def test_requested_order_and_unmodified_action_dispatch(self):
        from pathlib import Path
        from unittest.mock import Mock
        import yaml
        root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root / 'humanoid/scripts'))
        import open_loop
        config = yaml.safe_load((root / 'humanoid/config/open_loop.yaml').read_text())
        expected = ['stand', 'grab'] + ['forward'] * 10 + ['left'] * 10 + ['forward'] * 10 + ['right'] * 10 + ['forward'] * 10 + ['release']
        self.assertEqual(open_loop.plan(config), expected)
        motion = Mock()
        open_loop.run(expected, motion)
        self.assertEqual([call[0][0] for call in motion.action.call_args_list], expected)
        shared = open_loop.motion_config(config)
        self.assertEqual(shared['hold_arm_servos'], [])
        self.assertFalse(shared['obstacle_guard']['enabled'])
        self.assertEqual(shared['actions']['left'], '/home/pi/tonypi_web_control/actions/carry_left')
        self.assertEqual(shared['actions']['right'], '/home/pi/tonypi_web_control/actions/carry_right')
        self.assertEqual(shared['actions']['grab'], '/home/pi/tonypi_web_control/actions/grab')
        self.assertEqual(shared['actions']['retry_reset'], shared['actions']['stand'])
