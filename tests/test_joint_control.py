import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Joint controller requires Python 3')
class JointTests(unittest.TestCase):
    def setUp(self):
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'humanoid/scripts'))
        from joint_control import JointControl
        self.klass = JointControl
        self.config = {'hardware_verified': True, 'degrees_per_unit': .24,
                       'joints': {'a': {'id': 6, 'zero': 500, 'direction': 1, 'min': 400, 'max': 650},
                                  'b': {'id': 14, 'zero': 500, 'direction': -1, 'min': 400, 'max': 650}}}

    def test_angle_direction_and_roundtrip(self):
        c = self.klass(self.config)
        self.assertEqual(c.position(c.joint('a'), 24), 600)
        self.assertEqual(c.position(c.joint('b'), 24), 400)
        self.assertEqual(c.angle(c.joint('b'), 400), 24)

    def test_average_speed_sets_duration(self):
        command = self.klass(self.config).move_joint_at_speed('a', 0, 24, 12)
        self.assertEqual(command['duration_ms'], 2000)
        self.assertEqual(command['positions'], [[6, 600]])

    def test_no_travel_sends_nothing(self):
        from unittest.mock import Mock
        board = Mock()
        c = self.klass(self.config, execute=True, board=board)
        c.move_joint_at_speed('a', 0, 0, 12)
        board.bus_servo_set_position.assert_not_called()

    def test_group_sent_in_one_packet_with_seconds(self):
        from unittest.mock import Mock
        board = Mock()
        c = self.klass(self.config, execute=True, board=board)
        c.move_joint_group({'a': 600, 'b': 400}, 1000)
        board.bus_servo_set_position.assert_called_once_with(1., [[6, 600], [14, 400]])

    def test_unverified_limits_and_invalid_values_block_execution(self):
        import copy
        bad = copy.deepcopy(self.config)
        bad['hardware_verified'] = False
        with self.assertRaises(ValueError):
            self.klass(bad, execute=True)
        bad['hardware_verified'] = True
        bad['joints']['a']['min'] = None
        with self.assertRaises(ValueError):
            self.klass(bad, execute=True).move_joint_group({'a': 500}, 1000)
        for value in (700, -1, float('nan'), 500.5):
            with self.assertRaises(ValueError):
                self.klass(self.config).move_joint_group({'a': value}, 1000)

    def test_duplicate_id_alias_blocked(self):
        with self.assertRaises(ValueError):
            self.klass(self.config).move_joint_group({'a': 500, '6': 550}, 1000)


if __name__ == '__main__':
    unittest.main()
