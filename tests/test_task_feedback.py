import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'TonyPi task feedback requires Python 3')
class TaskTests(unittest.TestCase):
    def setUp(self):
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'humanoid/scripts'))
        import competition
        import task_feedback
        self.task, self.feedback = competition, task_feedback
        self.config = {'grasp_attempts': 3, 'settle_seconds': 0,
                       'route': [{'kind': 'forward', 'cycles': 1}],
                       'feedback': {'verify_carry': True}, 'hold_sensor_file': ''}
        from unittest.mock import Mock
        self.motion, self.heartbeat, self.approach = Mock(), Mock(), Mock()

    def run_task(self, results):
        from unittest.mock import Mock
        feedback = Mock()
        feedback.confirm.side_effect = results
        self.task.grasp_and_carry(self.config, self.motion, self.heartbeat, self.approach, feedback)
        return feedback

    def test_failed_grasp_resets_and_realigns_before_success(self):
        self.run_task([False, True, True, True, True, True])
        self.assertEqual(self.approach.call_count, 2)
        kinds = [c.args[0] for c in self.motion.action.call_args_list]
        self.assertEqual(kinds, ['grab', 'retry_reset', 'grab', 'forward', 'release'])
        states = [c.args[0] for c in self.heartbeat.set.call_args_list]
        self.assertIn('grasp_success', states)
        self.assertEqual(states[-1], 'done')

    def test_exhausted_grasp_never_starts_route(self):
        with self.assertRaisesRegex(RuntimeError, 'exhausted'):
            self.run_task([False, False, False])
        self.assertEqual(self.approach.call_count, 3)
        self.assertNotIn('forward', [c.args[0] for c in self.motion.action.call_args_list])

    def test_missing_grasp_feedback_does_not_trigger_blind_retry(self):
        with self.assertRaises(self.feedback.FeedbackUnavailable):
            self.run_task([self.feedback.FeedbackUnavailable('camera lost')])
        self.assertEqual([c.args[0] for c in self.motion.action.call_args_list], ['grab'])

    def test_drop_placement_and_release_failure_do_not_mark_done(self):
        for results, release_expected in (([True, False], False),
                                          ([True, True, False], False),
                                          ([True, True, True, False], False),
                                          ([True, True, True, True, False], True)):
            self.motion.reset_mock()
            self.heartbeat.reset_mock()
            with self.assertRaises(RuntimeError):
                self.run_task(results)
            kinds = [c.args[0] for c in self.motion.action.call_args_list]
            self.assertEqual('release' in kinds, release_expected)
            self.assertNotIn('done', [c.args[0] for c in self.heartbeat.set.call_args_list])

    def test_distance_missing_near_nonfinite_and_stale_stop(self):
        import tempfile
        import time
        from pathlib import Path
        from distance_adapter import write_sample
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'range.json'
            guard = self.feedback.DistanceGuard({'file': str(path), 'max_age': .3, 'stop_distance_m': .3})
            with self.assertRaises(self.feedback.CollisionBlocked):
                guard.check()
            for distance, age in ((.2, 0), (.3, 0), (float('nan'), 0), (0, 0), (2, 1), (2, -1)):
                write_sample(path, {'timestamp': time.time() - age, 'distance_m': distance})
                with self.assertRaises(self.feedback.CollisionBlocked):
                    guard.check()
            write_sample(path, {'timestamp': time.time(), 'distance_m': .8})
            guard.check()

    def test_sensor_failure_sentinel_is_not_clear_distance(self):
        from distance_adapter import distance_sample
        for reading in (99999, 5000, 0, None, float('nan')):
            self.assertFalse(distance_sample(reading)['valid'])
            self.assertEqual(distance_sample(reading)['distance_m'], 0)
        self.assertAlmostEqual(distance_sample(450)['distance_m'], .45)

    def test_motion_guard_blocks_before_commands_and_mid_wait(self):
        from unittest.mock import Mock
        import threading
        motion = self.task.Motion.__new__(self.task.Motion)
        motion.config = {'actions': {'forward': 'f'}, 'obstacle_guard': {'poll_seconds': .001}}
        motion.stop = threading.Event()
        motion.execute = True
        motion.controller = Mock()
        motion.frames = {'f': [(1, 20) + (500,) * 18]}
        motion.guard = Mock()
        motion.guard.check.side_effect = self.feedback.CollisionBlocked('near')
        with self.assertRaises(self.feedback.CollisionBlocked):
            motion.action('forward')
        motion.controller.set_bus_servo_pulse.assert_not_called()
        motion.guard.check.side_effect = [None, self.feedback.CollisionBlocked('near')]
        with self.assertRaises(self.feedback.CollisionBlocked):
            motion.wait(.1)
        motion.halt()
        motion.controller.board.bus_servo_stop.assert_called_once_with(list(range(1, 17)))

    def test_file_confirmation_rejects_pre_action_and_duplicate_samples(self):
        import tempfile
        import time
        from pathlib import Path
        from unittest.mock import Mock
        from distance_adapter import write_sample
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'event.json'
            spec = {'mode': 'file', 'file': str(path), 'max_age': 1, 'timeout': .03, 'stable_samples': 2}
            feedback = self.feedback.TaskFeedback({'held': spec}, {}, None, Mock(), None)
            write_sample(path, {'timestamp': time.time() - .01, 'state': 'held'})
            with self.assertRaises(self.feedback.FeedbackUnavailable):
                feedback.confirm('held')
            published = [False]
            def wait(unused):
                if not published[0]:
                    write_sample(path, {'timestamp': time.time(), 'state': 'held'})
                    published[0] = True
            feedback.motion.wait.side_effect = wait
            self.assertFalse(feedback.confirm('held'))
