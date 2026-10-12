import unittest
import sys
from pathlib import Path
from unittest.mock import Mock,patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'humanoid/scripts'))
import lane_trial


class TrialTests(unittest.TestCase):
    def test_startup_stands_then_grabs_without_visual_approach(self):
        motion,heartbeat,feedback=Mock(),Mock(),Mock()
        feedback.confirm.return_value=True
        lane_trial.startup({'settle_seconds':.2},motion,heartbeat,feedback)
        self.assertEqual([c[0][0] for c in motion.action.call_args_list],['stand','grab'])
        feedback.confirm.assert_called_once_with('held')

    def test_failed_initial_grab_does_not_start_walking(self):
        motion,heartbeat,feedback=Mock(),Mock(),Mock()
        feedback.confirm.return_value=False
        with self.assertRaises(RuntimeError):
            lane_trial.startup({'settle_seconds':.2},motion,heartbeat,feedback)
        self.assertEqual([c[0][0] for c in motion.action.call_args_list],['stand','grab'])

    def run_case(self,valid):
        import yaml
        config=yaml.safe_load((lane_trial.ROOT/'humanoid/config/lane_trial.yaml').read_text())
        clock=[0.]
        camera=Mock()
        def read(unused):
            clock[0]+=.01
            return object(),clock[0]
        camera.read_after.side_effect=read
        motion=Mock(); motion.wait.side_effect=lambda dt:clock.__setitem__(0,clock[0]+dt)
        heartbeat=Mock()
        obs=dict(valid=valid,front=False,error=0,heading=0)
        with patch.object(lane_trial,'observe',return_value=obs),patch.object(lane_trial,'roi_present',return_value=True),patch.object(lane_trial.time,'monotonic',side_effect=lambda:clock[0]):
            count=lane_trial.run_trial(config,camera,motion,heartbeat,3,2)
        return count,motion

    def test_invalid_corridor_sends_no_body_action(self):
        count,motion=self.run_case(False)
        self.assertEqual(count,0); motion.action.assert_not_called()

    def test_valid_loaded_corridor_is_bounded_and_never_grabs_or_releases(self):
        count,motion=self.run_case(True)
        self.assertEqual(count,3)
        self.assertEqual([c[0][0] for c in motion.action.call_args_list],['forward']*3)

    def test_front_barrier_refuses_forward(self):
        self.assertIsNone(lane_trial.decision(dict(valid=True,front=True),{}))
