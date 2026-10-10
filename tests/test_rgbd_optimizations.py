"""Behavioral regressions for identity, camera coverage, inertia and evidence."""
import copy
import math
import os
import sys
import unittest
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.tracking import Tracker, Completion
from rgbd_following_core.motion import Grid, stopping_clear
from rgbd_following_core.visibility import visible_path


class IdentityRecoveryTests(unittest.TestCase):
    def tracker(self):
        t = Tracker(confirmations=2, reacquire_confirmations=3)
        t.update([(1.,0.,.3)], 1., [True])
        t.update([(1.,0.,.3)], 1.1, [True])
        t.update([], 1.2)
        return t

    def test_verified_target_recovers_beyond_old_radius_after_confirmation(self):
        t = self.tracker()
        for stamp in (3.,3.1):
            self.assertIsNone(t.update([(1.6,0.,.3)], stamp, [True]))
            self.assertEqual(t.anchor, (1.,0.,.3))
        self.assertEqual(t.update([(1.6,0.,.3)], 3.2, [True]), (1.6,0.,.3))
        self.assertTrue(t.reacquired)

    def test_unverified_far_candidate_and_two_verified_candidates_never_recover(self):
        for verified, candidates in (([False], [(1.6,0.,.3)]), ([True,True], [(1.6,0.,.3),(1.1,0.,.3)])):
            t = self.tracker()
            for stamp in (3.,3.1,3.2,3.3):
                self.assertIsNone(t.update(candidates, stamp, verified))
            self.assertEqual(t.anchor, (1.,0.,.3))

    def test_recovery_confirmation_cannot_accumulate_across_missing_frames(self):
        t = self.tracker()
        t.update([(1.6,0.,.3)], 3., [True])
        t.update([], 3.1)
        t.update([(1.6,0.,.3)], 3.2, [True])
        self.assertIsNone(t.update([(1.6,0.,.3)], 3.3, [True]))

    def test_unconfirmed_initial_distractor_expires(self):
        t = Tracker(confirmations=2)
        t.update([(4.,0.,.3)], 1.)
        self.assertIsNone(t.update([(1.,0.,.3)], 2.))
        self.assertEqual(t.update([(1.,0.,.3)], 2.1), (1.,0.,.3))

    def test_unverified_recovery_does_not_walk_anchor_toward_distractor(self):
        t = self.tracker()
        self.assertIsNone(t.update([(1.4,0.,.3)], 3., [False]))
        self.assertIsNone(t.update([(1.5,0.,.3)], 3.1, [False]))
        self.assertEqual(t.anchor,(1.,0.,.3))


class VisibilityAndInertiaTests(unittest.TestCase):
    def visible(self, target, v, w):
        return visible_path((0.,0.,0.), target, v, w, (250.,250.,160.,120.,320,240),
                            ((0.,.5,0.),(.5,-.5,.5,.5)), 1., .12)

    def test_turn_away_loses_target_but_scaled_motion_retains_it(self):
        target = (1.,.18,.5)
        self.assertTrue(self.visible(target, .1, 0.))
        self.assertFalse(self.visible(target, .1, -.4))
        self.assertTrue(self.visible(target, .03, -.12))

    def test_behind_camera_and_outside_vertical_fov_rejected(self):
        self.assertFalse(self.visible((-1.,0.,.5), 0.,0.))
        self.assertFalse(self.visible((1.,0.,2.), 0.,0.))

    def test_zero_new_command_still_checks_actual_inertia(self):
        g = Grid([0]*10000,100,100,.02)
        g.data[50*100+70] = 100
        self.assertTrue(stopping_clear(g,(1.,1.,0.),0.,0.,.26,.15,.3))
        self.assertFalse(stopping_clear(g,(1.,1.,0.),.3,0.,.26,.15,.3))

    def test_angular_curvature_change_cannot_escape_stopping_disk(self):
        g = Grid([0]*10000,100,100,.02)
        g.data[69*100+50] = 100
        self.assertFalse(stopping_clear(g,(1.,1.,0.),.3,.03,.26,.15,.3))

    def test_finish_hold_starts_only_after_arrival_and_actual_stop(self):
        c = Completion(hold=2.)
        poly = [[0,0],[4,0],[4,4],[0,4]]
        def ready(t, robot=(1.,1.), stopped=True):
            return c.ready('done',(2.,1.),t,robot,(1.,1.),.15,poly,.26,stopped)
        self.assertFalse(ready(1.,(.5,1.)))
        self.assertFalse(ready(4.))
        self.assertFalse(ready(6.,stopped=False))
        self.assertFalse(ready(7.))
        self.assertTrue(ready(9.1))


@unittest.skipIf(sys.version_info[0] < 3, 'Report test loader uses Python 3')
class EvidenceTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('trial_report', os.path.join(ROOT,'tools/evaluate_rgbd_trials.py'))
        self.tool = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.tool)
        self.meta = dict(kind='metadata', scenario='occlusion', settings=dict(lost_failure_seconds=60., stopped_linear=.015, stopped_angular=.03))
        self.verdict = dict((k,True) for k in self.tool.VERDICTS)

    def records(self):
        return [self.meta] + [dict(kind='sample',t=i*.1,mission='done',target_valid=not 10<=i<20,
            odom_fresh=True,grid_fresh=True,speed=0.,angular_speed=0.,follow_status='finished' if i>30 else 'following',
            euclidean_gap_error_m=.1,costmap_clearance_m=.2) for i in range(40)]

    def test_recovery_interval_and_measured_metrics(self):
        result = self.tool.summarize(self.records(),self.verdict)
        self.assertTrue(result['accepted'])
        self.assertEqual(result['reacquisition_seconds'],[1.])
        self.assertAlmostEqual(result['euclidean_gap_error_p95_m'],.1)

    def test_telemetry_cannot_award_success_without_human_evidence(self):
        self.assertFalse(self.tool.summarize(self.records())['accepted'])

    def test_gaps_moving_finish_and_unfinished_loss_fail(self):
        for change in ('gap','moving','loss','stale'):
            rows = self.records()
            if change == 'gap':
                rows[-1]['t'] += 4
            elif change == 'moving':
                for r in rows[1:]: r['speed'] = .1
            elif change == 'loss':
                rows[0]['settings']['lost_failure_seconds'] = .5
            else:
                for r in rows[1:]: r['odom_fresh'] = False
            self.assertFalse(self.tool.summarize(rows,self.verdict)['accepted'],change)

    def test_team_finish_requires_separate_whole_team_evidence(self):
        rows = self.records()
        rows[0]['scenario'] = 'team_finish'
        self.assertFalse(self.tool.summarize(rows,self.verdict)['accepted'])
        self.verdict['whole_team_finished'] = True
        self.assertTrue(self.tool.summarize(rows,self.verdict)['accepted'])


if __name__ == '__main__':
    unittest.main()
