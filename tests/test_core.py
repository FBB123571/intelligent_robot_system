# -*- coding: utf-8 -*-
"""Core tests run on Python 2.7 (Kinetic) and Python 3, without ROS."""
import copy
import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot', 'follow2', 'multi_robot_following', 'src'))
from competition_core.planning import Trail, fresh, grid_clear, scan_clear, localization_ok
from competition_core.mission import Approach, validate_route


class TrailTests(unittest.TestCase):
    def test_corner_uses_arc_length(self):
        trail = Trail(jump_limit=2)
        for p in [(0, 0), (1, 0), (1, 1)]:
            self.assertTrue(trail.add(*p))
        x, y, yaw = trail.behind(1.5)
        self.assertAlmostEqual(x, 0.5)
        self.assertAlmostEqual(y, 0)
        self.assertAlmostEqual(yaw, 0)

    def test_start_waits_for_enough_trail(self):
        trail = Trail()
        trail.add(0, 0)
        trail.add(0.2, 0)
        self.assertIsNone(trail.behind(0.9))

    def test_jump_and_nan_do_not_poison_trail(self):
        trail = Trail()
        trail.add(0, 0)
        self.assertFalse(trail.add(9, 9))
        self.assertFalse(trail.add(float('nan'), 1))
        self.assertEqual(trail.points, [(0., 0.)])

    def test_stationary_jitter_not_added(self):
        trail = Trail()
        trail.add(0, 0)
        self.assertTrue(trail.add(.001, .001))
        self.assertEqual(len(trail.points), 1)

    def test_history_is_bounded(self):
        trail = Trail(max_length=1)
        for i in range(40):
            trail.add(i * 0.1, 0)
        self.assertLessEqual(len(trail.points), 12)


class GuardTests(unittest.TestCase):
    def test_timestamp_old_future_or_missing(self):
        self.assertTrue(fresh(9.5, 10, 1))
        for stamp in (None, 8, 11, float('nan')):
            self.assertFalse(fresh(stamp, 10, 1))

    def test_grid_blocks_unknown_obstacle_and_boundary(self):
        data = [0] * 100
        def clear(x, y):
            return grid_clear(data, 10, 10, 0.1, 0, 0, 0, x, y, .15)
        self.assertTrue(clear(.5, .5))
        self.assertFalse(clear(.02, .5))
        data[55] = -1
        self.assertFalse(clear(.5, .5))
        data[55] = 100
        self.assertFalse(clear(.5, .5))

    def test_rotated_grid_origin(self):
        self.assertTrue(grid_clear([0] * 100, 10, 10, .1, 1, 2, math.pi / 2, .5, 2.5, .1))

    def test_invalid_grid(self):
        self.assertFalse(grid_clear([], 10, 10, .1, 0, 0, 0, .5, .5, .1))

    def test_scan_missing_is_not_clear(self):
        args = (-.2, .1, .1, 10, .5, .3)
        self.assertTrue(scan_clear([2] * 5, *args))
        self.assertFalse(scan_clear([2, 2, .3, 2, 2], *args))
        self.assertFalse(scan_clear([float('inf')] * 5, *args))
        self.assertFalse(scan_clear([float('nan')] * 5, *args))
        self.assertFalse(scan_clear([0] * 5, *args))

    def test_localization_covariance(self):
        covariance = [0.] * 36
        self.assertTrue(localization_ok(covariance, .25, .35))
        for value in (.5, -1, float('nan')):
            covariance[0] = value
            self.assertFalse(localization_ok(covariance, .25, .35))


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.config = dict(center_x=.5, x_tolerance=.06, grab_bottom=.88,
                           grab_area=.025, stable_frames=3, stability_tolerance=.06)

    def test_alignment_and_approach(self):
        planner = Approach(self.config)
        self.assertEqual(planner.decide((.3, .9, .1)), 'align_left')
        self.assertEqual(planner.decide((.7, .9, .1)), 'align_right')
        self.assertEqual(planner.decide((.5, .5, .01)), 'approach')

    def test_grab_requires_consecutive_fresh_observations(self):
        planner = Approach(self.config)
        self.assertEqual(planner.decide((.5, .9, .1)), 'wait')
        self.assertEqual(planner.decide((.5, .9, .1)), 'wait')
        self.assertEqual(planner.decide(None), 'wait')
        self.assertEqual(planner.decide((.5, .9, .1)), 'wait')
        self.assertEqual(planner.decide((.5, .9, .1)), 'wait')
        self.assertEqual(planner.decide((.5, .9, .1)), 'grab')

    def test_jitter_resets_grasp_readiness(self):
        planner = Approach(self.config)
        planner.decide((.5, .9, .1))
        planner.decide((.5, .9, .1))
        self.assertEqual(planner.decide((.5, 1., .1)), 'wait')

    def test_route_requires_both_turn_directions(self):
        config = dict(actions=dict(grab='g', release='r', approach='a', align_left='l', align_right='r',
                                   forward='f', left='l', right='r'),
                      route=[dict(kind='left', cycles=1), dict(kind='right', cycles=1)])
        self.assertTrue(validate_route(config))
        bad = copy.deepcopy(config)
        bad['route'].pop()
        with self.assertRaises(ValueError):
            validate_route(bad)
        for count in (0, -1, True, 1.5):
            bad = copy.deepcopy(config)
            bad['route'][0]['cycles'] = count
            with self.assertRaises(ValueError):
                validate_route(bad)


if __name__ == '__main__':
    unittest.main()
