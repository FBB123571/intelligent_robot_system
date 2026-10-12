import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'humanoid/scripts'))


class LaneTests(unittest.TestCase):
    def setUp(self):
        import yaml
        from lane_control import Planner
        self.config = yaml.safe_load((ROOT / 'humanoid/config/competition_vision_lane.yaml').read_text())
        self.lane = self.config['lane_following']
        self.planner = Planner(self.lane)

    def observation(self, error=0, **extra):
        result = dict(valid=True, front=False, error=error, heading=0, visible=1,
                      boundaries=[(.2,.9,.4,.5),(.8,.9,.6,.5)], image_quality=True,
                      left_open=False, right_open=False)
        result.update(extra)
        return result

    def test_center_correction_and_uncertain_view_do_not_advance(self):
        self.assertEqual(self.planner.decide(self.observation(.2)), 'right')
        self.assertEqual(self.planner.decide(self.observation(-.2)), 'left')
        self.assertEqual(self.planner.decide(self.observation(valid=False)), 'wait')
        self.assertEqual(self.planner.advance, 0)
        self.assertEqual(self.planner.decide(self.observation()), 'forward')

    def test_corner_needs_progress_side_opening_and_distinct_confirmation(self):
        self.planner.advance = 3
        corner = self.observation(valid=False, front=True, left_open=True)
        for unused in range(self.lane['stable_frames']-1):
            self.assertEqual(self.planner.decide(corner), 'wait')
        self.assertEqual(self.planner.decide(corner), 'left')
        self.assertTrue(self.planner.turning)
        self.planner.turn_cycles = self.lane['turn_min_cycles']['left']
        for unused in range(self.lane['stable_frames']):
            self.assertEqual(self.planner.decide(self.observation()), 'wait')
        self.assertEqual(self.planner.stage, 1)
        self.assertFalse(self.planner.turning)

    def test_lost_boundaries_only_end_after_both_turns_and_final_progress(self):
        empty = self.observation(valid=False, boundaries=[])
        for unused in range(10): self.assertNotEqual(self.planner.decide(empty), 'done')
        self.planner.stage = 2
        for unused in range(10): self.assertNotEqual(self.planner.decide(empty), 'done')
        self.planner.advance = self.lane['endpoint_min_forward_cycles']
        for unused in range(self.lane['endpoint_stable_frames']-1):
            self.assertEqual(self.planner.decide(empty), 'wait')
        self.assertEqual(self.planner.decide(empty), 'done')

    def test_turn_timeout_and_uncalibrated_config_rejected(self):
        from lane_control import validate
        with self.assertRaises(ValueError): validate(self.lane)
        self.planner.turning = True
        self.planner.turn_cycles = self.lane['turn_max_cycles']['left']
        with self.assertRaises(RuntimeError): self.planner.decide(self.observation(valid=False))

    def test_red_hue_wrap_and_gray_corridor_geometry(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest('OpenCV/numpy needed')
        from competition import detect
        from lane_control import observe
        for color, expected in [((0,0,200),True),((20,0,200),True),((0,200,0),False),((200,80,30),False)]:
            image = np.zeros((240,320,3),np.uint8)
            cv2.rectangle(image,(120,140),(200,210),color,-1)
            self.assertEqual(detect(image,self.config['vision'])[0] is not None,expected)
        image = np.full((240,320,3),100,np.uint8)
        cv2.line(image,(50,220),(130,120),(210,210,210),3)
        cv2.line(image,(270,220),(190,120),(210,210,210),3)
        result = observe(image,self.lane)
        self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['error'],0,delta=.03)
        self.assertFalse(observe(np.full_like(image,100),self.lane)['valid'])
