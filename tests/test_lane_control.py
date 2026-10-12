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
        self.lane['endpoint_mode']='boundaries_disappear'  # optional legacy strategy
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
        cv2.line(image,(50,220),(50,90),(30,30,30),3)
        cv2.line(image,(270,220),(270,90),(30,30,30),3)
        result = observe(image,self.lane)
        self.assertTrue(result['valid'])
        self.assertAlmostEqual(result['error'],0,delta=.03)
        self.assertFalse(observe(np.full_like(image,100),self.lane)['valid'])

    def test_floor_tape_cannot_supply_panel_or_cone_evidence(self):
        try:
            import cv2
            import numpy as np
        except ImportError: self.skipTest('OpenCV/numpy needed')
        from lane_control import observe,cone_candidate
        from vision_guard import floor_obstacle
        image=np.full((240,320,3),150,np.uint8)
        # Both floor-only seams and diagonal yellow/black tape are distractors.
        cv2.line(image,(80,239),(130,105),(20,20,20),3)
        cv2.line(image,(20,180),(300,180),(20,20,20),3)
        for y in range(100,230,20):
            x=140+(y-100)//2
            cv2.line(image,(x,y),(x+10,y+18),(0,230,230) if y%40==20 else (10,10,10),10)
        result=observe(image,self.lane)
        self.assertFalse(result['valid'])
        self.assertFalse(result['front'])
        self.assertIsNone(cone_candidate(image,self.lane['cone']))
        self.assertFalse(floor_obstacle(image,self.config['obstacle_guard'],self.config['vision'],None,'forward')[0])

    def test_yellow_cone_shape_and_final_stage_gate(self):
        try:
            import cv2
            import numpy as np
        except ImportError: self.skipTest('OpenCV/numpy needed')
        from lane_control import cone_candidate
        image=np.full((240,320,3),120,np.uint8)
        cv2.fillPoly(image,[np.array([[150,70],[170,70],[190,190],[130,190]])],(0,220,220))
        cone=cone_candidate(image,self.lane['cone']); self.assertIsNotNone(cone)
        obs=self.observation(cone=cone)
        for unused in range(12): self.assertNotEqual(self.planner.decide(obs),'done')
        self.planner.stage=2; self.planner.advance=3
        for unused in range(self.lane['endpoint_stable_frames']-1):
            self.assertEqual(self.planner.decide(obs),'wait')
        self.assertEqual(self.planner.decide(obs),'done')

    def test_red_held_object_cannot_replace_gray_panels(self):
        try:
            import cv2
            import numpy as np
        except ImportError: self.skipTest('OpenCV/numpy needed')
        from lane_control import observe
        image=np.full((240,320,3),100,np.uint8)
        for vertices in ([[0,0],[130,0],[130,120],[50,220],[0,220]],
                         [[190,0],[320,0],[320,220],[270,220],[190,120]]):
            cv2.fillPoly(image,[np.array(vertices)],(0,0,220))
        cv2.line(image,(50,220),(50,90),(20,20,20),3)
        cv2.line(image,(270,220),(270,90),(20,20,20),3)
        self.assertFalse(observe(image,self.lane)['valid'])
