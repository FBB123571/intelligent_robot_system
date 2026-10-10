import copy
import os
import sys
import unittest
import numpy as np
import cv2
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.vision import detect
from rgbd_following_core.settings import validate


class BodyTests(unittest.TestCase):
    def setUp(self):
        with open(os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/config/follow.yaml')) as f:
            self.config = yaml.safe_load(f)
        self.config['perception']['proposal_mode'] = 'depth_body'
        self.camera = (260., 260., 159.5, 119.5)

    def scene(self, centers=(160,), color=None):
        rgb = np.full((240, 320, 3), 170, dtype=np.uint8)
        y = np.mgrid[0:240, 0:320][0]
        depth = np.full(y.shape, np.nan, dtype=np.float32)
        below = y > 142
        depth[below] = .34*260/(y[below]-119.5)
        for center in centers:
            depth[120:200, center-24:center+24] = 1.
            rgb[120:160, center-24:center+24] = 230 if color is None else color
            rgb[160:200, center-24:center+24] = 35 if color is None else color
        return rgb, depth

    def candidates(self, rgb, depth):
        return detect(rgb, depth, self.camera, self.config['perception'])[0]

    def enroll(self):
        candidates = self.candidates(*self.scene())
        self.assertEqual(len(candidates), 1)
        self.config['perception']['identity']['references'] = [candidates[0]['feature']]
        validate(self.config)

    def test_black_white_body_separated_from_gray_floor(self):
        candidates = self.candidates(*self.scene())
        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0]['depth'], 1.)
        self.assertEqual(candidates[0]['feature']['kind'], 'depth_body_v1')

    def test_duplicate_depth_slices_do_not_create_ambiguity(self):
        self.enroll()
        self.assertEqual(len(self.candidates(*self.scene())), 1)

    def test_two_similar_objects_remain_ambiguous(self):
        self.enroll()
        self.assertEqual(len(self.candidates(*self.scene((80, 240)))), 2)

    def test_same_shape_different_appearance_is_rejected(self):
        self.enroll()
        self.assertEqual(self.candidates(*self.scene(color=(0, 0, 220))), [])

    def test_no_floor_never_falls_back_to_color(self):
        rgb, depth = self.scene()
        depth[:] = 1.
        self.assertEqual(self.candidates(rgb, depth), [])

    def test_invalid_depth_returns_no_target(self):
        rgb, depth = self.scene()
        depth[:] = np.nan
        self.assertEqual(self.candidates(rgb, depth), [])

    def test_floating_body_is_rejected(self):
        rgb, depth = self.scene(())
        depth[30:110, 136:184] = 1.
        self.assertEqual(self.candidates(rgb, depth), [])

    def test_legacy_color_references_cannot_enable_body_mode(self):
        self.enroll()
        del self.config['perception']['identity']['references'][0]['kind']
        with self.assertRaises(ValueError):
            validate(self.config)

    def test_invalid_silhouette_is_rejected(self):
        self.enroll()
        self.config['perception']['identity']['references'][0]['silhouette'] = [0.]
        with self.assertRaises(ValueError):
            validate(self.config)


if __name__ == '__main__':
    unittest.main()
