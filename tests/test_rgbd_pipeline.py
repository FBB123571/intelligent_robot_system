"""Actual OpenCV + NumPy synthetic perception, not hardware validation."""
import copy
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))


class VisionPipelineTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2
            import numpy as np
            import yaml
        except ImportError:
            self.skipTest('Install OpenCV, NumPy and PyYAML for RGB-D pipeline tests')
        from rgbd_following_core import vision
        from rgbd_following_core.settings import validate
        self.np, self.cv2, self.vision, self.validate = np, cv2, vision, validate
        with open(os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/config/follow.yaml')) as stream:
            self.c = yaml.safe_load(stream)
        self.validate(self.c)
        self.rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        self.rgb[90:150, 130:190] = (0, 220, 0)
        self.depth = np.full((240, 320), 2.5, dtype=np.float32)
        self.depth[90:150, 130:190] = 1.2
        self.camera = (250., 250., 160., 120.)

    def detect(self):
        return self.vision.detect(self.rgb, self.depth, self.camera, self.c['perception'])[0]

    def test_color_and_depth_produce_measured_3d_candidate(self):
        items = self.detect()
        self.assertEqual(len(items), 1)
        self.assertAlmostEqual(items[0]['point'][2], 1.2, places=5)
        self.assertLess(abs(items[0]['point'][0]), .01)

    def test_blank_wrong_color_invalid_and_mixed_depth_rejected(self):
        for mode in ('blank', 'wrong_color', 'invalid_depth', 'mixed_depth'):
            rgb, depth = self.rgb.copy(), self.depth.copy()
            if mode == 'blank':
                self.rgb[:] = 0
            elif mode == 'wrong_color':
                self.rgb[:] = (220, 0, 0)
            elif mode == 'invalid_depth':
                self.depth[:] = float('nan')
            else:
                self.depth[:, 160:] = 2.5
            self.assertEqual(self.detect(), [], mode)
            self.rgb, self.depth = rgb, depth

    def test_second_same_color_object_stays_separate_for_ambiguity_check(self):
        from rgbd_following_core.tracking import Tracker
        self.rgb[90:150, 30:90] = (0, 220, 0)
        self.depth[90:150, 30:90] = 1.2
        candidates = self.detect()
        self.assertEqual(len(candidates), 2)
        tracker = Tracker()
        self.assertIsNone(tracker.update([c['point'] for c in candidates], 1.))
        self.assertEqual(tracker.reason, 'ambiguous_target')

    def test_cloud_uses_intrinsics_and_removes_invalid_depth(self):
        d = self.np.array([[1., float('nan')], [0., 2.]], dtype=self.np.float32)
        points = self.vision.cloud_points(d, (1., 1., 0., 0.), stride=1)
        self.np.testing.assert_allclose(points, [[0, 0, 1], [2, 2, 2]])

    def test_metric_depth_and_optical_to_base_transform(self):
        d = self.vision.metric_depth(self.np.array([[1200]], dtype=self.np.uint16), '16UC1')
        self.assertAlmostEqual(float(d[0, 0]), 1.2, places=5)
        # Optical z forward becomes base x; optical x right becomes base -y.
        point = self.vision.transform_points([[.1, .2, 1.]], (0, 0, .5), (-.5, .5, -.5, .5))
        self.np.testing.assert_allclose(point, [[1, -.1, .3]], atol=1e-6)

    def test_execution_requires_all_calibrations_and_finish_zone(self):
        with self.assertRaises(ValueError):
            self.validate(self.c, execute=True)
        for name in ('hardware_calibrated', 'identity_calibrated', 'depth_coverage_calibrated'):
            self.c[name] = True
        with self.assertRaises(ValueError):
            self.validate(self.c, execute=True)
        self.c['finish_polygon'] = [[0, 0], [2, 0], [2, 2], [0, 2]]
        self.validate(self.c, execute=True)

    def test_unsafe_config_rejected(self):
        for key, value in (('max_linear', 9.), ('footprint_radius', float('nan')), ('follow_distance', .2)):
            bad = copy.deepcopy(self.c)
            bad[key] = value
            with self.assertRaises(ValueError):
                self.validate(bad)

    def test_color_calibration_handles_red_hue_wrap(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('calibration_test', os.path.join(ROOT, 'tools/calibrate_rgbd_target.py'))
        tool = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tool)
        hsv = self.np.zeros((20, 20, 3), self.np.uint8)
        hsv[:, :, 1:] = 220
        hsv[:, :10, 0], hsv[:, 10:, 0] = 0, 179
        bands = tool.bands_from_crop(self.cv2.cvtColor(hsv, self.cv2.COLOR_HSV2BGR))
        self.assertEqual(len(bands), 2)
        self.assertTrue(any(b['lower'][0] == 0 for b in bands))
        self.assertTrue(any(b['upper'][0] == 179 for b in bands))
        with self.assertRaises(ValueError):
            tool.bands_from_crop(self.np.full((20, 20, 3), 200, self.np.uint8))

    def test_finish_polygon_rejects_self_intersecting_star(self):
        import math
        vertices = [[math.cos(a * math.pi / 2.5), math.sin(a * math.pi / 2.5)] for a in range(5)]
        self.c['finish_polygon'] = [vertices[i] for i in (0, 2, 4, 1, 3)]
        with self.assertRaises(ValueError):
            self.validate(self.c)


if __name__ == '__main__':
    unittest.main()
