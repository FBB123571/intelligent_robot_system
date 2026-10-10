import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Pi camera guard requires Python 3')
class CameraGuardTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest('Install OpenCV and numpy')
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'humanoid/scripts'))
        from vision_guard import floor_obstacle, VisionGuard
        self.cv2, self.np, self.analyze, self.guard = cv2, np, floor_obstacle, VisionGuard
        self.config = dict(roi=[.1, .5, .9, 1], floor_lower_hsv=[0, 0, 40],
                           floor_upper_hsv=[179, 65, 235], contour_fraction=.06,
                           occupancy_fraction=.15, min_visible_fraction=.5,
                           edge_fraction=.015, edge_contour_height=.3, max_age=.3)
        self.blue = dict(color_space='HSV', lower_hsv=[95, 70, 40], upper_hsv=[135, 255, 255])
        self.frame = np.full((240, 320, 3), 150, dtype=np.uint8)

    def test_grey_floor_clear_and_red_column_blocks(self):
        self.assertFalse(self.analyze(self.frame, self.config, self.blue, None, 'forward')[0])
        self.cv2.rectangle(self.frame, (140, 135), (180, 230), (20, 20, 220), -1)
        self.assertTrue(self.analyze(self.frame, self.config, self.blue, None, 'forward')[0])

    def test_grey_high_contrast_chair_leg_edge_blocks(self):
        self.cv2.rectangle(self.frame, (140, 130), (146, 230), (50, 50, 50), -1)
        self.assertTrue(self.analyze(self.frame, self.config, self.blue, None, 'forward')[0])

    def test_blue_exception_is_local_and_not_active_during_carry(self):
        self.config['interaction_roi'] = [.4, .5, .6, 1]
        self.cv2.rectangle(self.frame, (140, 145), (180, 220), (220, 40, 20), -1)
        self.assertFalse(self.analyze(self.frame, self.config, self.blue, None, 'grab')[0])
        self.assertTrue(self.analyze(self.frame, self.config, self.blue, None, 'forward')[0])
        self.cv2.rectangle(self.frame, (40, 145), (70, 220), (220, 40, 20), -1)
        self.assertTrue(self.analyze(self.frame, self.config, self.blue, None, 'grab')[0])

    def test_occluded_corridor_and_stale_camera_stop(self):
        from task_feedback import CollisionBlocked
        from unittest.mock import Mock
        import time
        with self.assertRaises(CollisionBlocked):
            self.analyze(self.frame, self.config, self.blue, [.1, .5, .9, 1], 'forward')
        camera = Mock()
        camera.read_after.return_value = (self.frame, time.monotonic() - 1)
        guard = self.guard(self.config, camera, self.blue, None)
        with self.assertRaises(CollisionBlocked):
            guard.check()
        camera.read_after.return_value = (self.frame, time.monotonic())
        guard.check()
