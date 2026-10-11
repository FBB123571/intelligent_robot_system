import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Pi vision runs on Python 3')
class VisionTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest('Install opencv-python-headless and numpy for synthetic vision tests')
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'humanoid/scripts'))
        from competition import detect
        self.cv2, self.np, self.detect = cv2, np, detect
        sample = np.uint8([[[0, 160, 0]]])
        lab = cv2.cvtColor(sample, cv2.COLOR_BGR2LAB)[0, 0].astype(int)
        self.config = dict(lower_lab=np.maximum(lab - 15, 0).tolist(),
                           upper_lab=np.minimum(lab + 15, 255).tolist(),
                           min_area=.003, max_area=.5)

    def test_colored_block_normalized_position_and_bottom(self):
        frame = self.np.zeros((240, 320, 3), dtype=self.np.uint8)
        self.cv2.rectangle(frame, (128, 160), (192, 224), (0, 160, 0), -1)
        observation, mask = self.detect(frame, self.config)
        self.assertIsNotNone(observation)
        self.assertAlmostEqual(observation[0], .5, delta=.01)
        self.assertAlmostEqual(observation[1], 224. / 240, delta=.02)
        self.assertGreater(observation[2], .025)

    def test_ball_does_not_require_block_shape(self):
        frame = self.np.zeros((240, 320, 3), dtype=self.np.uint8)
        self.cv2.circle(frame, (160, 160), 30, (0, 160, 0), -1)
        self.assertIsNotNone(self.detect(frame, self.config)[0])

    def test_blank_and_small_noise_rejected(self):
        frame = self.np.zeros((240, 320, 3), dtype=self.np.uint8)
        self.assertIsNone(self.detect(frame, self.config)[0])
        self.cv2.rectangle(frame, (160, 160), (162, 162), (0, 160, 0), -1)
        self.assertIsNone(self.detect(frame, self.config)[0])

    def test_default_blue_thresholds_reject_red_and_green(self):
        import yaml
        from pathlib import Path
        root = Path(__file__).resolve().parents[1]
        config = yaml.safe_load((root / 'humanoid/config/competition.yaml').read_text(encoding='utf-8'))['vision']
        for color, expected in (((200, 80, 30), True), ((0, 160, 0), False), ((0, 0, 220), False)):
            frame = self.np.zeros((240, 320, 3), dtype=self.np.uint8)
            self.cv2.rectangle(frame, (128, 160), (192, 224), color, -1)
            self.assertEqual(self.detect(frame, config)[0] is not None, expected)

    def test_release_requires_empty_hand_and_visible_placed_block(self):
        from task_feedback import TaskFeedback
        from unittest.mock import Mock, patch
        config = dict(color_space='HSV', lower_hsv=[95, 70, 40], upper_hsv=[135, 255, 255],
                      min_area=.03, max_area=.95)
        held_roi, placed_roi = [0, 0, .5, .5], [.5, .5, 1, 1]
        for held, placed, expected in ((False, False, False), (True, True, False),
                                      (True, False, False), (False, True, True)):
            frame = self.np.zeros((240, 320, 3), dtype=self.np.uint8)
            if held:
                self.cv2.rectangle(frame, (30, 30), (100, 90), (200, 80, 30), -1)
            if placed:
                self.cv2.rectangle(frame, (190, 150), (260, 210), (200, 80, 30), -1)
            camera = Mock()
            # Advance virtual time for each distinct frame and polling wait.
            # Real OpenCV latency on a Pi must not consume the test deadline.
            clock = [0.0]
            def read_after(unused):
                clock[0] += .005
                return frame.copy(), clock[0]
            camera.read_after.side_effect = read_after
            motion = Mock()
            motion.wait.side_effect = lambda seconds: clock.__setitem__(0, clock[0] + seconds)
            feedback_config = dict(held_roi=held_roi, placed_roi=placed_roi, camera_timeout=1,
                                   released=dict(mode='vision', timeout=.03, stable_samples=2))
            feedback = TaskFeedback(feedback_config, config, camera, motion, self.detect)
            with patch('task_feedback.time.monotonic', side_effect=lambda: clock[0]):
                self.assertEqual(feedback.confirm('released'), expected)


if __name__ == '__main__':
    unittest.main()
