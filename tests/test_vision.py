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


if __name__ == '__main__':
    unittest.main()
