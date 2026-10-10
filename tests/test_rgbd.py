# -*- coding: utf-8 -*-
"""Method 3 input and depth checks; no ROS, OpenCV or NumPy required."""
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot', 'follow3', 'rgbd_following', 'src'))
from rgbd_following_core.depth import check_pair, roi_bounds, summarize


class DepthTests(unittest.TestCase):
    def test_millimeters_and_meters_agree(self):
        mm = summarize([1200] * 40, '16UC1')
        meters = summarize([1.2] * 40, '32FC1')
        self.assertEqual(mm, meters)
        self.assertTrue(mm['usable'])

    def test_invalid_and_out_of_range_samples_are_not_free_space(self):
        result = summarize([0, float('nan'), float('inf'), -1, 0.2, 8], '32FC1')
        self.assertFalse(result['usable'])
        self.assertIsNone(result['median_m'])

    def test_sparse_depth_rejected_even_with_enough_valid_pixels(self):
        result = summarize([1.] * 40 + [0.] * 100, '32FC1')
        self.assertFalse(result['usable'])

    def test_small_roi_and_empty_depth_rejected(self):
        for values in ([], [1.] * 5):
            self.assertFalse(summarize(values, '32FC1')['usable'])

    def test_median_ignores_single_far_outlier(self):
        result = summarize([1.] * 40 + [3.], '32FC1')
        self.assertTrue(result['usable'])
        self.assertEqual(result['median_m'], 1.)

    def test_mixed_foreground_background_is_not_reliable_target_depth(self):
        result = summarize([1.] * 20 + [3.] * 20, '32FC1')
        self.assertFalse(result['usable'])
        self.assertEqual(result['reason'], 'mixed_depths')

    def test_unknown_encoding_and_bad_thresholds_rejected(self):
        with self.assertRaises(ValueError):
            summarize([1.] * 40, '8UC1')
        with self.assertRaises(ValueError):
            summarize([], '32FC1', max_depth=float('nan'))

    def test_roi_validation_and_full_image_bounds(self):
        self.assertEqual(roi_bounds([0, 0, 1, 1], 640, 480), (0, 0, 640, 480))
        for roi in ([-.1, 0, 1, 1], [0, 0, 0, 1], [0, 0, float('nan'), 1], [0, 0, .0001, 1]):
            with self.assertRaises(ValueError):
                roi_bounds(roi, 640, 480)

    def test_valid_registered_pair(self):
        check_pair([10, 10.02, 10], 10.1, .5, .05,
                   ['camera', '/camera', 'camera'], [(640, 480)] * 3, '16UC1')

    def test_stale_future_unsynchronized_zero_stamps_rejected(self):
        for stamps in ([9, 9, 9], [11, 11, 11], [10, 10.1, 10], [0, 0, 0]):
            with self.assertRaises(ValueError):
                check_pair(stamps, 10.2, .5, .05, ['camera'] * 3, [(640, 480)] * 3, '32FC1')

    def test_unregistered_frames_and_sizes_rejected(self):
        for frames, sizes in ((['rgb', 'depth', 'rgb'], [(640, 480)] * 3),
                              (['rgb'] * 3, [(640, 480), (320, 240), (640, 480)])):
            with self.assertRaises(ValueError):
                check_pair([10] * 3, 10.1, .5, .05, frames, sizes, '32FC1')


if __name__ == '__main__':
    unittest.main()
