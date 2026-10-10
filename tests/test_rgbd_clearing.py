"""Range separation for measured clearing rays; no fabricated free space."""
import copy
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))


class ClearingCloudTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        import yaml
        from rgbd_following_core.vision import observation_clouds
        from rgbd_following_core.settings import validate
        self.np, self.clouds, self.validate = np, observation_clouds, validate
        with open(os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/config/follow.yaml')) as f:
            self.config = yaml.safe_load(f)
        self.p = self.config['perception']
        self.p['cloud_stride'] = self.p['clearing_stride'] = 1
        self.camera = (525., 525., 2., 2.)

    def test_far_background_only_enters_clearing_stream(self):
        d = self.np.array([[1.2, 4., 4.3, 4.5, 4.6, 9.]], self.np.float32)
        obstacles, clearing = self.clouds(d, self.camera, self.p)
        self.np.testing.assert_allclose(obstacles[:, 2], [1.2, 4.])
        self.np.testing.assert_allclose(clearing[:, 2], [1.2, 4., 4.3, 4.5])

    def test_no_return_is_not_a_clearing_endpoint(self):
        d = self.np.array([[float('nan'), float('inf'), -float('inf'), 0., -.2, .2]], self.np.float32)
        obstacles, clearing = self.clouds(d, self.camera, self.p)
        self.assertEqual(obstacles.shape, (0, 3))
        self.assertEqual(clearing.shape, (0, 3))

    def test_endpoint_keeps_measured_range_and_ray_direction(self):
        d = self.np.array([[4.3]], self.np.float32)
        unused, clearing = self.clouds(d, (1., 1., 0., 1.), self.p)
        # Optical y=-4.3 is a high endpoint after mounting. Do not truncate its
        # height/range before VoxelLayer clips the ray in the map frame.
        self.np.testing.assert_allclose(clearing, [[0., -4.3, 4.3]])

    def test_legacy_calibration_does_not_silently_expand_sensor_range(self):
        del self.p['clearing_max_depth']
        del self.p['clearing_stride']
        self.validate(self.config)
        self.assertEqual(self.p['clearing_max_depth'], self.p['max_depth'])
        unused, clearing = self.clouds(self.np.array([[4.3]], self.np.float32), self.camera, self.p)
        self.assertEqual(len(clearing), 0)

    def test_invalid_clearing_configuration_is_rejected(self):
        for key, value in [('clearing_max_depth', 3.), ('clearing_max_depth', float('inf')),
                           ('clearing_stride', 0), ('clearing_stride', 1.5),
                           ('clearing_stride', True), ('clearing_stride', 8)]:
            c = copy.deepcopy(self.config)
            c['perception'][key] = value
            with self.assertRaises(ValueError):
                self.validate(c)


if __name__ == '__main__':
    unittest.main()
