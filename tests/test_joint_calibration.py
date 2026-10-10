import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Offline calibration requires Python 3')
class CalibrationTests(unittest.TestCase):
    def setUp(self):
        from pathlib import Path
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
        from analyze_joint_mapping import fit
        self.fit = fit

    def test_offset_and_negative_direction(self):
        result = self.fit([(420, 24), (520, 0), (620, -24)])
        self.assertAlmostEqual(result['zero'], 520)
        self.assertEqual(result['direction'], -1)
        self.assertAlmostEqual(result['rmse_deg'], 0)

    def test_noisy_measurements_report_residual(self):
        result = self.fit([(400, -24), (500, .2), (600, 24)])
        self.assertEqual(result['direction'], 1)
        self.assertGreater(result['rmse_deg'], 0)
        self.assertFalse(result['hardware_verified'])

    def test_unobservable_and_invalid_data(self):
        for samples in ([(500, 0)], [(500, 0), (600, 0)],
                        [(500, 0), (500, 24)], [(500, 0), (1001, 24)],
                        [(500, 0), (600, float('nan'))]):
            with self.assertRaises(ValueError):
                self.fit(samples)
