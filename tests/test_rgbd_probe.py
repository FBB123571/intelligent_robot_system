"""Exercise ROS probe callbacks with message doubles; not a ROS integration test."""
import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Adapter harness requires Python 3')
class ProbeTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        import threading
        from pathlib import Path
        from types import SimpleNamespace as NS, ModuleType
        from unittest.mock import Mock, MagicMock, patch
        self.NS, self.Mock = NS, Mock
        def module(name, **values):
            result = ModuleType(name)
            result.__dict__.update(values)
            return result
        fake = {
            'cv2': module('cv2', rectangle=Mock(), putText=Mock(), FONT_HERSHEY_SIMPLEX=0, error=RuntimeError),
            'cv_bridge': module('cv_bridge', CvBridge=Mock, CvBridgeError=TypeError),
            'message_filters': module('message_filters'),
            'rospy': module('rospy', Time=NS(now=lambda: NS(to_sec=lambda: 10.1))),
            'sensor_msgs': module('sensor_msgs'),
            'sensor_msgs.msg': module('sensor_msgs.msg', Image=object, CameraInfo=object),
            'std_msgs': module('std_msgs'),
            'std_msgs.msg': module('std_msgs.msg', String=lambda data: NS(data=data))
        }
        self.patcher = patch.dict(sys.modules, fake)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        root = Path(__file__).resolve().parents[1]
        package = root / 'turtlebot/follow3/rgbd_following'
        sys.path.insert(0, str(package / 'src'))
        spec = importlib.util.spec_from_file_location('probe_under_test', package / 'scripts/rgbd_probe.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.node = self.module.Probe.__new__(self.module.Probe)
        self.node.lock = threading.RLock()
        self.node.clock = lambda: 20.
        self.node.max_age, self.node.slop = .5, .05
        self.node.roi, self.node.stride = [0, 0, 1, 1], 2
        self.node.depth_args = {}
        self.node.last_pair = self.node.last_stamp = self.node.last_receipt = None
        self.node.status, self.node.preview, self.node.bridge = Mock(), Mock(), Mock()
        array = MagicMock()
        array.shape = (10, 10)
        array.__getitem__.return_value.ravel.return_value = [1.2] * 40
        view = MagicMock()
        view.shape = (10, 10, 3)
        view.copy.return_value = view
        self.node.bridge.imgmsg_to_cv2.side_effect = lambda msg, desired_encoding: array if desired_encoding == 'passthrough' else view
        self.array = array
        self.messages = [NS(header=NS(stamp=NS(to_sec=lambda: 10.), frame_id='rgb'),
                            width=10, height=10, encoding='32FC1') for unused in range(3)]
        self.messages[2].P = [10., 0., 5., 0., 0., 10., 5., 0., 0., 0., 1., 0.]

    def status(self):
        import json
        return json.loads(self.node.status.publish.call_args[0][0].data)

    def test_valid_input_publishes_diagnostic_without_claiming_identity(self):
        self.node.callback(*self.messages)
        result = self.status()
        self.assertEqual(result['reason'], 'roi_depth_ok')
        self.assertEqual(result['median_m'], 1.2)
        self.assertFalse(result['motion_enabled'])
        self.assertFalse(result['target_identified'])
        self.assertEqual(self.node.preview.publish.call_count, 1)
        self.assertEqual(self.node.last_stamp, 10.)

    def test_duplicate_frames_do_not_refresh_receipt_or_preview(self):
        self.node.callback(*self.messages)
        self.node.clock = lambda: 21.
        self.node.callback(*self.messages)
        self.assertEqual(self.status()['reason'], 'repeated_or_out_of_order_input')
        self.assertEqual(self.node.last_receipt, 20.)
        self.assertEqual(self.node.preview.publish.call_count, 1)

    def test_stale_frame_and_bad_calibration_never_publish_preview(self):
        self.messages[0].header.stamp.to_sec = lambda: 8.
        self.node.callback(*self.messages)
        self.assertFalse(self.status()['usable'])
        self.assertIsNone(self.node.last_receipt)
        self.messages[0].header.stamp.to_sec = lambda: 10.
        self.messages[2].P[0] = 0.
        self.node.callback(*self.messages)
        self.assertEqual(self.status()['reason'], 'invalid_camera_calibration')
        self.node.preview.publish.assert_not_called()

    def test_missing_depth_is_reported_as_unusable(self):
        self.array.__getitem__.return_value.ravel.return_value = [float('nan')] * 40
        self.node.callback(*self.messages)
        self.assertFalse(self.status()['usable'])
        self.assertIsNone(self.status()['median_m'])

    def test_watchdog_expires_even_if_ros_time_stalls(self):
        from unittest.mock import patch
        self.node.callback(*self.messages)
        self.node.clock = lambda: 21.
        with patch.object(self.module.rospy, 'is_shutdown', create=True, side_effect=[False, True]):
            with patch.object(self.module.time, 'sleep'):
                self.node.watchdog()
        self.assertEqual(self.status()['reason'], 'input_timeout')
        self.assertFalse(self.status()['usable'])


if __name__ == '__main__':
    unittest.main()
