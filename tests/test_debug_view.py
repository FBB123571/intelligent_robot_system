import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Pi debug view requires Python 3')
class DebugTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.skipTest('Install OpenCV and numpy')
        from pathlib import Path
        import yaml
        root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root / 'humanoid/scripts'))
        from competition import detect_details, detect, Stopped
        from debug_view import DebugView, render_frame
        self.cv2, self.np, self.details, self.detect = cv2, np, detect_details, detect
        self.view, self.render, self.stopped = DebugView, render_frame, Stopped
        self.config = yaml.safe_load((root / 'humanoid/config/competition.yaml').read_text(encoding='utf-8'))

    def test_actual_target_box_scales_to_original_frame_without_mutation(self):
        frame = self.np.zeros((480, 640, 3), dtype=self.np.uint8)
        self.cv2.rectangle(frame, (200, 260), (360, 420), (200, 80, 30), -1)
        original = frame.copy()
        observation, mask, boxes = self.details(frame, self.config['vision'])
        self.assertEqual(self.detect(frame, self.config['vision'])[0], observation)
        self.assertEqual(len(boxes), 1)
        x, y, w, h = boxes[0]['bbox']
        self.assertAlmostEqual(x * 640, 200, delta=4)
        self.assertAlmostEqual(y * 480, 260, delta=4)
        output = self.render(frame, boxes, self.config, 'verifying_grasp', 'grab', 'Feedback held yes 6/6')
        self.assertTrue(self.np.array_equal(frame, original))
        self.assertEqual(output[int(y * 480), int(x * 640)].tolist(), [255, 180, 0])
        self.assertFalse(self.np.array_equal(output, frame))

    def test_window_quit_propagates_stop_and_does_not_issue_actions(self):
        import time
        from unittest.mock import Mock
        frame = self.np.zeros((240, 320, 3), dtype=self.np.uint8)
        camera, heartbeat, motion = Mock(), Mock(), Mock()
        camera.read_after.return_value = frame, time.monotonic()
        camera.cv2.waitKey.return_value = ord('q')
        heartbeat.state = 'carrying'
        motion.current_action, motion.current_kind = 'grab_go_forward', 'forward'
        view = self.view(camera, self.config, heartbeat, motion, self.details, self.stopped)
        with self.assertRaises(self.stopped):
            view.poll()
        camera.cv2.imshow.assert_called()
        motion.action.assert_not_called()

    def test_gui_is_pumped_during_action_wait(self):
        from competition import Motion
        import threading
        from unittest.mock import Mock
        motion = Motion.__new__(Motion)
        motion.stop = threading.Event()
        motion.config = {}
        motion.debug_view = Mock()
        motion.debug_view.poll.side_effect = self.stopped('quit')
        with self.assertRaises(self.stopped):
            motion.wait(.5)
        motion.debug_view.poll.assert_called_once()

    def test_gtk_unsupported_visibility_keeps_display_open(self):
        import time
        from unittest.mock import Mock
        camera, heartbeat, motion = Mock(), Mock(), Mock()
        camera.read_after.return_value = self.np.zeros((240, 320, 3), dtype=self.np.uint8), time.monotonic()
        camera.cv2.waitKey.return_value = -1
        camera.cv2.getWindowProperty.return_value = -1
        heartbeat.state = 'vision_preview'
        motion.current_action = ''
        view = self.view(camera, self.config, heartbeat, motion, self.details, self.stopped)
        view.poll()
        camera.cv2.imshow.assert_called()
        view.last_poll = 0
        camera.cv2.getWindowProperty.return_value = 0
        with self.assertRaises(self.stopped):
            view.poll()
