import io
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock
import urllib.error

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'humanoid/scripts'))
from snapshot_camera import SnapshotCapture


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        try:
            import cv2
            import numpy as np
        except ImportError: self.skipTest('OpenCV/numpy needed')
        self.cv2,self.np=cv2,np
        self.config={'url':'http://127.0.0.1:9040/api/camera','max_fps':30}

    def test_cached_image_cannot_refresh_camera_timestamp(self):
        frame=self.np.zeros((20,30,3),self.np.uint8)
        first=self.cv2.imencode('.jpg',frame)[1].tobytes()
        frame[:]=(0,0,220)
        second=self.cv2.imencode('.jpg',frame)[1].tobytes()
        opener=Mock(); opener.open.side_effect=[io.BytesIO(first),io.BytesIO(first),io.BytesIO(second)]
        capture=SnapshotCapture(self.config,self.cv2,opener)
        self.assertTrue(capture.read()[0])
        self.assertEqual(capture.read(),(False,None))
        self.assertTrue(capture.read()[0])
        capture.release(); self.assertEqual(capture.read(),(False,None))
        self.assertEqual(opener.open.call_count,3)

    def test_disabled_camera_and_invalid_payload_do_not_supply_frames(self):
        opener=Mock(); opener.open.side_effect=[urllib.error.HTTPError(self.config['url'],503,'camera stale',{},None),io.BytesIO(b'not an image')]
        capture=SnapshotCapture(self.config,self.cv2,opener)
        self.assertEqual(capture.read(),(False,None))
        self.assertIn('503',capture.last_error)
        self.assertEqual(capture.read(),(False,None))

    def test_unbounded_timeout_is_rejected(self):
        with self.assertRaises(ValueError): SnapshotCapture(dict(self.config,timeout=100),self.cv2)
