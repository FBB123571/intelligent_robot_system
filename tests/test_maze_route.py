"""Offline checks for humanoid/maze_route (no camera, no servos).

Marker detection and pose are checked on synthetic perspective views; the
route state machine is run against a simulated robot with the gait measured on
the field (3.4 cm per step, drifting left, ~30 deg per catch_ball_turn).
"""
import math
import sqlite3
import sys
import tempfile
import threading
import types
import unittest
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:          # pragma: no cover - OpenCV is optional on CI
    cv2 = None

ROOT = Path(__file__).resolve().parents[1]
MAZE = ROOT / 'humanoid' / 'maze_route'


@unittest.skipIf(cv2 is None or sys.version_info[0] < 3, 'needs Python 3 with OpenCV and NumPy')
class MarkerTests(unittest.TestCase):
    K = None

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(MAZE))
        import markers
        cls.m = markers
        cls.K = np.float64([[425, 0, 320], [0, 425, 276], [0, 0, 1]])
        cls.det = markers.MarkerDetector(cls.K, np.zeros(5))

    def render(self, mid, dist_cm, robot_right_deg, pitch_down_deg=7, below_cm=13, seed=0):
        """Marker `mid` on a wall square to the corridor, seen by a camera
        `dist_cm` away, turned right by `robot_right_deg` and pitched down."""
        bits = np.array(list(self.m.TEMPLATES[mid]), np.uint8).reshape(5, 5)
        g = np.zeros((9, 9), np.uint8)
        g[2:7, 2:7] = bits
        tex = cv2.copyMakeBorder(cv2.resize(g * 255, (450, 450), interpolation=cv2.INTER_NEAREST),
                                 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
        half = 570 / 2 * self.m.SIZE_CM / 450
        P = np.float64([[-half, -half], [half, -half], [half, half], [-half, half]])
        yaw, pit = math.radians(robot_right_deg), math.radians(pitch_down_deg)
        world = np.c_[P[:, 0], P[:, 1] + below_cm, np.full(4, float(dist_cm))]
        Ry = np.array([[math.cos(yaw), 0, -math.sin(yaw)], [0, 1, 0], [math.sin(yaw), 0, math.cos(yaw)]])
        Rx = np.array([[1, 0, 0], [0, math.cos(pit), -math.sin(pit)], [0, math.sin(pit), math.cos(pit)]])
        cam = (Rx @ Ry @ world.T).T
        pts = (self.K @ (cam / cam[:, 2:3]).T).T[:, :2].astype(np.float32)
        H = cv2.getPerspectiveTransform(np.float32([[0, 0], [570, 0], [570, 570], [0, 570]]), pts)
        out = cv2.warpPerspective(tex, H, (640, 480), borderValue=110)
        noise = np.random.default_rng(seed).normal(0, 5, out.shape)
        out = np.clip(cv2.GaussianBlur(out, (0, 0), 0.8) + noise, 0, 255).astype(np.uint8)
        return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)

    def test_templates_are_distinct_under_rotation(self):
        T = {k: np.array(list(v), int).reshape(5, 5) for k, v in self.m.TEMPLATES.items()}
        for a in T:
            for b in T:
                if a < b:
                    d = min(int((np.rot90(T[b], r) != T[a]).sum()) for r in range(4))
                    self.assertGreater(d, 2 * self.m.MAX_DATA_ERR, (a, b))

    def test_each_marker_is_identified(self):
        for mid in 'ABC':
            found = self.det.detect(self.render(mid, 60, 0))
            self.assertEqual(list(found), [mid])

    def test_distance_bearing_and_yaw(self):
        for d in (40, 60, 100):
            for yr in (-20, 0, 20):
                m = self.det.detect(self.render('A', d, yr)).get('A')
                self.assertIsNotNone(m, (d, yr))
                self.assertAlmostEqual(m['dist_cm'], math.hypot(d, 13), delta=0.06 * d)
                self.assertAlmostEqual(m['yaw_deg'], yr, delta=5)          # + = robot right of square
                self.assertLess(m['bearing_deg'] * yr, 1e-6 if yr else 1)   # marker drifts the other way

    def test_too_close_with_head_at_neutral_leaves_the_image(self):
        # head at neutral (~7 deg down), marker centre 13 cm below the camera:
        # at 30 cm the bottom of the marker is below the image, at 40 cm it is in
        self.assertEqual(self.det.detect(self.render('C', 30, 0)), {})
        self.assertIn('C', self.det.detect(self.render('C', 40, 0)))

    def test_blank_wall_has_no_marker(self):
        self.assertEqual(self.det.detect(np.full((480, 640, 3), 110, np.uint8)), {})


@unittest.skipIf(cv2 is None or sys.version_info[0] < 3, 'needs Python 3 with OpenCV and NumPy')
class RouteSimulationTests(unittest.TestCase):
    """Top view, cm. Start at the origin facing +y; A ends the first leg, B the
    second (after a left turn), C the last (after a right turn)."""
    MARKERS = {'A': (0.0, 190.0, 90.0), 'B': (-160.0, 150.0, 180.0), 'C': (-120.0, 410.0, 90.0)}

    def run_route(self, drift_deg, step_cm=3.4, turn_deg=30.0, see_cm=300, min_cm=0.0, stop_cm=40):
        sys.path.insert(0, str(MAZE))
        import marker_route as mr
        mr.SETTLE_S = mr.WALK_PAUSE_S = mr.CORNER_PAUSE_S = 0
        pose = {'x': 0.0, 'y': 0.0, 'h': 90.0}
        played = []

        class Servos:
            def play(self, name, lock, stop_ev):
                if name == 'catch_ball_go':
                    r = math.radians(pose['h'])
                    pose['x'] += step_cm * math.cos(r)
                    pose['y'] += step_cm * math.sin(r)
                    pose['h'] += drift_deg
                elif name.endswith('turn_left'):
                    pose['h'] += turn_deg
                elif name.endswith('turn_right'):
                    pose['h'] -= turn_deg
                played.append(name)
                return True

        markers = self.MARKERS

        class Eyes:
            last_stats = (0, 3)

            def latest(self):
                return None, {}, 0

            def look(self, target, n=3, timeout=2):
                mx, my, square = markers[target]
                dx, dy = mx - pose['x'], my - pose['y']
                d = math.hypot(dx, dy)
                bearing = (pose['h'] - math.degrees(math.atan2(dy, dx)) + 180) % 360 - 180
                if abs(bearing) > 37 or d > see_cm or d < min_cm:
                    return None
                yaw = -((pose['h'] - square + 180) % 360 - 180)
                return dict(dist_cm=d, bearing_deg=bearing, yaw_deg=yaw, seen=3, of=3)

        tmp = Path(tempfile.mkdtemp())
        (tmp / 'snapshots').mkdir()
        actions = tmp / 'actions'
        actions.mkdir()
        db = sqlite3.connect(str(actions / 'grab_final_v2.d6a'))
        db.execute('create table ActionGroup ("Index" int, Time int, %s)' % ','.join('Servo%d int' % i for i in range(1, 19)))
        db.execute('insert into ActionGroup values (%s)' % ','.join(['1', '1000'] + ['500'] * 18))
        db.commit()
        db.close()
        mr.ACTIONS = actions
        a = types.SimpleNamespace(grab=False, release=True, dry_run=False, turn_cm=40, stop_cm=stop_cm, centre_deg=8,
                                  lost_near_cm=12,
                                  turn_step_deg=20, max_steps=150, stall_steps=20, stall_cm=5, max_misses=2,
                                  max_centre_turns=4, max_corner_turns=12, max_blind_steps=25, camera_wait=60,
                                  max_snapshots=0)
        route = mr.Route(a, Servos(), Eyes(), tmp, interactive=False)
        route.go.set()
        route.start()
        route.join(60)
        self.assertFalse(route.is_alive())
        return route, played, math.hypot(markers['C'][0] - pose['x'], markers['C'][1] - pose['y'])

    def test_measured_gait_reaches_c_and_puts_down(self):
        for drift in (8.0, 12.0, 15.0, -3.0):
            route, played, left_to_c = self.run_route(drift)
            self.assertIn('cube put down', route.status, drift)
            self.assertLessEqual(left_to_c, 40, drift)
            self.assertEqual(played[-1], 'put_down')

    def test_marker_lost_close_to_the_goal_counts_as_reached(self):
        # the camera loses markers closer than 37 cm; asking to stop at 30 cm
        # must still end at C (lost right next to it) and put the cube down
        route, played, left_to_c = self.run_route(12.0, min_cm=37, stop_cm=30)
        self.assertIn('cube put down', route.status)
        self.assertLessEqual(left_to_c, 40)

    def test_does_not_oscillate_with_large_turns(self):
        route, played, _ = self.run_route(0.0, turn_deg=32)
        turns = sum(1 for p in played if 'turn' in p)
        self.assertLess(turns, 15)


if __name__ == '__main__':
    unittest.main()
