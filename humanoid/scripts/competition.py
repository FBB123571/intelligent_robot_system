#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TonyPi: fresh vision -> grasp -> carry route -> release.

Single synchronous action executor prevents vision/motion command races.
Direct d6a frame execution propagates errors (some SDK action helpers swallow them).
Python >= 3.5. No ROS requirement on the Raspberry Pi.
"""
from __future__ import print_function
import argparse
import importlib
import json
import os
from pathlib import Path
import signal
import socket
import sqlite3
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'turtlebot/follow2/multi_robot_following/src'))
from competition_core.mission import Approach, validate_route


class Stopped(RuntimeError):
    pass


def action_frames(path):
    path = Path(path)
    if not path.is_file():
        raise RuntimeError('Missing action file: ' + str(path))
    connection = sqlite3.connect('file:' + path.resolve().as_posix() + '?mode=ro', uri=True)
    try:
        rows = connection.execute('SELECT * FROM ActionGroup ORDER BY "Index"').fetchall()
        if not rows or any(len(row) not in (18, 20) or not isinstance(row[1], int) or not 20 <= row[1] <= 30000 or
                           any(not isinstance(v, int) or not 0 <= v <= 1000 for v in row[2:]) for row in rows):
            raise ValueError('Invalid 16/18-channel TonyPi d6a: ' + str(path))
        return rows
    finally:
        connection.close()


class Heartbeat(object):
    def __init__(self, config):
        self.config = config
        self.state = 'idle'
        self.stop = threading.Event()
        self.sequence = 0
        self.run_id = str(uuid.uuid4())
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()

    def run(self):
        while not self.stop.is_set():
            if self.config['enabled']:
                data = dict(session=self.config['session'], state=self.state,
                            sequence=self.sequence, run_id=self.run_id)
                try:
                    self.sock.sendto(json.dumps(data).encode('utf-8'), (self.config['host'], self.config['port']))
                except OSError as exc:
                    print('[heartbeat]', exc, file=sys.stderr)
            self.sequence += 1
            self.stop.wait(0.2)

    def set(self, state):
        self.state = state
        print('[state]', state)

    def close(self):
        self.stop.set()
        self.worker.join(1)
        self.sock.close()


class Motion(object):
    def __init__(self, config, stop, execute):
        self.config, self.stop, self.execute = config, stop, execute
        self.frames = {}
        self.hold_values = {}
        for name in validate_route(config):
            if execute:
                self.frames[name] = action_frames(Path(config['action_dir']) / (name + '.d6a'))
        self.controller = self.old_board = None
        if execute:
            for directory in ('/home/pi/TonyPi', '/home/pi/TonyPi/HiwonderSDK',
                              '/home/pi/TonyPi/HiwonderSDK/build/lib'):
                sys.path.insert(0, directory)
            # Only import hardware drivers after all preflight checks succeed.
            try:
                sdk = importlib.import_module('hiwonder.ros_robot_controller_sdk')
                controller = importlib.import_module('hiwonder.Controller')
            except ImportError:
                self.old_board = importlib.import_module('hiwonder.Board')
            else:
                self.controller = controller.Controller(sdk.Board())

    def wait(self, seconds):
        if self.stop.wait(seconds):
            raise Stopped('stop requested')

    def head(self):
        if not self.execute:
            return
        head = self.config['head']
        for servo, pulse in ((head['pan_id'], head['pan']), (head['tilt_id'], head['tilt'])):
            if not 500 <= pulse <= 2500:
                raise ValueError('PWM head pulse outside 500..2500')
            if self.controller:
                self.controller.set_pwm_servo_pulse(servo, pulse, 500)
            else:
                self.old_board.setPWMServoPulse(servo, pulse, 500)
        self.wait(0.6)

    def action(self, kind):
        if self.stop.is_set():
            raise Stopped('stop requested')
        name = self.config['actions'][kind]
        print('[action]', kind, name)
        if not self.execute:
            self.wait(0.01)
            return
        for frame in self.frames[name]:
            if self.stop.is_set():
                raise Stopped('stop requested')
            if kind in ('forward', 'left', 'right'):
                check_hold(self.config)
            # Official TonyPi 1.20 diagram identifies 16 body bus servos.
            # Some local d6a files also contain 17/18 columns; keep them in the
            # parser but do not send commands to unverified extra bus channels.
            for servo, pulse in enumerate(frame[2:18], 1):
                if kind in ('forward', 'left', 'right'):
                    pulse = self.hold_values.get(servo, pulse)
                if self.controller:
                    self.controller.set_bus_servo_pulse(servo, pulse, frame[1])
                else:
                    self.old_board.setBusServoPulse(servo, pulse, frame[1])
            self.wait(frame[1] / 1000.0)
        if kind == 'grab':
            last = self.frames[name][-1]
            self.hold_values = {servo: last[servo + 1] for servo in self.config.get('hold_arm_servos', [])}
        # Keep the final pose. Never insert a generic stand action while carrying.


class Camera(object):
    """Continuously drain camera so actions cannot leave old frames queued."""
    def __init__(self, source, stop):
        import cv2
        self.cv2 = cv2
        self.stop = stop
        self.lock = threading.Lock()
        self.frame, self.stamp = None, None
        if source == 'auto':
            try:
                module = importlib.import_module('hiwonder.Camera')
            except ImportError:
                self.cam = cv2.VideoCapture(0)
            else:
                self.cam = module.Camera()
                self.cam.camera_open()
        else:
            self.cam = cv2.VideoCapture(int(source) if str(source).isdigit() else source)
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()

    def run(self):
        while not self.stop.is_set():
            ok, frame = self.cam.read()
            if ok and frame is not None:
                with self.lock:
                    self.frame, self.stamp = frame.copy(), time.monotonic()
            else:
                self.stop.wait(0.02)

    def read_after(self, stamp):
        with self.lock:
            if self.stamp is None or self.stamp <= stamp:
                return None, None
            return self.frame.copy(), self.stamp

    def close(self):
        self.stop.set()
        # Close to unblock a native capture read before joining.
        if hasattr(self.cam, 'camera_close'):
            self.cam.camera_close()
        else:
            self.cam.release()
        self.worker.join(1)


def detect(frame, config):
    import cv2
    import numpy as np
    image = cv2.resize(frame, (320, 240))
    lab = cv2.cvtColor(cv2.GaussianBlur(image, (3, 3), 0), cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(lab, np.array(config['lower_lab'], dtype=np.uint8), np.array(config['upper_lab'], dtype=np.uint8))
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    valid = [c for c in contours if config['min_area'] <= cv2.contourArea(c) / (320 * 240.0) <= config['max_area']]
    if not valid:
        return None, mask
    contour = max(valid, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    return ((x + w / 2.0) / 320, (y + h) / 240.0, cv2.contourArea(contour) / (320 * 240.0)), mask


def check_hold(config):
    path = config.get('hold_sensor_file')
    if path:
        if not os.path.isfile(path) or not 0 <= time.time() - os.path.getmtime(path) <= config['hold_sensor_timeout']:
            raise RuntimeError('Grasp sensor missing/stale; holding at current pose')
        if Path(path).read_text().strip() != 'held':
            raise RuntimeError('Object dropped/not held; return to drop location before resuming')


def carry(config, motion, heartbeat):
    heartbeat.set('carrying')
    for step in config['route']:
        if step['kind'] == 'wait':
            remaining = float(step['seconds'])
            while remaining > 0:
                check_hold(config)
                duration = min(0.1, remaining)
                motion.wait(duration)
                remaining -= duration
        else:
            for unused in range(step['cycles']):
                check_hold(config)
                motion.action(step['kind'])
                motion.wait(config['settle_seconds'])
    check_hold(config)
    heartbeat.set('releasing')
    motion.action('release')
    heartbeat.set('done')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(ROOT / 'humanoid/config/competition.yaml'))
    parser.add_argument('--execute', action='store_true', help='send servo commands after calibration checks')
    parser.add_argument('--simulate', action='store_true', help='synthetic vision and printed route, no camera/hardware')
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--check-actions', action='store_true', help='validate configured Pi action files without importing hardware SDK')
    args = parser.parse_args()
    import yaml
    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    validate_route(config)
    if any(not isinstance(i, int) or not 1 <= i <= 16 for i in config.get('hold_arm_servos', [])):
        parser.error('hold_arm_servos must contain bus servo IDs 1..16')
    if args.check_actions:
        for name in validate_route(config):
            rows = action_frames(Path(config['action_dir']) / (name + '.d6a'))
            print(name, 'frames=', len(rows), 'seconds=', sum(row[1] for row in rows) / 1000.0)
        return
    if args.execute and args.simulate:
        parser.error('--simulate cannot be combined with --execute')
    if args.execute and config['hardware_calibrated'] is not True:
        parser.error('Record/test right turn and release, calibrate route, then set hardware_calibrated: true')
    if not args.execute:
        config['heartbeat']['enabled'] = False  # dry runs must not launch the real followers
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *unused: stop.set())
    signal.signal(signal.SIGTERM, lambda *unused: stop.set())
    motion = Motion(config, stop, args.execute)
    heartbeat = Heartbeat(config['heartbeat'])
    camera = None
    try:
        motion.head()
        heartbeat.set('searching')
        planner = Approach(config['vision'])
        if args.simulate:
            for unused in range(config['vision']['stable_frames']):
                decision = planner.decide((config['vision']['center_x'], 0.95, 0.1))
            if decision != 'grab':
                raise RuntimeError('simulation did not reach grab state')
        else:
            camera = Camera(config['camera'], stop)
            deadline = time.monotonic() + config['search_timeout']
            last = time.monotonic()
            actions = 0
            while not stop.is_set():
                now = time.monotonic()
                if now >= deadline:
                    raise RuntimeError('Object search/approach timed out')
                frame, stamp = camera.read_after(last)
                if frame is None:
                    if now - last > config['camera_timeout']:
                        raise RuntimeError('Camera frames missing/stale')
                    motion.wait(0.02)
                    continue
                last = stamp
                observation, mask = detect(frame, config['vision'])
                decision = planner.decide(observation)
                if args.debug:
                    camera.cv2.imshow('object mask', mask)
                    if camera.cv2.waitKey(1) & 0xff in (27, ord('q')):
                        stop.set()
                        raise Stopped('debug quit')
                if decision == 'grab':
                    break
                if decision != 'wait':
                    actions += 1
                    if actions > config['approach_action_limit']:
                        raise RuntimeError('Approach action limit exceeded')
                    motion.action(decision)
                    motion.wait(config['settle_seconds'])
                    last = time.monotonic()  # require a new observation AFTER body stops
                    planner.ready = 0
                    planner.last = None
            if stop.is_set():
                raise Stopped('stop requested')
        heartbeat.set('grasping')
        motion.action('grab')
        motion.wait(config['settle_seconds'])
        carry(config, motion, heartbeat)
        # Keep done heartbeat alive while Turtlebot finishes its trailing goal.
        if args.execute:
            print('Route done; keep this process running until all followers stop. Ctrl+C exits.')
            while not stop.wait(0.2):
                pass
    except Stopped as exc:
        heartbeat.set('stopped')
        print(exc)
    except Exception:
        heartbeat.set('fault')
        time.sleep(0.25)
        raise
    finally:
        stop.set()
        if camera:
            camera.close()
        if args.debug:
            import cv2
            cv2.destroyAllWindows()
        heartbeat.close()


if __name__ == '__main__':
    main()
