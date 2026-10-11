#!/usr/bin/env python
"""Run ONLY in an isolated ROS master without a robot or real move_base."""
from __future__ import print_function
import os
import subprocess
import sys
import time

import actionlib
import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult

rospy.init_node('navigation_interface_check')
if rospy.get_published_topics('/odom'):
    raise RuntimeError('Refusing test: odometry publisher detected')
received = []
mode = ['success']
cancelled = [False]


def execute(goal):
    received.append(goal)
    if mode[0] == 'success':
        server.set_succeeded(MoveBaseResult())
    elif mode[0] == 'failure':
        server.set_aborted(MoveBaseResult())
    else:
        deadline = time.time() + 10
        while not server.is_preempt_requested() and time.time() < deadline:
            time.sleep(0.02)
        cancelled[0] = server.is_preempt_requested()
        server.set_preempted(MoveBaseResult())


server = actionlib.SimpleActionServer('/move_base', MoveBaseAction, execute_cb=execute, auto_start=False)
server.start()
broadcaster = tf.TransformBroadcaster()
timer = rospy.Timer(rospy.Duration(0.05), lambda event: broadcaster.sendTransform(
    (0, 0, 0), (0, 0, 0, 1), rospy.Time.now(), 'base_footprint', 'map'))
script = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'navigation_goal.py')
command = [sys.executable, script, '--goal', '1', '2', '90',
           '--goal', '2', '3', '0', '--goal', '0', '0', '180', '--timeout', '2']
assert subprocess.call(command) == 0
assert len(received) == 3
assert received[0].target_pose.header.frame_id == 'map'
assert abs(received[0].target_pose.pose.orientation.z - 2**-0.5) < 1e-6
mode[0] = 'failure'
received[:] = []
assert subprocess.call(command) == 1
assert len(received) == 1, 'Failure must stop the sequence'
mode[0] = 'timeout'
received[:] = []
assert subprocess.call(command) == 1
time.sleep(0.3)
assert len(received) == 1 and cancelled[0], 'Timeout must cancel current goal'
print('PASS: three ordered goals, map frame/yaw, failure stops, timeout cancels')
rospy.signal_shutdown('Checks completed')
