#!/usr/bin/env python
"""Send explicit map-frame goals; stop the sequence on failure or timeout."""
from __future__ import print_function
import argparse
import math
import sys
import time

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=3, type=float, action='append',
                        metavar=('X', 'Y', 'YAW_DEG'), required=True)
    parser.add_argument('--timeout', type=float, default=120.0)
    args = parser.parse_args(rospy.myargv()[1:])
    values = [args.timeout] + [v for goal in args.goal for v in goal]
    if any(math.isnan(v) or math.isinf(v) for v in values) or args.timeout <= 0:
        parser.error('Coordinates must be finite and timeout must be positive')
    rospy.init_node('lab3_navigation_goal')
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    active = [False]

    def cancel():
        if active[0]:
            client.cancel_goal()
            active[0] = False
            time.sleep(0.2)

    rospy.on_shutdown(cancel)
    if not client.wait_for_server(rospy.Duration(15)):
        rospy.logerr('No move_base action server')
        return 1
    listener = tf.TransformListener()
    try:
        listener.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(10))
    except tf.Exception as error:
        rospy.logerr('Missing localization TF: %s', error)
        return 1
    for x, y, yaw in args.goal:
        if rospy.is_shutdown():
            return 1
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(math.radians(yaw) / 2)
        goal.target_pose.pose.orientation.w = math.cos(math.radians(yaw) / 2)
        active[0] = True
        client.send_goal(goal)
        rospy.loginfo('Goal: x=%.3f y=%.3f yaw=%.1f deg', x, y, yaw)
        deadline = time.time() + args.timeout
        terminal = (GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED,
                    GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST)
        while not rospy.is_shutdown() and client.get_state() not in terminal:
            if time.time() >= deadline:
                cancel()
                rospy.logerr('Goal timed out; cancellation sent, sequence stopped')
                return 1
            time.sleep(0.1)
        if rospy.is_shutdown():
            return 1
        active[0] = False
        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('Navigation failed: %s', client.get_goal_status_text())
            return 1
        rospy.loginfo('Goal reached')
    return 0


if __name__ == '__main__':
    sys.exit(main())
