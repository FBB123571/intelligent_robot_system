#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Wall-clock watchdog + swept costmap footprint check, sole velocity output."""
import threading
import time
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from rgbd_following_core.depth import finite
from rgbd_following_core.motion import fresh, swept_clear
from rgbd_following_core.settings import validate
from rgbd_following_core.ros_helpers import grid_message, transform_pose


class Gate(object):
    def __init__(self):
        self.execute = rospy.get_param('~execute', False)
        self.c = validate(rospy.get_param('~settings'), self.execute)
        self.frame, self.base = rospy.get_param('~odom_frame', 'odom'), rospy.get_param('~base_frame', 'base_footprint')
        self.clock = getattr(time, 'monotonic', time.time)
        self.lock = threading.RLock()
        self.command = Twist()
        self.command_received = self.allow_received = self.target_received = None
        self.cloud_stamp = self.cloud_received = self.grid_stamp = self.grid_received = None
        self.grid = None
        self.allowed = self.target_valid = self.estop = False
        self.reason = 'waiting'
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        # Shadow mode has no publisher on the base's navigation input.
        self.pub = rospy.Publisher('cmd_vel_safe' if self.execute else 'cmd_vel_preview', Twist, queue_size=1)
        self.status = rospy.Publisher('gate_status', String, queue_size=1)
        self.subs = [rospy.Subscriber('cmd_vel_raw', Twist, self.on_command, queue_size=1),
                     rospy.Subscriber('motion_allowed', Bool, self.on_allowed, queue_size=1),
                     rospy.Subscriber('target_valid', Bool, self.on_target, queue_size=1),
                     rospy.Subscriber('estop', Bool, self.on_estop, queue_size=1),
                     rospy.Subscriber('obstacle_cloud', PointCloud2, self.on_cloud, queue_size=1),
                     rospy.Subscriber('move_base/local_costmap/costmap', OccupancyGrid, self.on_grid, queue_size=1)]
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()

    def on_command(self, msg):
        with self.lock:
            self.command, self.command_received = msg, self.clock()

    def on_allowed(self, msg):
        with self.lock:
            self.allowed, self.allow_received = msg.data, self.clock()
            if not msg.data:
                self.pub.publish(Twist())

    def on_target(self, msg):
        with self.lock:
            self.target_valid, self.target_received = msg.data, self.clock()
            if not msg.data:
                self.pub.publish(Twist())

    def on_estop(self, msg):
        with self.lock:
            self.estop = msg.data
            if msg.data:
                self.pub.publish(Twist())

    def on_cloud(self, msg):
        with self.lock:
            stamp = msg.header.stamp.to_sec()
            if (msg.width * msg.height >= self.c['perception']['min_cloud_points'] and
                    fresh(stamp, rospy.Time.now().to_sec(), self.c['target_timeout']) and
                    (self.cloud_stamp is None or stamp > self.cloud_stamp)):
                self.cloud_stamp, self.cloud_received = stamp, self.clock()

    def on_grid(self, msg):
        with self.lock:
            try:
                self.grid = grid_message(msg, self.frame)
                self.grid_stamp, self.grid_received = msg.header.stamp.to_sec(), self.clock()
            except ValueError:
                self.grid = None

    def output(self):
        c, wall, now = self.c, self.clock(), rospy.Time.now().to_sec()
        zero = Twist()
        self.reason = 'permission_or_command_stale'
        if (self.estop or not self.allowed or not self.target_valid or
                not fresh(self.command_received, wall, .3) or not fresh(self.allow_received, wall, .3) or
                not fresh(self.target_received, wall, c['target_timeout'])):
            return zero
        self.reason = 'depth_or_costmap_stale'
        if (self.grid is None or not fresh(self.cloud_received, wall, c['target_timeout']) or
                not fresh(self.cloud_stamp, now, c['target_timeout']) or
                not fresh(self.grid_received, wall, c['grid_timeout']) or
                not fresh(self.grid_stamp, now, c['grid_timeout'])):
            return zero
        v, w = self.command.linear.x, self.command.angular.z
        self.reason = 'invalid_navigation_velocity'
        all_values = (v, w, self.command.linear.y, self.command.linear.z, self.command.angular.x, self.command.angular.y)
        if not all(finite(x) for x in all_values) or v < 0 or any(abs(x) > 1e-6 for x in all_values[2:]):
            return zero
        # Reject oversized commands instead of independently clipping v/w and
        # changing the collision-checked curvature produced by DWA.
        if v > c['max_linear'] + 1e-6 or abs(w) > c['max_angular'] + 1e-6:
            return zero
        self.reason = 'base_tf_invalid'
        try:
            tf = self.buffer.lookup_transform(self.frame, self.base, rospy.Time(0), rospy.Duration(.02))
            if not fresh(tf.header.stamp.to_sec(), now, c['tf_timeout']):
                return zero
            pose = transform_pose(tf)
        except (ValueError, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return zero
        self.reason = 'swept_footprint_blocked'
        if not swept_clear(self.grid, pose, v, w, c['footprint_radius'], c['braking_acceleration'], c['reaction_time']):
            return zero
        self.reason = 'executing' if self.execute else 'shadow_velocity_only'
        out = Twist()
        out.linear.x, out.angular.z = v, w
        return out

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    self.pub.publish(self.output())
                    self.status.publish(self.reason)
                except Exception as exc:
                    self.pub.publish(Twist())
                    rospy.logerr_throttle(2, 'Gate error: ' + str(exc))
            time.sleep(.05)


if __name__ == '__main__':
    rospy.init_node('rgbd_velocity_gate')
    Gate()
    rospy.spin()
