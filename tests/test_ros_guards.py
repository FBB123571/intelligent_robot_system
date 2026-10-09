"""Exercise node stop/forward decisions with message doubles; no ROS master needed.

This verifies adapter control flow, not real navigation or sensor drivers.
"""
import sys
import unittest


@unittest.skipIf(sys.version_info[0] < 3, 'Adapter test harness runs on Python 3')
class RosGuardTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        from pathlib import Path
        from types import ModuleType, SimpleNamespace as NS
        from unittest.mock import patch
        self.NS = NS
        self.now = 100.0
        def module(name, **members):
            out = ModuleType(name)
            out.__dict__.update(members)
            return out
        class Stamp(object):
            def __init__(self, seconds):
                self.seconds = seconds
            def to_sec(self):
                return self.seconds
        self.Stamp = Stamp
        owner = self
        class Time(object):
            def __init__(self, value=0):
                self.value = value
            @staticmethod
            def now():
                return Stamp(owner.now)
        def twist():
            return NS(linear=NS(x=0., y=0., z=0.), angular=NS(x=0., y=0., z=0.))
        self.twist = twist
        def goal():
            return NS(target_pose=NS(header=NS(frame_id='', stamp=None),
                                     pose=NS(position=NS(x=0., y=0.), orientation=NS(z=0., w=0.))))
        fake = {
            'rospy': module('rospy', Time=Time, Duration=lambda x: x),
            'tf2_ros': module('tf2_ros', LookupException=LookupError, ConnectivityException=ConnectionError, ExtrapolationException=ValueError),
            'tf2_geometry_msgs': module('tf2_geometry_msgs'),
            'ar_track_alvar_msgs': module('ar_track_alvar_msgs'),
            'ar_track_alvar_msgs.msg': module('ar_track_alvar_msgs.msg', AlvarMarkers=object),
            'actionlib': module('actionlib'),
            'geometry_msgs': module('geometry_msgs'),
            'geometry_msgs.msg': module('geometry_msgs.msg', Twist=twist, PoseStamped=object, PoseWithCovarianceStamped=object),
            'sensor_msgs': module('sensor_msgs'),
            'sensor_msgs.msg': module('sensor_msgs.msg', LaserScan=object),
            'nav_msgs': module('nav_msgs'),
            'nav_msgs.msg': module('nav_msgs.msg', OccupancyGrid=object),
            'std_msgs': module('std_msgs'),
            'std_msgs.msg': module('std_msgs.msg', Bool=bool, String=str),
            'move_base_msgs': module('move_base_msgs'),
            'move_base_msgs.msg': module('move_base_msgs.msg', MoveBaseAction=object, MoveBaseGoal=goal),
        }
        self.patcher = patch.dict(sys.modules, fake)
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        root = Path(__file__).resolve().parents[1]
        sys.path.insert(0, str(root / 'turtlebot/follow2/multi_robot_following/src'))
        scripts = root / 'turtlebot/follow2/multi_robot_following/scripts'
        def load(name):
            spec = importlib.util.spec_from_file_location('test_' + name, scripts / (name + '.py'))
            out = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(out)
            return out
        self.gate_module = load('velocity_gate')
        self.follow_module = load('map_follower')
        self.marker_module = load('marker_target')

    def gate(self):
        gate = self.gate_module.Gate.__new__(self.gate_module.Gate)
        gate.clock = lambda: self.now
        gate.command = self.twist()
        gate.command.linear.x = .5
        gate.command.angular.z = .8
        gate.command_stamp = gate.allowed_stamp = gate.scan_stamp = self.now
        gate.allowed = True
        gate.estop = False
        gate.max_v, gate.max_w, gate.clearance = .15, .5, .5
        gate.require_scan = True
        gate.scan = self.NS(ranges=[2.] * 31, angle_min=-1.5, angle_increment=.1, range_min=.1, range_max=10.)
        return gate

    def follower(self):
        from competition_core.planning import Trail
        from unittest.mock import Mock
        node = self.follow_module.Follower.__new__(self.follow_module.Follower)
        node.frame, node.base = 'map', 'tb2/base_footprint'
        node.gap, node.min_gap, node.radius = .9, .65, .25
        node.target_timeout, node.tf_timeout = .8, .8
        node.goal_interval, node.goal_shift, node.finish_tolerance = 1., .25, .2
        node.trail = Trail(jump_limit=2.)
        node.trail.add(2., 2.)
        node.trail.add(3., 2.)
        node.target = (3., 2.)
        node.target_stamp = node.loc_stamp = node.mission_stamp = node.grid_stamp = self.now
        node.mission, node.loc_good = 'carrying', True
        node.estop = node.finished = node.failed = False
        node.goal = None
        node.goal_time = 0
        node.grid = self.NS(header=self.NS(frame_id='map'), data=[0] * 10000,
                            info=self.NS(width=100, height=100, resolution=.1,
                                         origin=self.NS(position=self.NS(x=0., y=0.), orientation=self.NS(x=0., y=0., z=0., w=1.))))
        node.allowed, node.status, node.client, node.buffer = Mock(), Mock(), Mock(), Mock()
        node.client.wait_for_server.return_value = True
        node.client.get_state.return_value = 1
        node.buffer.lookup_transform.return_value = self.NS(header=self.NS(stamp=self.Stamp(self.now)),
                                                            transform=self.NS(translation=self.NS(x=1.5, y=2.)))
        return node

    def assert_zero(self, gate):
        out = gate.output()
        self.assertEqual((out.linear.x, out.angular.z), (0., 0.))

    def test_gate_limits_navigation_speed(self):
        out = self.gate().output()
        self.assertEqual((out.linear.x, out.angular.z), (.15, .5))

    def test_gate_stale_permission_command_scan_and_estop(self):
        for key in ('allowed_stamp', 'command_stamp', 'scan_stamp'):
            gate = self.gate()
            setattr(gate, key, self.now - 2)
            self.assert_zero(gate)
        gate = self.gate()
        gate.estop = True
        self.assert_zero(gate)

    def test_gate_rejects_reverse_nonfinite_and_front_obstacle(self):
        for value in (-.1, float('nan'), float('inf')):
            gate = self.gate()
            gate.command.linear.x = value
            self.assert_zero(gate)
        gate = self.gate()
        gate.scan.ranges[15] = .3
        self.assert_zero(gate)

    def test_follow_sends_measured_trailing_goal(self):
        node = self.follower()
        node.step()
        goal = node.client.send_goal.call_args[0][0]
        self.assertAlmostEqual(goal.target_pose.pose.position.x, 2.1)
        node.allowed.publish.assert_called_with(True)

    def test_loss_cancels_goal_and_closes_gate(self):
        node = self.follower()
        node.goal = (2.1, 2., 0.)
        node.target_stamp -= 2
        node.step()
        node.client.cancel_goal.assert_called_once()
        node.allowed.publish.assert_called_with(False)
        node.status.publish.assert_called_with('target_lost_hold_position')

    def test_stale_tf_bad_localization_costmap_or_mission_stops(self):
        for key in ('loc_stamp', 'mission_stamp', 'grid_stamp'):
            node = self.follower()
            setattr(node, key, self.now - 5)
            node.step()
            node.client.send_goal.assert_not_called()
            node.allowed.publish.assert_called_with(False)
        node = self.follower()
        node.buffer.lookup_transform.return_value.header.stamp = self.Stamp(self.now - 2)
        node.step()
        node.status.publish.assert_called_with('base_tf_stale')

    def test_close_target_does_not_reverse(self):
        node = self.follower()
        node.target = (1.6, 2.)
        node.step()
        node.status.publish.assert_called_with('leader_too_close')
        node.client.send_goal.assert_not_called()

    def test_navigation_failure_latches_stop(self):
        node = self.follower()
        node.goal = (2.1, 2., 0.)
        node.client.get_state.return_value = 4
        node.step()
        self.assertTrue(node.failed)
        node.client.send_goal.assert_not_called()
        node.allowed.publish.assert_called_with(False)

    def test_finished_is_latched(self):
        node = self.follower()
        node.mission = 'done'
        node.buffer.lookup_transform.return_value.transform.translation.x = 2.1
        node.step()
        self.assertTrue(node.finished)
        node.allowed.publish.assert_called_with(False)

    def test_alvar_marker_header_used_when_nested_pose_header_is_empty(self):
        from unittest.mock import Mock
        node = self.marker_module.MarkerTarget.__new__(self.marker_module.MarkerTarget)
        node.marker_id, node.frame, node.timeout, node.max_range = 1, 'map', .5, 4.
        node.buffer, node.pub = Mock(), Mock()
        node.buffer.transform.side_effect = lambda pose, frame, timeout: pose
        pose = self.NS(header=self.NS(frame_id='', stamp=self.Stamp(0)),
                       pose=self.NS(position=self.NS(x=0., y=0., z=1.)))
        marker = self.NS(id=1, pose=pose, header=self.NS(frame_id='tb2/camera_rgb_optical_frame', stamp=self.Stamp(self.now)))
        message = self.NS(markers=[marker], header=self.NS(frame_id='', stamp=self.Stamp(0)))
        node.on_markers(message)
        published = node.pub.publish.call_args[0][0]
        self.assertEqual(published.header.stamp.to_sec(), self.now)
        self.assertEqual(node.buffer.transform.call_args[0][0].header.frame_id, marker.header.frame_id)
        self.assertEqual(pose.header.frame_id, '')  # original measurement is not mutated


if __name__ == '__main__':
    unittest.main()
