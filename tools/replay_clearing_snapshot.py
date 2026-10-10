#!/usr/bin/env python
"""Replay saved depth against seeded old voxels on a NEW loopback ROS master.

Requires ROS Kinetic, NumPy and the rgbd_following package. This is an isolated
VoxelLayer regression, not a recording replay or evidence of live clear latency.
No goals or velocity commands are published. The caller's ROS master is ignored.
"""
from __future__ import print_function
import argparse
import copy
import hashlib
import json
import os
import signal
import socket
import subprocess
import threading
import time

import numpy as np


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    deadline = time.time() + 8.0
    while process.poll() is None and time.time() < deadline:
        time.sleep(.1)
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait()


def rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True,
                        help='Directory containing metadata/depth/voxels/comparison')
    parser.add_argument('--package-dir', required=True)
    parser.add_argument('--output', required=True, help='Isolated logs and report directory')
    parser.add_argument('--stride', type=int, default=4)
    parser.add_argument('--ranges', type=float, nargs='+', default=[4.0, 4.5])
    args = parser.parse_args()
    if args.stride < 1 or any(not np.isfinite(v) or v <= .4 for v in args.ranges):
        parser.error('Invalid sampling stride or depth limits')
    source = os.path.abspath(args.dataset)
    output = os.path.abspath(args.output)
    if not os.path.isdir(output):
        os.makedirs(output)
    with open(os.path.join(source, 'metadata.json')) as stream:
        metadata = json.load(stream)
    with open(os.path.join(source, 'comparison.json')) as stream:
        comparison = json.load(stream)
    depth = np.load(os.path.join(source, 'depth.npy'))
    saved = np.load(os.path.join(source, 'voxels.npy'))
    cells = comparison['vacated_right_surface_cells']['cells']
    origin = np.asarray(metadata['voxel_origin'], dtype=float)
    resolution = np.asarray(metadata['voxel_resolution'], dtype=float)
    voxels = [(int(x), int(y), z) for x, y in cells
              for z in range(int(metadata['voxel_size_z']))
              if int(saved[y, x]) & (1 << (16+z))]
    if not voxels:
        raise RuntimeError('Dataset contains no marked voxels in selected old columns')
    world = origin + (np.asarray(voxels, dtype=float) + .5) * resolution
    sensor = np.asarray(metadata['odom_optical_translation'], dtype=float)
    rot = rotation(metadata['odom_optical_quaternion'])
    marking = (world - sensor).dot(rot).astype(np.float32).tolist()

    # Select a private port, launch the master ourselves, and never use 11311.
    sock = socket.socket()
    sock.bind(('127.0.0.1', 0))
    port = sock.getsockname()[1]
    sock.close()
    if port == 11311:
        raise RuntimeError('Refusing the hardware master port')
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%s' % port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_HOME'] = os.path.join(output, 'ros_home')
    master = launch = None
    tf_stop = tf_thread = None
    logs = []
    try:
        log = open(os.path.join(output, 'master.log'), 'w')
        logs.append(log)
        master = subprocess.Popen(['roscore', '-p', str(port)], stdout=log,
                                  stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        try:
            import xmlrpclib as xmlrpc
        except ImportError:
            import xmlrpc.client as xmlrpc
        deadline = time.time() + 15
        while True:
            if master.poll() is not None:
                raise RuntimeError('Private ROS master failed to start')
            try:
                pid = xmlrpc.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/snapshot_probe')
                if pid[0] == 1:
                    break
            except (socket.error, IOError):
                pass
            if time.time() > deadline:
                raise RuntimeError('Private ROS master startup timed out')
            time.sleep(.1)

        import rospy
        import tf2_ros
        import yaml
        from costmap_2d.msg import VoxelGrid
        from geometry_msgs.msg import TransformStamped
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Header

        rospy.init_node('snapshot_clearing_replay', disable_signals=True)
        broadcaster = tf2_ros.TransformBroadcaster()
        transforms = []
        for child, translation, quaternion in (
                ('snapshot_base', metadata['odom_base_translation'], metadata['odom_base_quaternion']),
                ('snapshot_camera', metadata['odom_optical_translation'], metadata['odom_optical_quaternion'])):
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id, msg.child_frame_id = 'snapshot_odom', child
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = translation
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = quaternion
            transforms.append(msg)
        tf_stop = threading.Event()
        def broadcast():
            while not tf_stop.is_set():
                for transform in transforms:
                    transform.header.stamp = rospy.Time.now()
                broadcaster.sendTransform(transforms)
                tf_stop.wait(.025)
        tf_thread = threading.Thread(target=broadcast)
        tf_thread.daemon = True
        tf_thread.start()
        with open(os.path.join(args.package_dir, 'config', 'navigation.yaml')) as stream:
            navigation = yaml.safe_load(stream)
        costmap_file = os.path.join(args.package_dir, 'config', 'costmap.yaml')
        with open(costmap_file) as stream:
            costmap = yaml.safe_load(stream)
        if costmap['obstacles']['observation_sources'].split() != ['depth', 'clearing']:
            raise RuntimeError('Replay requires the separate production marking/clearing sources')
        config_hash = hashlib.sha256(open(costmap_file, 'rb').read()).hexdigest()
        results = []
        for number, far in enumerate(args.ranges):
            ns = '/snapshot_%s' % number
            node = ns + '/move_base'
            params = copy.deepcopy(navigation)
            for scope in ('global_costmap', 'local_costmap'):
                params[scope].update(copy.deepcopy(costmap))
                params[scope].update(dict(global_frame='snapshot_odom', robot_base_frame='snapshot_base',
                    rolling_window=False, width=saved.shape[1]*resolution[0],
                    height=saved.shape[0]*resolution[1], resolution=resolution[0],
                    origin_x=float(origin[0]), origin_y=float(origin[1]),
                    update_frequency=10., publish_frequency=10.))
                params[scope]['obstacles'].update(origin_z=float(origin[2]),
                    z_resolution=float(resolution[2]), z_voxels=int(metadata['voxel_size_z']))
                params[scope]['obstacles']['depth']['topic'] = ns+'/obstacle_cloud'
                params[scope]['obstacles']['clearing']['topic'] = ns+'/clearing_cloud'
            # Convert NumPy scalars before XML-RPC serialization.
            params = json.loads(json.dumps(params))
            rospy.set_param(node, params)
            launch_path = os.path.join(output, 'trial_%s.launch' % number)
            with open(launch_path, 'w') as stream:
                stream.write('<launch><group ns="snapshot_%s"><node pkg="move_base" '
                    'type="move_base" name="move_base" output="screen">'
                    '<remap from="cmd_vel" to="unused_cmd_vel"/>'
                    '<remap from="odom" to="unused_odom"/>'
                    '</node></group></launch>\n' % number)
            log = open(os.path.join(output, 'trial_%s.log' % number), 'w')
            logs.append(log)
            launch = subprocess.Popen(['roslaunch', launch_path], stdout=log,
                                      stderr=subprocess.STDOUT, preexec_fn=os.setsid)
            latest = [None]
            def receive(msg):
                latest[0] = msg
            sub = rospy.Subscriber(node+'/local_costmap/obstacles/voxel_grid', VoxelGrid, receive)
            mark = rospy.Publisher(ns+'/obstacle_cloud', PointCloud2, queue_size=1)
            clear = rospy.Publisher(ns+'/clearing_cloud', PointCloud2, queue_size=1)
            deadline = time.time()+20
            # Global/local maps live in one roscpp process and share transport.
            while mark.get_num_connections()<1 or clear.get_num_connections()<1:
                if time.time()>deadline or launch.poll() is not None:
                    raise RuntimeError('Costmap observations not ready: mark=%s clear=%s' %
                                       (mark.get_num_connections(), clear.get_num_connections()))
                time.sleep(.1)
            def publish(mark_points, clear_points):
                header = Header(stamp=rospy.Time.now(), frame_id='snapshot_camera')
                mark.publish(point_cloud2.create_cloud_xyz32(header, mark_points))
                clear.publish(point_cloud2.create_cloud_xyz32(header, clear_points))
            def states():
                grid = latest[0]
                if grid is None:
                    return None
                observed_origin = np.array([grid.origin.x, grid.origin.y, grid.origin.z])
                observed_res = np.array([grid.resolutions.x, grid.resolutions.y, grid.resolutions.z])
                indices = np.floor((world-observed_origin)/observed_res).astype(int)
                result = []
                for x,y,z in indices:
                    if not (0<=x<grid.size_x and 0<=y<grid.size_y and 0<=z<grid.size_z):
                        raise RuntimeError('Seeded voxel outside replay map')
                    col = grid.data[y*grid.size_x+x]
                    result.append(int(((col>>z)&1)|(((col>>(z+16))&1)<<1)))
                return result
            deadline=time.time()+8
            while time.time()<deadline:
                publish(marking, [])
                time.sleep(.1)
                if states() == [3]*len(voxels):
                    break
            initial=states()
            if initial != [3]*len(voxels):
                raise RuntimeError('Failed to seed exact old occupied voxels: %r' % initial)
            fx,fy,cx,cy=metadata['camera']
            v,u=np.mgrid[0:depth.shape[0]:args.stride,0:depth.shape[1]:args.stride]
            z=depth[::args.stride,::args.stride]
            valid=np.isfinite(z)
            valid[valid] &= (z[valid]>=.4)&(z[valid]<=far)
            z=z[valid]
            points=np.column_stack(((u[valid]-cx)*z/fx,(v[valid]-cy)*z/fy,z)).astype(np.float32).tolist()
            series=[]
            deadline=time.time()+4
            previous=None
            while time.time()<deadline:
                publish([], points)
                time.sleep(.1)
                grid=latest[0]
                if grid is not None and grid.header.stamp!=previous:
                    series.append(states())
                    previous=grid.header.stamp
            final=states()
            remaining=[voxels[i] for i,state in enumerate(final) if state==3]
            result=dict(depth_limit=far,stride=args.stride,clearing_endpoints=len(points),
                seed_voxels=voxels,initial_states=initial,final_states=final,
                remaining_voxels=remaining,remaining_columns=len(set((v[0],v[1]) for v in remaining)),
                distinct_map_updates=len(series),state_series=series)
            results.append(result)
            print(json.dumps(dict(depth_limit=far, seeded=len(voxels),
                remaining_voxels=len(remaining), remaining_columns=result['remaining_columns'],
                distinct_map_updates=len(series))))
            sub.unregister(); mark.unregister(); clear.unregister()
            stop(launch); launch=None
        report=dict(dataset=source,private_master=os.environ['ROS_MASTER_URI'],
            production_costmap_sha256=config_hash,hardware_master_used=False,
            observation='Saved snapshot rays against synthetic seeded old occupied voxels; no live latency claim',
            trials=results)
        with open(os.path.join(output,'report.json'),'w') as stream:
            json.dump(report,stream,indent=2,sort_keys=True)
        print('Report: '+os.path.join(output,'report.json'))
    finally:
        if tf_stop is not None:
            tf_stop.set()
        if tf_thread is not None:
            tf_thread.join(2.)
        stop(launch)
        stop(master)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
