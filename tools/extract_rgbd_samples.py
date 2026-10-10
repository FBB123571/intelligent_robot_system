#!/usr/bin/env python
"""Export synchronized registered RGB-D calibration samples from a ROS bag.

Run under sourced ROS Kinetic. Edit generated manifest ROIs and view labels
before enrollment; keep separate bags for held-out validation.
"""
from __future__ import print_function
import argparse
import json
import os
import sys
import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT,'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.depth import check_pair


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag')
    p.add_argument('--output', required=True, help='New output directory')
    p.add_argument('--rgb', default='/camera/rgb/image_rect_color')
    p.add_argument('--depth', default='/camera/depth_registered/image_rect')
    p.add_argument('--info', default='/camera/rgb/camera_info')
    p.add_argument('--interval', type=float, default=1.)
    p.add_argument('--limit', type=int, default=40)
    args = p.parse_args()
    if args.interval <= 0 or args.limit < 1:
        p.error('interval and limit must be positive')
    os.mkdir(args.output)
    topics = [args.rgb,args.depth,args.info]
    latest, manifest, last = {}, [], None
    bridge = CvBridge()
    with rosbag.Bag(args.bag) as bag:
        for topic,msg,unused in bag.read_messages(topics=topics):
            latest[topic] = msg
            if len(latest) != 3:
                continue
            rgb,depth,info = [latest[t] for t in topics]
            stamps = [m.header.stamp.to_sec() for m in (rgb,depth,info)]
            if last is not None and stamps[1]-last < args.interval:
                continue
            try:
                check_pair(stamps,max(stamps),.5,.05,[m.header.frame_id for m in (rgb,depth,info)],
                           [(m.width,m.height) for m in (rgb,depth,info)],depth.encoding)
            except ValueError:
                continue
            name = 'view_%03d' % len(manifest)
            image = bridge.imgmsg_to_cv2(rgb,'bgr8')
            values = bridge.imgmsg_to_cv2(depth,'passthrough')
            if not cv2.imwrite(os.path.join(args.output,name+'.png'),image):
                raise IOError('Cannot save sample')
            np.save(os.path.join(args.output,name+'.npy'),values,allow_pickle=False)
            manifest.append(dict(rgb=name+'.png',depth=name+'.npy',encoding=depth.encoding,
                                 camera=[info.P[0],info.P[5],info.P[2],info.P[6]],
                                 roi=[0,0,rgb.width,rgb.height],label=name,stamp=stamps[1]))
            last = stamps[1]
            if len(manifest)>=args.limit:
                break
    with open(os.path.join(args.output,'manifest.json'),'w') as f:
        json.dump(manifest,f,indent=2)
    if not manifest:
        raise ValueError('No synchronized registered samples; check topic names and timestamps')
    print('Exported %d samples. Edit ROIs and labels before enrollment.' % len(manifest))


if __name__ == '__main__':
    main()
