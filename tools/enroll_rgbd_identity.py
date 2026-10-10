#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Enroll manually selected TonyPi views from registered RGB + depth samples.

Manifest JSON: [{"rgb":"front.png", "depth":"front.npy", "encoding":"16UC1",
 "camera":[fx,fy,cx,cy], "roi":[x,y,width,height], "label":"front-carrying"}, ...]
Paths are relative to the manifest. ROI must contain exactly one seed center.
"""
from __future__ import print_function
import argparse
import copy
import json
import os
import sys
import cv2
import numpy as np
import yaml
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/src'))
from rgbd_following_core.vision import detect, metric_depth
from rgbd_following_core.settings import validate


def enroll(config, samples, directory):
    result = copy.deepcopy(config)
    result['perception']['identity']['references'] = []
    references = []
    for sample in samples:
        image = cv2.imread(os.path.join(directory, sample['rgb']))
        depth = np.load(os.path.join(directory, sample['depth']), allow_pickle=False)
        if image is None or image.shape[:2] != depth.shape:
            raise ValueError('Missing or unregistered sample: '+sample['rgb'])
        camera = sample['camera']
        if len(camera) != 4 or not all(np.isfinite(camera)) or min(camera[:2]) <= 0:
            raise ValueError('Invalid sample camera intrinsics')
        x, y, w, h = sample['roi']
        if min(x,y) < 0 or min(w,h) <= 0 or x+w > image.shape[1] or y+h > image.shape[0]:
            raise ValueError('ROI outside sample')
        candidates, unused = detect(image, metric_depth(depth, sample['encoding']), camera, result['perception'])
        selected = [c for c in candidates if x <= c['bbox'][0]+c['bbox'][2]/2. < x+w and
                    y <= c['bbox'][1]+c['bbox'][3]/2. < y+h]
        if len(selected) != 1:
            raise ValueError('ROI must contain exactly one detected seed: '+sample['rgb'])
        ref = selected[0]['feature']
        ref['label'] = sample['label']
        references.append(ref)
    if not references:
        raise ValueError('No identity samples')
    result['perception']['identity']['references'] = references
    for flag in ('hardware_calibrated', 'identity_calibrated', 'depth_coverage_calibrated'):
        result[flag] = False
    return validate(result)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('manifest')
    p.add_argument('--config', default=os.path.join(ROOT, 'turtlebot/follow3/rgbd_following/config/follow.yaml'))
    p.add_argument('--output', required=True)
    p.add_argument('--evaluate', action='store_true', help='Held-out samples: roi=null means target absent. Output metrics JSON.')
    args = p.parse_args()
    with open(args.config) as f:
        config = yaml.safe_load(f)
    with open(args.manifest) as f:
        samples = json.load(f)
    directory = os.path.dirname(os.path.abspath(args.manifest))
    result = evaluate(config, samples, directory) if args.evaluate else enroll(config, samples, directory)
    # Exclusive creation: never overwrite field calibration.
    fd = os.open(args.output, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    with os.fdopen(fd, 'w') as f:
        if args.evaluate:
            json.dump(result, f, indent=2)
        else:
            yaml.safe_dump(result, f, default_flow_style=False)
    print('Evaluated %d held-out samples.' % len(samples) if args.evaluate else
          'Enrolled %d views; validate held-out target/distractor samples before enabling identity_calibrated.' % len(samples))


def evaluate(config, samples, directory):
    validate(config)
    if not config['perception']['identity']['references'] or not samples:
        raise ValueError('Evaluation requires enrolled references and held-out samples')
    tp = fp = fn = ambiguous = 0
    views = []
    for sample in samples:
        image = cv2.imread(os.path.join(directory,sample['rgb']))
        depth = np.load(os.path.join(directory,sample['depth']),allow_pickle=False)
        if image is None:
            raise ValueError('Missing sample image')
        candidates,unused = detect(image,metric_depth(depth,sample['encoding']),sample['camera'],config['perception'])
        roi = sample['roi']
        hits = []
        if roi is not None:
            x,y,w,h = roi
            hits = [c for c in candidates if x<=c['bbox'][0]+c['bbox'][2]/2.<x+w and
                    y<=c['bbox'][1]+c['bbox'][3]/2.<y+h]
        tp += int(bool(hits))
        fn += int(roi is not None and not hits)
        fp += len(candidates)-int(bool(hits))
        ambiguous += int(len(candidates)>1)
        views.append(dict(label=sample['label'],target_found=bool(hits),candidates=len(candidates)))
    return dict(samples=len(samples),true_positive=tp,false_positive=fp,false_negative=fn,
                ambiguous_frames=ambiguous,precision=tp/float(tp+fp) if tp+fp else None,
                recall=tp/float(tp+fn) if tp+fn else None,views=views,
                note='Candidate-level held-out evidence only; not temporal tracking or a competition score.')


if __name__ == '__main__':
    main()
