"""Calibrated color signature + registered depth. Not a generic TonyPi model."""
import cv2
import numpy as np
from .depth import finite, summarize


def intrinsics(projection, width, height):
    if (len(projection) != 12 or not all(finite(v) for v in projection) or
            projection[0] <= 0 or projection[5] <= 0 or
            not 0 <= projection[2] < width or not 0 <= projection[6] < height or
            abs(projection[3]) > 1e-6 or abs(projection[7]) > 1e-6):
        raise ValueError('invalid_rectified_rgb_calibration')
    return projection[0], projection[5], projection[2], projection[6]


def metric_depth(depth, encoding):
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError('unsupported_depth_encoding')
    return depth.astype(np.float32) * (0.001 if encoding == '16UC1' else 1.0)


def cloud_points(depth_m, camera, stride=4, near=0.4, far=4.0):
    fx, fy, cx, cy = camera
    rows, cols = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    z = depth_m[::stride, ::stride]
    good = np.isfinite(z) & (z >= near) & (z <= far)
    z, u, v = z[good], cols[good], rows[good]
    return np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z)).astype(np.float32)


def transform_points(points, translation, quaternion):
    t, q = np.asarray(translation, dtype=float), np.asarray(quaternion, dtype=float)
    if not np.all(np.isfinite(t)) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q) - 1.) > .05:
        raise ValueError('invalid_transform')
    q /= np.linalg.norm(q)
    x, y, z, w = q
    rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return np.asarray(points).reshape((-1, 3)).dot(rotation.T) + t


def detect(rgb, depth_m, camera, config):
    """Return independent candidates; tracker handles association and ambiguity."""
    if rgb.shape[:2] != depth_m.shape:
        raise ValueError('registered_size_mismatch')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
    mask = np.zeros(depth_m.shape, dtype=np.uint8)
    for band in config['hsv_bands']:
        mask |= cv2.inRange(hsv, np.array(band['lower'], dtype=np.uint8), np.array(band['upper'], dtype=np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    fx, fy, cx, cy = camera
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not config['min_area_ratio'] <= area / mask.size <= config['max_area_ratio']:
            continue
        component = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(component, [contour], -1, 255, -1)
        # Shrink edges to avoid depth registration holes and mixed background.
        component = cv2.erode(component & mask, kernel, iterations=1)
        values = depth_m[component > 0]
        summary = summarize(values, '32FC1', min_depth=config['min_depth'],
                            max_depth=config['max_depth'], min_samples=config['min_depth_samples'],
                            min_ratio=config['min_valid_ratio'], max_spread=config['max_depth_spread'])
        if not summary['usable']:
            continue
        z = summary['median_m']
        good = ((component > 0) & np.isfinite(depth_m) &
                (np.abs(depth_m - z) <= config['depth_cluster_band']))
        vs, us = np.nonzero(good)
        if len(us) < config['min_depth_samples']:
            continue
        x0, y0, width, height = cv2.boundingRect(contour)
        physical_width = width * z / fx
        if not config['min_width_m'] <= physical_width <= config['max_width_m']:
            continue
        u, v = float(np.median(us)), float(np.median(vs))
        point = ((u - cx) * z / fx, (v - cy) * z / fy, z)
        candidates.append(dict(point=point, bbox=(x0, y0, width, height), depth=z,
                               valid_ratio=summary['valid_ratio']))
    return candidates, mask
