#!/usr/bin/env python3
"""Estimate a COLOR signature from a tightly cropped target image; no hardware.

This produces an unvalidated config, not a trained TonyPi identity model.
Inspect several views and distractors before enabling identity_calibrated.
"""
import argparse
from pathlib import Path
import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def bands_from_crop(crop):
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    samples = hsv[(hsv[:, :, 1] >= 50) & (hsv[:, :, 2] >= 40)]
    if len(samples) < 30 or len(samples) / float(hsv.shape[0] * hsv.shape[1]) < .4:
        raise ValueError('Crop lacks a distinctive saturated color; choose another feature/detector')
    # Circular hue center handles red near both 0 and 179.
    angles = samples[:, 0].astype(float) * np.pi / 90
    center = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles))) * 90 / np.pi % 180
    offsets = (samples[:, 0].astype(float) - center + 90) % 180 - 90
    low, high = np.percentile(offsets, [5, 95])
    if high - low > 35:
        raise ValueError('Crop contains too many hues; select a single stable target color')
    lo, hi = int(np.floor(center + low - 5)), int(np.ceil(center + high + 5))
    s = max(40, int(np.percentile(samples[:, 1], 5)) - 35)
    v = max(30, int(np.percentile(samples[:, 2], 5)) - 35)
    if lo < 0:
        ranges = [(0, hi), (180 + lo, 179)]
    elif hi > 179:
        ranges = [(lo, 179), (0, hi - 180)]
    else:
        ranges = [(lo, hi)]
    return [dict(lower=[a, s, v], upper=[b, 255, 255]) for a, b in ranges]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image')
    parser.add_argument('--roi', required=True, type=int, nargs=4, metavar=('LEFT', 'TOP', 'RIGHT', 'BOTTOM'))
    parser.add_argument('--config', default=str(ROOT / 'turtlebot/follow3/rgbd_following/config/follow.yaml'))
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if Path(args.output).exists():
        parser.error('Output exists; choose a new filename')
    # imdecode supports Unicode file paths on Windows.
    image = cv2.imdecode(np.fromfile(args.image, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        parser.error('Unable to load image')
    x0, y0, x1, y1 = args.roi
    if not 0 <= x0 < x1 <= image.shape[1] or not 0 <= y0 < y1 <= image.shape[0]:
        parser.error('ROI is outside image')
    with open(args.config, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    config['perception']['hsv_bands'] = bands_from_crop(image[y0:y1, x0:x1])
    for flag in ('hardware_calibrated', 'identity_calibrated', 'depth_coverage_calibrated'):
        config[flag] = False
    with open(args.output, 'x', encoding='utf-8') as stream:
        yaml.safe_dump(config, stream, allow_unicode=True)
    print('Saved unvalidated color candidate:', args.output)
    print('Test front/back/side views and distractors before enabling hardware.')


if __name__ == '__main__':
    main()
