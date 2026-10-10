#!/usr/bin/env python3
"""Publish a short operator-confirmed feedback event; never drives hardware.

For manual placement confirmation or debugging only, not an automatic sensor.
"""
import argparse
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'humanoid/scripts'))
from distance_adapter import write_sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', required=True)
    parser.add_argument('--state', required=True, choices=('held', 'not_held', 'released', 'not_released', 'placement_ready', 'not_ready'))
    parser.add_argument('--duration', type=float, default=1)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 30:
        parser.error('--duration must be in (0,30] seconds')
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        write_sample(args.file, {'timestamp': time.time(), 'state': args.state, 'source': 'operator'})
        time.sleep(.05)
    print('Operator event published; samples expire automatically.')


if __name__ == '__main__':
    main()
