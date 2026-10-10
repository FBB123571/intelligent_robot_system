#!/usr/bin/env python3
"""Offline action priors and pulse/known-angle calibration; never sends commands."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'humanoid/scripts'))
from competition import action_frames


def analyze(path):
    rows = action_frames(str(path))
    pairs = []
    for left in range(1, 9):
        right = left + 8
        sums = [r[left + 1] + r[right + 1] for r in rows]
        pairs.append({'ids': [left, right], 'sum_min': min(sums),
                      'sum_max': max(sums),
                      'complement_rmse_units': round(math.sqrt(
                          sum((v - 1000) ** 2 for v in sums) / len(sums)), 3)})
    return {'file': str(path), 'frames': len(rows),
            'duration_ms': sum(r[1] for r in rows),
            'first_frame_positions': list(rows[0][2:18]),
            'observed_ranges_not_limits': {
                str(i): [min(r[i + 1] for r in rows), max(r[i + 1] for r in rows)]
                for i in range(1, 17)},
            'pairing_is_diagram_prior_not_inferred': pairs}


def fit(samples, degrees_per_unit=0.24):
    """Fit q = direction * scale * (p - zero), with nominal scale fixed."""
    if not math.isfinite(degrees_per_unit) or degrees_per_unit <= 0:
        raise ValueError('degrees_per_unit must be finite and positive')
    if len(samples) < 2:
        raise ValueError('at least two known-angle measurements required')
    if any(not math.isfinite(p) or not math.isfinite(q) or not 0 <= p <= 1000
           for p, q in samples):
        raise ValueError('finite angles and pulses in 0..1000 required')
    if max(q for p, q in samples) - min(q for p, q in samples) < 1e-6:
        raise ValueError('different known angles required to determine direction')
    if max(p for p, q in samples) - min(p for p, q in samples) < 1e-6:
        raise ValueError('different pulses required to determine direction')
    candidates = []
    for direction in (1, -1):
        zero = sum(p - direction * q / degrees_per_unit for p, q in samples) / len(samples)
        residuals = [direction * degrees_per_unit * (p - zero) - q for p, q in samples]
        candidates.append({'zero': zero, 'direction': direction,
                           'rmse_deg': math.sqrt(sum(r*r for r in residuals) / len(samples)),
                           'max_error_deg': max(abs(r) for r in residuals)})
    candidates.sort(key=lambda c: c['rmse_deg'])
    best = dict(candidates[0])
    best.update({'degrees_per_unit': degrees_per_unit, 'sample_count': len(samples),
                 'alternative_direction_rmse_deg': candidates[1]['rmse_deg'],
                 'hardware_verified': False,
                 'note': 'Candidate only: inspect residuals, axis convention and hold-out poses; no limits inferred.'})
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode')
    actions = sub.add_parser('actions')
    actions.add_argument('paths', nargs='+')
    measurements = sub.add_parser('fit')
    measurements.add_argument('csv', help='UTF-8 CSV columns: joint,position,angle_deg')
    measurements.add_argument('--degrees-per-unit', type=float, default=.24)
    args = parser.parse_args()
    if args.mode == 'actions':
        result = [analyze(p) for p in args.paths]
    elif args.mode == 'fit':
        groups = {}
        with open(args.csv, encoding='utf-8-sig', newline='') as stream:
            for row in csv.DictReader(stream):
                groups.setdefault(row['joint'], []).append(
                    (float(row['position']), float(row['angle_deg'])))
        if not groups:
            parser.error('CSV contains no measurements')
        result = {joint: fit(samples, args.degrees_per_unit)
                  for joint, samples in groups.items()}
    else:
        parser.error('choose actions or fit')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
