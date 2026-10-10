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
        if not Path(args.csv).is_file():
            parser.error('找不到测量文件：%s。请复制 humanoid/config/measurements.template.csv '
                         '到此路径，再填写实测数据。演示可使用 '
                         'humanoid/config/measurements.demo.csv（非实测）。' % args.csv)
        try:
            with open(args.csv, encoding='utf-8-sig', newline='') as stream:
                reader = csv.DictReader(stream)
                if not {'joint', 'position', 'angle_deg'}.issubset(reader.fieldnames or []):
                    parser.error('CSV 必须包含 joint,position,angle_deg 三列。')
                for line, row in enumerate(reader, 2):
                    joint = (row['joint'] or '').strip()
                    if not joint:
                        parser.error('CSV 第 %d 行缺少 joint。' % line)
                    groups.setdefault(joint, []).append(
                        (float(row['position']), float(row['angle_deg'])))
        except (OSError, UnicodeError, ValueError, TypeError) as exc:
            parser.error('无法读取测量数据：%s。position 和 angle_deg 必须填写数值。' % exc)
        if not groups:
            parser.error('CSV 目前只有表头，没有实测数据。请填写至少两个不同姿态的刻度和角度；'
                         '演示可使用 humanoid/config/measurements.demo.csv（非实测）。')
        try:
            result = {joint: fit(samples, args.degrees_per_unit)
                      for joint, samples in groups.items()}
        except ValueError as exc:
            parser.error('测量数据无法标定：%s' % exc)
    else:
        parser.error('choose actions or fit')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
