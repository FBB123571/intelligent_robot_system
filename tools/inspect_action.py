#!/usr/bin/env python3
"""Read d6a action frames, print JSON or export CSV without opening hardware."""
import argparse
import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'humanoid/scripts'))
from competition import action_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action_file')
    parser.add_argument('--csv', help='new CSV file; refuses to overwrite')
    args = parser.parse_args()
    rows = action_frames(args.action_file)
    header = ['Index', 'Time'] + ['Servo%d' % i for i in range(1, len(rows[0]) - 1)]
    if args.csv:
        with open(args.csv, 'x', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(rows)
    print(json.dumps({'file': str(args.action_file), 'frames': len(rows),
                      'total_duration_ms': sum(row[1] for row in rows),
                      'channel_count': len(rows[0]) - 2,
                      'data': [dict(zip(header, row)) for row in rows]}, indent=2))


if __name__ == '__main__':
    main()
