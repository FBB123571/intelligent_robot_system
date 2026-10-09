#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Generate UNVALIDATED right-turn and release candidates without overwriting originals.

TonyPi standard bilateral leg channels: 1..5 <-> 9..13, neutral sum 1000.
Arm channels stay fixed during the mirrored carrying turn. Confirm wiring first.
Reversing grab is only a release starting point, not a physically proven release.
"""
import argparse
import sqlite3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'humanoid/scripts'))
from competition import action_frames


def write_new(path, rows):
    # Exclusive creation; never clobber an action edited by a teammate.
    with path.open('xb'):
        pass
    conn = sqlite3.connect(str(path))
    try:
        conn.execute('CREATE TABLE ActionGroup ("Index" INTEGER PRIMARY KEY, Time INT, ' +
                     ', '.join('Servo%d INT' % i for i in range(1, 19)) + ')')
        conn.executemany('INSERT INTO ActionGroup VALUES (' + ','.join('?' for unused in range(20)) + ')', rows)
        conn.commit()
    finally:
        conn.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default=str(ROOT / 'humanoid/ActionGroup/candidates'))
    args = parser.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    turn = []
    for row in action_frames(ROOT / 'humanoid/ActionGroup/grab_turn_left.d6a'):
        new = list(row)
        for a in range(1, 6):
            b = a + 8
            new[a + 1], new[b + 1] = 1000 - row[b + 1], 1000 - row[a + 1]
        turn.append(new)
    release = []
    for index, row in enumerate(reversed(action_frames(ROOT / 'humanoid/ActionGroup/grab.d6a')), 1):
        new = list(row)
        new[0] = index
        new[1] = max(800, row[1])
        release.append(new)
    write_new(out / 'grab_turn_right.d6a', turn)
    write_new(out / 'grab_release.d6a', release)
    print('Generated candidates. Test/edit in the TonyPi action editor before deployment.')


if __name__ == '__main__':
    main()
