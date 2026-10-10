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


def mirror_frames(rows, scope='legs', neutral=500):
    """Swap paired channels and negate their displacement from neutral.

    This is an assumed pulse-space symmetry, not geometric calibration.
    Unselected arms and extra channels are preserved exactly.
    """
    if scope not in ('legs', 'arms', 'body'):
        raise ValueError('scope must be legs, arms or body')
    if neutral != 500:
        raise ValueError('This TonyPi mirror prior uses neutral=500')
    left_ids = {'legs': range(1, 6), 'arms': range(6, 9), 'body': range(1, 9)}[scope]
    mirrored = []
    for row in rows:
        new = list(row)
        for a in left_ids:
            b = a + 8
            new[a + 1], new[b + 1] = 2 * neutral - row[b + 1], 2 * neutral - row[a + 1]
        mirrored.append(new)
    return mirrored


def write_new(path, rows):
    if not rows or len(rows[0]) not in (18, 20) or any(len(r) != len(rows[0]) for r in rows):
        raise ValueError('Consistent 16/18-channel frames required')
    width = len(rows[0])
    # Exclusive creation; never clobber an action edited by a teammate.
    with path.open('xb'):
        pass
    conn = sqlite3.connect(str(path))
    try:
        conn.execute('CREATE TABLE ActionGroup ("Index" INTEGER PRIMARY KEY, Time INT, ' +
                     ', '.join('Servo%d INT' % i for i in range(1, width - 1)) + ')')
        conn.executemany('INSERT INTO ActionGroup VALUES (' + ','.join('?' for unused in range(width)) + ')', rows)
        conn.commit()
    finally:
        conn.close()


def compose_carry_turn(rows, grab_rows, entry_ms=600):
    """Keep original leg trajectory/timing; hold arms at grab's final pose.

    Extra unverified channels are stored at grab's final values, never executed.
    An entry frame blends from the current pose to the turn's initial leg pose.
    This remains an unvalidated loaded gait, not a guarantee of balance/yaw.
    """
    if not rows or not grab_rows or any(len(r) != len(grab_rows[-1]) for r in rows):
        raise ValueError('Turn and grab must have matching channel counts')
    if isinstance(entry_ms, bool) or not isinstance(entry_ms, int) or not 20 <= entry_ms <= 30000:
        raise ValueError('entry_ms must be 20..30000')
    last = grab_rows[-1]
    held = [6, 7, 8, 14, 15, 16] + list(range(17, len(last) - 1))
    output = []
    for row in rows:
        new = list(row)
        for servo in held:
            new[servo + 1] = last[servo + 1]
        output.append(new)
    entry = list(output[0])
    entry[1] = entry_ms
    output.insert(0, entry)
    for index, row in enumerate(output, 1):
        row[0] = index
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default=str(ROOT / 'humanoid/ActionGroup/candidates'))
    parser.add_argument('--source', help='mirror this existing d6a instead of generating default candidates')
    parser.add_argument('--carry-pose', help='grab.d6a: compose source legs with its final arm pose instead of mirroring')
    parser.add_argument('--entry-ms', type=int, default=600, help='composed turn entry transition duration')
    parser.add_argument('--output', help='new mirrored d6a path; never overwrites')
    parser.add_argument('--scope', choices=('legs', 'arms', 'body'), default='legs',
                        help='paired channels to mirror; default keeps carrying arms unchanged')
    args = parser.parse_args()
    if bool(args.source) != bool(args.output):
        parser.error('--source and --output must be supplied together')
    if args.carry_pose and not args.source:
        parser.error('--carry-pose requires --source and --output')
    if args.source:
        rows = (compose_carry_turn(action_frames(args.source), action_frames(args.carry_pose), args.entry_ms)
                if args.carry_pose else mirror_frames(action_frames(args.source), args.scope))
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        try:
            write_new(output, rows)
        except FileExistsError:
            parser.error('Output already exists; choose a new path: ' + str(output))
        print('Generated UNVALIDATED %s candidate:' % ('composed carry turn' if args.carry_pose else 'mirror'), output)
        return
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    turn = mirror_frames(action_frames(ROOT / 'humanoid/ActionGroup/grab_turn_left.d6a'))
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
