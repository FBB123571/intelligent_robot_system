#!/usr/bin/env python
"""Summarize passive trial evidence plus explicit human verdicts; never award points."""
from __future__ import division, print_function
import argparse
import json
import math
import os

SCENARIOS = ('left_right_turns', 'occlusion', 'same_color', 'crossing', 'obstacles', 'team_finish')
VERDICTS = ('scenario_exercised', 'collision_free', 'correct_identity', 'no_intervention', 'robot_stopped_in_finish')


def percentile(values, q):
    values = sorted(values)
    return values[int(math.ceil((len(values)-1)*q))] if values else None


def summarize(records, verdict=None):
    meta = [r for r in records if r.get('kind') == 'metadata']
    rows = [r for r in records if r.get('kind') == 'sample' and r.get('mission') in ('carrying','releasing','done')]
    result = dict(samples=len(rows), accepted=False, issues=[])
    if len(meta) != 1 or len(rows) < 2:
        result['issues'].append('missing_metadata_or_active_samples')
        return result
    c = meta[0]['settings']
    if any(r.get('kind') == 'invalid_sample' for r in records):
        result['issues'].append('invalid_telemetry')
    loss_start, recoveries, longest, visible_time, elapsed = None, [], 0., 0., 0.
    for i, row in enumerate(rows):
        if i:
            dt = row['t']-rows[i-1]['t']
            if dt <= 0 or dt > .5:
                result['issues'].append('sampling_gap_or_clock_reset')
            if dt > 0:
                elapsed += dt
                visible_time += dt if rows[i-1]['target_valid'] else 0.
        if not row['target_valid'] and loss_start is None:
            loss_start = row['t']
        if row['target_valid'] and loss_start is not None:
            recoveries.append(row['t']-loss_start)
            longest = max(longest, row['t']-loss_start)
            loss_start = None
    if loss_start is not None:
        longest = max(longest, rows[-1]['t']-loss_start)
    errors = [r['euclidean_gap_error_m'] for r in rows if r.get('euclidean_gap_error_m') is not None]
    clearances = [r['costmap_clearance_m'] for r in rows if r.get('costmap_clearance_m') is not None]
    result.update(duration_s=elapsed, visible_fraction=visible_time/elapsed if elapsed else 0.,
                  longest_loss_s=longest, reacquisition_seconds=recoveries,
                  euclidean_gap_error_p95_m=percentile(errors, .95),
                  min_costmap_clearance_m=min(clearances) if clearances else None,
                  odom_coverage=sum(bool(r.get('odom_fresh')) for r in rows)/len(rows),
                  grid_coverage=sum(bool(r.get('grid_fresh')) for r in rows)/len(rows))
    if longest >= c['lost_failure_seconds']:
        result['issues'].append('loss_over_limit')
    if result['odom_coverage'] < .95 or result['grid_coverage'] < .95:
        result['issues'].append('insufficient_sensor_evidence')
    finished = [r for r in rows if r.get('follow_status') == 'finished']
    if not finished or not any(r.get('odom_fresh') and r.get('speed') is not None and
                              r['speed'] <= c['stopped_linear'] and r.get('angular_speed') is not None and
                              r['angular_speed'] <= c['stopped_angular'] for r in finished):
        result['issues'].append('no_measured_stopped_finish')
    verdict = verdict or {}
    required = VERDICTS + (('whole_team_finished',) if meta[0]['scenario'] == 'team_finish' else ())
    for key in required:
        if verdict.get(key) is not True:
            result['issues'].append('human_verdict_missing_or_failed:'+key)
    result['issues'] = sorted(set(result['issues']))
    result['accepted'] = not result['issues']
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('manifest', help='JSON {trials:[{scenario, log, verdict:{...}}]}')
    p.add_argument('--repeats', type=int, default=3)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    if args.repeats < 1:
        p.error('repeats must be positive')
    with open(args.manifest) as f:
        trials = json.load(f)['trials']
    results, counts, seen = [], dict((s,0) for s in SCENARIOS), set()
    streaks, attempts = dict((s,0) for s in SCENARIOS), dict((s,0) for s in SCENARIOS)
    for trial in trials:
        path = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(args.manifest)), trial['log']))
        if path in seen:
            raise ValueError('A log cannot count as multiple trials')
        seen.add(path)
        with open(path) as f:
            records = [json.loads(line) for line in f if line.strip()]
        metadata = [r for r in records if r.get('kind') == 'metadata']
        if trial['scenario'] not in counts or len(metadata) != 1 or metadata[0]['scenario'] != trial['scenario']:
            raise ValueError('Scenario mismatch')
        result = summarize(records, trial.get('verdict'))
        result.update(log=trial['log'], scenario=trial['scenario'])
        results.append(result)
        counts[trial['scenario']] += int(result['accepted'])
        attempts[trial['scenario']] += 1
        streaks[trial['scenario']] = streaks[trial['scenario']]+1 if result['accepted'] else 0
    report = dict(trials=results, accepted_counts=counts, repeats_required=args.repeats,
                  ending_success_streaks=streaks,
                  observed_success_rates=dict((s,counts[s]/attempts[s] if attempts[s] else None) for s in SCENARIOS),
                  suite_accepted=all(v >= args.repeats for v in streaks.values()),
                  note='Engineering acceptance only. Physical collision/identity/team outcomes require human evidence; this is not a competition score.')
    with open(args.output, 'w') as f:
        json.dump(report, f, indent=2, allow_nan=False)
    print(json.dumps(dict(suite_accepted=report['suite_accepted'], accepted_counts=counts)))
    return 0 if report['suite_accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
