#!/usr/bin/env python3
"""Create named action copies with per-frame hip offsets, never overwrite."""
import argparse
import hashlib
import json
from pathlib import Path

from generate_action_candidates import action_frames, write_new


def offset_frames(rows, offsets):
    if not rows or len(rows[0]) not in (18,20) or any(len(r)!=len(rows[0]) for r in rows):
        raise ValueError('Consistent 16/18-channel body frames required')
    if any(isinstance(i,bool) or not isinstance(i,int) or not 1<=i<=16 or
           isinstance(d,bool) or not isinstance(d,int) for i,d in offsets.items()):
        raise ValueError('Offsets require body IDs 1..16 and integer pulse deltas')
    result=[]
    for row in rows:
        new=list(row)
        for servo,delta in offsets.items():
            new[servo+1]+=delta
            if not 0<=new[servo+1]<=1000:
                raise ValueError('Frame %s servo %d offset exceeds 0..1000; no clipping' % (row[0],servo))
        result.append(new)
    return result


def generate(sources, output_dir, offsets):
    output_dir=Path(output_dir)
    manifest_path=output_dir/'offset-manifest.json'
    pending=[]; names=set()
    for source in sources:
        source=Path(source)
        if source.stem.endswith('_offset'):
            raise ValueError('Refusing to apply a second offset to '+str(source))
        dest=output_dir/(source.stem+'_offset.d6a')
        if dest.name in names or dest.exists():
            raise FileExistsError('Output name collision: '+str(dest))
        names.add(dest.name)
        pending.append((source,dest,offset_frames(action_frames(source),offsets)))
    if not pending: raise ValueError('No source action files')
    if manifest_path.exists(): raise FileExistsError(str(manifest_path))
    # Preflight every action before creating any output.
    output_dir.mkdir(parents=True,exist_ok=True)
    report={'offsets':{str(k):v for k,v in offsets.items()},'validated_on_hardware':False,'files':[]}
    for source,dest,rows in pending:
        write_new(dest,rows)
        report['files'].append({'source':source.as_posix(),'output':dest.name,'frames':len(rows),
          'duration_ms':sum(r[1] for r in rows),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
          'output_sha256':hashlib.sha256(dest.read_bytes()).hexdigest()})
    with manifest_path.open('x',encoding='utf-8') as stream: json.dump(report,stream,indent=2)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--source',nargs='+',help='one or more original d6a files')
    group.add_argument('--source-dir',help='all original d6a files in this directory, excluding *_offset')
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--left-delta',type=int,required=True,help='ID4 pulse change; ID12 receives its opposite')
    args=parser.parse_args()
    if not 1<=abs(args.left_delta)<=100: parser.error('Candidate left-delta magnitude must be 1..100 pulse units')
    sources=args.source or sorted(p for p in Path(args.source_dir).glob('*.d6a') if not p.stem.endswith('_offset'))
    try: report=generate(sources,args.output_dir,{4:args.left_delta,12:-args.left_delta})
    except (ValueError,FileExistsError,RuntimeError) as exc: parser.error(str(exc))
    print('Generated %d UNVALIDATED copies: ID4 %+d, ID12 %+d' % (len(report['files']),args.left_delta,-args.left_delta))


if __name__=='__main__': main()
