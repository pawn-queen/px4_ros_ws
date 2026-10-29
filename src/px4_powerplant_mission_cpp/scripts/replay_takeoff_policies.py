#!/usr/bin/env python3
"""Replay already aligned historical tracking samples through the native core.

This compares risk/threshold decisions only. It is not a counterfactual flight,
and cannot prove that changing a threshold would change the plant response.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def replay(probe,root):
    records=[]
    for path in sorted(Path(root).glob('*/tracking.jsonl')):
        samples=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        payload={'op':'tracking_policy_replay','samples':samples}
        result=subprocess.run([str(Path(probe).resolve())],input=json.dumps(payload)+'\n',
            text=True,capture_output=True,check=True)
        values=json.loads(result.stdout)
        if not isinstance(values,list) or len(values)!=len(samples):
            raise RuntimeError('native core did not replay every historical sample')
        mismatch=sum(bool(s['accepted'])!=r['legacy']['hard_accepted'] for s,r in zip(samples,values))
        policies={name:{'soft_trigger_samples':sum(r[name]['soft_triggered'] for r in values),
            'hard_rejected_samples':sum(not r[name]['hard_accepted'] for r in values),
            'maximum_risk':max((r[name]['risk'] for r in values),default=None)}
            for name in ('legacy','A','B','C','D')}
        records.append({'run':path.parent.name,'source':str(path.resolve()),
            'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'samples':len(samples),
            'original_hard_decision_mismatches':mismatch,'policies':policies})
    return {'passed':bool(records) and all(r['samples'] and not r['original_hard_decision_mismatches'] for r in records),
        'scope':'native scalar/axis tracking and predictive risk replay only; not geometry or flight acceptance',
        'runs':records}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--logs',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=replay(a.probe,a.logs)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'passed':result['passed'],'runs':len(result['runs']),'output':str(a.output)}))
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
