#!/usr/bin/env python3
"""实际电厂中串行执行3次位置模式起飞与2次独立起飞保护注入。"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('runner_arguments',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    extra=args.runner_arguments[1:] if args.runner_arguments[:1]==['--'] else args.runner_arguments
    owned=('--scenario','--stop-after','--px4-position-takeoff','--takeoff-fault','--controller-profile',
        '--controller-calibration-candidate','--px4-hover-thrust','--px4-z-velocity-integral','--takeoff-policy',
        '--governor-ablation','--output-dir','--prepare-only','--image-chain-only','--roi-access-test')
    if any(x.split('=')[0] in owned for x in extra):parser.error('takeoff driver owns flight mode, stage, faults and output; controller overrides forbidden')
    output=args.output_dir.resolve()
    if output.exists() and any(output.iterdir()):parser.error('a new output directory is required')
    output.mkdir(parents=True,exist_ok=True)
    journal={'mode':'px4_position_only_stock_control','runs':[],'passed':False,
        'scope':'three actual 5m takeoffs to strict hover handoff, then drift and timeout AUTO_LAND guards; not full mission'}
    def save():(output/'takeoff_acceptance.json').write_text(json.dumps(journal,indent=2,allow_nan=False))
    save()
    for name,fault in [('hover_1',None),('hover_2',None),('hover_3',None),('guard_drift','drift'),('guard_timeout','timeout')]:
        run=output/name
        command=[sys.executable,str(Path(__file__).with_name('run_acceptance.py')),
            '--scenario','powerplant','--stop-after','hover','--px4-position-takeoff','--output-dir',str(run),*extra]
        if fault:command+=['--takeoff-fault',fault]
        result=subprocess.run(command)
        path=run/'report.json'
        report=json.loads(path.read_text()) if path.is_file() else {}
        journal['runs'].append({'name':name,'fault':fault,'command':command,'exit_code':result.returncode,
            'report':str(path),'passed':result.returncode==0 and report.get('passed') is True,
            'failed_checks':[k for k,v in report.get('checks',{}).items() if v is not True]})
        save()
    journal['three_consecutive_takeoffs_passed']=all(r['passed'] for r in journal['runs'][:3])
    journal['both_guard_injections_passed']=all(r['passed'] for r in journal['runs'][3:])
    journal['passed']=journal['three_consecutive_takeoffs_passed'] and journal['both_guard_injections_passed']
    save();print(json.dumps(journal,indent=2));return 0 if journal['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
