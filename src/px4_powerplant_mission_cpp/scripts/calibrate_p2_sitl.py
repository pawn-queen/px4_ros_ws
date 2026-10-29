#!/usr/bin/env python3
"""Run the fixed nonzero vertical-I grid serially, then freeze two stage passes.

Source the normal ROS/P0 environment first. Arguments after '--' are passed to
run_p2_sitl.py; scene, calibration candidate, stage and output are owned here.
All failures and attempts are retained. No flight-time parameter write occurs.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('controller',ROOT/'p2_sitl_controller.py')
controller=importlib.util.module_from_spec(spec);spec.loader.exec_module(controller)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('runner_arguments',nargs=argparse.REMAINDER)
    args=p.parse_args()
    extra=args.runner_arguments[1:] if args.runner_arguments[:1]==['--'] else args.runner_arguments
    owned=('--scenario','--stop-after','--controller-profile','--controller-calibration-candidate',
        '--px4-hover-thrust','--px4-z-velocity-integral','--takeoff-policy','--governor-ablation','--output-dir','--prepare-only','--image-chain-only','--roi-access-test')
    if any(value.split('=')[0] in owned for value in extra):p.error('calibration driver owns scenario, stage, profile, candidate and output flags')
    out=args.output_dir.resolve()
    if out.exists() and any(out.iterdir()):p.error('use a new output directory to preserve previous evidence')
    out.mkdir(parents=True,exist_ok=True)
    journal={'profile':str(args.profile.resolve()),'candidates':list(controller.CANDIDATES),'attempts':[],
        'passed':False,'frozen_profile':None}
    def save():
        (out/'calibration.json').write_text(json.dumps(journal,indent=2,allow_nan=False))
    save()
    for policy in controller.TRANSITION_POLICIES:
        rejected=[]
        for candidate in controller.CANDIDATES:
            pair=[]
            for attempt in (1,2):
                run=out/f'{policy}_I_{candidate:g}_attempt_{attempt}'
                command=[sys.executable,str(ROOT/'run_p2_sitl.py'),'--scenario','powerplant',
                    '--stop-after','scan','--controller-profile',str(args.profile.resolve()),
                    '--takeoff-policy',policy,'--controller-calibration-candidate',str(candidate),
                    '--output-dir',str(run),*extra]
                process=subprocess.run(command)
                path=run/'report.json'
                report=json.loads(path.read_text()) if path.is_file() else {}
                passed=process.returncode==0 and report.get('passed') is True
                journal['attempts'].append({'policy':policy,'candidate':candidate,'attempt':attempt,'command':command,
                    'exit_code':process.returncode,'report':str(path),'passed':passed})
                save()
                if not passed:
                    if path.is_file():rejected.append(path)
                    break
                pair.append(path)
            if len(pair)==2:
                frozen=out/'p2_sitl_controller_frozen.yaml'
                controller.freeze_profile(args.profile,pair,frozen,rejected)
                # Freeze first, then run the explicitly separate algorithm
                # ablation; its outcome is a comparison, never a calibration pass.
                ablation=out/'governor_off_ablation'
                command=[sys.executable,str(ROOT/'run_p2_sitl.py'),'--scenario','powerplant',
                    '--stop-after','scan','--controller-profile',str(frozen),
                    '--governor-ablation','--output-dir',str(ablation),*extra]
                process=subprocess.run(command)
                path=ablation/'report.json'
                journal['ablation']={'command':command,'exit_code':process.returncode,
                    'report':str(path),'report_available':path.is_file()}
                journal.update(passed=True,selected_integral=candidate,selected_transition_policy=policy,
                    frozen_profile=str(frozen))
                save();print(json.dumps(journal,indent=2));return 0
    journal['failure']='all bounded transition policies and nonzero integral candidates failed'
    save();print(json.dumps(journal,indent=2));return 1

if __name__=='__main__':raise SystemExit(main())
