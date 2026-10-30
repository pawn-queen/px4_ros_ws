#!/usr/bin/env python3
"""Compare three paired original-load real-world trials without hiding failures."""
import argparse,json,math,statistics
from pathlib import Path


def compare(root,regression=.05):
    reports={v:[] for v in ('complete','simplified')};missing=[]
    for variant in reports:
        for repeat in (1,2,3):
            path=root/f'bench_native_{repeat}_{variant}'/'report.json'
            if not path.exists():missing.append(str(path));continue
            value=json.loads(path.read_text());value['report_file']=str(path);reports[variant].append(value)
    if missing:return {'passed':False,'status':'UNCONFIRMED','missing_reports':missing}
    checks={'all_three_repeats':all(len(v)==3 for v in reports.values()),
        'original_sensor_samples_every_trial':all(r['native_sensor_samples_present'] and r['profile']=='native' for rows in reports.values() for r in rows),
        'required_warmup_and_duration':all(r['warmup_wall_s']>=30 and r['sample_wall_s']>=59.9 for rows in reports.values() for r in rows)}
    all_rows=sum(reports.values(),[])
    checks['resolved_collision_resource_every_trial']=all(
        r.get('resolved_collision_resource',{}).get('verified')
        and r['resolved_collision_resource'].get('sha256')==r['collision_resource_sha256']
        for r in all_rows)
    checks['different_actual_comparison_resources']=len({
        r.get('resolved_collision_resource',{}).get('sha256') for r in all_rows})==2
    renderers={next((line.split('GL_RENDERER =',1)[1].strip() for line in r.get('renderer_identification',[])
                    if 'GL_RENDERER =' in line),None) for r in all_rows}
    checks['same_identified_renderer']=len(renderers)==1 and None not in renderers
    for key in ('world_sha256','visual_mesh_sha256','fixtures_sha256','render_context'):
        checks['same_'+key]=len({r.get(key) for r in all_rows})==1
    for variant,rows in reports.items():checks['stable_'+variant+'_mesh']=len({r['collision_resource_sha256'] for r in rows})==1
    metrics={};policy={'cpu_core_percent':'lower','rss_median_bytes':'lower','rss_peak_bytes':'lower','loading_wall_s':'lower','measured_real_time_factor':'higher','measured_iteration_rate_hz':'higher'}
    for name,direction in policy.items():
        values={v:[r.get(name) for r in reports[v]] for v in reports}
        valid=all(isinstance(x,(float,int)) and math.isfinite(x) and x>0 for a in values.values() for x in a)
        if not valid:checks[name]=False;metrics[name]={'valid':False,'values':values};continue
        before,after=[statistics.median(values[v]) for v in ('complete','simplified')]
        worsening=(after/before-1) if direction=='lower' else (before/after-1)
        metrics[name]={'direction':direction,'complete':values['complete'],'simplified':values['simplified'],'complete_median':before,'simplified_median':after,'worsening_fraction':worsening,'within_limit':worsening<=regression}
        checks[name]=worsening<=regression
    for sensor in ('rgb','depth','lidar'):
        name=sensor+'_p95_wall_interval_s';values={v:[r['sensors'][sensor]['wall_interval_p50_p95_p99_s'][1] for r in reports[v]] for v in reports}
        before,after=[statistics.median(values[v]) for v in ('complete','simplified')];worsening=after/before-1
        metrics[name]={'complete':values['complete'],'simplified':values['simplified'],'complete_median':before,'simplified_median':after,'worsening_fraction':worsening,'within_limit':worsening<=regression};checks[name]=worsening<=regression
    passed=all(checks.values())
    return {'passed':passed,'status':'PASS' if passed else 'FAIL','allowed_regression_fraction':regression,'checks':checks,'metrics':metrics,'renderer_identification':[r.get('renderer_identification',[]) for r in all_rows],'reports':[r['report_file'] for r in all_rows],
        'scope':'Same actual powerplant world, original RGB/depth/lidar load, fixed sensor rig, and a slow dynamic probe against the perimeter wall. Measures whole-scene load/memory; does not isolate plant-mesh collision-query throughput or certify realtime flight.',
        'limitations':['Three trials use point medians, not a statistical guarantee.','Software rendering, when identified, remains software rendering; GPU visibility alone is not acceleration evidence.','No mission/YOLO workers run in this dedicated asset-cost comparison.']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_dir',type=Path);p.add_argument('--max-regression',type=float,default=.05);args=p.parse_args()
    if not math.isfinite(args.max_regression) or args.max_regression<0:p.error('regression must be finite and nonnegative')
    result=compare(args.run_dir,args.max_regression);target=args.run_dir/'performance_comparison.json';target.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'status':result['status'],'report':str(target),'failed_checks':[k for k,v in result.get('checks',{}).items() if not v]},indent=2));return 0 if result['passed'] else (2 if result['status']=='UNCONFIRMED' else 1)
if __name__=='__main__':raise SystemExit(main())
