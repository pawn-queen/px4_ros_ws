#!/usr/bin/env python3
"""Isolated SITL controller derivation, readback and calibration evidence.

No MAVLink writes are made here. The runner applies only the declared two
calibration values through the environment of its fresh, prearm PX4 rootfs.
"""
from __future__ import annotations
import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

CANDIDATES = (2.0, 1.0, .5, .25, .1)
TRANSITION_POLICIES = {
    'A': {'position_tolerance_m':.10,'body_radius_m':.50},
    'B': {'position_tolerance_m':.12,'body_radius_m':.50},
    'C': {'position_tolerance_m':.15,'body_radius_m':.50},
    'D': {'position_tolerance_m':.15,'body_radius_m':.48},
}
ESC_PARAMETERS = tuple(f'SIM_GZ_EC_{kind}{i}' for i in range(1,5) for kind in ('MIN','MAX'))
CONTROL_PARAMETERS = ('COM_SPOOLUP_TIME','MPC_TKO_RAMP_T','MPC_THR_HOVER',
    'MPC_Z_VEL_I_ACC','THR_MDL_FAC','MPC_THR_MAX') + ESC_PARAMETERS


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_profile(path, candidate=None):
    import yaml
    path=Path(path).resolve()
    value=yaml.safe_load(path.read_text())
    if not isinstance(value,dict) or value.get('schema_version')!=1:
        raise ValueError('unsupported SITL controller profile')
    if tuple(value.get('integral_candidates',()))!=CANDIDATES:
        raise ValueError('formal integral grid must be [2, 1, .5, .25, .1]')
    if value.get('transition_policy_order',list(TRANSITION_POLICIES))!=list(TRANSITION_POLICIES):
        raise ValueError('transition policy order must be A, B, C, D')
    if candidate is not None:
        if candidate not in CANDIDATES:
            raise ValueError('integral candidate outside the formal nonzero grid')
        mode='calibration_candidate'
        selected=float(candidate)
    else:
        selected=value.get('selected_integral')
        if selected not in CANDIDATES or value.get('status')!='frozen':
            raise ValueError('formal flight requires a frozen controller profile; use an explicit calibration candidate first')
        mode='frozen'
        validate_freeze(value)
    expected=value.get('expected_readback',{})
    expected_keys=set(CONTROL_PARAMETERS)-{'MPC_THR_HOVER','MPC_Z_VEL_I_ACC'}
    if set(expected)!=expected_keys or any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in expected.values()):
        raise ValueError('complete finite spool/ramp/ESC/thrust mapping readback is required')
    if expected['COM_SPOOLUP_TIME']!=1 or expected['MPC_TKO_RAMP_T']!=3:
        raise ValueError('spool/ramp are fixed at original 1/3 seconds')
    if expected['THR_MDL_FAC']!=0 or expected['MPC_THR_MAX']!=1:
        raise ValueError('this derivation requires the audited linear ESC mapping and full thrust setting')
    value={**value,'path':str(path),'profile_sha256':digest(path),'mode':mode,'selected_integral':float(selected)}
    value['profile_id']=f"{value.get('id','p2-sitl')}:{digest(path)[:12]}:I={selected:g}:{mode}"
    return value


def derive_model(models, world, expected):
    """Expand actual nested SDF and solve equal-speed static thrust balance.

    Absolute error here is floating point calculation error, not a calibrated
    flight model. Actual flight and readback remain separate acceptance gates.
    """
    models,world=Path(models).resolve(),Path(world).resolve()
    env=os.environ.copy()
    for key in ('SDF_PATH','GZ_SIM_RESOURCE_PATH'):
        env[key]=str(models)+os.pathsep+env.get(key,'')
    expanded=subprocess.run(['gz','sdf','-p',str(models/'x500_depth/model.sdf')],
        env=env,capture_output=True,text=True,timeout=30,check=True).stdout
    root=ET.fromstring(expanded)
    model=root.find('model')
    if model is None or model.findall('.//include') or model.findall('model'):
        raise ValueError('expected fully merged x500_depth model')
    links=model.findall('link')
    masses=[float(link.findtext('inertial/mass','nan')) for link in links]
    if not masses or any(not math.isfinite(v) or v<=0 for v in masses):
        raise ValueError('every expanded body link must have a positive inertial mass')
    gravity=[float(x) for x in ET.parse(world).getroot().findtext('world/gravity','0 0 -9.8').split()]
    if len(gravity)!=3 or abs(gravity[0])+abs(gravity[1])>1e-12 or gravity[2]>=0:
        raise ValueError('hover derivation requires vertical ENU gravity')
    motors=[p for p in model.findall('plugin') if p.find('motorConstant') is not None]
    if len(motors)!=4 or sorted(int(p.findtext('motorNumber')) for p in motors)!=list(range(4)):
        raise ValueError('exactly four independently identified lift motors required')
    constants=[float(p.findtext('motorConstant')) for p in motors]
    maxima=[float(p.findtext('maxRotVelocity')) for p in motors]
    if any(v<=0 or not math.isfinite(v) for v in constants+maxima) or max(constants)-min(constants)>1e-15:
        raise ValueError('equal positive motor constants required by this balance model')
    mass=sum(masses); weight=mass*-gravity[2]
    speed=math.sqrt(weight/sum(constants))
    normalized=[]
    for i,max_speed in enumerate(maxima,1):
        low,high=expected[f'SIM_GZ_EC_MIN{i}'],expected[f'SIM_GZ_EC_MAX{i}']
        if not 0<=low<high<=max_speed or not low<speed<high:
            raise ValueError('hover speed lies outside audited ESC/motor range')
        normalized.append((speed-low)/(high-low))
    if max(normalized)-min(normalized)>1e-9:
        raise ValueError('unequal ESC mapping is unsupported by equal-thrust hover derivation')
    sources={str(models/name/'model.sdf'):digest(models/name/'model.sdf')
        for name in ('x500_depth','x500','x500_base','OakD-Lite')}
    sources[str(world)]=digest(world)
    return {'total_mass_kg':mass,'gravity_mps2':-gravity[2],'weight_n':weight,
        'link_masses_kg':dict(zip((p.get('name') for p in links),masses)),
        'motor_constants':constants,'motor_max_speed_radps':maxima,
        'maximum_ideal_lift_n':sum(k*w*w for k,w in zip(constants,maxima)),
        'hover_motor_speed_radps':speed,'derived_hover_thrust':normalized[0],
        'expanded_sdf_sha256':hashlib.sha256(expanded.encode()).hexdigest(),
        'source_file_sha256':sources,
        'assumptions':'static equal-speed thrust; linear ESC mapping, THR_MDL_FAC=0; not a measured controller response'}


def resolve_profile(profile, model):
    value={**profile,'model_derivation':model}
    value['requested_parameters']={'MPC_THR_HOVER':model['derived_hover_thrust'],
        'MPC_Z_VEL_I_ACC':profile['selected_integral']}
    value['expected_parameters']={**profile['expected_readback'],**value['requested_parameters']}
    if profile['mode']=='frozen':
        frozen=profile['freeze']
        if frozen['model_source_sha256']!=model['source_file_sha256']:
            raise ValueError('frozen calibration model/world identity changed')
        if abs(frozen['derived_hover_thrust']-model['derived_hover_thrust'])>1e-10:
            raise ValueError('frozen hover derivation changed')
    return value


def readback_evidence(profile, observed, maximum_age_s=None, now_wall=None):
    expected=profile['expected_parameters']
    actual={name:observed.get(name,{}).get('value') for name in expected}
    checks={}
    for name,value in expected.items():
        observed_value=actual[name]
        checks[name]=(isinstance(observed_value,(int,float)) and not isinstance(observed_value,bool)
            and math.isfinite(observed_value) and abs(observed_value-value)<=max(1e-5,abs(value)*1e-6))
    return {'passed':all(checks.values()),'profile_id':profile['profile_id'],'mode':profile['mode'],
        'transition_policy':profile.get('transition_policy','legacy'),
        'acceptance_policy':profile.get('acceptance_policy'),
        'requested':profile['requested_parameters'],'expected':expected,'actual_read_only_values':actual,
        'checks':checks,'model_derivation':profile['model_derivation'],
        'profile_sha256':profile['profile_sha256'],
        'scope':'isolated temporary rootfs, applied before arming; full actual PX4 parameter readback required'}



def stable_code_lock(manifest):
    # Per-run generated YAML has a different pathname, while source, binaries,
    # shared YAML and the explicitly loaded profile must be byte-identical.
    return {row['path']:row['sha256'] for row in manifest['runtime_identity']['code_file_lock']
        if row.get('sha256') and 'runtime' not in Path(row['path']).parts}


def validate_acceptance_policy(parameters):
    policy=parameters.get('takeoff_transition_policy_id','legacy')
    if policy!='legacy' and policy not in TRANSITION_POLICIES:
        raise ValueError('unknown transition policy')
    fixed={'motion_tracking_position_tolerance_m':.10,'takeoff_ground_max_rise_m':.75,
        'motion_max_horizontal_velocity_mps':.6,'motion_max_vertical_velocity_mps':.3,
        'motion_max_horizontal_acceleration_mps2':.5,'motion_max_vertical_acceleration_mps2':.3,
        'motion_brake_horizontal_mps2':.4,'motion_brake_vertical_mps2':.25,
        'motion_reaction_time_s':.3,'motion_tracking_soft_trigger_ratio':.60,
        'motion_tracking_soft_recover_ratio':.35,'motion_tracking_recovery_wait_s':3.,
        'motion_tracking_recovery_dwell_s':.3,'motion_tracking_recovery_min_samples':3,
        'takeoff_tracking_max_recoveries':2,'takeoff_transition_hard_limit_s':35. if policy=='legacy' else 45.,
        'motion_takeoff_bootstrap_height_m':.02,'motion_takeoff_bootstrap_duration_s':4.5,
        'takeoff_response_probe_height_m':.20,'takeoff_response_clearance_m':.05,
        'takeoff_response_dwell_s':.3,'takeoff_response_min_samples':3}
    enabled=('motion_tracking_soft_guard_enabled','takeoff_response_sync_enabled',
        'takeoff_transition_budget_enabled','motion_model_configured')
    fixed.update(motion_body_radius_m=.5,motion_map_margin_m=.05,motion_sigma_multiplier=2.)
    if policy!='legacy':
        enabled+=('motion_rounded_envelope_enabled',)
        if type(parameters.get('takeoff_reference_governor_enabled')) is not bool:
            raise ValueError('reference governor must be explicit')
    bad=[name for name,value in fixed.items() if not isinstance(parameters.get(name),(int,float))
        or isinstance(parameters.get(name),bool) or abs(parameters[name]-value)>1e-10]
    bad += [name for name in enabled if parameters.get(name) is not True]
    if bad:raise ValueError('controller profile acceptance policy differs: '+', '.join(bad))
    return {'passed':True,'fixed_parameters':fixed,'enabled_policies':list(enabled),
        'transition_policy':policy,
        'transition_limits':({**TRANSITION_POLICIES[policy],'trigger_ratio':.8,'recover_ratio':.55,'recovery_wait_s':5.} if policy!='legacy' else None),
        'reference_governor_enabled':parameters.get('takeoff_reference_governor_enabled',False),
        'nominal_publication_rate_hz':50.,'actual_publication_rate_requires_runtime_evidence':True}


def executor_policy_sample(s, parameters):
    """Check an actual certificate against the named immutable policy table."""
    policy=parameters.get('takeoff_transition_policy_id','legacy')
    if policy not in TRANSITION_POLICIES:return None
    if not (s.get('motion_stop_certificate_valid') or s.get('motion_contact_certificate_valid')):return None
    expected=TRANSITION_POLICIES[policy]
    kind=s.get('motion_certificate_kind')
    strict=kind=='AIRBORNE_STRICT'
    values={'motion_effective_position_tolerance_m':.1 if strict else expected['position_tolerance_m'],
        'motion_effective_body_radius_m':.5 if strict else expected['body_radius_m'],
        'motion_effective_soft_trigger_ratio':.6 if strict else .8,
        'motion_effective_recover_ratio':.35 if strict else .55,
        'motion_effective_recovery_wait_s':3. if strict else 5.}
    identity='airborne-strict-v2' if strict else f'takeoff-{policy}-v2'
    valid=(kind in ('GROUND_CONTACT','LIFTOFF_TRANSITION','AIRBORNE_STRICT')
        and s.get('motion_policy_id')==identity and s.get('motion_policy_revision')==2
        and not ((s.get('phase') in ('HOVER_SCAN','EXPLORE','TARGETING_CYCLE','RETURN_HOME') or
            s.get('takeoff_safety_stage')=='AIRBORNE_STRICT') and not strict)
        and all(type(s.get(k)) in (int,float) and math.isfinite(s[k]) and abs(s[k]-v)<1e-9 for k,v in values.items())
        and s.get('motion_reference_governor_enabled')==(False if strict else parameters['takeoff_reference_governor_enabled']))
    return {'valid':valid,'strict':strict,'expected':values,'identity':identity}


def tracking_policy_schedule(statuses, parameters):
    """Prepare sampling-time policy brackets; uncertainty falls back to strict."""
    schedule=[]
    for s in statuses:
        stamp=s.get('mission_status_stamp_ns')
        if type(stamp) is not int or stamp<=0:continue
        checked=executor_policy_sample(s,parameters)
        schedule.append((stamp,s.get('localization_epoch'),
            checked['expected']['motion_effective_position_tolerance_m'] if checked and checked['valid'] else .1,
            checked['identity'] if checked and checked['valid'] else 'strict_conservative_fallback'))
    schedule.sort(key=lambda r:r[0])
    return schedule,[r[0] for r in schedule]


def tracking_policy_at(prepared, sample_ns, epoch, parameters):
    if parameters.get('takeoff_transition_policy_id','legacy')=='legacy':
        return float(parameters['motion_tracking_position_tolerance_m']),'legacy'
    rows,times=prepared
    i=bisect.bisect_right(times,sample_ns)
    if 0<i<len(rows):
        a,b=rows[i-1],rows[i]
        if a[1]==epoch==b[1] and b[0]-a[0]<=300_000_000:
            # A bracket spanning strict ACK always takes the tighter bound;
            # no delayed 10 Hz task status can extend a relaxed 50 Hz policy.
            bound=min(a[2],b[2])
            identity=a[3] if a[3]==b[3] else 'strict_conservative_fallback'
            return bound,identity
    return .1,'strict_conservative_fallback'


def phase_policy_evidence(statuses,parameters):
    policy=parameters.get('takeoff_transition_policy_id','legacy')
    if policy=='legacy':return {'passed':True,'scope':'legacy policy'}
    bad=[];ground=air=0
    for s in statuses:
        checked=executor_policy_sample(s,parameters)
        if checked is None:continue
        strict=checked['strict']
        if strict:air+=1
        else:ground+=1
        if not checked['valid']:
            values=checked['expected']
            bad.append({'sample_ns':s.get('mission_status_stamp_ns'),'expected':values,'actual':{k:s.get(k) for k in values},'policy_id':s.get('motion_policy_id')})
    return {'passed':bool(ground and air and not bad),'ground_samples':ground,'strict_samples':air,
        'violations':bad[:20],'transition_policy':policy,'scope':'actual executor policy values, including atomic strict restoration'}


def validate_freeze(profile):
    freeze=profile.get('freeze',{})
    evidence=freeze.get('reports',[])
    if len(evidence)!=2 or any(not isinstance(row,dict) or not row.get('path') or not row.get('sha256') for row in evidence):
        raise ValueError('freeze requires exactly two consecutive passing calibration reports')
    if len({row['path'] for row in evidence})!=2:
        raise ValueError('calibration reports must be independent runs')
    reports=[];code_sets=[];models=[]
    for row in evidence:
        path=Path(row['path'])
        if digest(path)!=row['sha256']:
            raise ValueError('frozen calibration report identity changed')
        report=json.loads(path.read_text());reports.append(report)
        if not report.get('passed') or report.get('acceptance_scope')!='powerplant_takeoff_scan':
            raise ValueError('freeze requires actual takeoff/scan stage pass, not another scope')
        required_checks=('actual_takeoff_and_two_second_stable_hover','ground_to_strict_runtime_handoff_verified',
            'requested_stage_observed','actual_physical_contacts_verified','no_mission_fault')
        checks=report.get('checks',{})
        if not checks or not all(v is True for v in checks.values()) or any(checks.get(k) is not True for k in required_checks):
            raise ValueError('calibration report lacks complete passing physical/stage checks')
        manifest=json.loads(Path(report['manifest']).read_text())
        code_sets.append(stable_code_lock(manifest))
        controller=report.get('isolated_controller_profile',{})
        selected_policy=profile.get('selected_transition_policy','legacy')
        if controller.get('transition_policy','legacy')!=selected_policy:
            raise ValueError('calibration pair transition policies differ')
        if selected_policy!='legacy' and checks.get('actual_phase_policy_matches') is not True:
            raise ValueError('actual strict restoration was not verified')
        models.append(controller.get('model_derivation'))
        if not controller.get('passed') or controller.get('requested',{}).get('MPC_Z_VEL_I_ACC')!=profile['selected_integral']:
            raise ValueError('calibration report does not match selected integral')
        if controller.get('model_derivation',{}).get('source_file_sha256')!=freeze.get('model_source_sha256'):
            raise ValueError('calibration pair model identities differ')
        if report.get('runtime_identity_validation',{}).get('code',{}).get('passed') is not True:
            raise ValueError('calibration code identity was not verified')
    if not code_sets[0] or code_sets[0]!=code_sets[1] or code_sets[0]!=freeze.get('code_file_sha256'):
        raise ValueError('two calibration runs must use identical frozen source and binary files')
    if models[0]!=models[1]:raise ValueError('two calibration model derivations differ')
    if reports[0].get('isolated_controller_profile',{}).get('acceptance_policy')!=reports[1].get('isolated_controller_profile',{}).get('acceptance_policy'):
        raise ValueError('two calibration acceptance policies differ')
    rejected=freeze.get('rejected_reports',[])
    for row in rejected:
        report=json.loads(Path(row['path']).read_text())
        if digest(row['path'])!=row['sha256'] or report.get('passed'):
            raise ValueError('rejected candidate report identity changed')
        observed=report.get('isolated_controller_profile',{})
        if not observed.get('passed') or observed.get('requested',{}).get('MPC_Z_VEL_I_ACC')!=row['integral']:
            raise ValueError('larger candidate lacks verified actual parameter readback')
        if observed.get('transition_policy','legacy')!=profile.get('selected_transition_policy','legacy'):
            raise ValueError('larger candidate used a different transition policy')
        manifest=json.loads(Path(report['manifest']).read_text())
        if stable_code_lock(manifest)!=code_sets[0] or observed.get('model_derivation')!=models[0]:
            raise ValueError('calibration code or model changed between candidates; restart the sweep')
    if any(i not in {row.get('integral') for row in rejected} for i in CANDIDATES[:CANDIDATES.index(profile['selected_integral'])]):
        raise ValueError('larger candidate failure evidence is missing')
    # Code files, not a git HEAD alone, bind uncommitted implementations too.
    if not freeze.get('code_file_sha256'):
        raise ValueError('freeze lacks controller/mission code identities')
    for path,sha in freeze['code_file_sha256'].items():
        if digest(path)!=sha:
            raise ValueError('code changed since controller calibration; recalibrate before formal acceptance')


def freeze_profile(source, reports, output, rejected_reports=()):
    import yaml
    source=Path(source)
    base=yaml.safe_load(source.read_text())
    if len(reports)!=2:
        raise ValueError('provide the two consecutive passing reports for the selected candidate')
    values=[json.loads(Path(p).read_text()) for p in reports]
    controllers=[v.get('isolated_controller_profile',{}) for v in values]
    selected=controllers[0].get('requested',{}).get('MPC_Z_VEL_I_ACC')
    if selected not in CANDIDATES or any(c.get('requested',{}).get('MPC_Z_VEL_I_ACC')!=selected for c in controllers):
        raise ValueError('calibration pair has different or invalid integrals')
    rejected=[]
    for path in rejected_reports:
        report=json.loads(Path(path).read_text())
        integral=report.get('isolated_controller_profile',{}).get('requested',{}).get('MPC_Z_VEL_I_ACC')
        if report.get('passed') or integral not in CANDIDATES:
            raise ValueError('rejected report must document an actually tested failed formal candidate')
        rejected.append({'path':str(Path(path).resolve()),'sha256':digest(path),'integral':integral})
    if any(i not in {row['integral'] for row in rejected} for i in CANDIDATES[:CANDIDATES.index(selected)]):
        raise ValueError('larger integral candidates lack actual failed calibration evidence')
    manifest=json.loads(Path(values[0]['manifest']).read_text())
    code=stable_code_lock(manifest)
    base.update(status='frozen',selected_integral=float(selected))
    base['selected_transition_policy']=controllers[0].get('transition_policy','legacy')
    base['freeze']={'reports':[{'path':str(Path(p).resolve()),'sha256':digest(p)} for p in reports],
        'model_source_sha256':controllers[0]['model_derivation']['source_file_sha256'],
        'derived_hover_thrust':controllers[0]['model_derivation']['derived_hover_thrust'],
        'code_file_sha256':code,'selection_order':list(CANDIDATES),'rejected_reports':rejected}
    validate_freeze(base)
    Path(output).write_text(yaml.safe_dump(base,sort_keys=False))
    return base


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--freeze-profile',type=Path,required=True)
    p.add_argument('--reports',nargs=2,type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rejected-reports',nargs='*',type=Path,default=[])
    a=p.parse_args()
    freeze_profile(a.freeze_profile,a.reports,a.output,a.rejected_reports)

if __name__=='__main__':main()
