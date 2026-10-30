#!/usr/bin/env python3
"""Offline paired navigation/Gazebo evidence, never an online flight input."""
from __future__ import annotations
import bisect
import math


def _finite3(value):
    return isinstance(value,(list,tuple)) and len(value)==3 and all(
        isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in value)


def _norm(value):return math.sqrt(sum(v*v for v in value))
def _sub(a,b):return [x-y for x,y in zip(a,b)]



def stopping_evidence(rows, stop_speed, maximum_gap_s):
    """Summarize observed BRAKING episodes without claiming causal control proof.

    Navigation positions and the executor box share the navigation frame. Raw
    Gazebo positions remain in the simulator world frame; no home-relative
    transform is invented to make them fit that navigation certificate.
    """
    episodes=[];active=[];settled=[];box=None;gap_ns=int(maximum_gap_s*1e9)
    def finish(reason, confirmed=False):
        if not active:return
        first,last=active[0],active[-1]
        valid_box=(isinstance(box,dict) and _finite3(box.get('minimum_m')) and _finite3(box.get('maximum_m'))
            and all(a<=b for a,b in zip(box['minimum_m'],box['maximum_m'])))
        contained=bool(valid_box and all(all(a<=v<=b for a,v,b in zip(box['minimum_m'],r['navigation'],box['maximum_m'])) for r in active))
        def length(field):return sum(_norm(_sub(b[field],a[field])) for a,b in zip(active,active[1:]))
        def number(value):return value if math.isfinite(value) else None
        episodes.append({'status':'observed_stopped' if confirmed else 'unconfirmed','termination_reason':reason,
            'measured_stop_confirmed':confirmed,'entry_sample_ns':first['sample_ns'],
            'confirmed_stop_sample_ns':last['sample_ns'] if confirmed else None,
            'first_sustained_low_speed_sample_ns':settled[0]['sample_ns'] if confirmed else None,
            'brake_to_first_sustained_low_speed_s':(settled[0]['sample_ns']-first['sample_ns'])*1e-9 if confirmed else None,
            'brake_to_confirmation_s':(last['sample_ns']-first['sample_ns'])*1e-9 if confirmed else None,
            'low_speed_dwell_s':(settled[-1]['sample_ns']-settled[0]['sample_ns'])*1e-9 if settled else 0.,
            'low_speed_independent_samples':len(settled),
            'entry_navigation_speed_mps':number(first['navigation_speed_mps']),
            'entry_truth_speed_mps':number(first['truth_speed_mps']),
            'navigation_displacement_to_confirmation_m':_norm(_sub(last['navigation'],first['navigation'])),
            'truth_displacement_to_confirmation_m':_norm(_sub(last['truth'],first['truth'])),
            'navigation_sampled_path_length_m':length('navigation'),'truth_sampled_path_length_m':length('truth'),
            'entry_raw_stopping_envelope_enu':box,'entry_motion_status_stamp_ns':first['state'].get('mission_status_stamp_ns'),
            'entry_stop_box_valid':valid_box,'sampled_navigation_reference_inside_entry_box':contained,
            'truth_inside_navigation_box':None,
            'truth_box_comparison':'not_asserted_without_independent_navigation_to_world_frame_certificate',
            'trigger_to_physical_response_latency_s':None,
            'response_latency_scope':'BRAKING is a sampled reported state; no exact trigger/actuation timestamp is available',
            'continuous_body_containment_proven':False,
            'trajectory':[{'sample_ns':r['sample_ns'],'motion_mode':r['state'].get('motion_mode'),
                'navigation_position_enu_m':r['navigation'],'truth_position_world_enu_m':r['truth'],
                'navigation_speed_mps':number(r['navigation_speed_mps']),
                'truth_speed_mps':number(r['truth_speed_mps'])} for r in active]})
    for row in rows:
        mode=row['state'].get('motion_mode')
        if active and (row['sample_ns']-active[-1]['sample_ns']>gap_ns
                or row['state'].get('localization_epoch')!=active[-1]['state'].get('localization_epoch')):
            finish('sample_gap_or_epoch_change');active=[];settled=[]
        if not active:
            if mode!='BRAKING':continue
            box=row['state'].get('motion_stopping_envelope_enu');active=[row];settled=[]
        else:
            if mode not in ('BRAKING','HOLD'):
                finish('motion_left_braking_or_hold_before_measured_stop');active=[];settled=[];continue
            active.append(row)
        low=(mode=='HOLD' and row['state'].get('motion_stopped')
            and row['navigation_speed_mps']<=stop_speed and row['truth_speed_mps']<=stop_speed)
        if low:
            settled.append(row)
            if len(settled)>=3 and (row['sample_ns']-settled[0]['sample_ns'])*1e-9>=.3:
                finish('measured_navigation_and_truth_low_speed_dwell',True);active=[];settled=[]
        else:settled=[]
    finish('recording_ended_before_confirmed_measured_stop')
    exercised=bool(episodes)
    return {'status':'not_exercised' if not exercised else 'observed_stopped' if all(e['measured_stop_confirmed'] for e in episodes) else 'unconfirmed',
        'exercised':exercised,'all_observed_episodes_stopped':all(e['measured_stop_confirmed'] for e in episodes) if exercised else None,
        'all_sampled_navigation_references_inside_entry_boxes':all(e['sampled_navigation_reference_inside_entry_box'] for e in episodes) if exercised else None,
        'episode_count':len(episodes),'required_low_speed_dwell_s':.3,'minimum_distinct_samples':3,
        'stop_speed_limit_mps':stop_speed,'episodes':episodes,
        'scope':'sampled physical stopping evidence; not a continuous swept-body certificate or trigger-to-actuator latency proof'}

def physical_flight_evidence(navigation, truth, statuses, parameters):
    """Require actual height and stopped return before the LANDING transition.

    Samples are paired by exposure time with bounded interpolation intervals;
    state strings alone, stale poses and one unverified landed flag never pass.
    The truth home is independently anchored at the same ground sample as the
    navigation home. Truth is used only in this post-run acceptance function.
    """
    truth=sorted((r for r in truth if type(r.get('sample_ns')) is int and _finite3(r.get('position'))),key=lambda r:r['sample_ns'])
    # Duplicate source timestamps do not become more independent samples.
    truth=list({r['sample_ns']:r for r in truth}.values())
    times=[r['sample_ns'] for r in truth]
    states=sorted((s for s in statuses if type(s.get('mission_status_stamp_ns')) is int),key=lambda s:s['mission_status_stamp_ns'])
    state_times=[s['mission_status_stamp_ns'] for s in states]
    max_gap=float(parameters.get('motion_reaction_time_s',.3))
    max_gap_ns=int(max_gap*1e9)
    def truth_at(t):
        i=bisect.bisect_left(times,t)
        if i<len(times) and times[i]==t:return truth[i]['position']
        if i==0 or i==len(times) or times[i]-times[i-1]>max_gap_ns:return None
        u=(t-times[i-1])/(times[i]-times[i-1])
        return [a+(b-a)*u for a,b in zip(truth[i-1]['position'],truth[i]['position'])]
    def state_at(t):
        i=bisect.bisect_right(state_times,t)-1
        if i<0 or t-state_times[i]>max_gap_ns:return None
        return states[i]
    rows=[];seen=set()
    for row in sorted(navigation,key=lambda r:r.get('sample_ns',0)):
        t=row.get('sample_ns')
        if type(t) is not int or t in seen or not row.get('covariance_valid') or not _finite3(row.get('position')) or not _finite3(row.get('velocity')):continue
        seen.add(t); p=truth_at(t);s=state_at(t)
        if p is None or s is None:continue
        # Independent truth velocity from a symmetric real-time interval.
        before,after=truth_at(t-50_000_000),truth_at(t+50_000_000)
        truth_speed=_norm(_sub(after,before))/.1 if before is not None and after is not None else math.inf
        rows.append({'sample_ns':t,'navigation':row['position'],'truth':p,'navigation_speed_mps':_norm(row['velocity']),
            'truth_speed_mps':truth_speed,'state':s})
    anchors=[r for r in rows if r['state'].get('phase')=='SAFE_TAKEOFF']
    anchor=anchors[0] if anchors else None
    home=anchor['state'].get('home_enu_m') if anchor else None
    if not _finite3(home):home=anchor['navigation'] if anchor else None
    truth_home=anchor['truth'] if anchor else None
    height=float(parameters.get('takeoff_height_m',5))
    return_height=float(parameters.get('return_home_height_m',height))
    acceptance=float(parameters.get('target_acceptance_m',.45))
    stop_speed=float(parameters.get('motion_settle_speed_mps',.08))
    takeoff_nav=[home[0],home[1],home[2]+height] if home else None
    takeoff_truth=[truth_home[0],truth_home[1],truth_home[2]+height] if truth_home else None
    return_nav=[home[0],home[1],home[2]+return_height] if home else None
    return_truth=[truth_home[0],truth_home[1],truth_home[2]+return_height] if truth_home else None
    landing_times=[s['mission_status_stamp_ns'] for s in states if s.get('phase')=='LANDING']
    first_landing=min(landing_times) if landing_times else None
    def interval(phase,duration,target_nav,target_truth):
        run=[];best=[]
        for row in rows:
            s=row['state'];t=row['sample_ns']
            valid=bool(target_nav and target_truth and s.get('phase') in phase
                and s.get('takeoff_safety_stage')=='AIRBORNE_STRICT'
                and s.get('motion_mode')=='HOLD' and s.get('motion_stopped')
                and s.get('motion_stop_certificate_valid')
                and row['navigation_speed_mps']<=stop_speed and row['truth_speed_mps']<=stop_speed
                and _norm(_sub(row['navigation'],target_nav))<=acceptance
                and _norm(_sub(row['truth'],target_truth))<=acceptance
                and (first_landing is None or t<first_landing))
            if not valid:
                run=[];continue
            if run and (t-run[-1]['sample_ns']>max_gap_ns or s.get('localization_epoch')!=run[-1]['state'].get('localization_epoch')):run=[]
            run.append(row)
            if len(run)>len(best):best=list(run)
            if len(run)>=3 and (t-run[0]['sample_ns'])*1e-9>=duration:break
        dwell=(best[-1]['sample_ns']-best[0]['sample_ns'])*1e-9 if best else 0
        return {'passed':len(best)>=3 and dwell>=duration,'required_stopped_duration_s':duration,
            'observed_stopped_duration_s':dwell,'independent_navigation_samples':len(best),
            'start_sample_ns':best[0]['sample_ns'] if best else None,'end_sample_ns':best[-1]['sample_ns'] if best else None,
            'navigation_target_enu_m':target_nav,'truth_target_enu_m':target_truth,
            'maximum_navigation_distance_m':max((_norm(_sub(r['navigation'],target_nav)) for r in best),default=None),
            'maximum_truth_distance_m':max((_norm(_sub(r['truth'],target_truth)) for r in best),default=None),
            'maximum_navigation_speed_mps':max((r['navigation_speed_mps'] for r in best),default=None),
            'maximum_truth_speed_mps':max((r['truth_speed_mps'] for r in best),default=None)}
    takeoff=interval(('SAFE_TAKEOFF','HOVER_SCAN'),2.,takeoff_nav,takeoff_truth)
    returned=interval(('RETURN_HOME',),.3,return_nav,return_truth)
    returned['landing_followed_stopped_return']=bool(first_landing and returned['end_sample_ns'] and returned['end_sample_ns']<first_landing)
    returned['passed']=bool(returned['passed'] and returned['landing_followed_stopped_return'])
    return {'actual_takeoff_and_stable_hover':takeoff,'actual_return_before_landing':returned,
        'anchor_sample_ns':anchor['sample_ns'] if anchor else None,'navigation_home_enu_m':home,'truth_home_enu_m':truth_home,
        'paired_navigation_truth_samples':len(rows),'maximum_pair_gap_s':max_gap,
        'position_acceptance_m':acceptance,'stop_speed_mps':stop_speed,
        'stopping_evidence':stopping_evidence(rows,stop_speed,max_gap),
        'scope':'offline independent Gazebo model-origin and navigation proof; truth never enters planning'}
