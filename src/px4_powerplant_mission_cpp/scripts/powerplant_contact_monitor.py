#!/usr/bin/env python3
"""Evaluation-only contact instrumentation and phase-bound physical evidence.

Only temporary model copies gain sensors. Collision geometry, inertias, joints,
controllers and the actual world remain unchanged; no flight/map data is emitted.
"""
from __future__ import annotations
import copy
import hashlib
import json
import math
import numbers
import numpy as np
import os
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def file_lock(paths):
    return [{'path':str(path),'sha256':sha(path) if path.is_file() else None}
            for path in sorted({Path(p).resolve() for p in paths})]


def verify_file_lock(records):
    changed=[]
    for row in records:
        path=Path(row['path'])
        try:actual=sha(path) if path.is_file() else None
        except OSError:actual=None
        if actual!=row['sha256'] or actual is None:
            changed.append({'path':str(path),'expected_sha256':row['sha256'],'actual_sha256':actual})
    return {'passed':bool(records) and not changed,'locked_files':len(records),'changed_files':changed}


def physical_digest(xml):
    root=ET.fromstring(xml)
    for parent in root.iter():
        for child in list(parent):
            if child.tag=='sensor':parent.remove(child)
    def canonical(node):
        return [node.tag,sorted(node.attrib.items()),' '.join((node.text or '').split()),
                [canonical(child) for child in node]]
    return hashlib.sha256(json.dumps(canonical(root),separators=(',',':')).encode()).hexdigest()


def _expand(path,environment):
    result=subprocess.run(['gz','sdf','-p',str(path)],env=environment,text=True,
                          capture_output=True,timeout=30,check=True)
    ET.fromstring(result.stdout)
    return result.stdout


def instrument_vehicle_contacts(models,overlay,output,world,support_collision_names,
                                world_name='powerplant',instance='x500_depth_0',support_surfaces=None,support_region=None):
    models,overlay,output,world=map(Path,(models,overlay,output,world))
    overlay.mkdir(parents=True,exist_ok=True);output.mkdir(parents=True,exist_ok=True)
    for name in ('x500_depth','x500','x500_base','OakD-Lite'):
        if not (overlay/name).exists():shutil.copytree(models/name,overlay/name)
    environment={**os.environ,'SDF_PATH':os.pathsep.join(map(str,(overlay,models))),
                 'GZ_SIM_RESOURCE_PATH':os.pathsep.join(map(str,(overlay,models,world.parent)))}
    vehicle=overlay/'x500_depth/model.sdf'
    before=_expand(vehicle,environment)
    (output/'vehicle_before_contacts.sdf').write_text(before)
    support_names={str(x).split('::')[-1] for x in support_collision_names}
    if not support_names:raise RuntimeError('contact instrumentation requires audited support collision identities')
    inserted=[]
    for model_name in ('x500_depth','x500','x500_base','OakD-Lite'):
        path=overlay/model_name/'model.sdf';tree=ET.parse(path)
        for link in tree.getroot().findall('model/link'):
            for collision in link.findall('collision'):
                link_name,collision_name=link.get('name'),collision.get('name')
                if not link_name or not collision_name:raise RuntimeError('unnamed physical link/collision')
                sensor_name='issue3_contact_'+re.sub('[^A-Za-z0-9_]','_',link_name+'_'+collision_name)
                if link.find(f"sensor[@name='{sensor_name}']") is not None:
                    raise RuntimeError('duplicate evaluation contact sensor')
                sensor=ET.SubElement(link,'sensor',{'name':sensor_name,'type':'contact'})
                ET.SubElement(sensor,'always_on').text='true'
                ET.SubElement(sensor,'update_rate').text='250'
                ET.SubElement(ET.SubElement(sensor,'contact'),'collision').text=collision_name
                inserted.append({'source_model':model_name,'link':link_name,'collision':collision_name,
                                 'sensor':sensor_name,'audited_landing_gear':collision_name in support_names})
        tree.write(path,encoding='utf-8',xml_declaration=True)
    after=_expand(vehicle,environment)
    (output/'vehicle_with_contacts.sdf').write_text(after)
    first,second=physical_digest(before),physical_digest(after)
    if first!=second:raise RuntimeError('contact instrumentation changed expanded physical model')
    model=ET.fromstring(after).find('model')
    actual=[]
    for link in model.findall('link'):
        for collision in link.findall('collision'):
            name=collision.get('name');matches=[s for s in link.findall('sensor')
                if s.get('type')=='contact' and s.findtext('contact/collision')==name]
            matches=[s for s in matches if s.get('name','').startswith('issue3_contact_')]
            if len(matches)!=1:raise RuntimeError('expanded collision lacks a unique evaluation sensor')
            link_name=link.get('name');sensor=matches[0].get('name')
            actual.append({'link':link_name,'collision':name,'sensor':sensor,
                'scope':f'{instance}::{link_name}::{name}',
                'topic':f'/world/{world_name}/model/{instance}/link/{link_name}/sensor/{sensor}/contact',
                'audited_landing_gear':name in support_names})
    if not actual or len(actual)!=len(inserted):raise RuntimeError('incomplete expanded contact coverage')
    ground=[]
    for m in ET.parse(world).getroot().findall('world/model'):
        for link in m.findall('link'):
            for collision in link.findall('collision'):
                if collision.find('geometry/plane') is not None:
                    ground.append(f"{m.get('name')}::{link.get('name')}::{collision.get('name')}")
    if support_surfaces is not None:
        audited={f"{v['model']}::{v['link']}::{v['collision']}" for v in support_surfaces}
        if not audited or not audited<=set(ground):
            raise RuntimeError('support collision scope contradicts the audited world')
        ground=sorted(audited)
    if not ground:raise RuntimeError('audited support plane collision scope is unavailable')
    result={'enabled':True,'evaluation_only':True,'external_checkout_modified':False,
            'physical_model_unchanged':True,'before_physical_sha256':first,'after_physical_sha256':second,
            'before_expanded_sha256':sha(output/'vehicle_before_contacts.sdf'),
            'after_expanded_sha256':sha(output/'vehicle_with_contacts.sdf'),
            'world_sha256':sha(world),'instance':instance,'sensors':actual,
            'support_collision_scopes':ground,'support_region':support_region,
            'scope':'contact sensors only; every expanded physical collision covered; no map/waypoint publication'}
    (output/'contact_instrumentation.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def contact_packet(message,topic,receipt_wall,clock_ns):
    stamp=lambda h:int(h.stamp.sec)*1_000_000_000+int(h.stamp.nsec)
    rows=[]
    for contact in message.contact:
        sample=stamp(contact.header) or stamp(message.header)
        rows.append({'sample_ns':sample,'receipt_wall':receipt_wall,'clock_at_receipt_ns':clock_ns,
                     'topic':topic,'collision1':contact.collision1.name,'collision2':contact.collision2.name,
                     'collision1_id':contact.collision1.id,'collision2_id':contact.collision2.id,
                     'world_name':contact.world.name,'world_id':contact.world.id,
                     'positions':[[p.x,p.y,p.z] for p in contact.position],
                     'depths':list(contact.depth)})
    return rows


class ContactEvidence:
    def __init__(self,instrumentation,plane_z,plane_error,maximum_age_s=.3):
        self.instrumentation=instrumentation;self.plane_z=plane_z;self.plane_error=plane_error
        self.maximum_age_s=maximum_age_s;self.rows=[];self.dropped=0
        self.subscribed=set();self.advertised=set();self.callback_counts={};self.world_steps=[]

    def observe(self,rows):
        for row in rows:
            if len(self.rows)>=100000:self.dropped+=1
            else:self.rows.append(row)

    def result(self,statuses,events=()):
        statuses=sorted((x for x in statuses if type(x.get('mission_status_stamp_ns')) is int
                         and x['mission_status_stamp_ns']>0),key=lambda x:x['mission_status_stamp_ns'])
        import bisect
        # Stage/phase changes may happen between the 10 Hz status messages.
        # Their source-time events delimit the actual interval; a later ground
        # status must not excuse a contact sampled after strict promotion.
        timeline=[(x['mission_status_stamp_ns'],0,x) for x in statuses]
        strict_epochs={}
        for x in events:
            sample=x.get('event_stamp_ns')
            if type(sample) is not int or sample<=0 or not x.get('phase') or x.get('localization_epoch') is None:continue
            if x.get('event')=='takeoff_strict_handoff_complete':
                actual=x.get('motion_strict_promotion_stamp_ns')
                if type(actual) is int and 0<actual<=sample and (sample-actual)*1e-9<=self.maximum_age_s:
                    sample=actual
                    strict_epochs[x['localization_epoch']]=min(sample,strict_epochs.get(x['localization_epoch'],sample))
            timeline.append((sample,1,x))
        statuses=[];previous={}
        for sample,kind,row in sorted(timeline,key=lambda item:(item[0],item[1])):
            if kind:
                if previous.get('localization_epoch')!=row.get('localization_epoch'):
                    continue
                row={**previous,**row,'mission_status_stamp_ns':sample}
            strict_since=strict_epochs.get(row.get('localization_epoch'))
            if strict_since is not None and sample>=strict_since:
                row={**row,'takeoff_safety_stage':'AIRBORNE_STRICT'}
            previous=row;statuses.append(row)
        times=[x['mission_status_stamp_ns'] for x in statuses]
        sensors={x['topic']:x for x in self.instrumentation.get('sensors',[])}
        support=set(self.instrumentation.get('support_collision_scopes',[]))
        positive=0;failure_support=0;positive_samples=[];unsafe=[];unknown=[];excluded=0;counts={};deferred_clock=0
        received_steps=sorted((r for r in self.world_steps
            if isinstance(r.get('receipt_wall'),(int,float)) and math.isfinite(r['receipt_wall']) and type(r.get('sample_ns')) is int
            and not r.get('paused')),key=lambda r:r['receipt_wall'])
        received_times=[r['receipt_wall'] for r in received_steps]
        for row in self.rows:
            if row.get('before_monitor_clock') is True:
                excluded+=1;continue
            sample=row.get('sample_ns');sensor=sensors.get(row.get('topic'))
            if type(sample) is not int or sample<=0 or sensor is None:
                unknown.append({**row,'reason':'invalid contact sample or unregistered sensor'});continue
            if not times or sample<times[0]:
                excluded+=1;continue
            index=bisect.bisect_right(times,sample)-1;state=statuses[index]
            if (sample-times[index])*1e-9>self.maximum_age_s:
                unknown.append({**row,'reason':'contact has no fresh phase interval'});continue
            epoch=state.get('localization_epoch');phase=state.get('phase');stage=state.get('takeoff_safety_stage')
            clock=row.get('clock_at_receipt_ns')
            clock_valid=isinstance(clock,int) and -.05<=(clock-sample)*1e-9<=self.maximum_age_s
            # Evaluation-only bounded waiting: an early contact callback can
            # precede ROS /clock. Confirm against a later actual world step in
            # the same phase epoch, not by changing the original clock sample.
            if (not clock_valid and isinstance(clock,int) and clock<sample
                    and isinstance(row.get('receipt_wall'),(int,float)) and math.isfinite(row['receipt_wall'])):
                i=bisect.bisect_left(received_times,row['receipt_wall'])
                for step in received_steps[i:]:
                    if step['receipt_wall']-row['receipt_wall']>self.maximum_age_s:break
                    j=bisect.bisect_right(times,step['sample_ns'])-1
                    if (0<=step['sample_ns']-sample<=self.maximum_age_s*1e9 and j>=0
                            and statuses[j].get('localization_epoch')==epoch
                            and step['sample_ns']-times[j]<=self.maximum_age_s*1e9):
                        clock_valid=True;deferred_clock+=1;break
            if not clock_valid or epoch is None:
                unknown.append({**row,'reason':'contact sampling clock/epoch is unconfirmed'});continue
            first,second=row.get('collision1'),row.get('collision2')
            if first==sensor['scope']:other=second
            elif second==sensor['scope']:other=first
            else:
                unknown.append({**row,'reason':'contact collision identity disagrees with its sensor'});continue
            positions=row.get('positions',[])
            finite=bool(positions) and all(len(p)==3 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in p) for p in positions)
            on_plane=finite and all(abs(p[2]-self.plane_z)<=self.plane_error for p in positions)
            prearm=(phase in ('IDLE','START','SAFE_TAKEOFF') and state.get('arming_state')==1)
            postland=(phase=='COMPLETE' and state.get('arming_state')==1 and state.get('land_detected') is True)
            allowed_phase=prearm or postland or phase=='LANDING' or (phase=='SAFE_TAKEOFF' and stage in ('GROUND_CONTACT','LIFTOFF_TRANSITION'))
            region=self.instrumentation.get('support_region')
            inside_region=bool(region and finite and all(
                math.hypot(p[0]-region['center_enu_m'][0],p[1]-region['center_enu_m'][1])<=region['radius_m']
                for p in positions))
            safe=(sensor.get('audited_landing_gear') and other in support and on_plane and allowed_phase
                  and (inside_region or phase in ('LANDING','COMPLETE')))
            # Physical classification only: a still-grounded gear on its
            # surveyed floor during pre-strict failure is expected support.
            # It grants no movement permission and supplies no positive control.
            fault_support=(phase=='FAULT' and stage in ('GROUND_CONTACT','LIFTOFF_TRANSITION')
                and sensor.get('audited_landing_gear') and other in support and on_plane and inside_region)
            key=f'{phase}/{stage}';counts[key]=counts.get(key,0)+1
            if safe:
                positive+=1;positive_samples.append(sample)
            elif fault_support:
                failure_support+=1
            else:unsafe.append({**row,'phase':phase,'stage':stage,'localization_epoch':epoch,
                                'reason':'contact outside audited gear/support/phase contract'})
        expected=set(sensors)
        steps=[r for r in self.world_steps if type(r.get('sample_ns')) is int
               and type(r.get('iterations')) is int and not r.get('paused')]
        advancing=bool(len(steps)>=2 and max(r['iterations'] for r in steps)>min(r['iterations'] for r in steps)
                       and max(r['sample_ns'] for r in steps)>min(r['sample_ns'] for r in steps))
        overlap=bool(advancing and any(min(r['sample_ns'] for r in steps)<=t<=max(r['sample_ns'] for r in steps)
                                      for t in positive_samples))
        checks={'actual_world_steps_advance':advancing,'positive_control_during_measured_world_steps':overlap,
                'physical_model_unchanged':self.instrumentation.get('physical_model_unchanged') is True,
                'all_collision_topics_advertised':bool(expected) and expected<=self.advertised,
                'all_collision_subscriptions_accepted':bool(expected) and expected<=self.subscribed,
                'actual_gear_ground_positive_control':positive>0,
                'no_unexpected_physical_contact':not unsafe,
                'all_in_window_contacts_classified':not unknown,
                'no_contact_record_overflow':self.dropped==0}
        return {'passed':all(checks.values()),'checks':checks,'allowed_support_contact_count':positive,
                'deferred_clock_contacts_confirmed':deferred_clock,
                'prestrict_failure_support_contact_count':failure_support,
                'unsafe_contact_count':len(unsafe),'unclassified_contact_count':len(unknown),
                'excluded_before_monitor_ready':excluded,'dropped_contact_records':self.dropped,
                'phase_contact_counts':counts,'unsafe_contacts':unsafe[:32],'unclassified_contacts':unknown[:32],
                'registered_collision_sensors':len(expected),'callback_counts':self.callback_counts,
                'world_statistics_samples':len(steps),
                'world_statistics_first_last':([steps[0],steps[-1]] if steps else []),
                'scope':'actual contact sensor evidence, sample-time phase association and audited landing-gear support; silence alone is not proof'}


def takeoff_handoff_evidence(statuses,events,landed_samples,setpoints,support_model,parameters,navigation_samples=()):
    """Check the runtime ACK independently of the top-level phase sequence.

    The raw stopping box and simultaneous typed certificate are retained. The
    sampled publication continuity check supplements the executor unit tests;
    it does not claim a continuous-time flight tracking guarantee.
    """
    transitions=sorted((e for e in events if e.get('event') in
        ('takeoff_liftoff_transition','takeoff_strict_handoff_complete')
        and type(e.get('event_stamp_ns')) is int),key=lambda e:e['event_stamp_ns'])
    lift=next((e for e in transitions if e['event']=='takeoff_liftoff_transition'),None)
    strict=next((e for e in transitions if e['event']=='takeoff_strict_handoff_complete'),None)
    ack_sample=strict.get('motion_strict_promotion_stamp_ns') if strict else None
    stamp=ack_sample if type(ack_sample) is int and ack_sample>0 else 0
    epoch=strict.get('localization_epoch') if strict else None
    ack=strict.get('motion_strict_ack') if strict else None
    ack=ack if isinstance(ack,dict) else {}
    current=[s for s in statuses if s.get('localization_epoch')==epoch]
    ground=[s for s in current if s.get('takeoff_safety_stage')=='GROUND_CONTACT'
            and type(s.get('mission_status_stamp_ns')) is int and s['mission_status_stamp_ns']<stamp]
    airborne=[s for s in current if s.get('phase') in
              ('HOVER_SCAN','EXPLORE','TARGETING_CYCLE','RETURN_HOME')]
    model_revision=support_model.get('ground_support_parameters',{}).get('takeoff_ground_model_revision')
    box=ack.get('stopping_envelope_enu')
    lower=box.get('minimum_m') if isinstance(box,dict) else None
    lower_z=lower[2] if isinstance(lower,list) and len(lower)==3 else None
    threshold=float(support_model.get('plane_z_enu_m',0))+float(support_model.get('plane_uncertainty_m',0))+float(parameters['motion_map_margin_m'])
    fresh_airborne=[r for r in landed_samples if r.get('landed') is False
                    and type(r.get('sample_ns')) is int
                    and 0<=(stamp-r['sample_ns'])*1e-9<=float(parameters['takeoff_ground_land_max_age_s'])]
    refs=sorted((r for r in setpoints if type(r.get('timestamp')) is int),key=lambda r:r['timestamp'])
    before=next((r for r in reversed(refs) if r['timestamp']*1000<stamp),None)
    after=next((r for r in refs if r['timestamp']*1000>=stamp),None)
    continuity={}
    bracket=bool(before and after and 0<(after['timestamp']-before['timestamp'])*1e-6<=.06)
    if bracket:
        for field in ('position','velocity','acceleration'):
            a,b=before.get(field,[]),after.get(field,[])
            finite=len(a)==len(b)==3 and all(isinstance(v,numbers.Real)
                and not isinstance(v,bool) and math.isfinite(v) for v in a+b)
            continuity[field+'_jump']=math.dist(a,b) if finite else None
    continuous=bool(bracket and all(isinstance(v,(int,float)) and math.isfinite(v)
                                   and v<=1e-4 for v in continuity.values()))
    def finite_vector(value):
        return isinstance(value,(list,tuple)) and len(value)==3 and all(
            isinstance(v,numbers.Real) and not isinstance(v,bool) and math.isfinite(v) for v in value)
    state_sample=ack.get('state_sample_ns')
    nav=next((r for r in navigation_samples if type(state_sample) is int and state_sample>0
        and r.get('sample_ns')==state_sample and isinstance(r.get('covariance_valid'),(bool,np.bool_))
        and bool(r['covariance_valid'])
        and isinstance(r.get('task_identity'),(list,tuple)) and tuple(r['task_identity'])[:1]==(epoch,)),None)
    state_matches=bool(nav and finite_vector(ack.get('position_enu_m')) and finite_vector(ack.get('velocity_enu_mps'))
        and finite_vector(nav.get('position')) and finite_vector(nav.get('velocity'))
        and math.dist(nav['position'],ack['position_enu_m'])<=1e-5
        and math.dist(nav['velocity'],ack['velocity_enu_mps'])<=1e-5
        and math.dist(nav['velocity'],[0,0,0])<=float(parameters.get('motion_settle_speed_mps',.08))
        and 0<=(stamp-nav['sample_ns'])*1e-9<=min(float(parameters.get('max_localization_age_s',.5)),float(parameters.get('motion_input_timeout_s',.5))))
    named=parameters.get('takeoff_transition_policy_id','legacy')!='legacy'
    policy_fields={'position_tolerance_m':parameters.get('motion_tracking_position_tolerance_m',.1),
        'body_radius_m':parameters.get('motion_body_radius_m',.5),
        'map_margin_m':parameters.get('motion_map_margin_m',.05),
        'sigma_multiplier':parameters.get('motion_sigma_multiplier',2.),
        'soft_trigger_ratio':parameters.get('motion_tracking_soft_trigger_ratio',.6),
        'recover_ratio':parameters.get('motion_tracking_soft_recover_ratio',.35),
        'recovery_wait_s':parameters.get('motion_tracking_recovery_wait_s',3.)}
    ack_policy=bool(ack.get('policy_id')==('airborne-strict-v2' if named else 'strict')
        and ack.get('policy_revision')==(2 if named else 0)
        and ack.get('reference_governor_enabled') is False
        and all(type(ack.get(k)) in (int,float) and math.isfinite(ack[k]) and abs(ack[k]-v)<1e-9 for k,v in policy_fields.items())
        and type(ack.get('tracking_risk_ratio')) in (int,float)
        and math.isfinite(ack['tracking_risk_ratio']) and 0<=ack['tracking_risk_ratio']<policy_fields['soft_trigger_ratio'])
    publication_matches=False
    if bracket and ack:
        pairs=(('position','reference_position_enu_m'),('velocity','reference_velocity_enu_mps'),
               ('acceleration','reference_acceleration_enu_mps2'))
        publication_matches=all(finite_vector(after.get(field)) and finite_vector(ack.get(key)) and
            math.dist([after[field][1],after[field][0],-after[field][2]],ack[key])<=1e-4 for field,key in pairs)
    checks={
        'support_model_verified':support_model.get('verified') is True and support_model.get('passed') is True,
        'executor_ack_timestamp_confirmed':bool(strict and stamp>0 and ack.get('stamp_ns')==stamp
            and 0<=(strict['event_stamp_ns']-stamp)*1e-9<=.3),
        'audited_ground_stage_observed':bool(ground and model_revision and all(
            s.get('takeoff_ground_model_revision')==model_revision for s in ground)),
        'ordered_liftoff_then_strict_ack':bool(lift and strict and lift['event_stamp_ns']<stamp
            and lift.get('localization_epoch')==epoch and lift.get('phase')==strict.get('phase')=='SAFE_TAKEOFF'
            and lift.get('takeoff_safety_stage')=='LIFTOFF_TRANSITION'),
        'strict_typed_runtime_ack':bool(strict and strict.get('takeoff_safety_stage')=='AIRBORNE_STRICT'
            and strict.get('motion_certificate_kind')=='AIRBORNE_STRICT'
            and strict.get('motion_stop_certificate_valid') is True
            and strict.get('motion_contact_certificate_valid') is False
            and ack.get('certificate_kind')=='AIRBORNE_STRICT'
            and ack.get('stopped') is True and ack.get('mode')=='HOLD'
            and ack.get('localization_epoch')==epoch and ack.get('survey_revision')==model_revision
            and type(ack.get('session')) is int and ack['session']>0
            and lift and type(strict.get('motion_authorization_revision')) is int
            and type(lift.get('motion_authorization_revision')) is int
            and strict['motion_authorization_revision']==ack.get('authorization_revision')
            and strict['motion_authorization_revision']>lift['motion_authorization_revision']),
        'ack_state_matches_recorded_navigation':state_matches,
        'ack_uses_target_strict_policy':ack_policy,
        'published_ack_matches_committed_reference':publication_matches,
        'raw_stop_box_clear_of_support_error_and_map_margin':bool(isinstance(lower_z,(int,float))
            and math.isfinite(lower_z) and lower_z>threshold-5e-7),
        'fresh_independent_airborne_measurement':bool(fresh_airborne),
        'scan_and_exploration_require_strict_stage':bool(airborne and all(
            s.get('takeoff_safety_stage')=='AIRBORNE_STRICT'
            and s.get('mission_status_stamp_ns',0)>=stamp for s in airborne)),
        'sampled_pva_continuity_at_strict_ack':continuous,
    }
    return {'passed':all(checks.values()),'checks':checks,'liftoff_event':lift,'strict_ack_event':strict,
            'support_clearance_threshold_m':threshold,'raw_stop_box_lower_z_m':lower_z,
            'independent_airborne_samples':len(fresh_airborne),
            'executor_ack_sample_ns':stamp or None,
            'executor_ack_snapshot':ack or None,
            'handoff_reference_bracket_us':([before['timestamp'],after['timestamp']] if bracket else None),
            'sampled_reference_jumps':continuity,
            'scope':'source-time typed runtime acknowledgement and sampled reference continuity; no contact exception after strict promotion'}


def exploration_progress_evidence(pairs,events,identity):
    valid=[x for x in pairs if tuple(x.get('identity',()))==tuple(identity)]
    navigation=truth=0.
    if valid:
        a=valid[0]
        for b in valid:
            navigation=max(navigation,math.dist(a['navigation'],b['navigation']))
            truth=max(truth,math.dist(a['truth'],b['truth']))
    observed=[x for x in events if x.get('event')=='explore_viewpoint_observed'
              and x.get('phase')=='EXPLORE' and x.get('motion_stopped') is True
              and x.get('motion_mode')=='HOLD'
              and (x.get('localization_epoch'),x.get('task_roi_revision'))==tuple(identity)
              and isinstance(x.get('actual_depth_sample_id'),int) and x['actual_depth_sample_id']>0
              and isinstance(x.get('actual_depth_sample_stamp_ns'),int) and valid
              and x['actual_depth_sample_stamp_ns']>=valid[0]['sample_ns']
              and x['actual_depth_sample_id']>valid[0].get('depth_base',0)]
    checks={'paired_exploration_samples_available':len(valid)>=2,
            'actual_navigation_exploration_displacement_0_3m':navigation>=.3,
            'actual_truth_exploration_displacement_0_3m':truth>=.3,
            'stopped_viewpoint_with_new_depth_observed':bool(observed)}
    return {'passed':all(checks.values()),'checks':checks,'paired_samples':len(valid),
            'maximum_navigation_displacement_m':navigation,'maximum_truth_displacement_m':truth,
            'observed_viewpoints':len(observed),'full_roi_coverage_claimed':False}
