"""Real-phase physical evidence must not be replaced by status strings."""
from pathlib import Path
import importlib.util
import json
import math
import shutil
from types import SimpleNamespace
import pytest

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/powerplant_contact_monitor.py'
spec=importlib.util.spec_from_file_location('contact_acceptance',SCRIPT)
contact=importlib.util.module_from_spec(spec);spec.loader.exec_module(contact)


def metadata():
    return {'physical_model_unchanged':True,'support_collision_scopes':['ground_plane::link::collision'],
        'support_region':{'center_enu_m':[0,0,0],'radius_m':1.5},
        'sensors':[{'topic':'/gear','scope':'x500_depth_0::base_link::gear','audited_landing_gear':True},
                   {'topic':'/rotor','scope':'x500_depth_0::rotor_0::blade','audited_landing_gear':False}]}


def evidence():
    value=contact.ContactEvidence(metadata(),0.,.01)
    value.advertised={'/gear','/rotor'};value.subscribed={'/gear','/rotor'}
    value.world_steps=[{'sample_ns':9_000_000_000,'iterations':1,'paused':False},
                       {'sample_ns':30_000_000_000,'iterations':5251,'paused':False}]
    return value


def state(sample=10_000_000_000,stage='GROUND_CONTACT',phase='SAFE_TAKEOFF',armed=2):
    return {'mission_status_stamp_ns':sample,'localization_epoch':7,'phase':phase,
            'takeoff_safety_stage':stage,'arming_state':armed,'land_detected':True}


def row(sample=10_100_000_000,**changes):
    value={'sample_ns':sample,'clock_at_receipt_ns':sample+10_000_000,'receipt_wall':30.,'topic':'/gear',
           'collision1':'x500_depth_0::base_link::gear','collision2':'ground_plane::link::collision',
           'positions':[[0.,0.,0.]],'depths':[.0001]}
    value.update(changes);return value


def test_sensor_instrumentation_cannot_change_physical_model():
    before='<sdf version="1.9"><model name="x"><link name="body"><inertial><mass>2</mass></inertial><collision name="box"><geometry><box><size>1 2 3</size></box></geometry></collision></link></model></sdf>'
    after=before.replace('</link>','<sensor name="contact" type="contact"><contact><collision>box</collision></contact></sensor></link>')
    assert contact.physical_digest(before)==contact.physical_digest(after)
    assert contact.physical_digest(before)!=contact.physical_digest(after.replace('<mass>2','<mass>3'))
    assert contact.physical_digest(before)!=contact.physical_digest(after.replace('<size>1 2 3','<size>1 2 4'))


def test_run_identity_rejects_changed_removed_or_missing_code_and_resources(tmp_path):
    path=tmp_path/'actual_model.sdf';path.write_text('physical baseline')
    lock=contact.file_lock([path]);assert contact.verify_file_lock(lock)['passed']
    path.write_text('different geometry');assert not contact.verify_file_lock(lock)['passed']
    path.unlink();assert not contact.verify_file_lock(lock)['passed']
    missing=contact.file_lock([tmp_path/'absent_executable'])
    assert not contact.verify_file_lock(missing)['passed']


@pytest.mark.parametrize('phase,stage,armed',[('IDLE','AIRBORNE_STRICT',1),('START','GROUND_CONTACT',1),
    ('SAFE_TAKEOFF','GROUND_CONTACT',2),('SAFE_TAKEOFF','LIFTOFF_TRANSITION',2),('LANDING','AIRBORNE_STRICT',2),
    ('COMPLETE','AIRBORNE_STRICT',1)])
def test_only_gear_contact_on_audited_support_is_accepted(phase,stage,armed):
    e=evidence();e.observe([row()]);r=e.result([state(stage=stage,phase=phase,armed=armed)])
    assert r['passed'] and r['allowed_support_contact_count']==1


@pytest.mark.parametrize('change',[
    {'positions':[[0,0,.02]]}, {'collision2':'power_plant_env::link::collision'},
    {'topic':'/rotor','collision1':'x500_depth_0::rotor_0::blade'},
])
def test_contact_geometry_and_body_identity_fail_closed(change):
    e=evidence();e.observe([row(**change)]);r=e.result([state()])
    assert not r['passed'] and r['unsafe_contact_count']==1


def test_sample_time_strict_phase_wins_over_later_ground_status():
    e=evidence();e.observe([row(sample=20_100_000_000)])
    r=e.result([state(),state(sample=20_000_000_000,stage='AIRBORNE_STRICT'),
                state(sample=21_000_000_000)])
    assert not r['passed'] and r['unsafe_contacts'][0]['stage']=='AIRBORNE_STRICT'


def test_exact_stage_event_closes_interval_between_status_updates():
    e=evidence();e.observe([row()])
    strict={'event_stamp_ns':10_050_000_000,'localization_epoch':7,'phase':'SAFE_TAKEOFF',
            'takeoff_safety_stage':'AIRBORNE_STRICT','event':'takeoff_strict_handoff_complete'}
    r=e.result([state()],[strict])
    assert not r['passed'] and r['unsafe_contacts'][0]['stage']=='AIRBORNE_STRICT'


def test_sparse_status_interval_cannot_authorize_contact():
    e=evidence();e.observe([row(sample=10_500_000_000)])
    r=e.result([state(),state(sample=11_000_000_000)])
    assert not r['passed'] and r['unclassified_contact_count']==1


def test_missing_positive_control_or_topic_is_not_physical_pass():
    e=evidence();assert not e.result([state()])['passed']
    e.observe([row()]);e.advertised.remove('/rotor')
    assert not e.result([state()])['passed']


def test_unknown_identity_and_late_transport_do_not_count_as_clearance():
    e=evidence();e.observe([row(collision1=''),row(clock_at_receipt_ns=11_000_000_000)])
    r=e.result([state()]);assert not r['passed'] and r['unclassified_contact_count']==2


def progress_event(**changes):
    event={'event':'explore_viewpoint_observed','phase':'EXPLORE','motion_stopped':True,'motion_mode':'HOLD',
           'localization_epoch':7,'task_roi_revision':2,'actual_depth_sample_id':8,
           'actual_depth_sample_stamp_ns':10_300_000_000}
    event.update(changes);return event


def progress_pairs(truth_end=.4):
    return [{'sample_ns':10_000_000_000,'navigation':[0,0,5],'truth':[0,0,5],'identity':(7,2),'depth_base':7},
            {'sample_ns':10_400_000_000,'navigation':[.4,0,5],'truth':[truth_end,0,5],'identity':(7,2),'depth_base':7}]


def test_exploration_requires_actual_motion_and_new_exposure():
    assert contact.exploration_progress_evidence(progress_pairs(),[progress_event()],(7,2))['passed']
    assert not contact.exploration_progress_evidence([],[],(7,2))['passed']
    assert not contact.exploration_progress_evidence(progress_pairs(.1),[progress_event()],(7,2))['passed']
    assert not contact.exploration_progress_evidence(progress_pairs(),[progress_event(actual_depth_sample_id=7)],(7,2))['passed']
    assert not contact.exploration_progress_evidence(progress_pairs(),[progress_event(actual_depth_sample_stamp_ns=9)],(7,2))['passed']
    assert not contact.exploration_progress_evidence(progress_pairs(),[progress_event()],(8,2))['passed']


def test_actual_sdformat_contact_overlay_preserves_expanded_geometry(tmp_path):
    if not shutil.which('gz'):pytest.skip('Gazebo parser unavailable')
    models=tmp_path/'models';models.mkdir()
    def write(name,body):
        folder=models/name;folder.mkdir()
        (folder/'model.config').write_text(f'<model><name>{name}</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
        (folder/'model.sdf').write_text(f'<sdf version="1.9"><model name="{name}">{body}</model></sdf>')
    box=lambda link,name:f'<link name="{link}"><collision name="{name}"><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link>'
    write('x500_base',box('base_link','base_link_collision_3'))
    write('x500','<include merge="true"><uri>model://x500_base</uri></include>')
    write('OakD-Lite',box('camera_link','camera_collision'))
    write('x500_depth','<include merge="true"><uri>model://x500</uri></include><include merge="true"><uri>model://OakD-Lite</uri></include>')
    world=tmp_path/'world.sdf';world.write_text('<sdf version="1.9"><world name="powerplant"><model name="ground_plane"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision></link></model></world></sdf>')
    result=contact.instrument_vehicle_contacts(models,tmp_path/'overlay',tmp_path/'output',world,['base_link_collision_3'])
    assert result['physical_model_unchanged'] and len(result['sensors'])==2
    assert result['before_physical_sha256']==result['after_physical_sha256']
    assert sum(x['audited_landing_gear'] for x in result['sensors'])==1
    assert not any('issue3_contact' in (models/x/'model.sdf').read_text() for x in ('x500_base','OakD-Lite'))


def test_contact_positive_control_requires_actual_advancing_world():
    e=evidence();e.observe([row()]);e.world_steps=[]
    assert not e.result([state()])['passed']
    e.world_steps=[{'sample_ns':10_000_000_000,'iterations':3,'paused':False},
                   {'sample_ns':10_000_000_000,'iterations':3,'paused':False}]
    assert not e.result([state()])['passed']


def test_grounded_failure_support_is_distinct_from_collision_or_flight_permission():
    e=evidence();e.observe([row(),row(sample=11_100_000_000)])
    statuses=[state(),state(sample=11_000_000_000,phase='FAULT',stage='LIFTOFF_TRANSITION')]
    result=e.result(statuses)
    assert result['passed'] and result['prestrict_failure_support_contact_count']==1
    assert result['allowed_support_contact_count']==1
    statuses[-1]['takeoff_safety_stage']='AIRBORNE_STRICT'
    assert not e.result(statuses)['passed']
    e=evidence();e.observe([row(sample=11_100_000_000)])
    statuses[-1]['takeoff_safety_stage']='LIFTOFF_TRANSITION'
    assert not e.result(statuses)['passed'] # failure support cannot be the positive control


def handoff_fixture():
    ground=state();ground['takeoff_ground_model_revision']=5
    lift={'event':'takeoff_liftoff_transition','event_stamp_ns':11_000_000_000,
          'phase':'SAFE_TAKEOFF','localization_epoch':7,'takeoff_safety_stage':'LIFTOFF_TRANSITION',
          'motion_authorization_revision':2}
    strict={'event':'takeoff_strict_handoff_complete','event_stamp_ns':12_000_000_000,
            'phase':'SAFE_TAKEOFF','localization_epoch':7,'takeoff_safety_stage':'AIRBORNE_STRICT',
            'motion_certificate_kind':'AIRBORNE_STRICT','motion_stop_certificate_valid':True,
            'motion_contact_certificate_valid':False,'motion_stopped':True,'motion_mode':'HOLD',
            'motion_authorization_revision':3,'motion_stopping_envelope_enu':{'minimum_m':[-.6,-.6,.061]}}
    scan=state(sample=12_100_000_000,phase='HOVER_SCAN',stage='AIRBORNE_STRICT')
    explore=state(sample=12_200_000_000,phase='EXPLORE',stage='AIRBORNE_STRICT')
    refs=[{'timestamp':t,'position':[0,0,.5],'velocity':[0,0,0],'acceleration':[0,0,0]}
          for t in (11_980_000,12_020_000)]
    return ([ground,scan,explore],[lift,strict],[{'sample_ns':11_950_000_000,'landed':False}],refs,
            {'verified':True,'passed':True,'plane_z_enu_m':0,'plane_uncertainty_m':.01,
             'ground_support_parameters':{'takeoff_ground_model_revision':5}},
            {'motion_map_margin_m':.05,'takeoff_ground_land_max_age_s':2.})


def test_strict_handoff_requires_runtime_ack_raw_box_and_fresh_airborne_sample():
    fixture=handoff_fixture()
    assert contact.takeoff_handoff_evidence(*fixture)['passed']
    fixture[1][1]['motion_certificate_kind']='LIFTOFF_TRANSITION'
    assert not contact.takeoff_handoff_evidence(*fixture)['passed']
    fixture=handoff_fixture();fixture[1][1]['motion_stopping_envelope_enu']['minimum_m'][2]=.059
    assert not contact.takeoff_handoff_evidence(*fixture)['passed']
    fixture=handoff_fixture();fixture[2].clear()
    assert not contact.takeoff_handoff_evidence(*fixture)['passed']


def test_handoff_does_not_accept_reference_reanchor_or_post_strict_stage_regression():
    fixture=handoff_fixture();fixture[3][1]['position'][0]=.01
    assert not contact.takeoff_handoff_evidence(*fixture)['passed']
    fixture=handoff_fixture();fixture[0][-1]['takeoff_safety_stage']='GROUND_CONTACT'
    assert not contact.takeoff_handoff_evidence(*fixture)['passed']


def test_explicit_controller_calibration_needs_real_readback():
    runner_path=SCRIPT.with_name('run_p2_sitl.py')
    spec=importlib.util.spec_from_file_location('runner_controller_calibration',runner_path)
    runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
    args=SimpleNamespace(px4_hover_thrust=.7463328867,px4_z_velocity_integral=None)
    assert not runner.controller_profile_evidence(args,{})['passed']
    assert not runner.controller_profile_evidence(args,{'MPC_THR_HOVER':{'value':.6}})['passed']
    assert runner.controller_profile_evidence(args,{'MPC_THR_HOVER':{'value':.7463329}})['passed']
    args.px4_z_velocity_integral=0.
    assert not runner.controller_profile_evidence(args,{'MPC_THR_HOVER':{'value':.7463329}})['passed']
