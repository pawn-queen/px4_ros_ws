"""Controller calibration must be model bound and physically witnessed."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
import yaml

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
def module(name):
    spec=importlib.util.spec_from_file_location(name,SCRIPTS/(name+'.py'))
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
controller=module('p2_sitl_controller')
flight=module('p2_flight_evidence')
BASE=Path(__file__).resolve().parents[2]/'px4_powerplant_mission/config/p2_sitl_controller.yaml'


def resolved(tmp_path,candidate=2):
    value=controller.load_profile(BASE,candidate)
    return controller.resolve_profile(value,{'derived_hover_thrust':.74633053,'source_file_sha256':{'model':'hash'}})


def test_profile_requires_explicit_candidate_or_validated_freeze(tmp_path):
    with pytest.raises(ValueError,match='frozen'):controller.load_profile(BASE)
    for value in controller.CANDIDATES:
        assert controller.load_profile(BASE,value)['selected_integral']==value
    with pytest.raises(ValueError,match='nonzero'):controller.load_profile(BASE,0)


def test_complete_actual_readback_not_only_hover_and_integral(tmp_path):
    profile=resolved(tmp_path)
    observed={name:{'value':value} for name,value in profile['expected_parameters'].items()}
    assert controller.readback_evidence(profile,observed)['passed']
    for name in profile['expected_parameters']:
        bad=copy.deepcopy(observed);bad.pop(name)
        assert not controller.readback_evidence(profile,bad)['passed'],name
        bad=copy.deepcopy(observed);bad[name]['value']+=.1
        assert not controller.readback_evidence(profile,bad)['passed'],name


def test_hover_uses_expanded_mass_world_gravity_and_esc_mapping(monkeypatch,tmp_path):
    models=tmp_path/'models';models.mkdir()
    for name in ('x500_depth','x500','x500_base','OakD-Lite'):
        folder=models/name;folder.mkdir();(folder/'model.sdf').write_text('<sdf/>')
    world=tmp_path/'world.sdf';world.write_text('<sdf><world><gravity>0 0 -9.8066</gravity></world></sdf>')
    masses=[2.,.0160769230769,.0160769230769,.0160769230769,.0160769230769,.061,.02]
    links=''.join(f'<link name="l{i}"><inertial><mass>{m}</mass></inertial></link>' for i,m in enumerate(masses))
    motors=''.join(f'<plugin><motorConstant>8.54858e-6</motorConstant><maxRotVelocity>1000</maxRotVelocity><motorNumber>{i}</motorNumber></plugin>' for i in range(4))
    expanded='<sdf><model>'+links+motors+'</model></sdf>'
    monkeypatch.setattr(controller.subprocess,'run',lambda *a,**k:SimpleNamespace(stdout=expanded))
    profile=controller.load_profile(BASE,2)
    actual=controller.derive_model(models,world,profile['expected_readback'])
    assert actual['total_mass_kg']==pytest.approx(2.1453076923076)
    assert actual['derived_hover_thrust']==pytest.approx(.74633053,abs=1e-7)
    assert actual['maximum_ideal_lift_n']==pytest.approx(34.19432)
    bad=profile['expected_readback'].copy();bad['SIM_GZ_EC_MAX4']=900
    with pytest.raises(ValueError,match='unequal ESC'):controller.derive_model(models,world,bad)


def physical_fixture():
    nav=[];truth=[];states=[]
    for i in range(101):
        t=10_000_000_000+i*100_000_000
        phase='SAFE_TAKEOFF' if i<20 else 'HOVER_SCAN' if i<55 else 'EXPLORE' if i<70 else 'RETURN_HOME' if i<90 else 'LANDING'
        z=0 if i<5 else 5
        nav.append({'sample_ns':t,'position':[0,0,z],'velocity':[0,0,0],'covariance_valid':True})
        truth.append({'sample_ns':t,'position':[0,0,z-.013]})
        states.append({'mission_status_stamp_ns':t,'phase':phase,'home_enu_m':[0,0,0],
            'takeoff_safety_stage':'AIRBORNE_STRICT','motion_mode':'HOLD','motion_stopped':True,
            'motion_stop_certificate_valid':True,'localization_epoch':3})
    return nav,truth,states,{'takeoff_height_m':5,'return_home_height_m':5}


def test_physical_takeoff_hover_and_return_require_both_navigation_and_truth():
    values=physical_fixture();result=flight.physical_flight_evidence(*values)
    assert result['actual_takeoff_and_stable_hover']['passed']
    assert result['actual_return_before_landing']['passed']
    for row in values[1]:row['position'][2]=-.013
    result=flight.physical_flight_evidence(*values)
    assert not result['actual_takeoff_and_stable_hover']['passed']
    assert not result['actual_return_before_landing']['passed']


def test_phase_presence_or_brief_arrival_does_not_prove_return():
    values=physical_fixture()
    for row in values[0]:
        if 17_000_000_000<=row['sample_ns']<18_900_000_000:row['position'][0]=1.
    assert not flight.physical_flight_evidence(*values)['actual_return_before_landing']['passed']
    values=physical_fixture()
    for state in values[2]:
        if state['phase']=='RETURN_HOME':state['motion_stopped']=False
    assert not flight.physical_flight_evidence(*values)['actual_return_before_landing']['passed']


def test_duplicate_samples_and_sparse_truth_cannot_accumulate_dwell():
    values=physical_fixture();values=(values[0]*4,[values[1][0],values[1][-1]],values[2],values[3])
    result=flight.physical_flight_evidence(*values)
    assert not result['actual_takeoff_and_stable_hover']['passed']
    assert not result['actual_return_before_landing']['passed']


def test_changed_spool_or_non_linear_esc_profile_is_refused(tmp_path):
    value=yaml.safe_load(BASE.read_text());value['expected_readback']['COM_SPOOLUP_TIME']=0
    path=tmp_path/'wrong.yaml';path.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError,match='fixed'):controller.load_profile(path,2)
    value['expected_readback']['COM_SPOOLUP_TIME']=1;value['expected_readback']['THR_MDL_FAC']=.5
    path.write_text(yaml.safe_dump(value))
    with pytest.raises(ValueError,match='linear ESC'):controller.load_profile(path,2)


def calibration_reports(tmp_path,integral=2):
    code=tmp_path/'controller.cpp';code.write_text('actual implementation')
    manifest=tmp_path/'manifest.json';manifest.write_text(json.dumps({'runtime_identity':{
        'code_file_lock':[{'path':str(code),'sha256':controller.digest(code)}]}}))
    paths=[]
    for i in range(2):
        value={'passed':True,'acceptance_scope':'powerplant_takeoff_scan',
            'isolated_controller_profile':{'passed':True,'requested':{'MPC_Z_VEL_I_ACC':integral},
                'actual_read_only_values':{'MPC_Z_VEL_I_ACC':integral},
                'model_derivation':{'source_file_sha256':{'model':'hash'},'derived_hover_thrust':.74633053}},
            'manifest':str(manifest),'runtime_identity_validation':{'code':{'passed':True}},
            'checks':{name:True for name in ('actual_takeoff_and_two_second_stable_hover',
                'ground_to_strict_runtime_handoff_verified','requested_stage_observed','actual_physical_contacts_verified','no_mission_fault')}}
        path=tmp_path/f'pass{i}.json';path.write_text(json.dumps(value));paths.append(path)
    return paths,code


def test_freeze_binds_two_reports_and_code_and_rejects_later_change(tmp_path):
    reports,code=calibration_reports(tmp_path)
    output=tmp_path/'frozen.yaml'
    controller.freeze_profile(BASE,reports,output)
    frozen=controller.load_profile(output)
    assert frozen['mode']=='frozen' and frozen['selected_integral']==2.
    code.write_text('implementation changed')
    with pytest.raises(ValueError,match='code changed'):controller.load_profile(output)


def test_lower_integral_cannot_be_selected_without_failed_larger_candidate(tmp_path):
    reports,_=calibration_reports(tmp_path,1.)
    with pytest.raises(ValueError,match='larger integral'):
        controller.freeze_profile(BASE,reports,tmp_path/'frozen.yaml')


def test_frozen_success_cannot_be_manufactured_from_wrong_scope_or_one_pass(tmp_path):
    reports,_=calibration_reports(tmp_path)
    value=json.loads(reports[-1].read_text());value['acceptance_scope']='powerplant_roi_access'
    reports[-1].write_text(json.dumps(value))
    with pytest.raises(ValueError,match='actual takeoff/scan'):
        controller.freeze_profile(BASE,reports,tmp_path/'frozen.yaml')
    with pytest.raises(ValueError,match='two consecutive'):
        controller.freeze_profile(BASE,reports[:1],tmp_path/'single.yaml')


def test_inherited_unlisted_pid_override_cannot_enter_profile(monkeypatch):
    runner=module('run_p2_sitl')
    monkeypatch.setenv('PX4_PARAM_MPC_Z_VEL_P_ACC','999')
    monkeypatch.setattr(runner.sys,'argv',['run','--scenario','powerplant','--stop-after','scan',
        '--controller-profile',str(BASE),'--controller-calibration-candidate','2'])
    with pytest.raises(SystemExit) as error:runner.parse_args()
    assert error.value.code==2


def test_frozen_acceptance_policy_cannot_widen_tracking_or_ground_height():
    config=BASE.with_name('powerplant.yaml');profile=BASE.with_name('p2_simulation.yaml')
    shared=yaml.safe_load(config.read_text())['powerplant_offboard_mission']['ros__parameters']
    sim=yaml.safe_load(profile.read_text())['powerplant_offboard_mission']['ros__parameters']
    values={**shared,**sim}
    assert not values['takeoff_response_sync_enabled']
    assert not values['takeoff_transition_budget_enabled']
    # Only the actual-world audit grants these in the derived runner profile.
    values.update(takeoff_response_sync_enabled=True,takeoff_transition_budget_enabled=True)
    assert controller.validate_acceptance_policy(values)['passed']
    for name,value in [('motion_tracking_position_tolerance_m',.11),('takeoff_ground_max_rise_m',.8),
            ('takeoff_response_sync_enabled',False),('motion_tracking_recovery_wait_s',4)]:
        changed={**values,name:value}
        with pytest.raises(ValueError,match=name):controller.validate_acceptance_policy(changed)


def braking_fixture(truth_keeps_moving=False):
    nav=[];truth=[];states=[]
    for i in range(12):
        t=10_000_000_000+i*100_000_000
        x=min(i,.3/.1)*.01
        nav.append({'sample_ns':t,'position':[x,0,0],
            'velocity':[.1 if i<4 else 0,0,0],'covariance_valid':True})
        truth.append({'sample_ns':t,'position':[100+(i*.02 if truth_keeps_moving else x),-3,2]})
        states.append({'mission_status_stamp_ns':t,'phase':'SAFE_TAKEOFF','home_enu_m':[0,0,0],
            'takeoff_safety_stage':'LIFTOFF_TRANSITION','localization_epoch':3,
            'motion_mode':'HOLD' if i==0 or i>=4 else 'BRAKING','motion_stopped':i>=4,
            'motion_stopping_envelope_enu':{'minimum_m':[-.1,-.1,-.1],'maximum_m':[.1,.1,.1]}})
    return nav,truth,states,{}


def test_braking_reports_actual_distance_dwell_and_keeps_truth_coordinates_independent():
    value=flight.physical_flight_evidence(*braking_fixture())['stopping_evidence']
    assert value['exercised'] and value['all_observed_episodes_stopped']
    episode=value['episodes'][0]
    assert episode['measured_stop_confirmed'] and episode['low_speed_dwell_s']>=.3
    assert episode['low_speed_independent_samples']>=3
    assert episode['navigation_displacement_to_confirmation_m']==pytest.approx(.02)
    assert episode['truth_displacement_to_confirmation_m']==pytest.approx(.02)
    assert episode['sampled_navigation_reference_inside_entry_box']
    assert episode['truth_inside_navigation_box'] is None
    assert episode['trajectory'][0]['truth_position_world_enu_m'][0]>100
    assert episode['trigger_to_physical_response_latency_s'] is None
    assert not episode['continuous_body_containment_proven']


def test_hold_label_without_physical_stop_does_not_confirm_braking():
    value=flight.physical_flight_evidence(*braking_fixture(True))['stopping_evidence']
    assert value['exercised'] and not value['all_observed_episodes_stopped']
    assert not value['episodes'][0]['measured_stop_confirmed']


def test_unexercised_braking_is_explicit_not_a_successful_stop_test():
    value=flight.physical_flight_evidence(*physical_fixture())['stopping_evidence']
    assert value['status']=='not_exercised' and not value['exercised']
    assert value['all_observed_episodes_stopped'] is None and value['episode_count']==0


def test_short_or_epoch_broken_hold_cannot_confirm_stopping_dwell():
    nav,truth,states,params=braking_fixture()
    for row in states:
        if row['motion_mode']=='HOLD':row['localization_epoch']=row['mission_status_stamp_ns']
    value=flight.physical_flight_evidence(nav,truth,states,params)['stopping_evidence']
    assert value['exercised'] and not value['all_observed_episodes_stopped']

@pytest.mark.parametrize('policy', ['A','B','C','D'])
def test_named_policy_keeps_strict_parameters_and_requires_actual_restoration(policy):
    shared=yaml.safe_load(BASE.with_name('powerplant.yaml').read_text())['powerplant_offboard_mission']['ros__parameters']
    sim=yaml.safe_load(BASE.with_name('p2_simulation.yaml').read_text())['powerplant_offboard_mission']['ros__parameters']
    parameters={**shared,**sim,'takeoff_response_sync_enabled':True,'takeoff_transition_budget_enabled':True,
        'takeoff_transition_policy_id':policy,'takeoff_transition_hard_limit_s':45.,'takeoff_reference_governor_enabled':True}
    validated=controller.validate_acceptance_policy(parameters)
    assert validated['transition_limits']=={**controller.TRANSITION_POLICIES[policy],
        'trigger_ratio':.8,'recover_ratio':.55,'recovery_wait_s':5.}
    ground={'motion_contact_certificate_valid':True,'motion_certificate_kind':'LIFTOFF_TRANSITION','motion_policy_id':f'takeoff-{policy}-v2',
        'motion_policy_revision':2,'motion_effective_position_tolerance_m':controller.TRANSITION_POLICIES[policy]['position_tolerance_m'],
        'motion_effective_body_radius_m':controller.TRANSITION_POLICIES[policy]['body_radius_m'],
        'motion_effective_soft_trigger_ratio':.8,'motion_effective_recover_ratio':.55,
        'motion_effective_recovery_wait_s':5.,'motion_reference_governor_enabled':True}
    strict={**ground,'motion_certificate_kind':'AIRBORNE_STRICT','motion_contact_certificate_valid':False,
        'motion_stop_certificate_valid':True,'motion_policy_id':'airborne-strict-v2',
        'motion_effective_position_tolerance_m':.1,'motion_effective_body_radius_m':.5,
        'motion_effective_soft_trigger_ratio':.6,'motion_effective_recover_ratio':.35,
        'motion_effective_recovery_wait_s':3.,'motion_reference_governor_enabled':False}
    assert controller.phase_policy_evidence([ground,strict],parameters)['passed']
    assert not controller.phase_policy_evidence([ground],parameters)['passed']
    unsafe={**strict,'motion_effective_position_tolerance_m':.15}
    assert not controller.phase_policy_evidence([ground,unsafe],parameters)['passed']
    for key,value in [('motion_sigma_multiplier',1.),('motion_map_margin_m',.03),('takeoff_transition_hard_limit_s',46.)]:
        with pytest.raises(ValueError):controller.validate_acceptance_policy({**parameters,key:value})


def test_sampled_tracking_uses_whitelisted_brackets_and_strict_at_handoff():
    params={'takeoff_transition_policy_id':'C','takeoff_reference_governor_enabled':True}
    ground={'mission_status_stamp_ns':1_000_000_000,'localization_epoch':7,
        'phase':'SAFE_TAKEOFF','takeoff_safety_stage':'LIFTOFF_TRANSITION',
        'motion_contact_certificate_valid':True,'motion_certificate_kind':'LIFTOFF_TRANSITION',
        'motion_policy_id':'takeoff-C-v2','motion_policy_revision':2,
        'motion_effective_position_tolerance_m':.15,'motion_effective_body_radius_m':.5,
        'motion_effective_soft_trigger_ratio':.8,'motion_effective_recover_ratio':.55,
        'motion_effective_recovery_wait_s':5.,'motion_reference_governor_enabled':True}
    strict={**ground,'mission_status_stamp_ns':1_200_000_000,'takeoff_safety_stage':'AIRBORNE_STRICT',
        'motion_certificate_kind':'AIRBORNE_STRICT','motion_policy_id':'airborne-strict-v2',
        'motion_effective_position_tolerance_m':.1,'motion_effective_soft_trigger_ratio':.6,
        'motion_effective_recover_ratio':.35,'motion_effective_recovery_wait_s':3.,
        'motion_reference_governor_enabled':False}
    rows=[ground,{**ground,'mission_status_stamp_ns':1_100_000_000},strict]
    schedule=controller.tracking_policy_schedule(rows,params)
    assert controller.tracking_policy_at(schedule,1_050_000_000,7,params)==(.15,'takeoff-C-v2')
    assert controller.tracking_policy_at(schedule,1_150_000_000,7,params)[0]==.1
    for sample,epoch in [(900_000_000,7),(1_300_000_000,7),(1_050_000_000,8)]:
        assert controller.tracking_policy_at(schedule,sample,epoch,params)==(.1,'strict_conservative_fallback')
    for invalid in ({'motion_effective_position_tolerance_m':.2},{'phase':'HOVER_SCAN'},
                    {'motion_policy_revision':3},{'motion_reference_governor_enabled':False}):
        schedule=controller.tracking_policy_schedule([ground,{**rows[1],**invalid}],params)
        assert controller.tracking_policy_at(schedule,1_050_000_000,7,params)[0]==.1
