"""位置模式起飞验收必须区分固定位置指令、真实到高和严格HOLD交接。"""
import copy
import importlib.util
import math
from pathlib import Path
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[2]
SITL=ROOT/'validation/sitl'
def load(name):
    spec=importlib.util.spec_from_file_location(name,SITL/(name+'.py'))
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value

acceptance=load('run_acceptance')
evidence=load('position_takeoff_evidence')
flight=load('flight_evidence')


def fixture():
    state={'mission_status_stamp_ns':10_000_000_000,'phase':'HOVER_SCAN','px4_takeoff_enabled':True,
        'px4_takeoff_target_enu_m':[1.,2.,5.], 'px4_takeoff_strict_ack':True,
        'px4_takeoff_anchor_enu_m':[1.,2.,5.], 'motion_mode':'HOLD',
        'motion_stop_certificate_valid':True,'takeoff_safety_stage':'AIRBORNE_STRICT'}
    refs=[{'phase':'SAFE_TAKEOFF','position_only_takeoff':True,'position':[2.,1.,-5.],
        'velocity':[math.nan]*3,'acceleration':[math.nan]*3,'yaw':.3,'yawspeed':math.nan} for _ in range(10)]
    refs.append({'phase':'SAFE_TAKEOFF','position_only_takeoff':False,'position':[2.,1.,-5.],
        'velocity':[0.]*3,'acceleration':[0.]*3,'yaw':.3,'yawspeed':0.})
    nav=[{'sample_ns':9_990_000_000,'position':[1.,2.,5.],'covariance_valid':True}]
    physical={'actual_takeoff_and_stable_hover':{'passed':True}}
    return [state],[],refs,nav,physical


def test_ack_requires_fixed_command_and_new_actual_hover_anchor():
    assert evidence.takeoff_evidence(*fixture())['passed']


@pytest.mark.parametrize('failure',['yaw','yawrate','velocity','target','anchor','strict_reference','ack','certificate','truth'])
def test_one_missing_fact_rejects_takeoff(failure):
    states,events,refs,nav,physical=fixture()
    if failure=='yaw':refs[1]['yaw']+=.5
    if failure=='yawrate':refs[1]['yawspeed']=0.
    if failure=='velocity':refs[1]['velocity'][0]=0.
    if failure=='target':refs[1]['position'][2]-=1.
    if failure=='anchor':states[0]['px4_takeoff_anchor_enu_m'][0]+=1.
    if failure=='strict_reference':refs[-1]['position'][0]+=1.
    if failure=='ack':states[0]['px4_takeoff_strict_ack']=False
    if failure=='certificate':states[0]['motion_stop_certificate_valid']=False
    if failure=='truth':physical['actual_takeoff_and_stable_hover']['passed']=False
    assert not evidence.takeoff_evidence(states,events,refs,nav,physical)['passed']


@pytest.mark.parametrize('kind,reason',[('drift','PX4_TAKEOFF_HORIZONTAL_DRIFT'),('timeout','PX4_TAKEOFF_TIMEOUT')])
def test_injected_guard_requires_actual_command_and_auto_land(kind,reason):
    states,events,refs,nav,physical=fixture();states[0]['phase']='LANDING'
    events=[{'event':'px4_takeoff_abort','event_stamp_ns':10,'px4_takeoff_abort_reason':reason}]
    command=[{'command':21,'sample_ns':11}];vehicle=[{'nav_state':18,'sample_ns':12}]
    result=lambda c,v:evidence.takeoff_evidence(states,events,refs,nav,physical,kind,[{'sample_ns':9}],c,v)['fault_injection']
    assert result(command,vehicle)['passed']
    assert not result([],vehicle)['passed']
    assert not result(command,[])['passed']
    assert not result([{'command':21,'sample_ns':8}],vehicle)['passed']


def test_inherited_pid_overrides_are_rejected_for_stock_mode(monkeypatch):
    monkeypatch.setenv('PX4_PARAM_MPC_Z_VEL_I_ACC','.1')
    result=subprocess.run([sys.executable,str(SITL/'run_acceptance.py'),'--scenario','powerplant',
        '--px4-position-takeoff','--stop-after','hover','--prepare-only'],capture_output=True,text=True)
    assert result.returncode==2 and 'rejects inherited PX4 parameter overrides' in result.stderr


@pytest.mark.parametrize('extra',[['--controller-profile','anything.yaml'],['--px4-z-velocity-integral','.1'],
    ['--px4-hover-thrust','.7'],['--takeoff-policy','A']])
def test_position_takeoff_does_not_accept_legacy_calibration(extra):
    result=subprocess.run([sys.executable,str(SITL/'run_acceptance.py'),'--scenario','powerplant',
        '--px4-position-takeoff','--stop-after','hover','--prepare-only',*extra],capture_output=True,text=True)
    assert result.returncode==2 and 'forbids controller overrides' in result.stderr


def test_stock_startup_requires_actual_full_readback_not_bootstrap():
    names=('MPC_THR_HOVER','MPC_Z_VEL_I_ACC','COM_SPOOLUP_TIME','MPC_TKO_RAMP_T',
        'MPC_Z_VEL_MAX_UP','THR_MDL_FAC','MPC_THR_MAX')+tuple(f'SIM_GZ_EC_{kind}{i}' for i in range(1,5) for kind in ('MIN','MAX'))
    observed={name:{'value':.6 if name=='MPC_THR_HOVER' else 1.} for name in names}
    for i in range(1,5):
        observed[f'SIM_GZ_EC_MIN{i}']={'value':150.}
        observed[f'SIM_GZ_EC_MAX{i}']={'value':1000.}
    assert acceptance.takeoff_startup_evidence({'px4_takeoff_enabled':True},observed)['passed']
    for name in names:
        incomplete=copy.deepcopy(observed);incomplete.pop(name)
        assert not acceptance.takeoff_startup_evidence({'px4_takeoff_enabled':True},incomplete)['passed']


def test_hover_phase_string_alone_never_passes():
    r=acceptance.limited_stage_progress('hover',['START','SAFE_TAKEOFF','HOVER_SCAN'],[],{'phase':'HOVER_SCAN','localization_ready':True})
    assert not r['passed']


def test_px4_hover_proof_precedes_strict_ack_and_still_requires_physical_height():
    nav=[];truth=[];states=[]
    for i in range(50):
        t=10_000_000_000+i*100_000_000;z=0. if i<3 else 5.
        nav.append({'sample_ns':t,'position':[0.,0.,z],'velocity':[0.,0.,0.],'covariance_valid':True})
        truth.append({'sample_ns':t,'position':[0.,0.,z]})
        states.append({'mission_status_stamp_ns':t,'phase':'SAFE_TAKEOFF','home_enu_m':[0.,0.,0.],
            'takeoff_safety_stage':'PX4_ASCENT','land_detected':False,'motion_mode':'DISABLED','localization_epoch':1})
    params={'px4_takeoff_enabled':True,'takeoff_height_m':5.,'px4_takeoff_arrival_tolerance_m':.2}
    assert flight.physical_flight_evidence(nav,truth,states,params)['actual_takeoff_and_stable_hover']['passed']
    for row in truth:row['position'][2]=0.
    assert not flight.physical_flight_evidence(nav,truth,states,params)['actual_takeoff_and_stable_hover']['passed']


def test_arrival_proof_uses_new_point_two_metre_limit():
    nav=[];truth=[];states=[]
    for i in range(50):
        t=10_000_000_000+i*100_000_000;z=0. if i<3 else 4.7
        nav.append({'sample_ns':t,'position':[0.,0.,z],'velocity':[0.,0.,0.],'covariance_valid':True})
        truth.append({'sample_ns':t,'position':[0.,0.,z]})
        states.append({'mission_status_stamp_ns':t,'phase':'SAFE_TAKEOFF','home_enu_m':[0.,0.,0.],
            'takeoff_safety_stage':'PX4_ASCENT','land_detected':False,'motion_mode':'DISABLED','localization_epoch':1})
    assert not flight.physical_flight_evidence(nav,truth,states,{'px4_takeoff_enabled':True,'takeoff_height_m':5.})['actual_takeoff_and_stable_hover']['passed']
