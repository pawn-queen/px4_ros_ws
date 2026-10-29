"""Static support geometry is evidence, never a fabricated navigation sample."""
from pathlib import Path
from types import SimpleNamespace
import hashlib
import importlib.util
import json
import struct

import pytest

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'

def load(name):
    spec=importlib.util.spec_from_file_location(name,SCRIPTS/(name+'.py'))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module

body=load('derive_sitl_body_envelope')
auditor=load('powerplant_takeoff_audit')


def expanded_body(*, missing=None, low_camera=False):
    # base_link is 24cm above the retained model-origin navigation reference.
    # The actual stance therefore comes from transformed gear vertices, not
    # from a negative conservative all-tilt body envelope.
    shapes={
        'base_link_collision_0':('0 0 .03','.2 .15 .08'),
        'base_link_collision_1':('-.08 0 -.14','.04 .04 .16'),
        'base_link_collision_2':('.08 0 -.14','.04 .04 .16'),
        'base_link_collision_3':('0 -.08 -.22','.2 .02 .014'),
        'base_link_collision_4':('0 .08 -.22','.2 .02 .014'),
    }
    xml=''.join(f'<collision name="{name}"><pose>{pose} 0 0 0</pose><geometry><box><size>{size}</size></box></geometry></collision>'
                for name,(pose,size) in shapes.items() if name!=missing)
    if low_camera:
        xml+='<collision name="camera"><pose>0 0 -.235 0 0 0</pose><geometry><box><size>.01 .01 .005</size></box></geometry></collision>'
    return '<sdf version="1.9"><model name="x500_depth"><link name="base_link"><pose>0 0 .24 0 0 0</pose>'+xml+'</link></model></sdf>'


def test_support_offsets_resolve_navigation_reference_and_complete_gear():
    report=body.derive_ground_support(expanded_body(),.35)
    assert report['verified']
    assert report['nominal_contact_offset_m']==pytest.approx(.013)
    assert len(report['support_points_flu'])==32
    assert len(report['lowest_rail_points_flu'])==8
    assert all(p[2]==pytest.approx(.013) for p in report['lowest_rail_points_flu'])
    assert report['support_first_height_gap_m']>0
    assert report['nominal_contact_offset_m']!=report['body_envelope']['minimum_z_m']
    assert report['support_geometry_sha256']==body.derive_ground_support(expanded_body(),.35)['support_geometry_sha256']


@pytest.mark.parametrize('missing',['base_link_collision_1','base_link_collision_3'])
def test_partial_landing_gear_cannot_issue_a_support_model(missing):
    with pytest.raises(ValueError,match='support collision'):
        body.derive_ground_support(expanded_body(missing=missing),.35)


def test_low_camera_contact_is_not_a_valid_landing_gear_support():
    with pytest.raises(ValueError,match='non-gear'):
        body.derive_ground_support(expanded_body(low_camera=True),.35)


@pytest.mark.parametrize('alpha',[-.01,float('nan'),2.])
def test_support_model_refuses_unbounded_tilt(alpha):
    with pytest.raises(ValueError,match='tilt'):
        body.derive_ground_support(expanded_body(),alpha)


@pytest.fixture
def scene(tmp_path,monkeypatch):
    models=tmp_path/'models';device=models/'device';device.mkdir(parents=True)
    (device/'model.config').write_text('<model><name>device</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
    def stl(path,x=0.,zlow=-1.,zhigh=1.):
        path.write_bytes(b'support audit fixture'.ljust(80,b' ')+struct.pack('<I',1)
                        +struct.pack('<12fH',1.,0.,0.,x,-1.,zlow,x,1.,zlow,x,0.,zhigh,0))
    for name in ('visual.stl','collision.stl'):stl(device/name)
    (device/'model.sdf').write_text('<sdf version="1.9"><model name="device"><link name="link">'
        '<visual name="v"><geometry><mesh><uri>model://device/visual.stl</uri></mesh></geometry></visual>'
        '<collision name="c"><geometry><mesh><uri>model://device/collision.stl</uri></mesh></geometry></collision>'
        '</link></model></sdf>')
    extra=''.join(f'<model name="indoor_enclosure_{name}"><pose>100 0 10 0 0 0</pose><link name="link"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model>'
        for name in ('west','east','north','south','ceiling'))
    extra+='<model name="ground"><link name="link"><collision name="floor"><geometry><plane><normal>0 0 1</normal><size>140 140</size></plane></geometry></collision></link></model>'
    world=tmp_path/'powerplant.sdf';world.write_text('<sdf version="1.9"><world name="powerplant"><include><uri>model://device</uri><pose>4 0 0 0 0 0</pose></include>'+extra+'</world></sdf>')
    expansion=expanded_body()
    sources={}
    for name in ('x500_depth','x500','x500_base','OakD-Lite'):
        p=models/name/'model.sdf';p.parent.mkdir();p.write_text(expansion if name=='x500_depth' else '<sdf version="1.9"><model name="unused"/></sdf>')
        sources[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
    audit=body.derive(expansion,.35)
    audit.update(source_sdf_sha256=sources,expanded_sdf_sha256=hashlib.sha256(expansion.encode()).hexdigest(),
                 configured_body_contains_collision_envelope=True)
    monkeypatch.setattr(auditor.subprocess,'run',lambda *args,**kwargs:SimpleNamespace(stdout=expansion,stderr=''))
    parameters={'takeoff_region_radius_m':1.5,'takeoff_region_height_m':6.2,'takeoff_height_m':5.,
        'motion_body_radius_m':.5,'motion_body_z_min_m':-.06,'motion_body_z_max_m':.5,
        'initial_scan_max_tilt_rad':.35,'explore_clearance_radius_m':.35}
    return world,models,audit,parameters,device,stl


def test_generated_support_is_bound_to_scene_body_and_explicit_parameters(scene):
    world,models,body_audit,parameters,_,_=scene
    report=auditor.generate_ground_support_model(world,models,body_audit,parameters)
    assert report['verified'] and report['ground_support_parameters']['takeoff_ground_support_enabled']
    assert report['nominal_reference_ground_z_enu_m']==pytest.approx(-.013)
    assert report['ground_contact_layer']['minimum_z_m']==0
    assert not report['ground_contact_layer']['support_is_free_space']
    assert report['evidence_prism']['minimum_enu_m'][2]==0
    assert report['ground_support_parameters']['takeoff_ground_land_max_age_s']==2
    assert report['ground_support_parameters']['takeoff_ground_transition_timeout_s']==12
    assert report['ground_support_parameters']['takeoff_ground_max_rise_m']==.75
    assert len(report['ground_support_parameters']['takeoff_ground_contact_points_flu'])==96
    assert 0<report['model_revision']<2**63
    again=auditor.generate_ground_support_model(world,models,body_audit,parameters)
    assert again['model_revision']==report['model_revision']
    changed=auditor.generate_ground_support_model(world,models,body_audit,parameters,floor_uncertainty_m=.02)
    assert changed['model_revision']!=report['model_revision']
    assert not report['runtime_contact_observation_proven']


def test_sub_four_centimetre_obstacle_is_not_waived_as_ground(scene):
    world,models,body_audit,parameters,device,stl=scene
    # The include translation is x=4. This triangle is at world x=1 and
    # z=.01.. .03: the old z>=.04 prism would miss it.
    stl(device/'collision.stl',x=-3,zlow=.01,zhigh=.03)
    report=auditor.generate_ground_support_model(world,models,body_audit,parameters)
    assert not report['verified']
    assert not report['ground_support_parameters']['takeoff_ground_support_enabled']
    assert not report['checks']['ground_to_takeoff_prism_separated']


def test_changed_body_source_invalidates_support_even_if_nominal_shape_looks_same(scene):
    world,models,body_audit,parameters,_,_=scene
    (models/'OakD-Lite/model.sdf').write_text('changed source')
    with pytest.raises(RuntimeError,match='changed since support audit'):
        auditor.generate_ground_support_model(world,models,body_audit,parameters)


def test_expansion_identity_and_body_limits_are_checked(scene):
    world,models,body_audit,parameters,_,_=scene
    wrong={**body_audit,'expanded_sdf_sha256':'wrong'}
    with pytest.raises(RuntimeError,match='expanded body changed'):
        auditor.generate_ground_support_model(world,models,wrong,parameters)
    with pytest.raises(RuntimeError,match='does not contain'):
        auditor.generate_ground_support_model(world,models,body_audit,{**parameters,'motion_body_radius_m':.01})


def test_support_plane_must_cover_complete_audited_footprint(scene):
    world,models,body_audit,parameters,_,_=scene
    world.write_text(world.read_text().replace('<size>140 140</size>','<size>.1 .1</size>'))
    with pytest.raises(RuntimeError,match='does not cover'):
        auditor.generate_ground_support_model(world,models,body_audit,parameters)
