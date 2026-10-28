"""Powerplant acceptance must use the locked real scene and explicit scope."""
from pathlib import Path
from types import SimpleNamespace
import hashlib
import importlib.util
import json
import sys
import struct
import pytest

SCRIPT=Path(__file__).resolve().parents[1]/'scripts/run_p2_sitl.py'
spec=importlib.util.spec_from_file_location('powerplant_sitl_acceptance',SCRIPT)
runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
audit_spec=importlib.util.spec_from_file_location('takeoff_auditor',SCRIPT.with_name('powerplant_takeoff_audit.py'))
auditor=importlib.util.module_from_spec(audit_spec);audit_spec.loader.exec_module(auditor)


def locked_scene(tmp_path):
    assets=tmp_path/'assets';model=assets/'models/device';model.mkdir(parents=True)
    (model/'model.config').write_text('<model><sdf>model.sdf</sdf></model>')
    (model/'surface.stl').write_text('solid test\nfacet normal 1 0 0\nouter loop\nvertex 0 -1 -1\nvertex 0 1 -1\nvertex 0 0 1\nendloop\nendfacet\nendsolid test\n')
    (model/'model.sdf').write_text('<sdf version="1.9"><model name="device"><static>true</static><link name="link"><visual name="v"><geometry><mesh><uri>model://device/surface.stl</uri></mesh></geometry></visual></link></model></sdf>')
    world=assets/'worlds/powerplant.sdf';world.parent.mkdir();world.write_text('<sdf version="1.9"><world name="powerplant"><include><uri>model://device</uri><pose>4 0 0 0 0 0</pose></include></world></sdf>')
    entries=[{'path':str(p.relative_to(assets)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(assets.rglob('*')) if p.is_file()]
    lock=tmp_path/'lock.json';lock.write_text(json.dumps({'schema_version':1,'files':entries}));return assets,world,lock


def test_complete_recursive_asset_lock(tmp_path):
    assets,world,lock=locked_scene(tmp_path)
    result=runner.read_asset_lock(lock,assets,world)
    assert result['passed'] and result['recursive_scene_references_verified']
    assert len(result['verified_files'])==4


def test_changed_mesh_and_pending_commit_are_rejected(tmp_path):
    assets,world,lock=locked_scene(tmp_path)
    payload=json.loads(lock.read_text());payload['asset_commit_pending']=True;lock.write_text(json.dumps(payload))
    with pytest.raises(RuntimeError,match='committed'):runner.read_asset_lock(lock,assets,world)
    payload['asset_commit_pending']=False;lock.write_text(json.dumps(payload))
    (assets/'models/device/surface.stl').write_text('changed geometry')
    with pytest.raises(RuntimeError,match='changed'):runner.read_asset_lock(lock,assets,world)


def test_unlocked_recursive_mesh_and_escaping_path_rejected(tmp_path):
    assets,world,lock=locked_scene(tmp_path)
    payload=json.loads(lock.read_text());payload['files']=[x for x in payload['files'] if not x['path'].endswith('.stl')];lock.write_text(json.dumps(payload))
    with pytest.raises(RuntimeError,match='not locked'):runner.read_asset_lock(lock,assets,world)
    payload['files'].append({'path':'../outside','sha256':'0'*64});lock.write_text(json.dumps(payload))
    with pytest.raises(RuntimeError,match='escaping'):runner.read_asset_lock(lock,assets,world)


def test_visual_mesh_transform_is_evaluation_only(tmp_path):
    pytest.importorskip('vtkmodules')
    assets,world,_=locked_scene(tmp_path)
    evaluator=runner.VisualSceneEvaluator(world,assets/'models')
    hit=evaluator.first_hit([0,0,0],[1,0,0])
    assert hit['kind']=='mesh' and hit['parameter']==pytest.approx(4)
    assert evaluator.first_hit([0,5,0],[1,0,0]) is None
    assert evaluator.visual_meshes[0]['triangles']==1


def test_limited_stage_requires_observation_and_actual_access_motion():
    phases=['START','SAFE_TAKEOFF','HOVER_SCAN','EXPLORE']
    state={'phase':'EXPLORE','localization_ready':True,'explore_roi_mode':'ROI_ACCESS','motion_mode':'TRACKING'}
    events=[{'event':'initial_scan_complete'}]
    assert not runner.limited_stage_progress('scan',phases,[],state)['passed']
    assert runner.limited_stage_progress('scan',phases,events,state)['passed']
    assert not runner.limited_stage_progress('roi-access',phases,events,state,.5,0)['passed']
    good=runner.limited_stage_progress('roi-access',phases,events,state,.5,.4)
    assert good['passed'] and not good['full_flight_acceptance'] and not good['task_roi_complete']


def test_real_world_entrypoint_preserves_distinct_acceptance_scope(tmp_path):
    args=SimpleNamespace(scenario='powerplant',px4_root=tmp_path,image_chain_only=False,stop_after='scan')
    assert runner.scenario_world(args)==tmp_path/'Tools/simulation/gz/worlds/powerplant.sdf'
    assert runner.acceptance_scope(args)=='powerplant_takeoff_scan'
    args.stop_after='mission'
    assert runner.acceptance_scope(args)=='powerplant_local_roi_mission'


def test_landing_requires_fresh_independent_px4_evidence():
    state={'phase':'COMPLETE'}
    status={'sample_ns':4_900_000_000,'receipt_wall':10.0,'arming_state':1}
    landed={'sample_ns':4_950_000_000,'receipt_wall':10.05,'landed':True}
    assert runner.landing_completion_evidence(state,status,landed,5_000_000_000,10.05,10.1,1)['passed']
    assert not runner.landing_completion_evidence(state,None,None,5_000_000_000,10.05,10.1,1)['passed']
    assert not runner.landing_completion_evidence(state,{**status,'arming_state':2},landed,5_000_000_000,10.05,10.1,1)['passed']
    assert not runner.landing_completion_evidence(state,status,{**landed,'landed':False},5_000_000_000,10.05,10.1,1)['passed']


@pytest.mark.parametrize('change', ['status_receipt','land_sample','future_land_sample','clock'])
def test_complete_phase_cannot_hide_stale_or_future_landing_evidence(change):
    state={'phase':'COMPLETE'}
    status={'sample_ns':4_900_000_000,'receipt_wall':10.0,'arming_state':1}
    landed={'sample_ns':4_950_000_000,'receipt_wall':10.05,'landed':True}
    clock_wall=10.05
    if change=='status_receipt':status['receipt_wall']=8.0
    if change=='land_sample':landed['sample_ns']=3_000_000_000
    if change=='future_land_sample':landed['sample_ns']=6_000_000_000
    if change=='clock':clock_wall=8.0
    result=runner.landing_completion_evidence(state,status,landed,5_000_000_000,clock_wall,10.1,1)
    assert not result['passed']


def test_access_motion_needs_new_paired_samples_in_same_task_and_epoch():
    first={'sample_ns':100,'navigation':[0,0,0],'truth':[0,0,0]}
    anchor,nav,truth=runner.access_motion_progress(None,first,(4,7),True)
    assert nav==truth==0
    later={'sample_ns':200,'navigation':[.4,0,0],'truth':[.41,0,0]}
    retained,nav,truth=runner.access_motion_progress(anchor,later,(4,7),True)
    assert retained is anchor and nav==pytest.approx(.4) and truth==pytest.approx(.41)
    for identity in ((5,7),(4,8)):
        changed,nav,truth=runner.access_motion_progress(anchor,later,identity,True)
        assert changed['identity']==identity and nav==truth==0
    for sample,active in ((None,True),(later,False)):
        assert runner.access_motion_progress(anchor,sample,(4,7),active)==(None,0,0)
    duplicate={**later,'sample_ns':100}
    assert runner.access_motion_progress(anchor,duplicate,(4,7),True)[1:]==(0,0)
    backwards={**later,'sample_ns':50}
    reset,nav,truth=runner.access_motion_progress(anchor,backwards,(4,7),True)
    assert reset['sample_ns']==50 and nav==truth==0


def test_visual_mesh_rotated_scaled_pose_preserves_axial_depth_parameter(tmp_path):
    pytest.importorskip('vtkmodules')
    assets,world,_=locked_scene(tmp_path)
    model=assets/'models/device/model.sdf'
    model.write_text(model.read_text().replace('<uri>model://device/surface.stl</uri>',
        '<uri>model://device/surface.stl</uri><scale>2 3 4</scale>'))
    world.write_text(world.read_text().replace('4 0 0 0 0 0','4 0 0 0 0 1.5707963267948966'))
    evaluator=runner.VisualSceneEvaluator(world,assets/'models')
    hit=evaluator.first_hit([4,-2,0],[0,2,0])
    assert hit['parameter']==pytest.approx(1)  # unnormalised pinhole ray
    assert evaluator.first_hit([4,-2,0],[0,2,0],maximum_parameter=.4) is None


@pytest.mark.parametrize('explicit_include_pose', [False,True])
def test_include_pose_overrides_model_pose_only_when_present(tmp_path,explicit_include_pose):
    pytest.importorskip('vtkmodules')
    assets,world,_=locked_scene(tmp_path)
    model=assets/'models/device/model.sdf'
    model.write_text(model.read_text().replace('<model name="device">',
        '<model name="device"><pose>6 0 0 0 0 0</pose>'))
    if not explicit_include_pose:world.write_text(world.read_text().replace('<pose>4 0 0 0 0 0</pose>',''))
    evaluator=runner.VisualSceneEvaluator(world,assets/'models')
    assert evaluator.first_hit([0,0,0],[1,0,0])['parameter']==pytest.approx(4 if explicit_include_pose else 6)


def test_visual_scene_includes_ceiling_walls_and_finite_floor(tmp_path):
    world=tmp_path/'world.sdf'
    world.write_text('''<sdf version="1.9"><world name="powerplant">
      <model name="indoor_enclosure_west"><pose>-58.25 10 9 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>0.5 136 18</size></box></geometry></visual></link></model>
      <model name="indoor_enclosure_ceiling"><pose>10 10 18.25 0 0 0</pose><link name="l"><visual name="v"><geometry><box><size>137 137 0.5</size></box></geometry></visual></link></model>
      <model name="ground_plane"><pose>10 10 0 0 0 0</pose><link name="l"><visual name="v"><geometry><plane><normal>0 0 1</normal><size>140 140</size></plane></geometry></visual></link></model>
      </world></sdf>''')
    evaluator=runner.VisualSceneEvaluator(world,tmp_path)
    assert evaluator.first_hit([0,0,5],[-1,0,0])['parameter']==pytest.approx(58)
    assert evaluator.first_hit([0,0,5],[0,0,1])['parameter']==pytest.approx(13)
    assert evaluator.first_hit([0,0,5],[0,0,-1])['parameter']==pytest.approx(5)
    assert evaluator.first_hit([100,100,5],[0,0,-1]) is None


def test_render_context_auto_retains_display_and_explicit_egl_choice():
    assert runner.resolve_render_context('auto',{'DISPLAY':':0'})=='display'
    assert runner.resolve_render_context('auto',{})=='egl'
    assert runner.resolve_render_context('egl',{'DISPLAY':':0'})=='egl'
    with pytest.raises(RuntimeError,match='DISPLAY'):runner.resolve_render_context('display',{})


def test_cpu_overlay_resolves_nested_sensor_with_actual_sdformat(tmp_path):
    import os
    import shutil
    import subprocess
    import xml.etree.ElementTree as ET
    if not shutil.which('gz'):pytest.skip('Gazebo CLI is unavailable')
    original=tmp_path/'original';overlay=tmp_path/'overlay'
    for directory,width,rate in ((original,1920,30),(overlay,640,10)):
        model=directory/'camera';model.mkdir(parents=True)
        (model/'model.config').write_text('<model><name>camera</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
        (model/'model.sdf').write_text(f'''<sdf version="1.9"><model name="camera"><link name="camera_link">
          <sensor name="rgb" type="camera"><update_rate>{rate}</update_rate><camera><image>
          <width>{width}</width><height>360</height></image></camera></sensor>
          </link></model></sdf>''')
    vehicle=overlay/'vehicle';vehicle.mkdir()
    sdf=vehicle/'model.sdf'
    sdf.write_text('<sdf version="1.9"><model name="vehicle"><include><uri>model://camera</uri></include></model></sdf>')
    env=os.environ.copy();env['SDF_PATH']=str(original)
    runner.configure_resource_paths(env,overlay,original,tmp_path)
    result=subprocess.run(['gz','sdf','-p',str(sdf)],env=env,text=True,capture_output=True,timeout=10)
    assert result.returncode==0,result.stderr
    sensor=ET.fromstring(result.stdout).find('.//sensor')
    assert sensor.findtext('camera/image/width')=='640'
    assert float(sensor.findtext('update_rate'))==10


def test_sensor_profile_acceptance_rejects_native_load_labeled_cpu():
    configured={'configured_sensors':{'rgb':{'width':640,'height':360,'configured_update_rate_hz':10},
        'depth':{'width':320,'height':240,'configured_update_rate_hz':10},
        'lidar':{'horizontal':{'samples':'360'},'configured_update_rate_hz':15}}}
    actual={name:{**dict(size),'unique_exposures':20,'observed_unique_exposure_rate_sim_hz':rate}
        for name,size,rate in [('rgb',{'width':640,'height':360},10),
            ('depth',{'width':320,'height':240},9.9),('lidar',{'range_samples':360},15)]}
    assert runner.sensor_profile_evidence(configured,actual)['passed']
    actual['rgb'].update(width=1920,height=1080,observed_unique_exposure_rate_sim_hz=30)
    failed=runner.sensor_profile_evidence(configured,actual)
    assert not failed['passed'] and not failed['checks']['rgb_dimensions_match']
    assert not failed['checks']['rgb_exposure_rate_matches']
    assert not runner.sensor_profile_evidence(configured,{})['passed']


@pytest.mark.parametrize('renderer,confirmed',[
    ('D3D12 (NVIDIA GeForce RTX 5060 Laptop GPU)',True),
    ('llvmpipe (LLVM 20.1.2, 256 bits)',False)])
def test_render_log_requires_this_run_live_writer_not_environment(tmp_path,renderer,confirmed):
    import os
    import subprocess
    import time
    source=tmp_path/'rendering/ogre2.log';source.parent.mkdir()
    source.write_text('old renderer\n')
    capture=runner.RendererLogCapture(source,tmp_path/'output',{'GALLIUM_DRIVER':'d3d12'})
    content='*** Starting GLX Subsystem ***\nGL_VENDOR = Microsoft Corporation\nGL_RENDERER = '+renderer+'\n'
    child=subprocess.Popen([sys.executable,'-c',
        'import pathlib,sys,time; f=pathlib.Path(sys.argv[1]).open("w"); f.write(sys.argv[2]); f.flush(); time.sleep(10)',
        str(source),content],start_new_session=True)
    try:
        deadline=time.monotonic()+3
        while source.read_text()!=content and time.monotonic()<deadline:time.sleep(.01)
        result=capture.finish(child.pid)
        assert result['log_provenance_confirmed']
        assert result['hardware_acceleration_confirmed']==confirmed
        assert result['renderer']==renderer and result['active_context_api']=='GLX'
        assert Path(result['log_copy']).read_text()==content
        assert any(row['pid']==child.pid for row in result['log_writers'])
    finally:child.terminate();child.wait(timeout=3)


def test_stale_render_log_cannot_confirm_acceleration_from_requested_adapter(tmp_path):
    source=tmp_path/'ogre2.log';source.write_text('GL_RENDERER = NVIDIA GeForce\n')
    capture=runner.RendererLogCapture(source,tmp_path/'output',{'GALLIUM_DRIVER':'d3d12',
        'MESA_D3D12_DEFAULT_ADAPTER_NAME':'NVIDIA'})
    result=capture.finish(None)
    assert not result['log_provenance_confirmed']
    assert result['status']=='UNCONFIRMED' and not result['hardware_acceleration_confirmed']


def test_foreign_render_log_writer_cannot_be_attributed_to_this_run(tmp_path):
    import os
    import subprocess
    import time
    source=tmp_path/'ogre2.log'
    capture=runner.RendererLogCapture(source,tmp_path/'output',{})
    child=subprocess.Popen([sys.executable,'-c',
        'import pathlib,sys,time; f=pathlib.Path(sys.argv[1]).open("w"); f.write("GL_RENDERER = NVIDIA GeForce\\n"); f.flush(); time.sleep(10)',
        str(source)],start_new_session=True)
    try:
        deadline=time.monotonic()+3
        while not source.exists() and time.monotonic()<deadline:time.sleep(.01)
        result=capture.finish(os.getpgrp())  # a different process group
        assert result['status']=='UNCONFIRMED' and not result['log_provenance_confirmed']
        assert not result['hardware_acceleration_confirmed']
    finally:child.terminate();child.wait(timeout=3)


def test_powerplant_cli_defaults_to_limited_no_detection_and_forbids_fake_replay(monkeypatch):
    monkeypatch.setattr(sys,'argv',['runner','--scenario','powerplant'])
    args=runner.parse_args()
    assert args.detection_mode=='none' and args.stop_after=='scan' and args.render_context=='auto'
    monkeypatch.setattr(sys,'argv',['runner','--scenario','powerplant','--roi-access-test'])
    assert runner.parse_args().stop_after=='roi-access'
    monkeypatch.setattr(sys,'argv',['runner','--scenario','powerplant','--detection-mode','replay'])
    with pytest.raises(SystemExit):runner.parse_args()


def test_takeoff_aabb_audit_only_zero_candidates_is_a_free_space_proof():
    far=[[[3,0,1],[3,1,1],[3,0,2]]]
    clear=auditor.triangle_aabb_audit(far,[-2,-2,.04],[2,2,6.3])
    assert clear['passed'] and clear['aabb_intersection_candidates']==0
    # This triangle is outside the prism, but its AABB overlaps it. The broad
    # phase may conservatively refuse, never invent a narrow-phase proof.
    conservative=[[[-10,0,1],[0,10,1],[-10,10,1]]]
    unproven=auditor.triangle_aabb_audit(conservative,[-2,-2,.04],[2,2,6.3])
    assert not unproven['passed'] and unproven['aabb_intersection_candidates']==1
    assert unproven['nonzero_candidates_interpreted_as'].startswith('unproven')
    touching=[[[2,0,1],[2,1,1],[2,0,2]]]
    assert not auditor.triangle_aabb_audit(touching,[-2,-2,.04],[2,2,6.3])['passed']


def tiny_takeoff_scene(tmp_path):
    assets,world,_=locked_scene(tmp_path)
    folder=assets/'models/device'
    def write_stl(path,x):
        vertices=(x,-1,-1,x,1,-1,x,0,1)
        path.write_bytes(b'audited triangle'.ljust(80,b' ')+struct.pack('<I',1)
            +struct.pack('<12fH',1,0,0,*vertices,0))
    write_stl(folder/'surface.stl',0)
    write_stl(folder/'collision.stl',0)
    model=folder/'model.sdf'
    model.write_text(model.read_text().replace('</link>',
        '<collision name="c"><geometry><mesh><uri>model://device/collision.stl</uri></mesh></geometry></collision></link>'))
    extras=''.join(f'<model name="indoor_enclosure_{name}"><pose>100 0 10 0 0 0</pose><link name="l"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model>'
        for name in ('west','east','north','south','ceiling'))
    extras+='<model name="ground"><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>140 140</size></plane></geometry></collision></link></model>'
    world.write_text(world.read_text().replace('</world>',extras+'</world>'))
    parameters={'takeoff_region_radius_m':1.5,'takeoff_region_height_m':6.2,'takeoff_height_m':5,
        'motion_body_radius_m':.5,'motion_body_z_min_m':-.06,'motion_body_z_max_m':.5,
        'initial_scan_max_tilt_rad':.35,'explore_clearance_radius_m':.35}
    return assets,world,parameters,write_stl


def test_actual_takeoff_audit_checks_full_height_and_separate_ground_contact(tmp_path):
    assets,world,parameters,_=tiny_takeoff_scene(tmp_path)
    result=auditor.audit_takeoff_domain(world,assets/'models',parameters)
    assert result['passed'] and result['operating_column']['z_max_m']==pytest.approx(6.3)
    assert len(result['meshes'])==2 and all(x['aabb_intersection_candidates']==0 for x in result['meshes'])
    assert not result['ground_contact_band']['claimed_obstacle_free']
    assert not result['navigation_error_envelope_proven'] and not result['actual_startup_response_proven']
    changed=auditor.audit_takeoff_domain(world,assets/'models',{**parameters,'takeoff_region_height_m':6.3})
    assert not changed['passed'] and not changed['checks']['operating_column_contained']


def test_takeoff_audit_refuses_collision_candidate_and_binds_actual_config(tmp_path):
    assets,world,parameters,write_stl=tiny_takeoff_scene(tmp_path)
    result=auditor.audit_takeoff_domain(world,assets/'models',parameters)
    config=tmp_path/'actual.yaml';config.write_text('takeoff_region_confirmed: true\n')
    auditor.bind_actual_configuration(result,config,parameters)
    assert result['actual_config_sha256']==hashlib.sha256(config.read_bytes()).hexdigest()
    with pytest.raises(RuntimeError,match='changed'):
        auditor.bind_actual_configuration(result,config,{**parameters,'takeoff_region_height_m':7})
    # Included model has x=4: local x=-3 places the actual collider at x=1.
    write_stl(assets/'models/device/collision.stl',-3)
    obstructed=auditor.audit_takeoff_domain(world,assets/'models',parameters)
    assert not obstructed['passed']
    assert next(x for x in obstructed['meshes'] if x['role']=='collision')['aabb_intersection_candidates']==1
    with pytest.raises(RuntimeError,match='invalid'):
        auditor.bind_actual_configuration(obstructed,config,parameters)


def test_collision_comparison_verifies_actual_nested_mesh_selection(tmp_path):
    import numpy as np
    s=SCRIPT.with_name('validate_powerplant_physics.py')
    spec=importlib.util.spec_from_file_location('collision_physics',s)
    physics=importlib.util.module_from_spec(spec);spec.loader.exec_module(physics)
    root=tmp_path/'assets';original=root/'models/power-plant';original.mkdir(parents=True)
    (original/'model.config').write_text('<model><name>power-plant</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
    def write_tetra(path,scale):
        points=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])*scale
        data=bytearray(b'valid resource binding fixture'.ljust(80,b' ')+struct.pack('<I',4))
        for ids in ((0,2,1),(0,1,3),(1,2,3),(2,0,3)):
            triangle=points[list(ids)];normal=np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0])
            normal/=np.linalg.norm(normal)
            data.extend(struct.pack('<12fH',*normal,*triangle.ravel(),0))
        path.write_bytes(data)
    simplified=original/'simplified.stl';write_tetra(simplified,1.)
    reference=tmp_path/'complete.stl';write_tetra(reference,2.)
    def model(uri):
        return '<sdf version="1.9"><model name="plant"><static>true</static><link name="link"><collision name="collision"><geometry><mesh><uri>'+uri+'</uri></mesh></geometry></collision></link></model></sdf>'
    (original/'model.sdf').write_text(model(simplified.as_uri()))
    overlay=tmp_path/'overlay';target=overlay/'power-plant';target.mkdir(parents=True)
    (target/'model.config').write_text((original/'model.config').read_text())
    (target/'model.sdf').write_text(model(reference.as_uri()))
    world=root/'worlds/powerplant.sdf';world.parent.mkdir();world.write_text('<sdf version="1.9"><world name="powerplant"><include><uri>model://power-plant</uri><name>power_plant_env</name></include></world></sdf>')
    out=tmp_path/'result';out.mkdir()
    env=physics.resource_environment(root,overlay,'test-resource-binding')
    evidence=physics.verify_resolved_collision(root,overlay,env,out)
    assert evidence['verified'] and evidence['sha256']==hashlib.sha256(reference.read_bytes()).hexdigest()
    assert evidence['mesh_path']==str(reference)
    env['SDF_PATH']=str(root/'models')  # exact original failure at the call site
    with pytest.raises(RuntimeError,match='does not match requested overlay'):
        physics.verify_resolved_collision(root,overlay,env,out)


@pytest.mark.parametrize('binding',['valid','missing','same_resource'])
def test_performance_comparison_requires_two_actual_bound_resources(tmp_path,binding):
    p=SCRIPT.with_name('summarize_powerplant_benchmarks.py');s=importlib.util.spec_from_file_location('benchmark_compare',p)
    compare=importlib.util.module_from_spec(s);s.loader.exec_module(compare)
    for variant in ('complete','simplified'):
        for repeat in (1,2,3):
            digest=variant if binding!='same_resource' else 'same-loaded-resource'
            row={'native_sensor_samples_present':True,'profile':'native','warmup_wall_s':30,'sample_wall_s':60,
                 'world_sha256':'world','visual_mesh_sha256':'visual','fixtures_sha256':'fixtures',
                 'render_context':'display','collision_resource_sha256':digest,
                 'resolved_collision_resource':{'verified':True,'sha256':digest},
                 'renderer_identification':['GL_RENDERER = explicit test fixture'],
                 'sensors':{v:{'wall_interval_p50_p95_p99_s':[.1,.2,.3]} for v in ('rgb','depth','lidar')}}
            row.update({v:1. for v in ('cpu_core_percent','rss_median_bytes','rss_peak_bytes','loading_wall_s','measured_real_time_factor','measured_iteration_rate_hz')})
            if binding=='missing':row.pop('resolved_collision_resource')
            folder=tmp_path/f'bench_native_{repeat}_{variant}';folder.mkdir();(folder/'report.json').write_text(json.dumps(row))
    result=compare.compare(tmp_path)
    assert result['passed']==(binding=='valid')
    if binding=='missing':assert not result['checks']['resolved_collision_resource_every_trial']
    if binding=='same_resource':assert not result['checks']['different_actual_comparison_resources']


def test_surface_separation_does_not_prove_seed_outside_a_closed_solid():
    import numpy as np
    vertices=np.array([[x,y,z] for x in (-10.,10.) for y in (-10.,10.) for z in (-10.,10.)])
    indices=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
                      [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    triangles=vertices[indices]
    assert auditor.triangle_aabb_audit(triangles,[-1,-1,-1],[1,1,1])['passed']
    assert not auditor.triangle_component_seed_audit(triangles,[0,0,0])['passed']
    assert auditor.triangle_component_seed_audit(triangles,[-11,0,0])['passed']
