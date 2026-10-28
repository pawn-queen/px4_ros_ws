#!/usr/bin/env python3
"""Measure the actual powerplant world: dynamic contacts and collision cost.

Probe coordinates are evaluation fixtures only. No map, waypoint or target is
published to the mission stack. An isolated resource overlay changes only the
collision URI for the complete-mesh comparison; the world and visuals are real.
"""
from __future__ import annotations
import argparse,atexit,copy,hashlib,json,math,os,signal,statistics,subprocess,sys,time,traceback
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def stamp(s):return s.sec+s.nsec*1e-9

def process_usage(root):
    pending=[root];seen=set();ticks=0;rss=0
    while pending:
        pid=pending.pop()
        if pid in seen:continue
        seen.add(pid)
        try:
            text=Path(f'/proc/{pid}/stat').read_text();fields=text[text.rfind(')')+2:].split()
            ticks+=int(fields[11])+int(fields[12]);rss+=int(fields[21])*os.sysconf('SC_PAGE_SIZE')
            pending.extend(map(int,Path(f'/proc/{pid}/task/{pid}/children').read_text().split()))
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return ticks/os.sysconf('SC_CLK_TCK'),rss,sorted(seen)


def resources(args,out,variant):
    root=args.px4_root/'Tools/simulation/gz';model=root/'models/power-plant'
    overlay=out/'models';folder=overlay/'power-plant';folder.mkdir(parents=True)
    # Symlink all immutable assets; edit only this run's model description.
    for p in model.iterdir():
        if p.name!='model.sdf':(folder/p.name).symlink_to(p,target_is_directory=p.is_dir())
    tree=ET.parse(model/'model.sdf')
    uri=tree.getroot().find('model/link/collision/geometry/mesh/uri')
    if uri is None:raise ValueError('Expected the repaired independent collision mesh')
    if variant=='complete':
        # Only exact duplicates, zero-area faces and unreferenced vertices are
        # removed. This is the full-surface reference, never a decimated mesh.
        reference=out.parent/'complete_reference.stl'
        if not reference.exists():
            sys.path.insert(0,str(model/'scripts'))
            from build_collision import read_clean,write_binary
            vertices,faces,clean=read_clean(model/'meshes/power-plant.stl')
            write_binary(reference,vertices,faces)
            (out.parent/'complete_reference_cleaning.json').write_text(json.dumps(clean,indent=2)+'\n')
        uri.text=reference.resolve().as_uri()
    tree.write(folder/'model.sdf',encoding='utf-8',xml_declaration=True)
    return root,overlay


def resource_environment(root,overlay,partition):
    # SDF_PATH resolves nested model:// includes before Gazebo's resource path.
    # Both must select the same per-run collision overlay.
    return {**os.environ,'GZ_PARTITION':partition,'GZ_IP':'127.0.0.1',
        'SDF_PATH':os.pathsep.join(map(str,[overlay,root/'models'])),
        'GZ_SIM_RESOURCE_PATH':os.pathsep.join(map(str,[overlay,root/'models',root/'worlds']))}


def verify_resolved_collision(root,overlay,env,out):
    world=root/'worlds/powerplant.sdf'
    expansion=subprocess.run(['gz','sdf','-p',str(world)],env=env,text=True,
        capture_output=True,check=True,timeout=30)
    (out/'expanded_world.sdf').write_text(expansion.stdout)
    (out/'sdf_expansion.log').write_text(expansion.stderr)
    uris=[x.text for x in ET.fromstring(expansion.stdout).findall(
        './/model[@name="power_plant_env"]/link/collision/geometry/mesh/uri')]
    requested=ET.parse(overlay/'power-plant/model.sdf').getroot().findtext(
        'model/link/collision/geometry/mesh/uri')
    if uris!=[requested]:
        raise RuntimeError('resolved world collision does not match requested overlay: '+str(uris))
    if requested.startswith('file://'):
        from urllib.parse import unquote
        path=Path(unquote(requested[7:])).resolve()
    elif requested.startswith('model://'):
        relative=requested[8:]
        path=next((base/relative for base in (overlay,root/'models') if (base/relative).is_file()),None)
        if path is None:raise RuntimeError('resolved collision resource is unavailable')
        path=path.resolve()
    else:raise RuntimeError('unsupported resolved collision resource URI')
    return {'verified':True,'uri':requested,'mesh_path':str(path),'sha256':sha(path),
        'expanded_world_sha256':sha(out/'expanded_world.sdf'),
        'scope':'actual expanded world using exactly the Gazebo server environment'}


class Scene:
    def __init__(self,args,out,variant):
        self.args,self.out,self.variant=args,out,variant;out.mkdir(parents=True)
        root,overlay=resources(args,out,variant)
        self.root=root;self.env=resource_environment(root,overlay,f'issue3-{os.getpid()}-{time.time_ns()}')
        self.resolved_collision=verify_resolved_collision(root,overlay,self.env,out)
        os.environ['GZ_PARTITION']=self.env['GZ_PARTITION'];os.environ['GZ_IP']='127.0.0.1'
        from gz.transport13 import Node
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.contacts_pb2 import Contacts
        self.node=Node();self.stats=[];self.poses={};self.contacts=[];self.sensor_rows={};self.sensor_first={};self.sensor_last={};self.capture_enabled=False
        self.node.subscribe(WorldStatistics,'/world/powerplant/stats',lambda m:self.stats.append((time.monotonic(),m.sim_time.sec+m.sim_time.nsec*1e-9,int(m.iterations),bool(m.paused),float(m.real_time_factor))))
        self.node.subscribe(Pose_V,'/world/powerplant/dynamic_pose/info',self.on_pose)
        self.node.subscribe(Contacts,'/issue3/contact',self.on_contact)
        self.node.subscribe(Contacts,'/world/powerplant/model/issue3_probe/link/body/sensor/probe_contact/contact',self.on_contact)
        self.log=(out/'gazebo.log').open('w')
        self.render_context=args.render_context
        if self.render_context=='auto':self.render_context='display' if self.env.get('DISPLAY') else 'egl'
        if self.render_context=='display' and not self.env.get('DISPLAY'):raise RuntimeError('Display rendering requires DISPLAY')
        argv=['gz','sim','-s','-v','3','--seed','301']
        if self.render_context=='egl':argv.append('--headless-rendering')
        argv.append(str(root/'worlds/powerplant.sdf'))
        self.process=subprocess.Popen(argv,env=self.env,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        atexit.register(self.close)
        deadline=time.monotonic()+40
        while time.monotonic()<deadline and not self.stats:
            if self.process.poll() is not None:raise RuntimeError('Gazebo exited during startup; see '+str(out/'gazebo.log'))
            time.sleep(.1)
        if not self.stats:raise RuntimeError('No statistics from isolated real world')
        self.loading_wall_s=time.monotonic()-deadline+40
        self.control(pause=True)
        self.create_probe()
        self.loaded_libraries=[]
        for pid in process_usage(self.process.pid)[2]:
            try:
                for line in Path(f'/proc/{pid}/maps').read_text().splitlines():
                    if any(x in line for x in ('dartsim','libdart','libode','libfcl')):
                        path=line.split()[-1]
                        if path not in self.loaded_libraries:self.loaded_libraries.append(path)
            except FileNotFoundError:pass

    def on_pose(self,msg):
        now=time.monotonic()
        for p in msg.pose:
            if p.name=='issue3_probe':self.poses[p.name]=(now,[p.position.x,p.position.y,p.position.z])

    def on_contact(self,msg):
        for c in msg.contact:
            self.contacts.append({'wall':time.monotonic(),'sim_stamp_s':stamp(msg.header.stamp),
                'collision1':c.collision1.name,'collision2':c.collision2.name,
                'positions':[[p.x,p.y,p.z] for p in c.position],'depths':list(c.depth)})

    def request(self,name,req,reply):
        # Separate request clients avoid transport13 Python synchronous RPC
        # starving the monitor callbacks. Publish/subscribe remains native.
        command=['gz','service','-s','/world/powerplant/'+name,
            '--reqtype',req.DESCRIPTOR.full_name,'--reptype',reply.DESCRIPTOR.full_name,
            '--timeout','5000','--req',str(req)]
        for attempt in range(3):
            result=subprocess.run(command,env=self.env,text=True,capture_output=True,timeout=8)
            if result.returncode==0 and 'data: true' in result.stdout:return reply(data=True)
            with (self.out/'service_retries.jsonl').open('a') as stream:
                stream.write(json.dumps({'wall':time.monotonic(),'service':name,'request':str(req),'attempt':attempt+1,'response':result.stdout+result.stderr,'last_stats':self.stats[-1] if self.stats else None})+'\n')
            time.sleep(.2)
        raise RuntimeError(f'Gazebo request failed: {name}; response={result.stdout+result.stderr}, process={self.process.poll()}, last_stats={self.stats[-1] if self.stats else None}')

    def control(self,pause=None,steps=0):
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.msgs10.boolean_pb2 import Boolean
        m=WorldControl();m.pause=True if pause is None else pause
        if steps:m.multi_step=steps
        self.request('control',m,Boolean)

    def steps(self,n):
        before=self.stats[-1][2];self.control(pause=True,steps=n);deadline=time.monotonic()+max(45,n*.02)
        while self.stats[-1][2]<before+n:
            if time.monotonic()>deadline:raise RuntimeError('World stepping did not complete')
            time.sleep(.005)

    def create(self,sdf):
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.boolean_pb2 import Boolean
        req=EntityFactory();req.sdf=sdf;self.request('create',req,Boolean)
        self.steps(2)

    def create_probe(self):
        from gz.msgs10.twist_pb2 import Twist
        self.twist_type=Twist;self.velocity=self.node.advertise('/issue3/cmd_vel',Twist)
        self.create('''<sdf version="1.9"><model name="issue3_probe"><pose>0 0 10 0 0 0</pose><link name="body"><gravity>false</gravity><inertial><mass>0.1</mass><inertia><ixx>.0001</ixx><iyy>.0001</iyy><izz>.0001</izz></inertia></inertial><collision name="probe_collision"><geometry><sphere><radius>.05</radius></sphere></geometry></collision><visual name="probe_visual"><geometry><sphere><radius>.05</radius></sphere></geometry></visual><sensor name="probe_contact" type="contact"><always_on>true</always_on><update_rate>250</update_rate><topic>/issue3/contact</topic><contact><collision>probe_collision</collision></contact></sensor></link><plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"><topic>/issue3/cmd_vel</topic></plugin></model></sdf>''')

    def set_velocity(self,v):
        m=self.twist_type();m.linear.x,m.linear.y,m.linear.z=map(float,v);self.velocity.publish(m)

    def set_pose(self,p):
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
        self.set_velocity([0,0,0]);m=Pose();m.name='issue3_probe';m.position.x,m.position.y,m.position.z=map(float,p);m.orientation.w=1
        self.request('set_pose',m,Boolean);self.steps(4);self.contacts.clear()

    def create_sensors(self):
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.laserscan_pb2 import LaserScan
        from gz.msgs10.camera_info_pb2 import CameraInfo
        def camera_info(message):
            path=self.out/f'camera_info_{message.width}_{message.height}.pb'
            if self.capture_enabled and not path.exists():path.write_bytes(message.SerializeToString())
        self.node.subscribe(CameraInfo,'/issue3/camera_info',camera_info)
        rig=ET.Element('sdf',version='1.9');model=ET.SubElement(rig,'model',name='issue3_sensor_rig');ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=' '.join(map(str,self.args.sensor_pose));link=ET.SubElement(model,'link',name='rig')
        oak=ET.parse(self.root/'models/OakD-Lite/model.sdf');vehicle=ET.parse(self.root/'models/x500_depth/model.sdf')
        for name,source,pose,topic,kind in [('rgb',oak.find(".//sensor[@name='IMX214']"),'.13233 0 .26078 0 0 0','/issue3/rgb',Image),('depth',oak.find(".//sensor[@name='StereoOV7251']"),'.13233 0 .26078 0 0 0','/issue3/depth',Image),('lidar',vehicle.find(".//sensor[@name='mapping_lidar']"),'0 0 .42 0 0 0','/issue3/lidar',LaserScan)]:
            sensor=copy.deepcopy(source);assert sensor is not None
            pose_el=sensor.find('pose')
            if pose_el is None:pose_el=ET.SubElement(sensor,'pose')
            pose_el.text=pose
            topic_el=sensor.find('topic')
            if topic_el is None:topic_el=ET.SubElement(sensor,'topic')
            topic_el.text=topic;link.append(sensor);self.sensor_rows[name]=[]
            def callback(m,n=name):
                wall=time.monotonic();ts=stamp(m.header.stamp);self.sensor_rows[n].append((wall,ts))
                if self.capture_enabled:
                    if n not in self.sensor_first:self.sensor_first[n]=m.SerializeToString()
                    if len(self.sensor_rows[n])%100==0:self.sensor_last[n]=m.SerializeToString()
            self.node.subscribe(kind,topic,callback)
        sdf=ET.tostring(rig,encoding='unicode')
        (self.out/'sensor_rig.sdf').write_text(sdf)
        self.create(sdf)

    def snapshot_scene(self):
        from gz.msgs10.scene_pb2 import Scene as SceneMessage
        from google.protobuf import text_format
        command=['gz','service','-s','/world/powerplant/scene/info','--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene','--timeout','5000','--req','']
        for _ in range(3):
            response=subprocess.run(command,env=self.env,text=True,capture_output=True,timeout=8)
            if response.returncode==0 and response.stdout.strip():
                message=SceneMessage()
                try:text_format.Parse(response.stdout,message)
                except text_format.ParseError:continue
                if any(m.name=='issue3_sensor_rig' for m in message.model):
                    (self.out/'scene_snapshot.pb').write_bytes(message.SerializeToString())
                    (self.out/'scene_snapshot.pbtxt').write_text(response.stdout)
                    return
        raise RuntimeError('Actual sensor parent Scene snapshot unavailable')

    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(self.process.pid,signal.SIGTERM);self.process.wait(timeout=8)
        self.log.close()


def probe_suite(scene,fixtures,max_cases):
    report={'variant':scene.variant,'world':str(scene.root/'worlds/powerplant.sdf'),'surfaces':[],'free_paths':[],'positive_control':None,'passed':False}
    directions=[{'id':'outer_wall_positive','start_world_m':[-57.8,0,5],'expected_source_surface_world_m':[-58,0,5],'expected_collision_surface_world_m':[-58,0,5],'velocity_world_mps':[-.05,0,0],'side':1}]
    for case in fixtures['surface_cases'][:max_cases]:
        directions.extend([{**d,'id':case['id']} for d in case['directions']])
    for case in directions:
        start=np.array(case['start_world_m']);expected=np.array(case['expected_source_surface_world_m']);vel=np.array(case['velocity_world_mps']);speed=np.linalg.norm(vel)
        expected_time=case.get('source_sphere_contact_time_s',(np.linalg.norm(expected-start)-.05)/speed)
        scene.set_pose(start);scene.set_velocity(vel);start_stamp=scene.stats[-1][1];last=0
        scene.control(pause=False);deadline=time.monotonic()+max(15,(expected_time+1)*10)
        while scene.stats[-1][1]-start_stamp < expected_time+.8 and not scene.contacts:
            if time.monotonic()>deadline:raise RuntimeError(f'Contact probe simulation stopped advancing: {case["id"]}, last_stats={scene.stats[-1]}, pose={scene.poses.get("issue3_probe")}')
            scene.set_velocity(vel);time.sleep(.025);last+=1
        scene.set_velocity([0,0,0]);scene.control(pause=True);scene.steps(2)
        valid=[r for r in scene.contacts if 'probe' in r['collision1']+' '+r['collision2']]
        # Names may be ID-only in some builds: a populated contact with the
        # probe's dedicated contact sensor still identifies its source.
        if not valid:valid=list(scene.contacts)
        points=valid[0]['positions'] if valid else []
        error=min((float(np.linalg.norm(np.array(p)-expected)) for p in points),default=None)
        row={**case,'expected_contact_sim_time_s':expected_time,'contact_messages':len(valid),'contact_error_m':error,'final_pose':scene.poses.get('issue3_probe'),'contacts':valid[:8],'passed':bool(points and error<=.025)}
        if case['id']=='outer_wall_positive':report['positive_control']=row
        else:report['surfaces'].append(row)
        (scene.out/'partial_probes.json').write_text(json.dumps(report,indent=2)+'\n')
        print('probe',scene.variant,case['id'],case['side'],row['passed'],error,flush=True)
    for case in fixtures['free_paths']:
        start=np.array(case['start_world_m'],float);end=np.array(case['end_world_m'],float);length=float(np.linalg.norm(end-start));speed=.2;velocity=(end-start)/length*speed
        scene.set_pose(start);scene.set_velocity(velocity);base=scene.stats[-1][1]
        scene.control(pause=False);deadline=time.monotonic()+max(20,length/speed*10)
        while True:
            pose=scene.poses.get('issue3_probe')
            if pose and np.dot(np.array(pose[1])-start,(end-start)/length)>=length-.008:break
            if time.monotonic()>deadline:raise RuntimeError('Free-path probe did not reach its measured endpoint')
            scene.set_velocity(velocity);time.sleep(.015)
        scene.set_velocity([0,0,0]);scene.control(pause=True);scene.steps(2);pose=scene.poses.get('issue3_probe');err=float(np.linalg.norm(np.array(pose[1])-end)) if pose else None
        report['free_paths'].append({**case,'executed_speed_mps':speed,'sample_step_displacement_m':speed*.004,'contact_messages':len(scene.contacts),'final_pose':pose,'endpoint_error_m':err,'passed':bool(pose and not scene.contacts and err<.03)})
    report['passed']=bool(report['positive_control']['passed'] and all(r['passed'] for r in report['surfaces']+report['free_paths']))
    report['loaded_libraries']=scene.loaded_libraries
    return report


def benchmark(scene,warmup,duration,profile):
    if profile=='native':scene.create_sensors()
    scene.set_pose([-57.94,0,5]);scene.set_velocity([-.005,0,0]);scene.control(pause=False)
    time.sleep(warmup)
    if profile=='native':scene.snapshot_scene()
    # Newly created Gazebo sensors can publish a first frame before their
    # parent's world transform has propagated. Preserve only post-warmup
    # exposures for geometric validation; interval evidence remains complete.
    scene.sensor_first.clear();scene.sensor_last.clear();scene.capture_enabled=True
    begin=time.monotonic();cpu0,rss0,_=process_usage(scene.process.pid);rows=[]
    while time.monotonic()-begin<duration:
        cpu,rss,_=process_usage(scene.process.pid);rows.append((time.monotonic(),cpu,rss));time.sleep(.5)
    end=time.monotonic();cpu1,rss1,_=process_usage(scene.process.pid);scene.capture_enabled=False;scene.control(pause=True)
    stats=[r for r in scene.stats if begin<=r[0]<=end]
    sensors={}
    for name,values in scene.sensor_rows.items():
        values=[r for r in values if begin<=r[0]<=end];dt=np.diff([r[0] for r in values]);simdt=np.diff([r[1] for r in values])
        sensors[name]={'messages':len(values),'wall_interval_p50_p95_p99_s':np.quantile(dt,[.5,.95,.99]).tolist() if len(dt) else None,'sim_interval_p50_p95_p99_s':np.quantile(simdt,[.5,.95,.99]).tolist() if len(simdt) else None}
        if name in scene.sensor_first:(scene.out/(name+'_first.pb')).write_bytes(scene.sensor_first[name])
        if scene.sensor_last.get(name):(scene.out/(name+'_last.pb')).write_bytes(scene.sensor_last[name])
    real_factor=(stats[-1][1]-stats[0][1])/(stats[-1][0]-stats[0][0]) if len(stats)>1 else None
    result={'variant':scene.variant,'profile':profile,'warmup_wall_s':warmup,'sample_wall_s':end-begin,'cpu_core_percent':100*(cpu1-cpu0)/(end-begin),'rss_median_bytes':statistics.median([r[2] for r in rows]),'rss_peak_bytes':max(r[2] for r in rows),'loading_wall_s':scene.loading_wall_s,'measured_real_time_factor':real_factor,'stats_messages':len(stats),'sensors':sensors,'loaded_libraries':scene.loaded_libraries,'raw_stats_file':'stats.json','native_sensor_samples_present':profile!='native' or all(v['messages']>=2 for v in sensors.values()) and len(sensors)==3}
    result['measured_iteration_rate_hz']=(stats[-1][2]-stats[0][2])/(stats[-1][0]-stats[0][0]) if len(stats)>1 else None
    (scene.out/'stats.json').write_text(json.dumps(stats)+'\n');(scene.out/'process_usage.json').write_text(json.dumps(rows)+'\n')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=('probes','benchmark'));p.add_argument('--px4-root',type=Path,default=Path('/home/pawn/drone-simulation'));p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--variant',choices=('complete','simplified'),default='simplified');p.add_argument('--warmup',type=float,default=30);p.add_argument('--duration',type=float,default=60);p.add_argument('--profile',choices=('physics','native'),default='native');p.add_argument('--max-cases',type=int,default=12);p.add_argument('--render-context',choices=('auto','display','egl'),default='auto');p.add_argument('--sensor-pose',type=float,nargs=6,default=(0,0,5,0,0,.65))
    p.add_argument('--fixtures',type=Path,help='Evaluation-only runtime fixtures; asset hashes must still match the actual world')
    args=p.parse_args()
    if not all(math.isfinite(x) for x in args.sensor_pose):p.error('sensor pose must be finite')
    args.output_dir=args.output_dir.resolve();scene=None
    try:
        scene=Scene(args,args.output_dir,args.variant)
        fixture=args.fixtures or args.px4_root/'Tools/simulation/gz/models/power-plant/probe_fixtures.json'
        fixture_data=json.loads(fixture.read_text())
        model=scene.root/'models/power-plant'
        if fixture_data['source_sha256']!=sha(model/'meshes/power-plant.stl') or fixture_data['collision_sha256']!=sha(model/'meshes/power-plant-collision.stl'):
            raise RuntimeError('Probe fixtures do not match the actual visual/collision assets')
        if fixture_data.get('world_sha256',sha(scene.root/'worlds/powerplant.sdf')) != sha(scene.root/'worlds/powerplant.sdf'):
            raise RuntimeError('Probe fixtures do not match the actual world')
        if args.mode=='probes' and (args.max_cases<=0 or not fixture_data.get('evaluation_only') or not fixture_data.get('surface_cases') or not fixture_data.get('free_paths')):
            raise RuntimeError('Probe acceptance requires evaluation-only fixtures with surface and free-passage cases')
        if args.mode=='probes' and not fixture_data.get('fixture_qualification',{'passed':True}).get('passed'):
            raise RuntimeError('Supplemental fixture generation remains UNCONFIRMED; cannot declare physical acceptance')
        if args.mode=='probes':result=probe_suite(scene,fixture_data,args.max_cases)
        else:result=benchmark(scene,args.warmup,args.duration,args.profile)
        result['world_sha256']=sha(scene.root/'worlds/powerplant.sdf');result['model_sdf_sha256']=sha(scene.out/'models/power-plant/model.sdf');result['fixtures_sha256']=sha(fixture)
        if sha(scene.resolved_collision['mesh_path'])!=scene.resolved_collision['sha256']:
            raise RuntimeError('actual collision resource changed during this run')
        result['resolved_collision_resource']=scene.resolved_collision
        result['visual_mesh_sha256']=fixture_data['source_sha256']
        result['runtime_physics_libraries']=[{'path':path,'sha256':sha(path)}
            for path in scene.loaded_libraries]
        result['runtime_sensor_libraries']=[]
        result['runtime_renderer_libraries']=[]
        for pid in process_usage(scene.process.pid)[2]:
            try:
                for line in Path(f'/proc/{pid}/maps').read_text().splitlines():
                    if any(token in line for token in ('libgz-sensors','libgz-rendering','libOgreNextMain','RenderSystem_GL3Plus')):
                        library=Path(line.split()[-1]).resolve()
                        entry={'path':str(library),'sha256':sha(library)}
                        bucket='runtime_sensor_libraries' if 'libgz-sensors' in line else 'runtime_renderer_libraries'
                        if entry not in result[bucket]:result[bucket].append(entry)
            except FileNotFoundError:pass
        result['render_context']=scene.render_context
        renderer_log=Path.home()/'.gz/rendering/ogre2.log'
        if renderer_log.exists():
            text=renderer_log.read_text(errors='replace')
            result['renderer_identification']=[line for line in text.splitlines() if any(key in line for key in ('GL_RENDERER =','GL_VENDOR =','Device Name:','EGL Device:'))]
            (scene.out/'ogre2.log').write_text(text)
            from powerplant_lidar_rays import capture_renderer_resources
            result['runtime_renderer_resources']=capture_renderer_resources(text)
            result['renderer_resource_log_sha256']=sha(scene.out/'ogre2.log')
        result['collision_resource_kind']='cleaned_complete_surface_without_decimation' if args.variant=='complete' else 'validated_componentwise_simplification'
        result['collision_resource_sha256']=sha(scene.out.parent/'complete_reference.stl') if args.variant=='complete' else fixture_data['collision_sha256']
        (args.output_dir/'report.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ('surfaces','free_paths','positive_control')},indent=2),flush=True)
        if args.mode=='probes' and not result['passed']:raise SystemExit(1)
    except BaseException as error:
        args.output_dir.mkdir(parents=True,exist_ok=True)
        (args.output_dir/'failure.json').write_text(json.dumps({'passed':False,'error':str(error),'traceback':traceback.format_exc()},indent=2)+'\n')
        raise
    finally:
        if scene:scene.close()
if __name__=='__main__':main()
