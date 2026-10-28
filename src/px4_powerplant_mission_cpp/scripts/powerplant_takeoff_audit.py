#!/usr/bin/env python3
"""Read-only, sufficient geometric proof for the real scene's takeoff prism.

A triangle AABB touching the prism is a candidate, not proof of collision.
Zero candidates prove surface separation. A separate external seed check
prevents mistaking a prism entirely inside a closed solid for free space.
"""
from pathlib import Path
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET

PARAMETERS = ("takeoff_region_radius_m", "takeoff_region_height_m", "takeoff_height_m",
              "motion_body_radius_m", "motion_body_z_min_m", "motion_body_z_max_m",
              "initial_scan_max_tilt_rad", "explore_clearance_radius_m")


def parameter_fingerprint(parameters):
    values = {key:float(parameters[key]) for key in PARAMETERS}
    if not all(math.isfinite(value) for value in values.values()):
        raise RuntimeError("nonfinite takeoff audit parameter")
    return values,hashlib.sha256(json.dumps(values,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()


def triangle_aabb_audit(triangles, minimum, maximum):
    import numpy as np
    triangles=np.asarray(triangles,dtype=float)
    if triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or not np.isfinite(triangles).all():
        raise RuntimeError("invalid takeoff audit triangles")
    minimum,maximum=np.asarray(minimum,dtype=float),np.asarray(maximum,dtype=float)
    if minimum.shape!=(3,) or maximum.shape!=(3,) or not np.isfinite([minimum,maximum]).all() or np.any(minimum>=maximum):
        raise RuntimeError("invalid takeoff evidence prism")
    candidates=((triangles.max(axis=1)>=minimum).all(axis=1)
                & (triangles.min(axis=1)<=maximum).all(axis=1))
    count=int(np.count_nonzero(candidates))
    return {"passed":count==0,"triangle_count":len(triangles),"aabb_intersection_candidates":count,
            "method":"closed triangle AABB separation; external seed also required to rule out solid containment",
            "nonzero_candidates_interpreted_as":"unproven, not an asserted physical collision"}


def triangle_component_seed_audit(triangles, seed):
    """A seed outside every connected component AABB cannot be inside a solid.

    Together with surface separation, one free seed proves a connected prism
    exterior to the mesh. An AABB containing the seed is merely UNCONFIRMED;
    this conservative check never invents an inside/outside parity answer.
    """
    import numpy as np
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    triangles,seed=np.asarray(triangles,float),np.asarray(seed,float)
    if (triangles.ndim!=3 or triangles.shape[1:]!=(3,3) or seed.shape!=(3,)
            or not np.isfinite(triangles).all() or not np.isfinite(seed).all()):
        raise RuntimeError("invalid component/seed evidence")
    cross=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    triangles=triangles[np.any(cross!=0,axis=1)]
    if not len(triangles):raise RuntimeError("empty nondegenerate mesh in seed audit")
    vertices,inverse=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    faces=inverse.reshape(-1,3)
    a=np.concatenate((faces[:,0],faces[:,1]));b=np.concatenate((faces[:,1],faces[:,2]))
    graph=coo_matrix((np.ones(len(a),np.uint8),(a,b)),shape=(len(vertices),len(vertices)))
    count,labels=connected_components(graph,directed=False)
    low=np.full((count,3),np.inf);high=np.full((count,3),-np.inf)
    np.minimum.at(low,labels,vertices);np.maximum.at(high,labels,vertices)
    candidates=np.flatnonzero(np.all(low<=seed,axis=1)&np.all(seed<=high,axis=1))
    return {"passed":len(candidates)==0,"seed_enu_m":seed.tolist(),
        "connected_components":int(count),"component_aabbs_containing_seed":candidates.tolist(),
        "method":"exact-coordinate connectivity; seed strictly exterior to every component AABB"}


def audit_takeoff_domain(world, models, parameters, home=(0.0,0.0,0.1)):
    import numpy as np
    world,models=Path(world).resolve(),Path(models).resolve()
    values,parameters_hash=parameter_fingerprint(parameters)
    minimum=np.array([-2.,-2.,.04]);maximum=np.array([2.,2.,6.3])
    home=np.asarray(home,dtype=float)
    if home.shape!=(3,) or not np.isfinite(home).all(): raise RuntimeError("invalid takeoff home")
    radius,height=values["takeoff_region_radius_m"],values["takeoff_region_height_m"]
    checks={"positive_operating_column":radius>0 and height>0 and values["takeoff_height_m"]>0,
        "nominal_body_parameters_valid":values["motion_body_radius_m"]>0
            and values["motion_body_z_min_m"]<values["motion_body_z_max_m"]
            and values["initial_scan_max_tilt_rad"]>=0 and values["explore_clearance_radius_m"]>=0,
        "operating_column_contained":bool(np.all(home[:2]-radius>=minimum[:2]-1e-10)
            and np.all(home[:2]+radius<=maximum[:2]+1e-10)
            and home[2]>=minimum[2]-1e-10 and home[2]+height<=maximum[2]+1e-10),
        "nominal_body_xy_sweep_contained":bool(np.all(home[:2]-radius-values["motion_body_radius_m"]>=minimum[:2]-1e-10)
            and np.all(home[:2]+radius+values["motion_body_radius_m"]<=maximum[:2]+1e-10)),
        "nominal_takeoff_body_z_sweep_contained":bool(home[2]+values["motion_body_z_min_m"]>=minimum[2]-1e-10
            and home[2]+values["takeoff_height_m"]+values["motion_body_z_max_m"]<=maximum[2]+1e-10)}
    sources={};meshes=[];boxes=[];ground=[]
    def tracked(path):
        path=Path(path).resolve();digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if path in sources and sources[path]!=digest: raise RuntimeError("scene asset changed during takeoff audit")
        sources[path]=digest;return path
    def pose(element):
        if element is None:return np.eye(4)
        if element.get("relative_to"):raise RuntimeError("takeoff audit requires resolved relative poses")
        v=[float(x) for x in (element.text or "").split()]
        if len(v)!=6 or not all(math.isfinite(x) for x in v):raise RuntimeError("invalid scene pose")
        x,y,z,r,p,w=v;cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(w),math.sin(w)
        out=np.eye(4);out[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
            [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]];out[:3,3]=[x,y,z]
        return out
    def mesh(path,transform,role,name):
        path=tracked(path);raw=path.read_bytes()
        if len(raw)<84:raise RuntimeError("invalid takeoff audit STL")
        count=struct.unpack_from("<I",raw,80)[0]
        if not count or len(raw)!=84+50*count:raise RuntimeError("takeoff audit requires a nonempty binary STL")
        dtype=np.dtype([("normal","<f4",(3,)),("vertices","<f4",(3,3)),("attribute","<u2")])
        triangles=np.frombuffer(raw,dtype=dtype,offset=84)["vertices"].astype(float)
        triangles=triangles@transform[:3,:3].T+transform[:3,3]
        report=triangle_aabb_audit(triangles,minimum,maximum)
        seed=triangle_component_seed_audit(triangles,[0.,0.,.25])
        report['free_seed']=seed
        report['passed']=report['passed'] and seed['passed']
        meshes.append({"path":str(path),"sha256":sources[path],"role":role,"model":name,**report})
    def geometry(element,transform,role,name,folder):
        if element.find("mesh") is not None:
            uri=element.findtext("mesh/uri","")
            path=models/uri[len("model://"):] if uri.startswith("model://") else folder/uri.removeprefix("file://")
            scale=np.array([float(x) for x in element.findtext("mesh/scale","1 1 1").split()])
            if scale.shape!=(3,) or not np.isfinite(scale).all() or np.any(scale<=0):raise RuntimeError("invalid audited mesh scale")
            transformed=transform.copy();transformed[:3,:3]=transformed[:3,:3]@np.diag(scale)
            mesh(path,transformed,role,name)
        elif element.find("box") is not None:
            size=np.array([float(x) for x in element.findtext("box/size").split()])
            if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):raise RuntimeError("invalid audited box")
            corners=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*size
            corners=corners@transform[:3,:3].T+transform[:3,3]
            overlap=bool(np.all(corners.max(0)>=minimum) and np.all(corners.min(0)<=maximum))
            boxes.append({"model":name,"role":role,"passed":not overlap,
                "world_minimum":corners.min(0).tolist(),"world_maximum":corners.max(0).tolist()})
        elif element.find("plane") is not None and role=="collision":
            normal=np.array([float(x) for x in element.findtext("plane/normal","0 0 1").split()])
            normal=transform[:3,:3]@normal
            is_ground=bool(np.allclose(normal,[0,0,1],atol=1e-10) and abs(transform[2,3])<=1e-10)
            if not is_ground:raise RuntimeError("unaudited collision plane; only world ground z=0 is a contact band")
            ground.append({"model":name,"z_m":0.0,"separate_contact_surface":True,"claimed_obstacle_free":False})
        elif role=="collision":
            raise RuntimeError("unsupported collision geometry in takeoff audit")
    def contents(element,parent,folder,ancestry):
        for included in element.findall("include"):
            uri=included.findtext("uri","")
            if not uri.startswith("model://"):raise RuntimeError("unsupported takeoff include URI")
            directory=models/uri[len("model://"):];config=tracked(directory/"model.config")
            source=tracked(directory/ET.parse(config).getroot().findtext("sdf"))
            if source in ancestry:raise RuntimeError("cyclic takeoff audit include")
            model=ET.parse(source).getroot().find("model")
            inspect_model(model,parent@pose(included.find("pose")),directory,ancestry+(source,),
                included.findtext("name",model.get("name","")),included.find("pose") is not None)
        for model in element.findall("model"):
            inspect_model(model,parent,folder,ancestry,model.get("name",""),False)
    def inspect_model(model,parent,folder,ancestry,name,overridden):
        transform=parent if overridden else parent@pose(model.find("pose"))
        for link in model.findall("link"):
            body=transform@pose(link.find("pose"))
            for role in ("visual","collision"):
                for shape in link.findall(role):
                    value=shape.find("geometry")
                    if value is not None:geometry(value,body@pose(shape.find("pose")),role,name,folder)
        contents(model,transform,folder,ancestry)
    root=ET.parse(tracked(world)).getroot().find("world")
    if root is None:raise RuntimeError("takeoff world is missing")
    contents(root,np.eye(4),world.parent,(world,))
    checks["source_visual_stl_checked"]=any(x["role"]=="visual" for x in meshes)
    checks["actual_collision_stl_checked"]=any(x["role"]=="collision" for x in meshes)
    checks["all_mesh_triangle_aabbs_separated"]=bool(meshes) and all(x["aabb_intersection_candidates"]==0 for x in meshes)
    checks["seed_outside_all_mesh_components"]=bool(meshes) and all(x["free_seed"]["passed"] for x in meshes)
    enclosure={x["model"] for x in boxes if x["role"]=="collision" and x["model"].startswith("indoor_enclosure_")}
    checks["all_five_enclosure_collision_boxes_checked"]=enclosure=={"indoor_enclosure_"+x for x in ("west","east","north","south","ceiling")}
    checks["all_boxes_separated"]=all(x["passed"] for x in boxes)
    checks["ground_contact_surface_recorded"]=bool(ground)
    for path,digest in sources.items():
        if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:raise RuntimeError("scene asset changed during takeoff audit")
    return {"passed":all(checks.values()),"checks":checks,"world":str(world),
        "world_sha256":sources[world],"sources":[{"path":str(p),"sha256":d} for p,d in sorted(sources.items())],
        "evidence_prism":{"minimum_enu_m":minimum.tolist(),"maximum_enu_m":maximum.tolist()},
        "operating_column":{"center_enu_m":home.tolist(),"radius_m":radius,"z_min_m":float(home[2]),"z_max_m":float(home[2]+height)},
        "nominal_body_parameters":values,"parameters_sha256":parameters_hash,"meshes":meshes,"boxes":boxes,
        "ground_contact_band":{"minimum_z_m":float(home[2]-values["explore_clearance_radius_m"]),
            "maximum_z_m":.04,"ground_surfaces":ground,"claimed_obstacle_free":False},
        "navigation_error_envelope_proven":False,"actual_startup_response_proven":False,
        "scope":"static nominal geometry and configured operating column only; motion uncertainty and actual response remain runtime requirements"}


def bind_actual_configuration(audit, config_path, parameters):
    _,digest=parameter_fingerprint(parameters)
    if not audit.get("passed") or digest!=audit.get("parameters_sha256"):
        raise RuntimeError("takeoff geometry/configuration audit is invalid or changed")
    audit["actual_config"]=str(Path(config_path).resolve())
    audit["actual_config_sha256"]=hashlib.sha256(Path(config_path).read_bytes()).hexdigest()
    return audit
