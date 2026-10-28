#!/usr/bin/env python3
"""Offline Ogre2 lidar directions, explicitly bound to a captured renderer.

The nominal LaserScan direction remains the default everywhere. This optional
model predicts the fixed nearest-neighbour cubemap sampling of verified
8.2.3 builds. Neither directions nor eligibility depend on measured ranges.
"""
from __future__ import annotations
import hashlib
import math
from pathlib import Path
import re
import numpy as np


class RendererEvidenceUnavailable(ValueError):
    """The capture cannot establish a supported sampling implementation."""


class RendererEvidenceMismatch(ValueError):
    """Recorded runtime evidence changed or contradicts the supported build."""


# Builds present on this validation host and checked against the 8.2.3 source.
# Other builds require an explicit source/build audit, never a version guess.
SUPPORTED_LIBRARIES = {
    'libgz-rendering8.so.8.2.3': {
        '142f611e51fe6c67daa2af53ddca355fdbb825004264798955ae08177211d074',
        'a416c2f5abf4b735e79c40a75b7b420b4b8e966708497176cd5eac61fb8611be'},
    'libgz-rendering8-ogre2.so.8.2.3': {
        '3a54fcd4678dc6840041a8c8ad88d4119c7e415964e26733f506075b1796d400',
        '89f66860b2b7212f2e6c51d2b5dcc4c23cc790d47a4b615f621cb6c06cf82e20'},
    'libOgreNextMain.so.2.3.3': {
        'ad86552b7c83c02ad10d505bb98089e173bdc6b6ac38af74250139e99f7ff457'},
    'RenderSystem_GL3Plus.so.2.3.3': {
        'e8adac4943114c8a28e725d6798e7f71eb27aeb3f144833d52c322517380a88d'},
}
SUPPORTED_RESOURCES = {
    'gpu_rays.material': '700d644d61446919201c294a054cd735b5f4512696108fae534ba9f9b8453413',
    'GpuRays.compositor': 'd3e9077225de4febc2e00b9aa9e3cba27f82d77ff30b2ac7fcd9047c4cade93c',
    'gpu_rays_1st_pass_vs.glsl': '95dd4a6ece6b89949d3f0d91cb4c508d1b9c11ce0c5e1f1a706601a4c29dcdfc',
    'gpu_rays_1st_pass_fs.glsl': '15e833808a3c7c8559976f4728a84f0f7581ac00a80a5bbf92028494862f1515',
    'gpu_rays_2nd_pass_fs.glsl': '674e53c9d60431a1a2feef562582f9057d40ca74f71074a692ed897ee11dfc9e',
}
SOURCE_URL = 'https://github.com/gazebosim/gz-rendering/blob/gz-rendering8_8.2.3/ogre2/src/Ogre2GpuRays.cc'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def capture_renderer_resources(ogre_log):
    """Hash actual registered GLSL/material directories from this Ogre log."""
    directories = {Path(x).resolve() for x in re.findall(r"Added resource location '([^']+)'", ogre_log)}
    entries = []
    for name in SUPPORTED_RESOURCES:
        paths = {folder/name for folder in directories if (folder/name).is_file()}
        for path in sorted(paths):
            entries.append({'path': str(path), 'sha256': digest(path), 'resource_name': name,
                            'registered_resource_directory': str(path.parent)})
    return entries


def verify_renderer_evidence(report, ogre_log_path):
    """Require fresh capture metadata, supported binaries and live file hashes."""
    libraries = report.get('runtime_renderer_libraries', [])
    resources = report.get('runtime_renderer_resources', [])
    if not libraries or not resources:
        raise RendererEvidenceUnavailable('capture lacks runtime renderer/library and shader hashes')
    log = Path(ogre_log_path)
    if not log.is_file() or digest(log) != report.get('renderer_resource_log_sha256'):
        raise RendererEvidenceMismatch('captured renderer resource log is unavailable or changed')
    # This implementation models the GL3Plus shader only, never Metal/Vulkan.
    text = log.read_text(errors='replace')
    if 'GL3PlusRenderSystem' not in text or 'GL_RENDERER =' not in text or 'Version 2.3.3' not in text:
        raise RendererEvidenceUnavailable('capture does not identify the supported GL3Plus renderer')
    recorded_resources = capture_renderer_resources(text)
    by_name = {}
    for entry in resources:
        path = Path(entry.get('path', ''))
        name = path.name
        if name not in SUPPORTED_RESOURCES:
            raise RendererEvidenceMismatch('unexpected resource in cubemap evidence')
        if entry.get('sha256') != SUPPORTED_RESOURCES[name]:
            raise RendererEvidenceUnavailable(f'unsupported renderer shader/material build: {name}')
        if not path.is_file() or digest(path) != entry['sha256']:
            raise RendererEvidenceMismatch(f'captured renderer resource changed: {path}')
        if not any(x['path'] == str(path.resolve()) and x['sha256'] == entry['sha256'] for x in recorded_resources):
            raise RendererEvidenceMismatch('captured renderer resource is not registered by the runtime log')
        by_name.setdefault(name, []).append(entry)
    if set(by_name) != set(SUPPORTED_RESOURCES) or any(len(v) != 1 for v in by_name.values()):
        raise RendererEvidenceUnavailable('renderer resource resolution is missing or ambiguous')
    found = {}
    for entry in libraries:
        path = Path(entry.get('path', ''))
        name = path.name
        if name.startswith(('libgz-rendering8', 'libOgreNextMain', 'RenderSystem_GL3Plus')):
            if name not in SUPPORTED_LIBRARIES or entry.get('sha256') not in SUPPORTED_LIBRARIES[name]:
                raise RendererEvidenceUnavailable(f'unsupported renderer binary/version: {name}')
            if not path.is_file() or digest(path) != entry.get('sha256'):
                raise RendererEvidenceMismatch(f'captured renderer binary changed: {path}')
            found.setdefault(name, []).append(entry)
    if set(found) != set(SUPPORTED_LIBRARIES) or any(len(v) != 1 for v in found.values()):
        raise RendererEvidenceUnavailable('runtime renderer version is missing or ambiguous')
    return {'verified': True, 'version': 'gz-rendering 8.2.3', 'source': SOURCE_URL,
            'libraries': [v[0] for v in found.values()],
            'resources': [v[0] for v in by_name.values()],
            'resource_log_sha256': report['renderer_resource_log_sha256'],
            'direction_uses_measured_ranges': False}


def cubemap_uv(horizontal, vertical):
    # Preserve the source's (yaw * pitch) * +Z quaternion operation order,
    # including axis-angle normalization before the float32 UV table write.
    def axis_angle(axis, angle):
        q = (math.cos(angle/2), *(x*math.sin(angle/2) for x in axis))
        norm = math.sqrt(sum(x*x for x in q))
        return tuple(x/norm for x in q)
    aw, ax, ay, az = axis_angle((0., 1., 0.), -horizontal)
    bw, bx, by, bz = axis_angle((1., 0., 0.), -vertical)
    w, x, y, z = (aw*bw-ax*bx-ay*by-az*bz,
                  aw*bx+ax*bw+ay*bz-az*by,
                  aw*by-ax*bz+ay*bw+az*bx,
                  aw*bz+ax*by-ay*bx+az*bw)
    # qvec cross +Z, then qvec cross that intermediate (gz-math Quaternion).
    uv = (y, -x, 0.)
    uuv = (y*uv[2]-z*uv[1], z*uv[0]-x*uv[2], x*uv[1]-y*uv[0])
    direction = np.asarray([uv[0]*(2*w)+2*uuv[0], uv[1]*(2*w)+2*uuv[1],
                            1.+uv[2]*(2*w)+2*uuv[2]])
    x, y, z = direction
    ax, ay, az = np.abs(direction)
    if az >= ax and az >= ay:
        face, major = (5 if z < 0 else 4), az
        u, v = (-x if z < 0 else x), -y
    elif ay >= ax:
        face, major = (3 if y < 0 else 2), ay
        u, v = x, (-z if y < 0 else z)
    else:
        face, major = (1 if x < 0 else 0), ax
        u, v = (z if x < 0 else -z), -y
    # CPU sampling table is PFG_RGBA32_FLOAT, before nearest lookup.
    return face, np.asarray([.5+.5*u/major, .5+.5*v/major], dtype=np.float32)


def cube_pixel_to_flu(face, u, v):
    s, t = 2*u-1, 2*v-1
    standard = {0: [1., -t, -s], 1: [-1., -t, s], 2: [s, 1., t],
                3: [s, -1., -t], 4: [s, -t, 1.], 5: [-s, -t, -1.]}[face]
    x, y, z = standard
    flu = np.asarray([z, -x, y])
    return flu/np.linalg.norm(flu)


def first_texture_size(scan):
    hfov = max(1e-4, scan.angle_max-scan.angle_min)
    vfov = max(1e-4, scan.vertical_angle_max-scan.vertical_angle_min) if scan.vertical_count > 1 else 0.
    hs = scan.count if hfov < math.pi/2 else int(math.pi/2/hfov*scan.count)
    vs = scan.vertical_count if vfov < math.pi/2 else int(math.pi/2/vfov*scan.vertical_count)
    return min(1024, max(128, 1 << (max(hs, vs)-1).bit_length()))


def predict_scan(scan, world_pose):
    """Pure source-derived directions; intentionally never reads scan.ranges."""
    endpoints = [scan.angle_min, scan.angle_max, scan.vertical_angle_min, scan.vertical_angle_max]
    if scan.count < 1 or scan.vertical_count < 1 or not all(math.isfinite(x) for x in endpoints):
        raise ValueError('invalid cubemap angular endpoints/count')
    if scan.angle_max < scan.angle_min or scan.vertical_angle_max < scan.vertical_angle_min:
        raise ValueError('reversed cubemap angle endpoints')
    pose = np.asarray(world_pose, dtype=float)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError('invalid finite cubemap sensor pose')
    size = first_texture_size(scan)
    dh = max(1e-4, scan.angle_max-scan.angle_min)/(scan.count-1) if scan.count > 1 else 0.
    dv = max(1e-4, scan.vertical_angle_max-scan.vertical_angle_min)/(scan.vertical_count-1) if scan.vertical_count > 1 else 0.
    records, world_rays = [], []
    vertical = float(scan.vertical_angle_min)
    for row in range(scan.vertical_count):
        horizontal = float(scan.angle_min)
        for column in range(scan.count):
            face, uv = cubemap_uv(horizontal, vertical)
            # OgreNext 2.3.3 HlmsSamplerblock defaults to TAM_CLAMP. A face
            # seam at UV=1 therefore uses the last texel, never wraps to zero.
            pixel = np.minimum(size-1, np.maximum(0, np.floor(uv.astype(float)*size).astype(int)))
            center = (pixel+.5)/size
            ray = cube_pixel_to_flu(face, *center)
            world_rays.append(pose[:3, :3]@ray)
            nominal = np.asarray([math.cos(vertical)*math.cos(horizontal),
                                  math.cos(vertical)*math.sin(horizontal), math.sin(vertical)])
            records.append({'index': [column, row], 'face': face,
                            'uv_float32': uv.astype(float).tolist(), 'pixel': pixel.tolist(),
                            'pixel_center_uv': center.tolist(),
                            'angular_difference_deg': math.degrees(math.acos(np.clip(nominal@ray, -1., 1.)))})
            horizontal += dh
        vertical += dv
    return {'first_pass_texture_size': size, 'world_rays': world_rays, 'rays': records,
            'source': SOURCE_URL, 'directions_use_measured_ranges': False}
