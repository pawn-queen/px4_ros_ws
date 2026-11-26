"""Pinhole registration for the co-located, rectified Gazebo OakD sensors."""

from functools import lru_cache

import cv2
import numpy as np


def pinhole(info, image):
    if (info.width, info.height) != (image.width, image.height):
        raise ValueError('CameraInfo dimensions differ from image dimensions')
    k = np.asarray(info.k).reshape(3, 3)
    if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0., 0., 1.])
            or not np.allclose([k[0, 1], k[1, 0]], 0.) or np.any(info.d)):
        raise ValueError('expected finite rectified pinhole calibration')
    return (int(info.width), int(info.height), k[0, 0], k[1, 1], k[0, 2], k[1, 2])


@lru_cache(maxsize=2)
def depth_coordinates(rgb_pinhole, depth_pinhole):
    """Inverse map RGB rays into depth pixels for coincident optical centres."""
    w, h, fx, fy, cx, cy = rgb_pinhole
    _, _, dfx, dfy, dcx, dcy = depth_pinhole
    u = (np.arange(w, dtype=np.float32) - cx) * (dfx / fx) + dcx
    v = (np.arange(h, dtype=np.float32) - cy) * (dfy / fy) + dcy
    col, row = np.meshgrid(u, v)
    return cv2.convertMaps(col.astype('float32'), row.astype('float32'),
                           cv2.CV_16SC2, nninterpolation=True)


def register_depth(image, rgb_pinhole, depth_pinhole):
    if image.encoding not in ('32FC1', '16UC1'):
        raise ValueError('expected depth in float metres or uint16 millimetres')
    kind = 'f4' if image.encoding == '32FC1' else 'u2'
    dtype = np.dtype(('>' if image.is_bigendian else '<') + kind)
    if image.step < image.width * dtype.itemsize or len(image.data) != image.height * image.step:
        raise ValueError('invalid depth row stride or buffer size')
    depth = np.ndarray((image.height, image.width), dtype, image.data,
                       strides=(image.step, dtype.itemsize)).astype('float32', copy=False)
    if kind == 'u2':
        depth = depth * np.float32(0.001)
    coords, unused = depth_coordinates(rgb_pinhole, depth_pinhole)
    registered = cv2.remap(depth, coords, unused, cv2.INTER_NEAREST,
                           borderMode=cv2.BORDER_CONSTANT, borderValue=float('nan'))
    registered[~np.isfinite(registered) | (registered <= 0)] = np.nan
    return registered


class Occlusion:
    def __init__(self, start_s, duration_s):
        if not np.isfinite(start_s) or not np.isfinite(duration_s) or duration_s < 0:
            raise ValueError('expected finite occlusion start and nonnegative duration')
        self.start_s = start_s
        self.duration_s = duration_s
        self.external = False
        self.first_ns = None

    def masked(self, stamp_ns):
        if self.first_ns is None:
            self.first_ns = stamp_ns
        elapsed = (stamp_ns - self.first_ns) * 1e-9
        return self.external or (self.start_s >= 0 and
                                 self.start_s <= elapsed < self.start_s + self.duration_s)
