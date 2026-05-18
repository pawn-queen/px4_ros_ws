"""Coordinate frame helpers for PX4 NED/FRD and ROS ENU/FLU conventions."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

import numpy as np

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass
class Pose3:
    """A lightweight pose used internally by the mapping and control nodes."""

    position: np.ndarray
    orientation_xyzw: np.ndarray


def as_vector3(values: Iterable[float]) -> np.ndarray:
    return np.asarray(list(values), dtype=float).reshape(3)


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def ned_to_enu(ned: Sequence[float]) -> np.ndarray:
    """Convert a vector from PX4 local NED to ROS ENU."""

    n, e, d = ned
    return np.array([e, n, -d], dtype=float)


def enu_to_ned(enu: Sequence[float]) -> np.ndarray:
    """Convert a vector from ROS ENU to PX4 local NED."""

    e, n, u = enu
    return np.array([n, e, -u], dtype=float)


def frd_to_flu(frd: Sequence[float]) -> np.ndarray:
    """Convert a body vector from PX4 FRD to ROS FLU."""

    f, r, d = frd
    return np.array([f, -r, -d], dtype=float)


def flu_to_frd(flu: Sequence[float]) -> np.ndarray:
    """Convert a body vector from ROS FLU to PX4 FRD."""

    f, l, u = flu
    return np.array([f, -l, -u], dtype=float)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    """PX4 heading/yaw in NED to ROS yaw in ENU."""

    return wrap_pi(math.pi / 2.0 - yaw_ned)


def yaw_enu_to_ned(yaw_enu: float) -> float:
    """ROS yaw in ENU to PX4 heading/yaw in NED."""

    return wrap_pi(math.pi / 2.0 - yaw_enu)


def yaw_to_quaternion_xyzw(yaw: float) -> np.ndarray:
    half = yaw * 0.5
    return np.array([0.0, 0.0, math.sin(half), math.cos(half)], dtype=float)


def quaternion_xyzw_to_yaw(q: Sequence[float]) -> float:
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_xyzw_to_roll_pitch(q: Sequence[float]) -> tuple[float, float]:
    x, y, z, w = normalize_quaternion_xyzw(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch


def quaternion_wxyz_to_xyzw(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = q
    return np.array([x, y, z, w], dtype=float)


def quaternion_multiply_xyzw(a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=float,
    )


def quaternion_conjugate_xyzw(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=float)


def normalize_quaternion_xyzw(q: Sequence[float]) -> np.ndarray:
    arr = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-9 or not np.isfinite(norm):
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return arr / norm


def rotate_vector_xyzw(q: Sequence[float], vector: Sequence[float]) -> np.ndarray:
    """Rotate vector by quaternion in ROS xyzw convention."""

    qn = normalize_quaternion_xyzw(q)
    vq = np.array([vector[0], vector[1], vector[2], 0.0], dtype=float)
    return quaternion_multiply_xyzw(
        quaternion_multiply_xyzw(qn, vq), quaternion_conjugate_xyzw(qn)
    )[:3]


def pose_from_xyzyaw(x: float, y: float, z: float, yaw: float) -> Pose3:
    return Pose3(
        position=np.array([x, y, z], dtype=float),
        orientation_xyzw=yaw_to_quaternion_xyzw(yaw),
    )


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """Convert WGS84 latitude, longitude, altitude to ECEF Cartesian meters."""

    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * math.cos(lon)
    y = (n + alt_m) * cos_lat * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return np.array([x, y, z], dtype=float)


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
) -> np.ndarray:
    """Convert WGS84 coordinates into an ENU vector around a local reference."""

    point = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    ref = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    delta = point - ref

    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    transform = np.array(
        [
            [-sin_lon, cos_lon, 0.0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat],
        ],
        dtype=float,
    )
    return transform @ delta
