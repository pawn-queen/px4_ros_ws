"""UWB anchor parsing, range simulation, and trilateration helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class Anchor:
    name: str
    position_enu: np.ndarray


def parse_anchor_list(values: Sequence[float], names: Sequence[str] | None = None) -> list[Anchor]:
    if len(values) % 3 != 0:
        raise ValueError("uwb_anchors must contain x,y,z triples in ENU meters")
    anchors: list[Anchor] = []
    count = len(values) // 3
    for index in range(count):
        name = names[index] if names and index < len(names) else f"anchor_{index}"
        start = index * 3
        anchors.append(Anchor(name=name, position_enu=np.asarray(values[start:start + 3], dtype=float)))
    return anchors


def simulate_ranges(
    position_enu: Sequence[float],
    anchors: Iterable[Anchor],
    noise_std_m: float,
    rng: np.random.Generator,
) -> list[float]:
    position = np.asarray(position_enu, dtype=float)
    ranges = []
    for anchor in anchors:
        distance = float(np.linalg.norm(position - anchor.position_enu))
        if noise_std_m > 0.0:
            distance += float(rng.normal(0.0, noise_std_m))
        ranges.append(max(distance, 0.0))
    return ranges


def trilaterate(
    anchors_enu: Sequence[Sequence[float]],
    ranges_m: Sequence[float],
    initial_enu: Sequence[float] | None = None,
    weights: Sequence[float] | None = None,
    max_iterations: int = 12,
) -> tuple[np.ndarray, float]:
    """Estimate ENU position from anchor ranges with damped Gauss-Newton.

    Returns the position and the RMS range residual in meters. Four non-coplanar
    anchors are preferred for 3D; with fewer anchors the initial z is retained.
    """

    anchors = np.asarray(anchors_enu, dtype=float)
    ranges = np.asarray(ranges_m, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] != 3:
        raise ValueError("anchors_enu must be Nx3")
    if len(anchors) != len(ranges):
        raise ValueError("anchors and ranges must have the same length")
    if len(anchors) < 3:
        raise ValueError("at least three UWB anchors are required")

    valid = np.isfinite(ranges) & (ranges > 0.0)
    anchors = anchors[valid]
    ranges = ranges[valid]
    if len(anchors) < 3:
        raise ValueError("at least three valid UWB ranges are required")

    if initial_enu is None:
        x = np.mean(anchors, axis=0)
    else:
        x = np.asarray(initial_enu, dtype=float).copy()

    if len(anchors) == 3:
        solve_axes = np.array([True, True, False])
    else:
        solve_axes = np.array([True, True, True])

    if weights is None:
        w = np.ones(len(anchors), dtype=float)
    else:
        w = np.asarray(weights, dtype=float)[valid]
        w = np.clip(w, 1e-6, None)

    damping = 1e-3
    for _ in range(max_iterations):
        diff = x[None, :] - anchors
        dist = np.linalg.norm(diff, axis=1)
        dist = np.maximum(dist, 1e-6)
        residual = dist - ranges
        jacobian = diff / dist[:, None]
        jacobian = jacobian[:, solve_axes]

        sqrt_w = np.sqrt(w)
        a = jacobian * sqrt_w[:, None]
        b = -residual * sqrt_w
        lhs = a.T @ a + damping * np.eye(a.shape[1])
        rhs = a.T @ b
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
        x[solve_axes] += step
        if float(np.linalg.norm(step)) < 1e-4:
            break

    residual = np.linalg.norm(x[None, :] - anchors, axis=1) - ranges
    rms = float(np.sqrt(np.mean(residual * residual)))
    return x, rms

