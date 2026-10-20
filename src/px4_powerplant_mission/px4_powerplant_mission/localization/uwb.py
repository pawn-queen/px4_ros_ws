"""UWB anchor parsing, range simulation, and trilateration helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


@dataclass(frozen=True)
class Anchor:
    name: str
    position_enu: np.ndarray


@dataclass(frozen=True)
class UwbSolution:
    """A 3D range solution and the measurements that support it."""

    position_enu: np.ndarray
    position_covariance_m2: np.ndarray
    residual_rms_m: float
    used_indices: tuple[int, ...]
    rejected_indices: tuple[int, ...]
    geometry_condition: float


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


def _range_solution(
    anchors: np.ndarray,
    ranges: np.ndarray,
    initial: np.ndarray | None,
    max_geometry_condition: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """Solve one valid subset; return position, residuals, Jacobian and condition."""

    # Four coplanar anchors can have two mirror-image 3D solutions. Reject this
    # ambiguity even if a local Jacobian at one of the solutions has full rank.
    anchor_singular = np.linalg.svd(anchors - np.mean(anchors, axis=0), compute_uv=False)
    if anchor_singular[-1] <= anchor_singular[0] * 1e-10:
        raise ValueError("UWB anchors do not span three dimensions")
    anchor_condition = float(anchor_singular[0] / anchor_singular[-1])
    if anchor_condition > max_geometry_condition:
        raise ValueError(
            f"UWB anchor geometry is poorly conditioned ({anchor_condition:.1f})"
        )

    relative = anchors[1:] - anchors[0]
    rhs = (
        ranges[0] ** 2 - ranges[1:] ** 2
        + np.sum(anchors[1:] ** 2, axis=1) - np.sum(anchors[0] ** 2)
    )
    linear_start = np.linalg.lstsq(2.0 * relative, rhs, rcond=None)[0]
    seeds = [linear_start]
    if initial is not None:
        seeds.insert(0, initial)

    best: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
    for seed in seeds:
        position = seed.copy()
        damping = 1e-3
        for _ in range(50):
            difference = position[None, :] - anchors
            distance = np.maximum(np.linalg.norm(difference, axis=1), 1e-9)
            residual = distance - ranges
            jacobian = difference / distance[:, None]
            cost = float(residual @ residual)
            gradient = jacobian.T @ residual
            information = jacobian.T @ jacobian
            accepted = False
            for _ in range(12):
                step = np.linalg.solve(
                    information + damping * np.eye(3), -gradient
                )
                trial = position + step
                trial_residual = np.linalg.norm(trial[None, :] - anchors, axis=1) - ranges
                if float(trial_residual @ trial_residual) < cost:
                    position = trial
                    damping = max(damping * 0.3, 1e-9)
                    accepted = True
                    break
                damping *= 10.0
            if not accepted or np.linalg.norm(step) < 1e-7:
                break

        difference = position[None, :] - anchors
        distance = np.maximum(np.linalg.norm(difference, axis=1), 1e-9)
        residual = distance - ranges
        jacobian = difference / distance[:, None]
        if best is None or residual @ residual < best[1] @ best[1]:
            best = position, residual, jacobian

    assert best is not None
    position, residual, jacobian = best
    singular = np.linalg.svd(jacobian, compute_uv=False)
    geometry_condition = max(
        anchor_condition,
        float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else np.inf,
    )
    if not np.isfinite(geometry_condition) or geometry_condition > max_geometry_condition:
        raise ValueError(
            f"UWB geometry is poorly conditioned ({geometry_condition:.1f})"
        )
    return position, residual, jacobian, geometry_condition


def solve_uwb_ranges(
    anchors_enu: Sequence[Sequence[float]],
    ranges_m: Sequence[float],
    *,
    range_std_m: float,
    min_range_m: float,
    max_range_m: float,
    max_residual_rms_m: float,
    max_geometry_condition: float,
    initial_enu: Sequence[float] | None = None,
) -> UwbSolution:
    """Estimate 3D ENU position from quality-checked per-anchor ranges.

    Invalid or out-of-range measurements are discarded, never clipped into the
    accepted interval. With at least five valid anchors, one gross range error
    may be removed after leave-one-out re-solving. A failed quality gate raises
    ValueError so callers cannot silently publish an unreliable pose.
    """

    anchors = np.asarray(anchors_enu, dtype=float)
    ranges = np.asarray(ranges_m, dtype=float)
    if anchors.ndim != 2 or anchors.shape[1] != 3 or not np.all(np.isfinite(anchors)):
        raise ValueError("anchors_enu must be a finite Nx3 array")
    if ranges.ndim != 1 or len(ranges) != len(anchors):
        raise ValueError("ranges_m must have one value per anchor")
    if not np.isfinite(range_std_m) or range_std_m <= 0.0:
        raise ValueError("range_std_m must be positive and finite")
    if not (np.isfinite(min_range_m) and np.isfinite(max_range_m)) or not (
        0.0 < min_range_m < max_range_m
    ):
        raise ValueError("range limits must satisfy 0 < min_range_m < max_range_m")
    if not np.isfinite(max_residual_rms_m) or max_residual_rms_m <= 0.0:
        raise ValueError("max_residual_rms_m must be positive and finite")
    if not np.isfinite(max_geometry_condition) or max_geometry_condition <= 1.0:
        raise ValueError("max_geometry_condition must be finite and greater than one")
    initial = None if initial_enu is None else np.asarray(initial_enu, dtype=float)
    if initial is not None and (initial.shape != (3,) or not np.all(np.isfinite(initial))):
        raise ValueError("initial_enu must be a finite 3D position")

    valid = np.isfinite(ranges) & (ranges > min_range_m) & (ranges < max_range_m)
    valid_indices = np.flatnonzero(valid)
    if len(valid_indices) < 4:
        raise ValueError("at least four valid UWB ranges are required for 3D")

    def solve_subset(indices: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        return _range_solution(
            anchors[indices], ranges[indices], initial, max_geometry_condition
        )

    all_solution: tuple[np.ndarray, np.ndarray, np.ndarray, float] | None = None
    all_error: ValueError | None = None
    try:
        all_solution = solve_subset(valid_indices)
    except ValueError as exc:
        all_error = exc

    def quality_ok(residual: np.ndarray) -> bool:
        rms = float(np.sqrt(np.mean(residual ** 2)))
        degrees_of_freedom = max(len(residual) - 3, 1)
        # Wilson-Hilferty approximation to the 99th percentile of Chi-square.
        # RMS alone can look small after 3D position absorbs a biased range.
        chi_limit = degrees_of_freedom * (
            1.0 - 2.0 / (9.0 * degrees_of_freedom)
            + 2.326 * np.sqrt(2.0 / (9.0 * degrees_of_freedom))
        ) ** 3
        normalized_error = float(residual @ residual) / range_std_m ** 2
        return rms <= max_residual_rms_m and normalized_error <= chi_limit

    selected_indices = valid_indices
    selected = all_solution if all_solution is not None and quality_ok(all_solution[1]) else None
    if selected is None and len(valid_indices) >= 5:
        candidates = []
        for omitted in valid_indices:
            indices = valid_indices[valid_indices != omitted]
            try:
                candidate = solve_subset(indices)
            except ValueError:
                continue
            position, residual, jacobian, _ = candidate
            omitted_residual = float(np.linalg.norm(position - anchors[omitted]) - ranges[omitted])
            # NLOS commonly lengthens a range. Require that signed pattern;
            # otherwise dropping a good anchor can make a wrong 4-point fit
            # look deceptively exact.
            if not quality_ok(residual) or omitted_residual > -4.0 * range_std_m:
                continue
            score = float(residual @ residual) + range_std_m * abs(omitted_residual)
            candidates.append((score, indices, candidate))
        candidates.sort(key=lambda item: item[0])
        if candidates:
            _, selected_indices, selected = candidates[0]
            if len(candidates) > 1 and candidates[1][0] <= 1.3 * candidates[0][0]:
                other_position = candidates[1][2][0]
                other_jacobian = candidates[1][2][2]
                selected_covariance = range_std_m ** 2 * np.linalg.inv(
                    selected[2].T @ selected[2]
                )
                other_covariance = range_std_m ** 2 * np.linalg.inv(
                    other_jacobian.T @ other_jacobian
                )
                uncertainty = 3.0 * np.sqrt(
                    max(float(np.linalg.eigvalsh(selected_covariance + other_covariance)[-1]), 0.0)
                )
                if np.linalg.norm(selected[0] - other_position) > uncertainty:
                    raise ValueError("UWB range solution is ambiguous after outlier rejection")

    if selected is None:
        if all_error is not None:
            raise all_error
        raise ValueError("UWB ranges fail normalized residual quality gate")
    position, residual, jacobian, geometry_condition = selected
    rms = float(np.sqrt(np.mean(residual ** 2)))

    # For an overdetermined solve, observed residuals may justify a larger
    # uncertainty than the nominal ranging noise, but never a smaller one.
    degrees_of_freedom = max(len(selected_indices) - 3, 1)
    residual_variance = float(residual @ residual) / degrees_of_freedom
    variance = max(range_std_m ** 2, residual_variance)
    covariance = variance * np.linalg.inv(jacobian.T @ jacobian)
    covariance = (covariance + covariance.T) * 0.5
    selected_set = set(selected_indices.tolist())
    return UwbSolution(
        position_enu=position,
        position_covariance_m2=covariance,
        residual_rms_m=rms,
        used_indices=tuple(int(index) for index in selected_indices),
        rejected_indices=tuple(index for index in range(len(ranges)) if index not in selected_set),
        geometry_condition=geometry_condition,
    )


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

