"""Small, testable helpers for the takeoff hover and yaw scan."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from px4_powerplant_mission.common.frames import wrap_pi


def has_observed_cells(grid: np.ndarray | None, minimum: int) -> bool:
    """A fixed-size occupancy grid can exist while every cell is still unknown."""
    return grid is not None and int(np.count_nonzero(grid >= 0)) >= max(1, minimum)


@dataclass
class YawScanSequence:
    """Visit successive yaw headings so the final wrapped heading cannot end the scan early."""

    start_yaw_rad: float
    steps: int
    dwell_s: float
    step_index: int = 0
    reached_since_ns: int = 0

    def __post_init__(self) -> None:
        if self.steps < 4:
            raise ValueError("yaw scan needs at least four headings")

    @property
    def complete(self) -> bool:
        return self.step_index >= self.steps

    def target_yaw(self) -> float:
        if self.complete:
            return wrap_pi(self.start_yaw_rad)
        return wrap_pi(self.start_yaw_rad + (self.step_index + 1) * 2.0 * math.pi / self.steps)

    def update(self, actual_yaw: float, now_ns: int, ready: bool, tolerance_rad: float) -> bool:
        if self.complete:
            return True
        if not ready or abs(wrap_pi(self.target_yaw() - actual_yaw)) > tolerance_rad:
            self.reached_since_ns = 0
            return False
        if self.reached_since_ns == 0:
            self.reached_since_ns = now_ns
            return False
        if (now_ns - self.reached_since_ns) * 1e-9 >= self.dwell_s:
            self.step_index += 1
            self.reached_since_ns = 0
        return self.complete
