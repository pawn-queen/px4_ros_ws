"""Small complementary pose fusion helper used by the localization node."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from px4_powerplant_mission.common.frames import yaw_to_quaternion_xyzw


@dataclass
class FusionState:
    position_enu: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    velocity_enu: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    yaw_enu: float = 0.0
    has_position: bool = False

    def predict_from_imu_delta(
        self,
        delta_velocity_enu: Sequence[float],
        dt_s: float,
        max_speed_mps: float,
    ) -> None:
        if dt_s <= 0.0 or dt_s > 1.0:
            return
        dv = np.asarray(delta_velocity_enu, dtype=float)
        if not np.all(np.isfinite(dv)):
            return
        self.velocity_enu += dv
        speed = float(np.linalg.norm(self.velocity_enu))
        if speed > max_speed_mps > 0.0:
            self.velocity_enu *= max_speed_mps / speed
        self.position_enu += self.velocity_enu * dt_s

    def correct_position(
        self,
        measurement_enu: Sequence[float],
        alpha: float,
        velocity_measurement_enu: Sequence[float] | None = None,
    ) -> None:
        measurement = np.asarray(measurement_enu, dtype=float)
        if not np.all(np.isfinite(measurement)):
            return
        alpha = float(np.clip(alpha, 0.0, 1.0))
        if not self.has_position:
            self.position_enu = measurement.copy()
            self.has_position = True
        else:
            self.position_enu = (1.0 - alpha) * self.position_enu + alpha * measurement
        if velocity_measurement_enu is not None:
            velocity = np.asarray(velocity_measurement_enu, dtype=float)
            if np.all(np.isfinite(velocity)):
                self.velocity_enu = (1.0 - alpha) * self.velocity_enu + alpha * velocity

    def correct_position_axes(
        self,
        measurement_enu: Sequence[float],
        alpha_enu: Sequence[float],
        velocity_measurement_enu: Sequence[float] | None = None,
    ) -> None:
        measurement = np.asarray(measurement_enu, dtype=float)
        alpha = np.clip(np.asarray(alpha_enu, dtype=float), 0.0, 1.0)
        if not np.all(np.isfinite(measurement)) or not np.all(np.isfinite(alpha)):
            return
        if not self.has_position:
            self.position_enu = measurement.copy()
            self.has_position = True
        else:
            self.position_enu = (1.0 - alpha) * self.position_enu + alpha * measurement
        if velocity_measurement_enu is not None:
            velocity = np.asarray(velocity_measurement_enu, dtype=float)
            if np.all(np.isfinite(velocity)):
                velocity_alpha = float(np.max(alpha))
                self.velocity_enu = (1.0 - velocity_alpha) * self.velocity_enu + velocity_alpha * velocity

    @property
    def orientation_xyzw(self) -> np.ndarray:
        return yaw_to_quaternion_xyzw(self.yaw_enu)
