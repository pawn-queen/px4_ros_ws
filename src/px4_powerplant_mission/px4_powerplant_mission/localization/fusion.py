"""Small complementary pose fusion helper used by the localization node."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

from px4_powerplant_mission.common.frames import (
    normalize_quaternion_xyzw,
    quaternion_conjugate_xyzw,
    quaternion_multiply_xyzw,
    quaternion_xyzw_to_yaw,
    yaw_to_quaternion_xyzw,
)


@dataclass
class FusionState:
    position_enu: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    position_variance_enu_m2: np.ndarray = field(default_factory=lambda: np.full(3, 4.0, dtype=float))
    velocity_enu: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    yaw_enu: float = 0.0
    tilt_xyzw: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    has_tilt: bool = False
    has_position: bool = False

    def correct_tilt(self, orientation_xyzw: Sequence[float]) -> bool:
        """Keep measured roll/pitch while allowing yaw to follow its own source."""

        q = np.asarray(orientation_xyzw, dtype=float)
        if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) <= 1e-6:
            return False
        q = normalize_quaternion_xyzw(q)
        measured_yaw = quaternion_xyzw_to_yaw(q)
        self.tilt_xyzw = normalize_quaternion_xyzw(
            quaternion_multiply_xyzw(
                quaternion_conjugate_xyzw(yaw_to_quaternion_xyzw(measured_yaw)), q
            )
        )
        self.has_tilt = True
        return True

    def clear_tilt(self) -> None:
        self.tilt_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.has_tilt = False

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

    def correct_position_with_variance(
        self,
        measurement_enu: Sequence[float],
        measurement_variance_enu_m2: Sequence[float],
        predicted_position_enu: Sequence[float] | None = None,
        predicted_variance_enu_m2: Sequence[float] | None = None,
    ) -> np.ndarray:
        """Apply an independent position measurement with per-axis Kalman gains.

        The caller supplies a motion prediction and its variance when available.
        A first independent measurement initializes the position directly.
        """
        measurement = np.asarray(measurement_enu, dtype=float)
        variance = np.asarray(measurement_variance_enu_m2, dtype=float)
        if (measurement.shape != (3,) or variance.shape != (3,)
                or not np.all(np.isfinite(measurement))
                or not np.all(np.isfinite(variance)) or np.any(variance <= 0.0)):
            raise ValueError("position measurement and variances must be finite 3-vectors")

        if not self.has_position:
            self.position_enu = measurement.copy()
            self.position_variance_enu_m2 = variance.copy()
            self.has_position = True
            return np.ones(3, dtype=float)

        predicted = (
            self.position_enu if predicted_position_enu is None
            else np.asarray(predicted_position_enu, dtype=float)
        )
        prior_variance = (
            self.position_variance_enu_m2 if predicted_variance_enu_m2 is None
            else np.asarray(predicted_variance_enu_m2, dtype=float)
        )
        if (predicted.shape != (3,) or prior_variance.shape != (3,)
                or not np.all(np.isfinite(predicted))
                or not np.all(np.isfinite(prior_variance))
                or np.any(prior_variance <= 0.0)):
            raise ValueError("predicted position and variances must be finite 3-vectors")

        gain = prior_variance / (prior_variance + variance)
        self.position_enu = predicted + gain * (measurement - predicted)
        self.position_variance_enu_m2 = (1.0 - gain) * prior_variance
        return gain

    @property
    def orientation_xyzw(self) -> np.ndarray:
        yaw_q = yaw_to_quaternion_xyzw(self.yaw_enu)
        if not self.has_tilt:
            return yaw_q
        return normalize_quaternion_xyzw(quaternion_multiply_xyzw(yaw_q, self.tilt_xyzw))
