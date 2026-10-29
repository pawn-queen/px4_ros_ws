"""Shared sensor-input contract for simulated and hardware UWB.

This module never consumes PX4 estimates. A qualified measurement remains an
independent measurement; navigation fusion must not be routed back into EV.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math

import numpy as np

from px4_powerplant_mission.common.frames import (
    quaternion_multiply_xyzw, yaw_to_quaternion_xyzw,
)


def stamp_ns(stamp) -> int:
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def set_stamp(stamp, ns: int) -> None:
    stamp.sec, stamp.nanosec = divmod(int(ns), 1_000_000_000)


def declare_input_parameters(node) -> None:
    for name, value in {
        "uwb_pose_topic": "/uwb/pose",
        "uwb_quality_topic": "/uwb/quality",
        "uwb_status_topic": "/uwb/status",
        "uwb_frame": "map",
        "uwb_to_map_translation_m": [0.0, 0.0, 0.0],
        "uwb_to_map_yaw_rad": 0.0,
    }.items():
        node.declare_parameter(name, value)


class UwbInput:
    def __init__(self, source_frame="map", map_frame="map", translation=(0., 0., 0.),
                 yaw=0., timeout_s=.5, max_xy=2., max_z=.5):
        self.source_frame, self.map_frame = source_frame, map_frame
        self.translation = np.asarray(translation, dtype=float)
        if (self.translation.shape != (3,) or not np.all(np.isfinite(self.translation))
                or not all(math.isfinite(v) for v in (yaw, timeout_s, max_xy, max_z))
                or timeout_s <= 0 or max_xy <= 0 or max_z <= 0):
            raise ValueError("invalid UWB transform or quality limits")
        c, s = math.cos(yaw), math.sin(yaw)
        self.rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        self.rotation_q = yaw_to_quaternion_xyzw(yaw)
        self.timeout_ns = int(timeout_s * 1e9)
        self.limits = np.array([max_xy, max_xy, max_z])
        self.last_sample_ns = 0
        self.invalid_before_ns = 0
        self.last_now_ns = 0
        self.valid = False
        self.reason = "waiting for independent UWB"
        self.reset_counter = 0

    @classmethod
    def from_node(cls, node, max_xy, max_z, timeout_s):
        p = lambda name: node.get_parameter(name).value
        return cls(p("uwb_frame"), p("map_frame"), p("uwb_to_map_translation_m"),
                   p("uwb_to_map_yaw_rad"), timeout_s, max_xy, max_z)

    def clock(self, now_ns):
        if now_ns < self.last_now_ns:
            self.last_sample_ns = self.invalid_before_ns = 0
            self.valid = False
            self.reason = "clock reset"
            self.reset_counter = (self.reset_counter + 1) % 256
        self.last_now_ns = now_ns

    def healthy(self, now_ns):
        self.clock(now_ns)
        return self.valid and 0 <= now_ns - self.last_sample_ns <= self.timeout_ns

    def health(self, payload, now_ns):
        """Reject bad samples without extending the last good sample's life.

        Device faults revoke trust immediately. A rejected *new* measurement
        does not erase a previous, still-fresh measurement, but its stamp is
        barred from later admission. Rejecting a sample already in use revokes
        it; only a newer qualified pose can restore trust.
        """
        self.clock(now_ns)
        try:
            event = json.loads(payload)
            if not isinstance(event, dict):
                return
            sample = max(0, int(event.get("sample_stamp_ns", 0) or 0))
        except (ValueError, TypeError, OverflowError):
            return
        kind = event.get("kind", "")
        hard_failure = kind in {
            "session_started", "session_ended", "device_fault", "clock_rejected",
            "serial_unavailable", "stale",
        } or event.get("confirmed_running") is False
        # Simulator quality messages have no kind. A statistical range gate
        # rejects that observation, not the device or the prior valid sample.
        measurement_rejected = kind == "observation_rejected" or (
            not kind and event.get("accepted") is False
        )
        if not (hard_failure or measurement_rejected) or (
                sample > 0 and sample < self.last_sample_ns):
            return
        self.invalid_before_ns = max(self.invalid_before_ns, sample or now_ns)
        self.reason = str(event.get("reason", kind or "UWB quality rejected"))
        if measurement_rejected and not hard_failure and sample > self.last_sample_ns:
            # Retain the original stamp and validity; healthy() still expires
            # it at the original TTL, and no old measurement is republished.
            return
        self.valid = False
        if kind in {"session_started", "session_ended"}:
            self.reset_counter = (self.reset_counter + 1) % 256

    def accept(self, msg, now_ns):
        self.clock(now_ns)
        sample = stamp_ns(msg.header.stamp)
        if sample <= 0 or sample <= self.last_sample_ns or sample <= self.invalid_before_ns:
            self.reason = "missing, duplicate, out-of-order or invalidated measurement stamp"
            return None
        if not -100_000_000 <= now_ns - sample <= self.timeout_ns:
            self.reason = "measurement outside clock/freshness window"
            return None
        if msg.header.frame_id != self.source_frame:
            self.valid, self.reason = False, "unexpected UWB frame"
            return None
        p = msg.pose.pose.position
        position = np.array([p.x, p.y, p.z])
        covariance = np.asarray(msg.pose.covariance).reshape(6, 6)[:3, :3]
        if (not np.all(np.isfinite(position)) or not np.all(np.isfinite(covariance))
                or not np.allclose(covariance, covariance.T, atol=1e-9)
                or np.min(np.linalg.eigvalsh(covariance)) < -1e-8):
            self.valid, self.reason = False, "invalid position/covariance"
            return None
        covariance = self.rotation @ covariance @ self.rotation.T
        if np.any(np.diag(covariance) <= 0) or np.any(np.diag(covariance) > self.limits):
            self.valid, self.reason = False, "excessive or unknown position covariance"
            return None
        result = deepcopy(msg)
        position = self.rotation @ position + self.translation
        result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = map(float, position)
        # Rotate all covariance blocks consistently, including position/attitude correlations.
        transform = np.zeros((6, 6))
        transform[:3, :3] = transform[3:, 3:] = self.rotation
        full = np.asarray(msg.pose.covariance).reshape(6, 6)
        # Unknown attitude is allowed but must not contaminate a valid position.
        if np.all(np.isfinite(full)):
            result.pose.covariance = (transform @ full @ transform.T).ravel().tolist()
        else:
            result.pose.covariance = [0.] * 36
            for i in (21, 28, 35):
                result.pose.covariance[i] = 1e6
        for i in range(3):
            for j in range(3):
                result.pose.covariance[6*i+j] = float(covariance[i, j])
        q = msg.pose.pose.orientation
        orientation = np.array([q.x, q.y, q.z, q.w])
        if np.all(np.isfinite(orientation)) and np.linalg.norm(orientation) > 1e-6:
            orientation = quaternion_multiply_xyzw(self.rotation_q, orientation / np.linalg.norm(orientation))
            q = result.pose.pose.orientation
            q.x, q.y, q.z, q.w = map(float, orientation)
        result.header.frame_id = self.map_frame
        self.last_sample_ns, self.valid, self.reason = sample, True, ""
        return result
