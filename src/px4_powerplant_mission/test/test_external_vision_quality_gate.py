"""External vision publishes only bounded, explicitly uncertain positions."""

from types import SimpleNamespace

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

from px4_powerplant_mission.nodes.external_vision_bridge_node import ExternalVisionBridgeNode


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _FakeBridge:
    _has_publishable_position = ExternalVisionBridgeNode._has_publishable_position
    _position_variance = ExternalVisionBridgeNode._position_variance
    _velocity_variance = ExternalVisionBridgeNode._velocity_variance
    _positive_or_default = ExternalVisionBridgeNode._positive_or_default
    _positive_or_value = staticmethod(ExternalVisionBridgeNode._positive_or_value)
    _float32_list = staticmethod(ExternalVisionBridgeNode._float32_list)
    _timestamp_us = ExternalVisionBridgeNode._timestamp_us

    def __init__(self):
        self.visual_odom_pub = _Publisher()
        self.parameters = {
            "require_independent_position": True,
            "max_publish_position_variance_m2": 4.0,
            "max_publish_height_variance_m2": 4.0,
            "position_variance_m2": 0.08,
            "height_variance_m2": 0.12,
            "yaw_variance_rad2": 0.05,
            "velocity_variance_m2ps2": 0.25,
            "use_velocity": False,
        }

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2_000_000_000))

    def get_logger(self):
        return SimpleNamespace(warn=lambda *args, **kwargs: None)


def _odom():
    msg = Odometry()
    msg.pose.pose.position.x = 1.0  # east
    msg.pose.pose.position.y = 2.0  # north
    msg.pose.pose.position.z = 3.0  # up
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.12  # east variance
    msg.pose.covariance[7] = 0.34  # north variance
    msg.pose.covariance[14] = 0.56  # height variance
    msg.pose.covariance[35] = 0.07
    return msg


def test_publish_converts_position_and_axis_variances_without_claiming_quality():
    bridge = _FakeBridge()
    ExternalVisionBridgeNode._odom_callback(bridge, _odom())

    assert len(bridge.visual_odom_pub.messages) == 1
    visual = bridge.visual_odom_pub.messages[0]
    assert visual.pose_frame == VehicleOdometry.POSE_FRAME_NED
    assert np.allclose(visual.position, [2.0, 1.0, -3.0])
    assert np.allclose(visual.position_variance, [0.34, 0.12, 0.56])
    assert visual.quality == 0


@pytest.mark.parametrize("variance_index", [0, 7, 14])
def test_high_position_variance_blocks_external_vision(variance_index):
    bridge = _FakeBridge()
    msg = _odom()
    msg.pose.covariance[variance_index] = 4.1

    ExternalVisionBridgeNode._odom_callback(bridge, msg)

    assert bridge.visual_odom_pub.messages == []
