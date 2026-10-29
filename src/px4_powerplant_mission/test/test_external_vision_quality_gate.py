"""Independent input must not leak PX4 priors, invented attitude, or fresh stamps."""
import json
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleOdometry
from std_msgs.msg import String

from px4_powerplant_mission.common.frames import px4_to_ros_quaternion
from px4_powerplant_mission.localization.inputs import UwbInput, set_stamp
from px4_powerplant_mission.nodes.external_vision_bridge_node import ExternalVisionBridgeNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeBridge:
    _pose_callback = ExternalVisionBridgeNode._pose_callback
    _health_callback = ExternalVisionBridgeNode._health_callback

    def __init__(self):
        self.now_ns = 2_000_000_000
        self.system_ns = 1_800_000_000_000_000_000
        self.system_clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.system_ns))
        self.input = UwbInput(max_z=2.)
        self.visual_odom_pub = Publisher()
        self.parameters = {"use_independent_orientation": False, "max_orientation_variance_rad2": .5,
                           "px4_timestamp_domain": "system", "use_sim_time": False}

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.now_ns))


def pose(stamp=1_950_000_000):
    msg = PoseWithCovarianceStamped()
    set_stamp(msg.header.stamp, stamp)
    msg.header.frame_id = "map"
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1., 2., 3.
    msg.pose.pose.orientation.w = 1.
    msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14] = .12, .34, .56
    for i in (21, 28, 35):
        msg.pose.covariance[i] = 1e6
    return msg


def test_hardware_position_only_preserves_acquisition_time_and_covariance():
    bridge = FakeBridge()
    bridge._pose_callback(pose())
    visual, = bridge.visual_odom_pub.messages
    assert visual.pose_frame == VehicleOdometry.POSE_FRAME_NED
    assert np.allclose(visual.position, [2., 1., -3.])
    assert np.allclose(visual.position_variance, [.34, .12, .56])
    assert visual.timestamp_sample == 1_950_000
    assert visual.timestamp == 2_000_000
    assert all(np.isnan(visual.q)) and all(np.isnan(visual.velocity))
    assert visual.quality == 0


@pytest.mark.parametrize("index,value", [(0, 4.1), (7, 4.1), (14, 4.1), (0, 0.), (7, float('nan'))])
def test_invalid_position_variance_blocks_external_vision(index, value):
    bridge = FakeBridge()
    msg = pose()
    msg.pose.covariance[index] = value
    bridge._pose_callback(msg)
    assert not bridge.visual_odom_pub.messages


def test_simulation_independent_full_attitude_requires_explicit_opt_in():
    bridge = FakeBridge()
    msg = pose()
    q = px4_to_ros_quaternion([.9, .1, .2, .3])
    msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
    for i in (21, 28, 35):
        msg.pose.covariance[i] = .02
    bridge._pose_callback(msg)
    assert all(np.isnan(bridge.visual_odom_pub.messages[-1].q))
    bridge.parameters['use_independent_orientation'] = True
    set_stamp(msg.header.stamp, 1_960_000_000)
    bridge._pose_callback(msg)
    restored = px4_to_ros_quaternion(bridge.visual_odom_pub.messages[-1].q)
    assert abs(np.dot(restored, q)) == pytest.approx(1.)


def test_simulation_transport_conversion_preserves_sample_age():
    bridge = FakeBridge()
    bridge.parameters['use_sim_time'] = True
    bridge._pose_callback(pose())
    visual, = bridge.visual_odom_pub.messages
    assert visual.timestamp == bridge.system_ns // 1000
    assert visual.timestamp - visual.timestamp_sample == 50_000


def test_duplicates_future_and_stale_samples_are_not_republished():
    bridge = FakeBridge()
    for stamp in [1_950_000_000, 1_950_000_000, 1_940_000_000, 1_000_000_000, 2_200_000_000, 0]:
        bridge._pose_callback(pose(stamp))
    assert len(bridge.visual_odom_pub.messages) == 1


def test_stop_invalidates_queued_pose_and_positive_quality_cannot_restore_it():
    bridge = FakeBridge()
    bridge._pose_callback(pose())
    bridge._health_callback(String(data=json.dumps({'kind': 'device_running_state', 'confirmed_running': False})))
    bridge._pose_callback(pose(1_990_000_000))
    bridge._health_callback(String(data='{"accepted":true}'))
    assert not bridge.input.healthy(bridge.now_ns)
    assert len(bridge.visual_odom_pub.messages) == 1
    bridge.now_ns += 100_000_000
    bridge._pose_callback(pose(bridge.now_ns))
    assert bridge.input.healthy(bridge.now_ns)
    assert len(bridge.visual_odom_pub.messages) == 2


def test_rewind_allows_new_epoch_and_advances_reset_counter():
    bridge = FakeBridge()
    bridge._pose_callback(pose())
    bridge.now_ns = 1_000_000_000
    bridge._pose_callback(pose(900_000_000))
    assert len(bridge.visual_odom_pub.messages) == 2
    assert bridge.visual_odom_pub.messages[-1].reset_counter == 1


def test_survey_frame_transform_rotates_covariance_and_rejects_other_frames():
    gate = UwbInput(source_frame='survey', translation=[10., 20., 1.], yaw=np.pi/2, max_z=2.)
    msg = pose()
    msg.header.frame_id = 'survey'
    transformed = gate.accept(msg, 2_000_000_000)
    p = transformed.pose.pose.position
    assert [p.x, p.y, p.z] == pytest.approx([8., 21., 4.])
    assert np.array(transformed.pose.covariance)[[0, 7, 14]] == pytest.approx([.34, .12, .56])
    msg.header.frame_id = 'wrong'
    set_stamp(msg.header.stamp, 1_960_000_000)
    assert gate.accept(msg, 2_000_000_000) is None
    assert not gate.healthy(2_000_000_000)
