"""Hardware-shaped and simulated samples exercise the same real ROS nodes."""
import json
import math
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.parameter import Parameter
from std_msgs.msg import String

from px4_powerplant_mission.common.frames import (
    px4_to_ros_quaternion, ros_to_px4_quaternion, rotate_vector_xyzw,
    quaternion_multiply_xyzw, yaw_to_quaternion_xyzw,
)
from px4_powerplant_mission.localization.inputs import set_stamp, stamp_ns
from px4_powerplant_mission.nodes.localization_node import LocalizationNode
from px4_powerplant_mission.nodes.external_vision_bridge_node import ExternalVisionBridgeNode


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(deepcopy(msg))


@pytest.fixture
def pipeline():
    rclpy.init(args=[])
    nav, ev = LocalizationNode(), ExternalVisionBridgeNode()
    nav.set_parameters([Parameter('publish_tf', value=False)])
    nav.odom_pub = Capture()
    nav.pose_pub = Capture()
    nav.status_pub = Capture()
    ev.visual_odom_pub = Capture()
    yield nav, ev
    nav.destroy_node()
    ev.destroy_node()
    rclpy.shutdown()


def uwb(ns):
    msg = PoseWithCovarianceStamped()
    set_stamp(msg.header.stamp, ns)
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = 1., 2., 3.
    msg.pose.pose.orientation.w = 1.
    for i in (0, 7, 14):
        msg.pose.covariance[i] = .1
    for i in (21, 28, 35):
        msg.pose.covariance[i] = 1e6
    return msg


def px4(ns):
    msg = VehicleOdometry()
    msg.timestamp_sample = ns // 1000
    msg.timestamp = msg.timestamp_sample + 5000
    msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
    msg.position = [2.1, 1.1, -3.1]
    q = quaternion_multiply_xyzw(yaw_to_quaternion_xyzw(.8), [.1, -.15, 0., .98])
    msg.q = ros_to_px4_quaternion(q).tolist()
    msg.orientation_variance = [.01, .02, .03]
    msg.position_variance = [.1, .2, .3]
    msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
    msg.velocity = [1., 2., -3.]
    msg.velocity_variance = [.1, .2, .3]
    return msg


def test_navigation_uses_full_px4_pose_ev_uses_only_independent_position(pipeline):
    nav, ev = pipeline
    ns = nav.get_clock().now().nanoseconds - 50_000_000
    measurement, state = uwb(ns), px4(ns)
    nav._uwb_pose_callback(measurement)
    ev._pose_callback(measurement)
    nav._vehicle_odometry_callback(state)
    nav._publish_state()
    odom, = nav.odom_pub.messages
    assert [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z] == pytest.approx([1.1, 2.1, 3.1])
    assert np.allclose(ev.visual_odom_pub.messages[-1].position, [2., 1., -3.])
    q = odom.pose.pose.orientation
    assert abs(np.dot([q.x, q.y, q.z, q.w], px4_to_ros_quaternion(state.q))) == pytest.approx(1.)
    assert stamp_ns(odom.header.stamp) == state.timestamp_sample * 1000
    assert ev.visual_odom_pub.messages[-1].timestamp_sample == ns // 1000
    assert all(np.isnan(ev.visual_odom_pub.messages[-1].q))
    # Publishing navigation, or receiving more PX4 estimates, cannot generate EV.
    state.timestamp_sample += 1000
    nav._vehicle_odometry_callback(state)
    nav._publish_state()
    assert len(ev.visual_odom_pub.messages) == 1
    assert nav._has_independent_position()
    assert nav.input.reset_counter == 0


def test_idle_publish_preserves_stamp_stop_degrades_covariance(pipeline):
    nav, _ = pipeline
    ns = nav.get_clock().now().nanoseconds - 50_000_000
    nav._uwb_pose_callback(uwb(ns))
    nav._vehicle_odometry_callback(px4(ns))
    nav._publish_state()
    nav._publish_state()
    assert nav.odom_pub.messages[0].header.stamp == nav.odom_pub.messages[1].header.stamp
    nav._uwb_health_callback(String(data=json.dumps({'kind':'session_ended'})))
    nav._publish_state()
    assert not nav._has_independent_position()
    assert nav.odom_pub.messages[-1].pose.covariance[0] >= 25.
    assert nav.odom_pub.messages[-1].pose.covariance[14] >= 9.
    assert nav.odom_pub.messages[-1].header.stamp == nav.odom_pub.messages[0].header.stamp


@pytest.mark.parametrize('rejection', [
    {'accepted': False}, {'kind': 'observation_rejected'},
])
def test_new_rejected_measurement_preserves_only_original_good_sample_ttl(
        pipeline, monkeypatch, rejection):
    nav, ev = pipeline
    current = [nav.get_clock().now().nanoseconds]
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=current[0]))
    for node in (nav, ev):
        monkeypatch.setattr(node, 'get_clock', lambda: clock)
    good = current[0] - 50_000_000
    rejected = good + 10_000_000
    nav._uwb_pose_callback(uwb(good))
    ev._pose_callback(uwb(good))
    nav._vehicle_odometry_callback(px4(good))
    event = String(data=json.dumps({**rejection, 'sample_stamp_ns': rejected,
                                   'reason': 'range residual rejected'}))
    nav._uwb_health_callback(event)
    ev._health_callback(event)
    assert nav._has_independent_position()
    assert ev.input.healthy(current[0])
    assert nav.input.last_sample_ns == ev.input.last_sample_ns == good
    assert nav.input.invalid_before_ns == ev.input.invalid_before_ns == rejected
    nav._publish_state()
    assert nav.odom_pub.messages[-1].pose.covariance[0] < 25.
    assert stamp_ns(nav.odom_pub.messages[-1].header.stamp) == good // 1000 * 1000

    # The rejected pose cannot arrive later and replace the last good sample.
    nav._uwb_pose_callback(uwb(rejected))
    ev._pose_callback(uwb(rejected))
    assert nav.input.last_sample_ns == ev.input.last_sample_ns == good
    assert nav._has_independent_position()
    assert len(ev.visual_odom_pub.messages) == 1
    assert ev.visual_odom_pub.messages[0].timestamp_sample == good // 1000

    assert nav.input.timeout_ns == ev.input.timeout_ns
    current[0] = good + nav.input.timeout_ns
    assert nav.input.healthy(current[0]) and ev.input.healthy(current[0])
    current[0] += 1
    assert not nav._has_independent_position()
    assert not ev.input.healthy(current[0])
    nav._vehicle_odometry_callback(px4(current[0] - 1_000_000))
    nav._publish_state()
    assert nav.odom_pub.messages[-1].pose.covariance[0] >= 25.
    assert len(ev.visual_odom_pub.messages) == 1  # No fresh EV from old evidence.


@pytest.mark.parametrize('rejection', [
    {'accepted': False}, {'kind': 'observation_rejected'},
])
def test_rejection_of_current_used_measurement_revokes_until_new_pose(pipeline, rejection):
    nav, ev = pipeline
    sample = nav.get_clock().now().nanoseconds - 50_000_000
    nav._uwb_pose_callback(uwb(sample))
    ev._pose_callback(uwb(sample))
    event = String(data=json.dumps({**rejection, 'sample_stamp_ns': sample}))
    nav._uwb_health_callback(event)
    ev._health_callback(event)
    assert not nav._has_independent_position()
    assert not ev.input.valid
    # A positive quality result and replayed pose do not resurrect it.
    positive = String(data=json.dumps({'accepted': True, 'sample_stamp_ns': sample}))
    nav._uwb_health_callback(positive)
    ev._health_callback(positive)
    nav._uwb_pose_callback(uwb(sample))
    ev._pose_callback(uwb(sample))
    assert not nav._has_independent_position()
    assert not ev.input.valid
    assert len(ev.visual_odom_pub.messages) == 1
    nav._uwb_pose_callback(uwb(sample + 1_000_000))
    ev._pose_callback(uwb(sample + 1_000_000))
    assert nav._has_independent_position()
    assert ev.input.valid
    assert len(ev.visual_odom_pub.messages) == 2


@pytest.mark.parametrize('failure', [
    {'kind': 'device_fault'}, {'kind': 'clock_rejected'},
    {'kind': 'session_ended'}, {'kind': 'stale'},
    {'accepted': False}, {'kind': 'observation_rejected'},
])
def test_hard_or_unstamped_negative_event_still_revokes_fresh_uwb(pipeline, failure):
    nav, ev = pipeline
    sample = nav.get_clock().now().nanoseconds - 50_000_000
    nav._uwb_pose_callback(uwb(sample))
    ev._pose_callback(uwb(sample))
    event = String(data=json.dumps(failure))
    nav._uwb_health_callback(event)
    ev._health_callback(event)
    assert not nav._has_independent_position()
    assert not ev.input.valid
    assert nav.input.last_sample_ns == ev.input.last_sample_ns == sample
    nav._uwb_pose_callback(uwb(sample))
    ev._pose_callback(uwb(sample))
    assert len(ev.visual_odom_pub.messages) == 1
    assert not nav._has_independent_position()


def test_health_epoch_changes_on_estimator_reset_without_retiming_samples(pipeline):
    nav, _ = pipeline
    sample = nav.get_clock().now().nanoseconds - 50_000_000
    nav._uwb_pose_callback(uwb(sample))
    nav._vehicle_odometry_callback(px4(sample))
    before = nav._status()
    repeated = nav._status()
    assert repeated['localization_epoch'] == before['localization_epoch'] > 0
    assert repeated['status_sequence'] > before['status_sequence']
    assert repeated['navigation_sample_ns'] == before['navigation_sample_ns']
    state = px4(sample + 1_000_000)
    state.reset_counter = 1
    nav._vehicle_odometry_callback(state)
    after = nav._status()
    assert after['localization_epoch'] != before['localization_epoch']
    assert after['status_sequence'] == 1
    assert after['navigation_sample_ns'] == state.timestamp_sample * 1000
    assert after['status_stamp_ns'] >= after['navigation_sample_ns']
    assert not after['has_independent_position']


def test_stale_or_unknown_px4_pose_never_becomes_fresh_navigation(pipeline):
    nav, _ = pipeline
    ns = nav.get_clock().now().nanoseconds
    msg = px4(ns - 2_000_000_000)
    nav._vehicle_odometry_callback(msg)
    nav._publish_state()
    assert not nav.odom_pub.messages
    msg = px4(ns - 50_000_000)
    msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
    nav._vehicle_odometry_callback(msg)
    assert nav.navigation_odom is None
    msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
    msg.orientation_variance = [math.nan] * 3
    nav._vehicle_odometry_callback(msg)
    assert nav.navigation_odom is None


def test_single_tag_cannot_erase_fresh_px4_tilt_in_adaptive_mode(pipeline):
    nav, _ = pipeline
    nav.navigation_mode = 'adaptive'
    nav.set_parameters([Parameter('use_uwb_orientation', value=False)])
    ns = nav.get_clock().now().nanoseconds - 50_000_000
    msg = px4(ns)
    nav._vehicle_odometry_callback(msg)
    nav._uwb_pose_callback(uwb(ns))
    assert nav.state.has_tilt
    q = nav.state.orientation_xyzw
    assert abs(np.dot(q, px4_to_ros_quaternion(msg.q))) == pytest.approx(1.)


def test_full_attitude_basis_transform_is_not_only_yaw_conversion():
    # Independent vector equivalence: ENU←NED × R_PX4 × FRD←FLU.
    q = [.81, .21, -.32, .43]
    converted = px4_to_ros_quaternion(q)
    r_world = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
    r_body = np.diag([1., -1., -1.])
    for axis in np.eye(3):
        expected = r_world @ rotate_vector_xyzw([q[1], q[2], q[3], q[0]], r_body @ axis)
        assert rotate_vector_xyzw(converted, axis) == pytest.approx(expected)


@pytest.mark.parametrize('independent_attitude', [False, True])
def test_ros_executor_routes_real_topics_without_navigation_feedback(independent_attitude):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from px4_powerplant_mission.common.qos import px4_qos_profile, reliable_qos_profile
    import time

    rclpy.init(args=[])
    nav, ev, source = LocalizationNode(), ExternalVisionBridgeNode(), Node('input_contract_source')
    ev.set_parameters([Parameter('use_independent_orientation', value=independent_attitude)])
    executor = SingleThreadedExecutor()
    for node in (nav, ev, source):
        executor.add_node(node)
    up = source.create_publisher(PoseWithCovarianceStamped, '/uwb/pose', reliable_qos_profile())
    pp = source.create_publisher(VehicleOdometry, '/fmu/out/vehicle_odometry', px4_qos_profile())
    received_nav, received_ev = [], []
    source.create_subscription(Odometry, '/powerplant/localization/odom', received_nav.append, reliable_qos_profile())
    source.create_subscription(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', received_ev.append, px4_qos_profile())
    try:
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline and not (received_nav and received_ev):
            ns = source.get_clock().now().nanoseconds - 20_000_000
            u = uwb(ns)
            if independent_attitude:
                for i in (21, 28, 35):
                    u.pose.covariance[i] = .02
            up.publish(u)
            pp.publish(px4(ns))
            for _ in range(10):
                executor.spin_once(timeout_sec=.01)
        assert received_nav and received_ev
        assert all(np.isnan(received_ev[-1].q)) != independent_attitude
        assert received_ev[-1].timestamp - received_ev[-1].timestamp_sample >= 19000
        assert received_nav[-1].pose.covariance[21] < .5
        assert not any(s.topic_name == '/powerplant/localization/odom' for s in ev.subscriptions)
    finally:
        executor.shutdown()
        for node in (nav, ev, source):
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('profile,expected', [('simulation', True), ('hardware', False)])
def test_launch_profile_disables_simulation_sources_for_hardware(profile, expected):
    import importlib.util
    from pathlib import Path
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    path = Path(__file__).parents[1] / 'launch/powerplant_system.launch.py'
    spec = importlib.util.spec_from_file_location('shared_localization_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['localization_profile'] = profile
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    for name in ('use_uwb', 'use_gazebo_truth_bridge', 'use_gazebo_clock_bridge', 'use_sim_time'):
        assert (LaunchConfiguration(name).perform(context).lower() == 'true') == expected
    assert LaunchConfiguration('use_control').perform(context) == 'false'
