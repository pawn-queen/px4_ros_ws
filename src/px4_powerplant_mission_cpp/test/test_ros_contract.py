"""Sequential, bounded ROS wire checks. No PX4/Gazebo or vehicle is started."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Image, CameraInfo
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLandDetected, VehicleOdometry

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = Path(__file__).resolve().parent / "fixtures"
BUILD = Path(os.environ.get("POWERPLANT_CPP_BUILD", ROOT.parent / "build/px4_powerplant_mission_cpp"))
SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
PX4 = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture
def transport():
    rclpy.init()
    node = Node("native_contract_test")
    processes = []

    def launch(name, *parameters):
        command = ([sys.executable, "-m", name[7:], "--ros-args"] if name.startswith("python:")
                   else [str(BUILD / name), "--ros-args"])
        for name_value in parameters:
            command += (["--params-file", name_value[1:]] if name_value.startswith("@")
                        else ["-p", name_value])
        process = subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        processes.append(process)
        return process

    def wait(predicate, timeout=5.0, pump=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pump:
                pump()
            rclpy.spin_once(node, timeout_sec=.03)
            assert all(p.poll() is None for p in processes), "native node exited unexpectedly"
            if predicate():
                return
        pytest.fail("timed out waiting for ROS contract")

    yield node, launch, wait
    errors = []
    for process in processes:
        process.send_signal(signal.SIGINT)
        try:
            _, error = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            _, error = process.communicate()
            errors.append("native shutdown hung")
        python_interrupt = len(process.args) > 1 and process.args[1] == "-m" and process.returncode == -signal.SIGINT
        if process.returncode != 0 and not python_interrupt:
            errors.append(error)
    node.destroy_node()
    rclpy.shutdown()
    assert not errors, "\n".join(errors)


def odometry(stamp=None, z=5.0):
    if stamp is None:
        stamp = time.time()
    m = Odometry()
    m.header.stamp.sec = int(stamp)
    m.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    m.header.frame_id = "map"
    m.child_frame_id = "base_link"
    m.pose.pose.position.z = z
    m.pose.pose.orientation.w = 1.0
    for k in (0, 7, 14, 21, 28, 35):
        m.pose.covariance[k] = .001
    for k in (0, 7, 14):
        m.twist.covariance[k] = .001
    return m


def health(node, publisher, sample_ns, sequence, epoch=1, healthy=True):
    publisher.publish(String(data=json.dumps({
        "status_stamp_ns": node.get_clock().now().nanoseconds,
        "status_sequence": sequence, "localization_epoch": epoch,
        "navigation_sample_ns": sample_ns, "px4_reset_counter": 0,
        "uwb_input_healthy": healthy, "has_independent_position": healthy,
    })))


@pytest.mark.parametrize("executable,node_name", [
    ("voxel_mapper_node", "powerplant_voxel_mapper"),
    ("offboard_mission_node", "powerplant_offboard_mission"),
])
def test_all_parameter_defaults_match_frozen_baseline(transport, executable, node_name):
    fixture = json.loads((FIXTURES / "parameter_defaults.json").read_text())["nodes"][executable]
    assert fixture["node"] == node_name
    defaults = fixture["parameters"]
    node, launch, wait = transport
    launch(executable)
    client = AsyncParameterClient(node, node_name)
    wait(client.services_are_ready)
    future = client.get_parameters(list(defaults))
    wait(future.done)
    for (name, spec), actual in zip(defaults.items(), future.result().values):
        expected = spec["value"]
        if spec["type"] == "bool":
            value = actual.bool_value
        elif spec["type"] == "int":
            value = actual.integer_value
        elif spec["type"] == "double":
            value = actual.double_value
        elif spec["type"] == "string":
            value = actual.string_value
        elif spec["type"] == "double_array":
            value = list(actual.double_array_value)
        elif spec["type"] == "string_array":
            value = list(actual.string_array_value)
        else:
            raise AssertionError(f"unsupported fixture type {spec['type']} for {name}")
        assert value == expected, name


@pytest.mark.parametrize("profile", ["p2_simulation.yaml", "p2_hardware.yaml"])
def test_ordinary_motion_profile_loads_idle_without_audited_ground_authority(transport, profile):
    node, launch, wait = transport
    statuses = []
    node.create_subscription(String, "/powerplant/control/mission_status",
                             lambda m: statuses.append(json.loads(m.data)), 10)
    config = ROOT / "px4_powerplant_mission/config"
    launch("offboard_mission_node", "@"+str(config/"powerplant.yaml"),
           "@"+str(config/profile), "use_sim_time:=false")
    wait(lambda: bool(statuses))
    assert statuses[-1]["phase"] == "IDLE"
    assert not statuses[-1]["takeoff_ground_support_enabled"]


def test_mapper_stamp_fields_counts_and_cells(transport):
    node, launch, wait = transport
    clouds, statuses, grids = {}, {}, []
    node.create_subscription(PointCloud2, "/powerplant/map/observed_voxels", lambda m: clouds.__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m), 10)
    node.create_subscription(String, "/powerplant/map/fusion_status", lambda m: statuses.__setitem__(json.loads(m.data)["map_stamp_ns"], json.loads(m.data)), 10)
    node.create_subscription(OccupancyGrid, "/powerplant/map/occupancy_grid", grids.append, 10)
    op = node.create_publisher(Odometry, "/powerplant/localization/odom", 10)
    hp = node.create_publisher(String, "/powerplant/localization/status", 10)
    sp = node.create_publisher(LaserScan, "/scan", SENSOR)
    dp = node.create_publisher(Image, "/depth_camera", SENSOR)
    cp = node.create_publisher(CameraInfo, "/depth_camera/camera_info", SENSOR)
    launch("voxel_mapper_node", "startup_gate_enabled:=false", "publish_rate_hz:=5.0", "use_fixed_grid_bounds:=true")
    wait(lambda: op.get_subscription_count() and hp.get_subscription_count() and sp.get_subscription_count() and dp.get_subscription_count() and cp.get_subscription_count())
    base = time.time() - .25
    first_pose = odometry(base)
    first_sample = first_pose.header.stamp.sec * 10**9 + first_pose.header.stamp.nanosec
    health(node, hp, first_sample, 1)
    op.publish(first_pose)
    op.publish(odometry(base + .2))
    # Give the two bracketing odometry samples time to arrive in order.
    end = time.monotonic() + .15
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
    scan = LaserScan()
    scan.header.stamp = odometry(base + .1).header.stamp
    scan.header.frame_id = "base_link"
    scan.range_min, scan.range_max, scan.angle_increment = .1, 10.0, .5
    scan.ranges = [2.0, 3.0]
    sp.publish(scan)
    info = CameraInfo()
    info.header = scan.header
    info.header.frame_id = "camera_optical"
    info.width = info.height = 1
    info.k = [100., 0., 0., 0., 100., 0., 0., 0., 1.]
    cp.publish(info)
    image = Image()
    image.header = info.header
    image.height = image.width = 1
    image.encoding = "32FC1"
    image.step = 4
    image.data = np.array([[2.0]], dtype=np.float32).tobytes()
    dp.publish(image)

    def paired():
        for stamp, cloud in clouds.items():
            status = statuses.get(stamp[0]*10**9 + stamp[1])
            if status and status["lidar_fused_count"] == 1 and status["depth_fused_count"] == 1:
                return cloud, status
        return None

    sequence = 1
    def refresh_pose():
        nonlocal sequence
        sequence += 1
        m = odometry()
        health(node, hp, m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, sequence)
        op.publish(m)
    wait(lambda: paired() is not None and grids, pump=refresh_pose)
    cloud, status = paired()
    assert cloud.header.frame_id == "map"
    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        (name, j*4, PointField.FLOAT32, 1) for j, name in enumerate(("x", "y", "z", "state"))]
    expected = json.loads((FIXTURES / "mapper_case.json").read_text())
    actual = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z", "state"))
    assert sorted(map(tuple, actual.tolist())) == sorted(map(tuple, expected["expected_points"]))
    assert status["observed_voxels_complete"] == expected["expected_complete"]
    assert status["observed_voxel_count"] == expected["expected_observed_count"]
    sample = scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec
    assert status["depth_last_sample_stamp_ns"] == sample
    assert status["lidar_last_sample_stamp_ns"] == sample
    assert status["localization_healthy"] and status["map_epoch"] > 0
    assert status["observed_cloud_fingerprint"]
    assert status["map_revision"] == expected["expected_revision"]
    assert status["camera_intrinsics"]["camera_info_valid"]
    assert status["camera_intrinsics"]["fx"] == 100.
    observation = status["depth_observations"][-1]
    assert observation["sample_id"] == 1 and observation["sample_stamp_ns"] == sample
    assert observation["sampled_pixel_count"] == 1
    assert observation["valid_yaw_sectors"] == [36]
    assert observation["coverage_qualified"]


def test_offboard_services_qos_warmup_commands_and_ned_setpoint(transport):
    node, launch, wait = transport
    statuses, heartbeats, setpoints, commands = [], [], [], []
    node.create_subscription(String, "/powerplant/control/mission_status", lambda m: statuses.append(json.loads(m.data)), 10)
    node.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode", heartbeats.append, PX4)
    node.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", setpoints.append, PX4)
    node.create_subscription(VehicleCommand, "/fmu/in/vehicle_command", commands.append, PX4)
    op = node.create_publisher(Odometry, "/powerplant/localization/odom", 10)
    hp = node.create_publisher(String, "/powerplant/localization/status", 10)
    vp = node.create_publisher(VehicleStatus, "/fmu/out/vehicle_status", PX4)
    lp = node.create_publisher(VehicleLandDetected, "/fmu/out/vehicle_land_detected", PX4)
    launch("offboard_mission_node", "offboard_warmup_setpoints:=3", "command_retry_interval_s:=0.1",
           "motion_tracking_position_tolerance_m:=0.1", "motion_model_configured:=true", "offboard_warmup_min_s:=0.3", "takeoff_region_confirmed:=true", "takeoff_region_height_m:=6.0", "landing_confirm_dwell_s:=0.1")
    start = node.create_client(Trigger, "/start_powerplant_mission")
    land_service = node.create_client(Trigger, "/land_powerplant_mission")
    wait(lambda: start.service_is_ready() and land_service.service_is_ready() and statuses)
    assert statuses[-1]["phase"] == "IDLE" and not commands and not heartbeats
    assert statuses[-1]["sitl_controller_profile_id"] == ""
    assert statuses[-1]["home_enu_m"] is None
    assert not statuses[-1]["motion_tracking_recovery_active"]
    reply = start.call_async(Trigger.Request())
    wait(reply.done)
    assert not reply.result().success
    vehicle = VehicleStatus()
    vehicle.arming_state = VehicleStatus.ARMING_STATE_DISARMED
    vehicle.pre_flight_checks_pass = True
    sequence = 0
    landed = False
    high_velocity_uncertainty = True

    def pump():
        nonlocal sequence
        sequence += 1
        m = odometry(z=10.)  # A raised platform verifies relative takeoff.
        m.pose.pose.position.x, m.pose.pose.position.y = 1., 2.
        if high_velocity_uncertainty:
            for index in (0, 7, 14):
                m.twist.covariance[index] = .16
        sample = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        health(node, hp, sample, sequence)
        op.publish(m)
        vehicle.timestamp = sample // 1000
        vp.publish(vehicle)
        detector = VehicleLandDetected()
        detector.timestamp = vehicle.timestamp
        detector.landed = landed
        lp.publish(detector)

    wait(lambda: statuses[-1]["localization_ready"] and statuses[-1]["vehicle_status_fresh"], pump=pump)
    reply = start.call_async(Trigger.Request())
    wait(reply.done, pump=pump)
    assert reply.result().success
    policy_client = AsyncParameterClient(node, "powerplant_offboard_mission")
    wait(policy_client.services_are_ready, pump=pump)
    from rclpy.parameter import Parameter
    change = policy_client.set_parameters_atomically([
        Parameter("motion_tracking_soft_guard_enabled", value=True),
        Parameter("sitl_controller_profile_id", value="late-profile")])
    wait(change.done, pump=pump)
    assert not change.result().result.successful
    assert "IDLE" in change.result().result.reason
    # Waiting without a first safe certificate is not Offboard prestream time.
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        pump()
        rclpy.spin_once(node, timeout_sec=.02)
    assert not setpoints and not commands
    high_velocity_uncertainty = False
    wait(lambda: any(c.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE for c in commands), pump=pump)
    first_mode = next(c for c in commands if c.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE)
    assert first_mode.timestamp - setpoints[0].timestamp >= 280_000
    assert all(c.command != VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM for c in commands)
    assert len(setpoints) >= 3
    assert setpoints[-1].position[0] == pytest.approx(2.)
    assert setpoints[-1].position[1] == pytest.approx(1.)
    assert setpoints[-1].position[2] <= -10.
    assert all(np.isfinite(v) for v in setpoints[-1].velocity)
    assert all(np.isfinite(v) for v in setpoints[-1].acceleration)
    assert np.isfinite(setpoints[-1].yawspeed)
    assert statuses[-1]["motion_stop_certificate_valid"]
    assert statuses[-1]["home_enu_m"] == pytest.approx([1., 2., 10.])
    assert statuses[-1]["takeoff_target_enu_m"] == pytest.approx([1., 2., 15.])
    assert statuses[-1]["return_home_target_enu_m"] == pytest.approx([1., 2., 15.])
    assert "motion_tracking_risk_ratio" in statuses[-1]
    assert len(statuses[-1]["linear_velocity_covariance_enu_m2ps2"]) == 9
    assert all(np.isfinite(v) for v in statuses[-1]["linear_velocity_covariance_enu_m2ps2"])
    envelope = statuses[-1]["motion_stopping_envelope_enu"]
    assert len(envelope["minimum_m"]) == len(envelope["maximum_m"]) == 3
    assert all(lo <= hi for lo, hi in zip(envelope["minimum_m"], envelope["maximum_m"]))
    margin = statuses[-1]["motion_stop_margin_lower_bound_m"]
    assert margin is None or margin >= 0  # No map evidence means unknown extra clearance.
    vehicle.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    wait(lambda: any(c.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM for c in commands), pump=pump)
    response = land_service.call_async(Trigger.Request())
    wait(response.done, pump=pump)
    assert response.result().message == "land command sent"
    wait(lambda: any(c.command == VehicleCommand.VEHICLE_CMD_NAV_LAND for c in commands))
    assert statuses[-1]["phase"] != "COMPLETE"
    landed = True
    wait(lambda: statuses[-1]["phase"] == "COMPLETE", pump=pump)
    assert all(c.target_system == 1 and c.from_external for c in commands)


def test_replayed_navigation_cannot_keep_active_mission_alive(transport):
    node, launch, wait = transport
    statuses, setpoints = [], []
    node.create_subscription(String, "/powerplant/control/mission_status", lambda m: statuses.append(json.loads(m.data)), 10)
    node.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", setpoints.append, PX4)
    op = node.create_publisher(Odometry, "/powerplant/localization/odom", 10)
    hp = node.create_publisher(String, "/powerplant/localization/status", 10)
    vp = node.create_publisher(VehicleStatus, "/fmu/out/vehicle_status", PX4)
    launch("offboard_mission_node", "motion_tracking_position_tolerance_m:=0.1", "motion_model_configured:=true", "offboard_warmup_min_s:=0.3", "takeoff_region_confirmed:=true", "takeoff_region_height_m:=6.0")
    start = node.create_client(Trigger, "/start_powerplant_mission")
    sequence = 0
    last = None
    replay = False

    def pump():
        nonlocal sequence, last
        sequence += 1
        if not replay:
            last = odometry(z=0.)
        sample = last.header.stamp.sec * 10**9 + last.header.stamp.nanosec
        health(node, hp, sample, sequence)
        op.publish(last)
        vehicle = VehicleStatus()
        vehicle.timestamp = node.get_clock().now().nanoseconds // 1000
        vehicle.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        vehicle.pre_flight_checks_pass = True
        vp.publish(vehicle)

    wait(lambda: start.service_is_ready() and statuses, pump=pump)
    wait(lambda: statuses[-1]["localization_ready"], pump=pump)
    reply = start.call_async(Trigger.Request())
    wait(reply.done, pump=pump)
    assert reply.result().success
    wait(lambda: statuses[-1]["phase"] == "SAFE_TAKEOFF" and setpoints, pump=pump)
    replay = True
    wait(lambda: statuses[-1]["phase"] == "FAULT", pump=pump)
    # Allow in-flight DDS packets to drain, then verify new commands stop.
    end = time.monotonic() + .2
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
    count = len(setpoints)
    end = time.monotonic() + .25
    while time.monotonic() < end:
        pump()
        rclpy.spin_once(node, timeout_sec=.02)
    assert len(setpoints) == count
    replay = False
    end = time.monotonic() + .2
    while time.monotonic() < end:
        pump()
        rclpy.spin_once(node, timeout_sec=.02)
    assert statuses[-1]["phase"] == "FAULT"


@pytest.mark.parametrize("profile", ["simulation", "hardware"])
def test_localization_and_ev_use_independent_sources_across_processes(transport, profile):
    node, launch, wait = transport
    navigation, external, health_status = [], [], []
    node.create_subscription(Odometry, "/powerplant/localization/odom", navigation.append, 10)
    node.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", external.append, PX4)
    node.create_subscription(String, "/powerplant/localization/status", lambda m: health_status.append(json.loads(m.data)), 10)
    topic = "/uwb/hardware/pose" if profile == "hardware" else "/uwb/pose"
    uwb_pub = node.create_publisher(PoseWithCovarianceStamped, topic, 10)
    px4_pub = node.create_publisher(VehicleOdometry, "/fmu/out/vehicle_odometry", PX4)
    config = "@" + str(ROOT / "px4_powerplant_mission/config" / ("localization_" + profile + ".yaml"))
    launch("python:px4_powerplant_mission.nodes.localization_node", config, "publish_tf:=false")
    launch("python:px4_powerplant_mission.nodes.external_vision_bridge_node", config)
    publish_uwb = True

    def pump():
        ns = node.get_clock().now().nanoseconds - 20_000_000
        if publish_uwb:
            measurement = PoseWithCovarianceStamped()
            measurement.header.stamp.sec, measurement.header.stamp.nanosec = divmod(ns, 10**9)
            measurement.header.frame_id = "map"
            measurement.pose.pose.position.x = .9
            measurement.pose.pose.position.y = 1.9
            measurement.pose.pose.position.z = 2.9
            measurement.pose.pose.orientation.w = 1.
            for index in (0, 7, 14):
                measurement.pose.covariance[index] = .1
            for index in (21, 28, 35):
                measurement.pose.covariance[index] = 1e6 if profile == "hardware" else .1
            uwb_pub.publish(measurement)
        state = VehicleOdometry()
        state.timestamp_sample = ns // 1000
        state.timestamp = state.timestamp_sample
        state.pose_frame = VehicleOdometry.POSE_FRAME_NED
        state.position = [2., 1., -3.]
        state.position_variance = [.1, .1, .1]
        state.q = [1., 0., 0., 0.]
        state.orientation_variance = [.01, .02, .03]
        state.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        state.velocity = [0., 0., 0.]
        state.velocity_variance = [.1, .1, .1]
        px4_pub.publish(state)

    wait(lambda: navigation and external and health_status and health_status[-1]["has_independent_position"],
         timeout=8., pump=pump)
    p = navigation[-1].pose.pose.position
    assert [p.x, p.y, p.z] == pytest.approx([1., 2., 3.])
    assert external[-1].position == pytest.approx([1.9, .9, -2.9])
    assert health_status[-1]["localization_epoch"] > 0
    assert all(np.isnan(v) for v in external[-1].q) if profile == "hardware" else all(np.isfinite(v) for v in external[-1].q)
    publish_uwb = False
    wait(lambda: not health_status[-1]["has_independent_position"] and
         navigation[-1].pose.covariance[0] >= 25., timeout=3., pump=pump)
    # After UWB is stale and all its packets drained, fresh PX4 navigation must
    # not create new EV measurements (the feedback loop prohibited by design).
    count = len(external)
    deadline = time.monotonic() + .2
    while time.monotonic() < deadline:
        pump()
        rclpy.spin_once(node, timeout_sec=.02)
    assert len(external) == count
