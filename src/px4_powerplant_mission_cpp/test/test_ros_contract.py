"""Sequential, bounded ROS wire checks. No PX4/Gazebo or vehicle is started."""
import ast
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter_client import AsyncParameterClient
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Image, CameraInfo
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
from px4_powerplant_mission.mapping.voxel_grid import VoxelGrid

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path(os.environ.get("POWERPLANT_CPP_BUILD", ROOT.parent / "build/px4_powerplant_mission_cpp"))
SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
PX4 = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL)


@pytest.fixture
def transport():
    rclpy.init()
    node = Node("native_contract_test")
    processes = []

    def launch(name, *parameters):
        command = [str(BUILD / name), "--ros-args"]
        for name_value in parameters:
            command += ["-p", name_value]
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
    for process in processes:
        process.send_signal(signal.SIGINT)
        try:
            _, error = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            _, error = process.communicate()
            pytest.fail("native shutdown hung")
        assert process.returncode == 0, error
    node.destroy_node()
    rclpy.shutdown()


def odometry(stamp=1.0, z=5.0):
    m = Odometry()
    m.header.stamp.sec = int(stamp)
    m.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    m.header.frame_id = "map"
    m.child_frame_id = "base_link"
    m.pose.pose.position.z = z
    m.pose.pose.orientation.w = 1.0
    for k in (0, 7, 14, 21, 28):
        m.pose.covariance[k] = .1
    return m


@pytest.mark.parametrize("executable,node_name", [
    ("voxel_mapper_node", "powerplant_voxel_mapper"),
    ("offboard_mission_node", "powerplant_offboard_mission"),
])
def test_all_parameter_defaults_match_retained_python(transport, executable, node_name):
    node, launch, wait = transport
    launch(executable)
    client = AsyncParameterClient(node, node_name)
    wait(client.services_are_ready)
    source = ROOT / "px4_powerplant_mission/px4_powerplant_mission/nodes" / f"{executable}.py"
    tree = ast.parse(source.read_text())
    declaration = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_declare_parameters")
    defaults = {ast.literal_eval(stmt.value.args[0]): ast.literal_eval(stmt.value.args[1]) for stmt in declaration.body}
    future = client.get_parameters(list(defaults))
    wait(future.done)
    for (name, expected), actual in zip(defaults.items(), future.result().values):
        if isinstance(expected, bool):
            value = actual.bool_value
        elif isinstance(expected, int):
            value = actual.integer_value
        elif isinstance(expected, float):
            value = actual.double_value
        elif isinstance(expected, str):
            value = actual.string_value
        else:
            value = list(actual.double_array_value)
        assert value == expected, name


def test_mapper_stamp_fields_counts_and_cells(transport):
    node, launch, wait = transport
    clouds, statuses, grids = {}, {}, []
    node.create_subscription(PointCloud2, "/powerplant/map/observed_voxels", lambda m: clouds.__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m), 10)
    node.create_subscription(String, "/powerplant/map/fusion_status", lambda m: statuses.__setitem__(json.loads(m.data)["map_stamp_ns"], json.loads(m.data)), 10)
    node.create_subscription(OccupancyGrid, "/powerplant/map/occupancy_grid", grids.append, 10)
    op = node.create_publisher(Odometry, "/powerplant/localization/odom", 10)
    sp = node.create_publisher(LaserScan, "/scan", SENSOR)
    dp = node.create_publisher(Image, "/depth_camera", SENSOR)
    cp = node.create_publisher(CameraInfo, "/depth_camera/camera_info", SENSOR)
    launch("voxel_mapper_node", "startup_gate_enabled:=false", "publish_rate_hz:=5.0", "use_fixed_grid_bounds:=true")
    wait(lambda: op.get_subscription_count() and sp.get_subscription_count() and dp.get_subscription_count() and cp.get_subscription_count())
    op.publish(odometry(1.0))
    op.publish(odometry(1.2))
    # Give the two bracketing odometry samples time to arrive in order.
    end = time.monotonic() + .15
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
    scan = LaserScan()
    scan.header.stamp = odometry(1.1).header.stamp
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

    wait(lambda: paired() is not None and grids)
    cloud, status = paired()
    assert cloud.header.frame_id == "map"
    assert [(f.name, f.offset, f.datatype, f.count) for f in cloud.fields] == [
        (name, j*4, PointField.FLOAT32, 1) for j, name in enumerate(("x", "y", "z", "state"))]
    expected = VoxelGrid(.25)
    origin = np.array([0., 0., 5.])
    expected.insert_rays(origin, [origin + [2., 0., 0.], origin + [3*np.cos(.5), 3*np.sin(.5), 0.]])
    camera = origin + [.12, -.03, .24]
    expected.insert_rays(camera, [camera + [2., 0., 0.]])
    actual = point_cloud2.read_points_numpy(cloud, field_names=("x", "y", "z", "state"))
    exp = expected.observed_snapshot().points
    assert sorted(map(tuple, actual.tolist())) == sorted(map(tuple, exp.tolist()))
    assert status["observed_voxels_complete"]
    assert status["observed_voxel_count"] == len(expected.cells)
    assert status["depth_last_sample_stamp_ns"] == 1100000000
    assert status["lidar_last_sample_stamp_ns"] == 1100000000
    assert status["map_revision"] == expected.revision


def test_offboard_services_qos_warmup_commands_and_ned_setpoint(transport):
    node, launch, wait = transport
    statuses, heartbeats, setpoints, commands = [], [], [], []
    node.create_subscription(String, "/powerplant/control/mission_status", lambda m: statuses.append(json.loads(m.data)), 10)
    node.create_subscription(OffboardControlMode, "/fmu/in/offboard_control_mode", heartbeats.append, PX4)
    node.create_subscription(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", setpoints.append, PX4)
    node.create_subscription(VehicleCommand, "/fmu/in/vehicle_command", commands.append, PX4)
    op = node.create_publisher(Odometry, "/powerplant/localization/odom", 10)
    vp = node.create_publisher(VehicleStatus, "/fmu/out/vehicle_status", PX4)
    launch("offboard_mission_node", "offboard_warmup_setpoints:=3", "command_retry_interval_s:=0.1")
    start = node.create_client(Trigger, "/start_powerplant_mission")
    land = node.create_client(Trigger, "/land_powerplant_mission")
    wait(lambda: start.service_is_ready() and land.service_is_ready() and heartbeats and statuses)
    assert statuses[-1]["phase"] == "IDLE"
    assert not commands
    reply = start.call_async(Trigger.Request())
    wait(reply.done)
    assert reply.result().success and reply.result().message == "mission requested: START"
    wait(lambda: statuses[-1]["phase"] == "START")
    assert not commands
    vehicle = VehicleStatus()
    vehicle.pre_flight_checks_pass = True
    odom = odometry(z=0.)
    odom.pose.pose.position.x, odom.pose.pose.position.y = 1., 2.

    def pump():
        op.publish(odom)
        vp.publish(vehicle)

    wait(lambda: any(c.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE for c in commands), pump=pump)
    assert all(c.command != VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM for c in commands)
    assert len(setpoints) >= 3
    assert setpoints[-1].position[0] == pytest.approx(2.)
    assert setpoints[-1].position[1] == pytest.approx(1.)
    assert setpoints[-1].position[2] <= 0.
    assert all(np.isnan(v) for v in setpoints[-1].velocity)
    vehicle.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
    wait(lambda: any(c.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM for c in commands), pump=pump)
    assert statuses[-1]["localization_ready"]
    response = land.call_async(Trigger.Request())
    wait(response.done)
    assert response.result().message == "land command sent"
    wait(lambda: any(c.command == VehicleCommand.VEHICLE_CMD_NAV_LAND for c in commands))
    assert all(c.target_system == 1 and c.from_external for c in commands)
