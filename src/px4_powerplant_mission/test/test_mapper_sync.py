import math

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, LaserScan
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from px4_powerplant_mission.common.frames import yaw_to_quaternion_xyzw
from px4_powerplant_mission.nodes.voxel_mapper_node import (
    VoxelMapperNode,
    _PoseHistory,
    _StampedPose,
)


def _stamp(seconds: float) -> Time:
    nanoseconds = round(seconds * 1_000_000_000)
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


def _odom(seconds: float, x: float, yaw: float = 0.0, z: float = 5.0) -> Odometry:
    msg = Odometry()
    msg.header.stamp = _stamp(seconds)
    msg.header.frame_id = "map"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.z = z
    q = yaw_to_quaternion_xyzw(yaw)
    msg.pose.pose.orientation.x = float(q[0])
    msg.pose.pose.orientation.y = float(q[1])
    msg.pose.pose.orientation.z = float(q[2])
    msg.pose.pose.orientation.w = float(q[3])
    msg.pose.covariance[21] = 0.02
    msg.pose.covariance[28] = 0.02
    return msg


@pytest.fixture
def mapper():
    rclpy.init(args=[])
    node = VoxelMapperNode()
    node.startup_gate_open = True
    node.set_parameters([Parameter("ray_clear_enabled", value=False)])
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_pose_history_interpolates_translation_and_shortest_quaternion_arc():
    history = _PoseHistory()
    history.add(_StampedPose(1_000_000_000, np.array([0.0, 0.0, 0.0]), yaw_to_quaternion_xyzw(0.0)))
    history.add(_StampedPose(1_200_000_000, np.array([2.0, 0.0, 0.0]), yaw_to_quaternion_xyzw(math.pi)))
    halfway = history.at(1_100_000_000)

    assert halfway is not None
    assert np.allclose(halfway.position, [1.0, 0.0, 0.0])
    assert math.isclose(abs(halfway.orientation_xyzw[2]), math.sqrt(0.5), abs_tol=1e-6)
    assert history.at(999_999_999) is None
    assert history.at(1_200_000_001) is None

    history.add(_StampedPose(1_600_000_000, np.array([4.0, 0.0, 0.0]), yaw_to_quaternion_xyzw(math.pi)))
    assert history.at(1_400_000_000) is None


def test_depth_waits_for_bracketing_pose_and_uses_matching_camera_info(mapper):
    mapper._odom_callback(_odom(1.0, 0.0))
    info = CameraInfo()
    info.header.stamp = _stamp(1.1)
    info.header.frame_id = "depth_optical"
    info.width = 1
    info.height = 1
    info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    mapper._camera_info_callback(info)

    image = mapper.bridge.cv2_to_imgmsg(np.array([[1.0]], dtype=np.float32), encoding="32FC1")
    image.header.stamp = _stamp(1.1)
    image.header.frame_id = "depth_optical"
    mapper._depth_callback(image)

    assert len(mapper.pending_measurements) == 1
    assert not mapper.grid.cells

    mapper._odom_callback(_odom(1.2, 2.0))

    assert not mapper.pending_measurements
    assert mapper.grid.key_from_point([2.12, -0.03, 5.24]) in mapper.grid.cells


def test_moving_lidar_scan_uses_each_beam_time(mapper):
    mapper._odom_callback(_odom(2.0, 0.0, yaw=0.0))
    scan = LaserScan()
    scan.header.stamp = _stamp(2.0)
    scan.header.frame_id = "mapping_lidar_link"
    scan.angle_min = 0.0
    scan.angle_increment = 0.0
    scan.time_increment = 0.05
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [2.0, 2.0, 2.0]
    mapper._laser_scan_callback(scan)

    assert len(mapper.pending_measurements) == 1
    mapper._odom_callback(_odom(2.1, 0.0, yaw=math.pi / 2.0))
    keys = set(mapper.grid.cells)

    assert mapper.grid.key_from_point([2.0, 0.0, 5.0]) in keys
    assert mapper.grid.key_from_point([math.sqrt(2.0), math.sqrt(2.0), 5.0]) in keys
    assert mapper.grid.key_from_point([0.0, 2.0, 5.0]) in keys


def test_pointcloud_uses_sample_pose_and_rejects_unknown_frame(mapper):
    mapper._odom_callback(_odom(3.0, 0.0))
    body_header = Header(stamp=_stamp(3.1), frame_id="base_link")
    cloud = point_cloud2.create_cloud_xyz32(body_header, [[1.0, 0.0, 0.0]])
    mapper._pointcloud_callback(cloud)
    mapper._odom_callback(_odom(3.2, 2.0))

    expected = mapper.grid.key_from_point([2.0, 0.0, 5.0])
    assert expected in mapper.grid.cells
    before = len(mapper.grid.cells)

    camera_header = Header(stamp=_stamp(3.1), frame_id="unknown_sensor")
    unknown = point_cloud2.create_cloud_xyz32(camera_header, [[50.0, 0.0, 0.0]])
    mapper._pointcloud_callback(unknown)
    assert len(mapper.grid.cells) == before


def test_sensor_is_dropped_when_pose_gap_is_too_large(mapper):
    mapper._odom_callback(_odom(4.0, 0.0))
    scan = LaserScan()
    scan.header.stamp = _stamp(4.5)
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [2.0]
    mapper._laser_scan_callback(scan)
    mapper._odom_callback(_odom(5.0, 10.0))

    assert not mapper.pending_measurements
    assert not mapper.grid.cells
    assert mapper.depth_fused_count == 0
    assert mapper.lidar_fused_count == 0


def test_clock_rewind_clears_mapping_history_and_voxels(mapper):
    mapper._odom_callback(_odom(5.0, 0.0))
    scan = LaserScan()
    scan.header.stamp = _stamp(5.0)
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0]
    mapper._laser_scan_callback(scan)
    assert mapper.last_selected_scan_stamp_ns == 5_000_000_000

    mapper._odom_callback(_odom(1.0, 0.0))

    assert not mapper.grid.cells
    assert len(mapper.pose_history.samples) == 1
    assert mapper.pose_history.samples[0].stamp_ns == 1_000_000_000
    assert mapper.last_selected_scan_stamp_ns is None
    assert mapper.lidar_fused_count == 0
    assert mapper.lidar_last_sample_stamp_ns == 0

    scan.header.stamp = _stamp(1.0)
    mapper._laser_scan_callback(scan)
    assert mapper.last_selected_scan_stamp_ns == 1_000_000_000
    assert mapper.grid.cells


def test_depth_and_scan_fusion_rates_use_message_stamps(mapper):
    mapper._odom_callback(_odom(6.0, 0.0))
    info = CameraInfo()
    info.header.stamp = _stamp(6.0)
    info.header.frame_id = "depth_optical"
    info.width = 1
    info.height = 1
    info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    mapper._camera_info_callback(info)

    def depth_at(seconds, distance):
        image = mapper.bridge.cv2_to_imgmsg(np.array([[distance]], dtype=np.float32), encoding="32FC1")
        image.header.stamp = _stamp(seconds)
        image.header.frame_id = "depth_optical"
        mapper._depth_callback(image)

    def scan_at(seconds, distance):
        scan = LaserScan()
        scan.header.stamp = _stamp(seconds)
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [distance]
        mapper._laser_scan_callback(scan)

    depth_at(6.0, 1.0)
    scan_at(6.0, 1.0)
    depth_at(6.1, 2.0)
    scan_at(6.1, 2.0)

    assert mapper.grid.key_from_point([1.12, -0.03, 5.24]) in mapper.grid.cells
    assert mapper.grid.key_from_point([1.0, 0.0, 5.0]) in mapper.grid.cells
    assert not mapper.pending_measurements

    mapper._odom_callback(_odom(6.4, 0.0))
    depth_at(6.4, 3.0)
    scan_at(6.4, 3.0)

    assert mapper.grid.key_from_point([3.12, -0.03, 5.24]) in mapper.grid.cells
    assert mapper.grid.key_from_point([3.0, 0.0, 5.0]) in mapper.grid.cells


def test_missing_stamps_and_mismatched_camera_info_do_not_modify_map(mapper):
    mapper._odom_callback(_odom(7.0, 0.0))
    scan = LaserScan()
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0]
    mapper._laser_scan_callback(scan)
    assert not mapper.pending_measurements

    info = CameraInfo()
    info.header.stamp = _stamp(7.1)
    info.header.frame_id = "another_camera"
    info.width = 1
    info.height = 1
    info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    mapper._camera_info_callback(info)
    image = mapper.bridge.cv2_to_imgmsg(np.array([[1.0]], dtype=np.float32), encoding="32FC1")
    image.header.stamp = _stamp(7.1)
    image.header.frame_id = "depth_optical"
    mapper._depth_callback(image)
    mapper._odom_callback(_odom(7.2, 0.0))

    assert not mapper.grid.cells
    assert not mapper.pending_measurements


def test_vectorized_rays_match_voxel_grid_for_random_and_overlapping_hits(mapper):
    from px4_powerplant_mission.mapping.voxel_grid import VoxelGrid

    rng = np.random.default_rng(42)
    origins = [np.array([0.15, -0.2, 5.1]), np.array([-1.1, 0.35, 4.8])]
    for origin in origins:
        hits = rng.uniform([-3.0, -3.0, 3.0], [3.0, 3.0, 7.0], size=(50, 3))
        hits = np.vstack((hits, origin + [1.0, 0.0, 0.0], origin + [2.0, 0.0, 0.0]))
        reference = VoxelGrid(mapper.grid.resolution)
        reference.insert_rays(origin, hits)
        mapper.grid.cells.clear()
        mapper._insert_rays_with_origins(np.broadcast_to(origin, hits.shape), hits)

        assert mapper.grid.cells == reference.cells


def test_future_sensor_queue_stays_bounded_without_new_odometry(mapper):
    mapper.set_parameters([Parameter("lidar_fusion_rate_hz", value=0.0)])
    mapper._odom_callback(_odom(8.0, 0.0))
    for index in range(40):
        scan = LaserScan()
        scan.header.stamp = _stamp(8.01 + index * 0.01)
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [1.0]
        mapper._laser_scan_callback(scan)

    assert len(mapper.pending_measurements) == 32
    assert not mapper.grid.cells


def test_delayed_pre_gate_scan_is_not_inserted_after_takeoff_gate_opens(mapper):
    mapper.startup_gate_open = False
    mapper._odom_callback(_odom(9.0, 0.0, z=1.9))
    mapper._odom_callback(_odom(9.1, 0.0, z=2.1))
    scan = LaserScan()
    scan.header.stamp = _stamp(9.05)
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [1.0]
    mapper._laser_scan_callback(scan)

    assert mapper.startup_gate_open
    assert mapper.mapping_start_stamp_ns == 9_100_000_000
    assert not mapper.grid.cells


def test_fusion_status_counts_only_frames_that_insert_geometry(mapper):
    import json
    from types import SimpleNamespace

    mapper.set_parameters(
        [
            Parameter("depth_fusion_rate_hz", value=0.0),
            Parameter("lidar_fusion_rate_hz", value=0.0),
        ]
    )
    mapper._odom_callback(_odom(11.0, 0.0))
    mapper._odom_callback(_odom(11.2, 0.0))
    info = CameraInfo()
    info.header.stamp = _stamp(11.1)
    info.header.frame_id = "depth_optical"
    info.width = 1
    info.height = 1
    info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    mapper._camera_info_callback(info)

    empty_image = mapper.bridge.cv2_to_imgmsg(
        np.array([[float("nan")]], dtype=np.float32), encoding="32FC1"
    )
    empty_image.header.stamp = _stamp(11.1)
    empty_image.header.frame_id = "depth_optical"
    mapper._depth_callback(empty_image)
    empty_scan = LaserScan()
    empty_scan.header.stamp = _stamp(11.1)
    empty_scan.range_min = 0.1
    empty_scan.range_max = 10.0
    empty_scan.ranges = [float("nan")]
    mapper._laser_scan_callback(empty_scan)

    assert mapper.depth_fused_count == 0
    assert mapper.lidar_fused_count == 0
    assert not mapper.grid.cells

    valid_image = mapper.bridge.cv2_to_imgmsg(
        np.array([[1.0]], dtype=np.float32), encoding="32FC1"
    )
    valid_image.header.stamp = _stamp(11.2)
    valid_image.header.frame_id = "depth_optical"
    mapper._depth_callback(valid_image)
    valid_scan = LaserScan()
    valid_scan.header.stamp = _stamp(11.2)
    valid_scan.range_min = 0.1
    valid_scan.range_max = 10.0
    valid_scan.ranges = [1.0]
    mapper._laser_scan_callback(valid_scan)

    assert mapper.depth_fused_count == 1
    assert mapper.lidar_fused_count == 1
    assert mapper.depth_last_sample_stamp_ns == 11_200_000_000
    assert mapper.lidar_last_sample_stamp_ns == 11_200_000_000
    messages = []
    mapper.fusion_status_pub = SimpleNamespace(publish=messages.append)
    mapper._publish_fusion_status(Header(stamp=_stamp(11.3)))
    status = json.loads(messages[0].data)
    assert status["depth_fused_count"] == 1
    assert status["lidar_fused_count"] == 1
    assert status["map_stamp_ns"] == 11_300_000_000


def test_high_roll_or_pitch_variance_is_rejected(mapper):
    uncertain_roll = _odom(12.0, 0.0)
    uncertain_roll.pose.covariance[21] = 1.0
    mapper._odom_callback(uncertain_roll)
    assert not mapper.pose_history.samples

    valid = _odom(12.1, 1.0)
    mapper._odom_callback(valid)
    assert len(mapper.pose_history.samples) == 1

    uncertain_pitch = _odom(12.2, 2.0)
    uncertain_pitch.pose.covariance[28] = float("inf")
    mapper._odom_callback(uncertain_pitch)
    assert len(mapper.pose_history.samples) == 1
    assert np.allclose(mapper.position, [1.0, 0.0, 5.0])


def test_fusion_status_reports_zero_while_mapping_gate_is_closed(mapper):
    import json
    from types import SimpleNamespace

    mapper.startup_gate_open = False
    mapper.have_pose = False
    messages = []
    mapper.fusion_status_pub = SimpleNamespace(publish=messages.append)
    mapper._publish_map()

    assert len(messages) == 1
    status = json.loads(messages[0].data)
    assert status["depth_fused_count"] == 0
    assert status["lidar_fused_count"] == 0


def test_observed_voxel_cloud_classifies_free_occupied_and_uncertain(mapper):
    import json
    from types import SimpleNamespace

    # Unknown space has no key; uncertain space has a key but is not flight-safe.
    mapper.grid.cells = {
        (0, -1, 20): -0.4,
        (1, -1, 20): 0.85,
        (2, -1, 20): -0.1,
    }
    mapper.grid.revision = 7
    observed_messages = []
    status_messages = []
    mapper.observed_voxel_pub = SimpleNamespace(publish=observed_messages.append)
    mapper.fusion_status_pub = SimpleNamespace(publish=status_messages.append)
    mapper.voxel_pub = SimpleNamespace(publish=lambda _: None)
    mapper.grid_pub = SimpleNamespace(publish=lambda _: None)
    mapper.marker_pub = SimpleNamespace(publish=lambda _: None)

    mapper._publish_map()

    assert len(observed_messages) == len(status_messages) == 1
    cloud = observed_messages[0]
    assert [field.name for field in cloud.fields] == ["x", "y", "z", "state"]
    points = list(point_cloud2.read_points(cloud, field_names=("x", "y", "z", "state")))
    assert len(points) == 3
    assert [float(point[3]) for point in points] == [0.0, 1.0, -1.0]
    assert np.allclose([points[0][0], points[0][1], points[0][2]], [0.125, -0.125, 5.125])
    assert all(float(point[0]) != 0.875 for point in points)

    status = json.loads(status_messages[0].data)
    assert status["map_stamp_ns"] == _stamp_ns_for_test(cloud.header.stamp)
    assert status["map_revision"] == 7
    assert status["voxel_resolution_m"] == 0.25
    assert status["observed_bounds_key"] == [0, -1, 20, 2, -1, 20]
    assert status["observed_voxel_count"] == 3
    assert status["free_voxel_count"] == 1
    assert status["occupied_voxel_count"] == 1
    assert status["uncertain_voxel_count"] == 1
    assert status["observed_voxels_complete"] is True
    assert status["mapping_enabled"] is True


def test_observed_voxel_cloud_reports_truncation(mapper):
    import json
    from types import SimpleNamespace

    mapper.set_parameters([Parameter("max_publish_points", value=2)])
    mapper.grid.cells = {
        (0, 0, 20): -0.4,
        (1, 0, 20): -0.4,
        (2, 0, 20): 0.85,
    }
    observed_messages = []
    status_messages = []
    mapper.observed_voxel_pub = SimpleNamespace(publish=observed_messages.append)
    mapper.fusion_status_pub = SimpleNamespace(publish=status_messages.append)
    mapper.voxel_pub = SimpleNamespace(publish=lambda _: None)
    mapper.grid_pub = SimpleNamespace(publish=lambda _: None)
    mapper.marker_pub = SimpleNamespace(publish=lambda _: None)

    mapper._publish_map()

    assert observed_messages[0].width == 2
    status = json.loads(status_messages[0].data)
    assert status["observed_voxel_count"] == 3
    assert status["observed_voxels_point_count"] == 2
    assert status["observed_voxels_complete"] is False


def _stamp_ns_for_test(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
