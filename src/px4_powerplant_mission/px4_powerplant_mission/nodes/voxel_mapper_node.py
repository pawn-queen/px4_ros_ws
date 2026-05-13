"""Depth/radar/lidar based voxel mapping for the powerplant world."""

from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from px4_powerplant_mission.common.frames import rotate_vector_xyzw
from px4_powerplant_mission.common.qos import reliable_qos_profile, sensor_qos_profile
from px4_powerplant_mission.mapping.voxel_grid import VoxelGrid


class VoxelMapperNode(Node):
    """Build a lightweight occupied voxel map from Gazebo depth/lidar topics."""

    def __init__(self) -> None:
        super().__init__("powerplant_voxel_mapper")
        self._declare_parameters()
        self.bridge = CvBridge()
        self.grid = VoxelGrid(float(self.get_parameter("voxel_resolution_m").value))
        self.camera_info: CameraInfo | None = None
        self.position = np.zeros(3, dtype=float)
        self.orientation_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        self.have_pose = False

        self.voxel_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter("occupied_voxels_topic").value,
            reliable_qos_profile(),
        )
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter("occupancy_grid_topic").value,
            reliable_qos_profile(),
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter("voxel_marker_topic").value,
            reliable_qos_profile(),
        )

        self.create_subscription(
            Odometry,
            self.get_parameter("localized_odom_topic").value,
            self._odom_callback,
            reliable_qos_profile(),
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_image_topic").value,
            self._depth_callback,
            sensor_qos_profile(),
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_camera_info_topic").value,
            self._camera_info_callback,
            sensor_qos_profile(),
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("laser_scan_topic").value,
            self._laser_scan_callback,
            sensor_qos_profile(),
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("pointcloud_topic").value,
            self._pointcloud_callback,
            sensor_qos_profile(),
        )

        period = 1.0 / float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(period, self._publish_map)
        self.get_logger().info("voxel mapper node started")

    def _declare_parameters(self) -> None:
        self.declare_parameter("localized_odom_topic", "/powerplant/localization/odom")
        self.declare_parameter("depth_image_topic", "/depth_camera")
        self.declare_parameter("depth_camera_info_topic", "/depth_camera/camera_info")
        self.declare_parameter("laser_scan_topic", "/scan")
        self.declare_parameter("pointcloud_topic", "/points")
        self.declare_parameter("occupied_voxels_topic", "/powerplant/map/occupied_voxels")
        self.declare_parameter("occupancy_grid_topic", "/powerplant/map/occupancy_grid")
        self.declare_parameter("voxel_marker_topic", "/powerplant/map/voxel_markers")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("voxel_resolution_m", 0.25)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("depth_stride_px", 8)
        self.declare_parameter("max_depth_range_m", 18.0)
        self.declare_parameter("min_depth_range_m", 0.25)
        self.declare_parameter("camera_fx", 430.0)
        self.declare_parameter("camera_fy", 430.0)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)
        self.declare_parameter("camera_translation_flu", [0.12, -0.03, 0.24])
        self.declare_parameter("lidar_translation_flu", [0.0, 0.0, 0.0])
        self.declare_parameter("pointcloud_frame", "base_link")
        self.declare_parameter("max_publish_points", 60000)
        self.declare_parameter("grid_z_min_m", -0.3)
        self.declare_parameter("grid_z_max_m", 4.0)

    def _odom_callback(self, msg: Odometry) -> None:
        self.position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        self.orientation_xyzw = np.array(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ],
            dtype=float,
        )
        self.have_pose = True

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _depth_callback(self, msg: Image) -> None:
        if not self.have_pose:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"depth image conversion failed: {exc}")
            return
        depth = np.asarray(image, dtype=np.float32)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        if msg.encoding in ("16UC1", "mono16"):
            depth = depth * 0.001

        points = self._depth_to_map_points(depth)
        self.grid.insert_points(points)

    def _depth_to_map_points(self, depth: np.ndarray) -> np.ndarray:
        height, width = depth.shape[:2]
        stride = int(self.get_parameter("depth_stride_px").value)
        min_range = float(self.get_parameter("min_depth_range_m").value)
        max_range = float(self.get_parameter("max_depth_range_m").value)

        if self.camera_info is not None and self.camera_info.k[0] > 0.0:
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
        else:
            fx = float(self.get_parameter("camera_fx").value)
            fy = float(self.get_parameter("camera_fy").value)
            cx = float(self.get_parameter("camera_cx").value)
            cy = float(self.get_parameter("camera_cy").value)

        ys = np.arange(0, height, stride)
        xs = np.arange(0, width, stride)
        u, v = np.meshgrid(xs, ys)
        z = depth[v, u]
        mask = np.isfinite(z) & (z >= min_range) & (z <= max_range)
        if not np.any(mask):
            return np.empty((0, 3), dtype=float)

        z = z[mask]
        u = u[mask].astype(float)
        v = v[mask].astype(float)
        optical_x = (u - cx) * z / fx
        optical_y = (v - cy) * z / fy
        optical_z = z

        body = np.stack([optical_z, -optical_x, -optical_y], axis=1)
        body += np.asarray(self.get_parameter("camera_translation_flu").value, dtype=float)
        return np.asarray([self._body_to_map(point) for point in body], dtype=float)

    def _laser_scan_callback(self, msg: LaserScan) -> None:
        if not self.have_pose:
            return
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges), dtype=float) * msg.angle_increment
        mask = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(mask):
            return
        x = ranges[mask] * np.cos(angles[mask])
        y = ranges[mask] * np.sin(angles[mask])
        z = np.zeros_like(x)
        body = np.stack([x, y, z], axis=1)
        body += np.asarray(self.get_parameter("lidar_translation_flu").value, dtype=float)
        points = np.asarray([self._body_to_map(point) for point in body], dtype=float)
        self.grid.insert_points(points)

    def _pointcloud_callback(self, msg: PointCloud2) -> None:
        if not self.have_pose:
            return
        frame = str(self.get_parameter("pointcloud_frame").value)
        points = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            arr = np.array([point[0], point[1], point[2]], dtype=float)
            if frame == str(self.get_parameter("map_frame").value):
                points.append(arr)
            else:
                points.append(self._body_to_map(arr))
        self.grid.insert_points(points)

    def _body_to_map(self, point_flu: np.ndarray) -> np.ndarray:
        return self.position + rotate_vector_xyzw(self.orientation_xyzw, point_flu)

    def _publish_map(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = str(self.get_parameter("map_frame").value)
        max_points = int(self.get_parameter("max_publish_points").value)
        occupied = self.grid.occupied_points(max_points=max_points)
        cloud = point_cloud2.create_cloud_xyz32(header, occupied.tolist())
        self.voxel_pub.publish(cloud)
        self._publish_occupancy_grid(header)
        self._publish_marker(header, occupied)

    def _publish_occupancy_grid(self, header: Header) -> None:
        result = self.grid.to_occupancy_grid_2d(
            float(self.get_parameter("grid_z_min_m").value),
            float(self.get_parameter("grid_z_max_m").value),
        )
        if result is None:
            return
        grid_array, origin_xy, resolution = result
        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = float(resolution)
        msg.info.width = int(grid_array.shape[1])
        msg.info.height = int(grid_array.shape[0])
        msg.info.origin.position.x = float(origin_xy[0])
        msg.info.origin.position.y = float(origin_xy[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_array.reshape(-1).astype(np.int8).tolist()
        self.grid_pub.publish(msg)

    def _publish_marker(self, header: Header, occupied: np.ndarray) -> None:
        marker = Marker()
        marker.header = header
        marker.ns = "occupied_voxels"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        scale = float(self.get_parameter("voxel_resolution_m").value)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = 0.0
        marker.color.g = 0.55
        marker.color.b = 0.9
        marker.color.a = 0.45
        for point in occupied:
            marker.points.append(Point(x=float(point[0]), y=float(point[1]), z=float(point[2])))
        self.marker_pub.publish(MarkerArray(markers=[marker]))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VoxelMapperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
