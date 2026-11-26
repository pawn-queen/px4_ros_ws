"""Prepare simulated image pairs and inertial measurements for the estimator."""

import json
import math
import sys
from collections import OrderedDict

import message_filters
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info, OdomInfo
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Bool, String

from .geometry import Occlusion, pinhole, register_depth


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class SimulationInputs(Node):
    def __init__(self):
        super().__init__('simulation_inputs')
        self.occlusion = Occlusion(self.declare_parameter('occlusion_start_s', -1.0).value,
                                   self.declare_parameter('occlusion_duration_s', 0.0).value)
        self.capture_started = not self.declare_parameter('wait_for_capture_start', False).value
        self.frame_masks = OrderedDict()
        self.loop_edges = set()
        self.graph_nodes = set()
        self.last_loop_id = 0
        self.last_frame_ns = -1
        self.depth = self.create_publisher(Image, 'depth_registered/image', 5)
        self.rgb = self.create_publisher(Image, 'rgb/image', 5)
        self.info = self.create_publisher(CameraInfo, 'rgb/camera_info', 5)
        self.imu = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        self.status = self.create_publisher(String, 'status', 20)
        self.imu_callbacks = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Imu, 'input/imu', self.on_imu, qos_profile_sensor_data,
                                 callback_group=self.imu_callbacks)
        self.create_subscription(Bool, 'input/occluded', self.on_occluded, 10)
        self.create_subscription(OdomInfo, 'odom_info', self.on_odom_info, qos_profile_sensor_data)
        self.create_subscription(Info, 'info', self.on_info, qos_profile_sensor_data)
        topics = [(Image, '/camera'), (Image, '/depth_camera'),
                  (CameraInfo, '/camera/camera_info'), (CameraInfo, '/depth_camera/camera_info')]
        self.input_streams = [message_filters.Subscriber(self, kind, name,
                              qos_profile=qos_profile_sensor_data) for kind, name in topics]
        # Both simulator cameras expose their own observation and CameraInfo timestamps.
        # Accept only equal acquisition stamps, preserving timing even on slow bag replay.
        self.sync = message_filters.TimeSynchronizer(self.input_streams, queue_size=30)
        self.sync.registerCallback(self.on_rgbd)

    def on_imu(self, msg):
        if not self.capture_started:
            return
        # Gazebo orientation may come directly from the simulated world. Supply only
        # angular rates and specific force to the six-axis attitude filter.
        if msg.header.frame_id != 'base_link' or stamp_ns(msg.header) <= 0:
            self.get_logger().warn('IMU needs a stamped base_link sample', throttle_duration_sec=3.)
            return
        rates, acc = msg.angular_velocity, msg.linear_acceleration
        if not all(math.isfinite(v) for v in (rates.x, rates.y, rates.z, acc.x, acc.y, acc.z)):
            return
        msg.header.frame_id = 'slam_body'
        msg.orientation.x = msg.orientation.y = msg.orientation.z = msg.orientation.w = 0.
        msg.orientation_covariance = [-1.] + [0.] * 8
        self.imu.publish(msg)

    def on_occluded(self, msg):
        self.occlusion.external = msg.data
        if not self.capture_started:
            # Discard partial image sets queued before the recorder's first marker.
            # RGB-D and marker subscriptions share the default callback group.
            for queue in self.sync.queues:
                queue.clear()
            self.capture_started = True

    def on_rgbd(self, rgb, depth, rgb_info, depth_info):
        if not self.capture_started:
            return
        ns = stamp_ns(rgb.header)
        if ns <= self.last_frame_ns or ns <= 0:
            return
        try:
            if any(m.header.frame_id != 'camera_link' for m in (rgb, rgb_info, depth, depth_info)):
                raise ValueError('expected both images and CameraInfo from the Gazebo camera_link rig')
            rgb_k, depth_k = pinhole(rgb_info, rgb), pinhole(depth_info, depth)
            registered = register_depth(depth, rgb_k, depth_k)
        except (TypeError, ValueError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.)
            return
        self.last_frame_ns = ns
        masked = self.occlusion.masked(ns)
        if masked:
            rgb.data = bytes(len(rgb.data))
            registered.fill(np.nan)
        self.frame_masks[ns] = masked
        if len(self.frame_masks) > 1024:
            self.frame_masks.popitem(last=False)
        rgb.header.frame_id = 'slam_camera_optical'
        rgb_info.header = rgb.header
        aligned = Image()
        aligned.header = rgb.header
        aligned.width, aligned.height = rgb.width, rgb.height
        aligned.encoding = '32FC1'
        aligned.is_bigendian = int(sys.byteorder == 'big')
        aligned.step = rgb.width * 4
        aligned.data = registered.tobytes()
        self.info.publish(rgb_info)
        self.depth.publish(aligned)
        self.rgb.publish(rgb)

    def on_info(self, msg):
        if msg.ref_id > 0:
            self.graph_nodes.add(msg.ref_id)
        if msg.loop_closure_id > 0:
            self.loop_edges.add((msg.ref_id, msg.loop_closure_id))
            self.last_loop_id = msg.loop_closure_id

    def on_odom_info(self, msg):
        ns = stamp_ns(msg.header)
        tracking = 'LOST' if msg.lost else 'TRACKING'
        if not msg.lost and msg.interval <= 0:
            tracking = 'INITIALIZING'
        status = dict(stamp_ns=ns, tracking=tracking, masked=self.frame_masks.get(ns),
                      keyframes=len(self.graph_nodes), loops=len(self.loop_edges),
                      loop_id=self.last_loop_id, inliers=msg.inliers)
        self.status.publish(String(data=json.dumps(status, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = SimulationInputs()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
