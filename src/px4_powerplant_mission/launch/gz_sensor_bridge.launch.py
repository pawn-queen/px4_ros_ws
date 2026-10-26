"""Bridge selected Gazebo Transport sensor topics into ROS 2."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    bridges = []
    remappings = []
    topic_specs = [
        ("depth_gz_topic", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        ("depth_info_gz_topic", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        ("rgb_gz_topic", "sensor_msgs/msg/Image", "gz.msgs.Image"),
        ("rgb_info_gz_topic", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
        ("scan_gz_topic", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
        ("points_gz_topic", "sensor_msgs/msg/PointCloud2", "gz.msgs.PointCloudPacked"),
    ]
    for launch_arg, ros_type, gz_type in topic_specs:
        topic = LaunchConfiguration(launch_arg).perform(context)
        if topic:
            bridges.append(f"{topic}@{ros_type}[{gz_type}")
            ros_arg = launch_arg.replace("_gz_topic", "_ros_topic")
            ros_topic = LaunchConfiguration(ros_arg).perform(context)
            if ros_topic and ros_topic != topic:
                remappings.append((topic, ros_topic))

    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="powerplant_gz_sensor_bridge",
            output="screen",
            arguments=bridges,
            remappings=remappings,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("depth_gz_topic", default_value="/depth_camera"),
            DeclareLaunchArgument("depth_info_gz_topic", default_value="/camera_info"),
            DeclareLaunchArgument("rgb_gz_topic", default_value="/world/powerplant/model/x500_depth_0/link/camera_link/sensor/IMX214/image"),
            DeclareLaunchArgument("rgb_info_gz_topic", default_value="/world/powerplant/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info"),
            DeclareLaunchArgument("scan_gz_topic", default_value="/mapping_lidar"),
            DeclareLaunchArgument("points_gz_topic", default_value=""),
            DeclareLaunchArgument("depth_ros_topic", default_value="/depth_camera"),
            DeclareLaunchArgument("depth_info_ros_topic", default_value="/depth_camera/camera_info"),
            DeclareLaunchArgument("rgb_ros_topic", default_value="/camera"),
            DeclareLaunchArgument("rgb_info_ros_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("scan_ros_topic", default_value="/mapping_lidar"),
            DeclareLaunchArgument("points_ros_topic", default_value="/points"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
