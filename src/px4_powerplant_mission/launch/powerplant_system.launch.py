"""Launch the full PX4 powerplant simulation application stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")

    default_config = PathJoinSubstitution(
        [FindPackageShare("px4_powerplant_mission"), "config", "powerplant.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("use_localization", default_value="true"),
            DeclareLaunchArgument("use_external_vision", default_value="true"),
            DeclareLaunchArgument("use_uwb", default_value="true"),
            DeclareLaunchArgument("use_gazebo_truth_bridge", default_value="true"),
            DeclareLaunchArgument("use_mapping", default_value="true"),
            DeclareLaunchArgument("use_yolo", default_value="false"),
            DeclareLaunchArgument("use_control", default_value="false"),
            DeclareLaunchArgument(
                "yolo_python",
                default_value="/usr/bin/python3",
                description="Python interpreter used to run the YOLO detector.",
            ),
            Node(
                package="px4_powerplant_mission",
                executable="localization_node",
                name="powerplant_localization",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(LaunchConfiguration("use_localization")),
            ),
            Node(
                package="px4_powerplant_mission",
                executable="external_vision_bridge_node",
                name="powerplant_external_vision_bridge",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(LaunchConfiguration("use_external_vision")),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="powerplant_gz_truth_bridge",
                output="screen",
                arguments=[
                    "/model/x500_depth/odometry_with_covariance"
                    "@nav_msgs/msg/Odometry"
                    "@gz.msgs.OdometryWithCovariance"
                ],
                condition=IfCondition(LaunchConfiguration("use_gazebo_truth_bridge")),
            ),
            Node(
                package="px4_powerplant_mission",
                executable="uwb_simulator_node",
                name="powerplant_uwb_simulator",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(LaunchConfiguration("use_uwb")),
            ),
            Node(
                package="px4_powerplant_mission",
                executable="voxel_mapper_node",
                name="powerplant_voxel_mapper",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(LaunchConfiguration("use_mapping")),
            ),
            Node(
                package="px4_powerplant_mission",
                executable="yolo_detector_node",
                name="powerplant_yolo_detector",
                output="screen",
                prefix=LaunchConfiguration("yolo_python"),
                parameters=[config_file],
                condition=IfCondition(LaunchConfiguration("use_yolo")),
            ),
            Node(
                package="px4_powerplant_mission",
                executable="offboard_mission_node",
                name="powerplant_offboard_mission",
                output="screen",
                parameters=[config_file],
                condition=IfCondition(LaunchConfiguration("use_control")),
            ),
        ]
    )
