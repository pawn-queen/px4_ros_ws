"""Launch the full PX4 powerplant simulation application stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from px4_msgs.msg import BatteryStatus, VehicleCommandAck, VehicleLocalPosition, VehicleStatus


def generate_launch_description() -> LaunchDescription:
    # PX4's generated DDS topics carry a suffix for versioned schemas. Keep
    # direct-node/default parameter compatibility; launch maps only the default
    # base names to the schema supplied by the installed px4_msgs package.
    px4_versioned_topics = [
        (base, f"{base}_v{message.MESSAGE_VERSION}")
        for base, message in (
            ("/fmu/out/vehicle_status", VehicleStatus),
            ("/fmu/out/vehicle_command_ack", VehicleCommandAck),
            ("/fmu/out/battery_status", BatteryStatus),
            ("/fmu/out/vehicle_local_position", VehicleLocalPosition),
        ) if message.MESSAGE_VERSION > 0
    ]
    config_file = LaunchConfiguration("config_file")
    profile = LaunchConfiguration("localization_profile")
    simulation = PythonExpression(["'", profile, "' == 'simulation'"])
    profile_config = PathJoinSubstitution([
        FindPackageShare("px4_powerplant_mission"), "config",
        PythonExpression(["'localization_", profile, ".yaml'"]),
    ])

    default_motion_profile = PathJoinSubstitution([
        FindPackageShare("px4_powerplant_mission"), "config",
        PythonExpression(["'p2_", profile, ".yaml'"]),
    ])

    motion_profile_config = LaunchConfiguration("motion_config_file")

    default_config = PathJoinSubstitution(
        [FindPackageShare("px4_powerplant_mission"), "config", "powerplant.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("localization_profile", default_value="simulation",
                                  choices=["simulation", "hardware"]),
            DeclareLaunchArgument("motion_config_file", default_value=default_motion_profile),
            DeclareLaunchArgument("use_localization", default_value="true"),
            DeclareLaunchArgument("use_external_vision", default_value="true"),
            DeclareLaunchArgument("use_uwb", default_value=simulation),
            DeclareLaunchArgument("use_gazebo_truth_bridge", default_value=simulation),
            DeclareLaunchArgument("use_gazebo_clock_bridge", default_value=simulation),
            DeclareLaunchArgument("use_sim_time", default_value=simulation),
            DeclareLaunchArgument("use_mapping", default_value="true"),
            DeclareLaunchArgument("use_yolo", default_value="false"),
            DeclareLaunchArgument("use_control", default_value="false"),
            DeclareLaunchArgument(
                "yolo_python",
                default_value="/home/pawn/px4_ros_ws/.venv/bin/python",
                description="Python interpreter used to run the YOLO detector.",
            ),
            Node(
                package="px4_powerplant_mission",
                executable="localization_node",
                name="powerplant_localization",
                output="screen",
                remappings=px4_versioned_topics,
                parameters=[config_file, profile_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("use_localization")),
            ),
            Node(
                package="px4_powerplant_mission",
                executable="external_vision_bridge_node",
                name="powerplant_external_vision_bridge",
                output="screen",
                parameters=[config_file, profile_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("use_external_vision")),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="powerplant_gz_clock_bridge",
                output="screen",
                arguments=[
                    "/world/powerplant/clock"
                    "@rosgraph_msgs/msg/Clock"
                    "[gz.msgs.Clock",
                ],
                remappings=[("/world/powerplant/clock", "/clock")],
                condition=IfCondition(LaunchConfiguration("use_gazebo_clock_bridge")),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="powerplant_gz_truth_bridge",
                output="screen",
                arguments=[
                    "/model/x500_depth/odometry_with_covariance"
                    "@nav_msgs/msg/Odometry"
                    "[gz.msgs.OdometryWithCovariance",
                    "/world/powerplant/dynamic_pose/info"
                    "@tf2_msgs/msg/TFMessage"
                    "[gz.msgs.Pose_V",
                ],
                condition=IfCondition(LaunchConfiguration("use_gazebo_truth_bridge")),
            ),
            Node(
                package="px4_powerplant_mission",
                executable="uwb_simulator_node",
                name="powerplant_uwb_simulator",
                output="screen",
                parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("use_uwb")),
            ),
            Node(
                package="px4_powerplant_mission_cpp",
                executable="voxel_mapper_node",
                name="powerplant_voxel_mapper",
                output="screen",
                parameters=[config_file, motion_profile_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("use_mapping")),
            ),
            Node(
                package="px4_powerplant_mission",
                executable="yolo_detector_node",
                name="powerplant_yolo_detector",
                output="screen",
                prefix=LaunchConfiguration("yolo_python"),
                parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("use_yolo")),
            ),
            Node(
                package="px4_powerplant_mission_cpp",
                executable="offboard_mission_node",
                name="powerplant_offboard_mission",
                output="screen",
                remappings=px4_versioned_topics,
                parameters=[config_file, motion_profile_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
                condition=IfCondition(LaunchConfiguration("use_control")),
            ),
        ]
    )
