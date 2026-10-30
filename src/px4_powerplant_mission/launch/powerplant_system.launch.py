"""Launch the full PX4 powerplant simulation application stack.

The profile-to-config mapping lives in ``config/runtime_profiles.json`` and is
resolved by ``px4_powerplant_mission.runtime_profile``. The old launch arguments
remain supported; ``profile`` is the preferred entry and overrides
``localization_profile`` when set.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import BatteryStatus, VehicleCommandAck, VehicleLocalPosition, VehicleStatus
from px4_powerplant_mission.runtime_profile import (
    DEFAULT_KEYS,
    load_manifest,
    resolve_defaults,
    resolve_profile,
)


PACKAGE = "px4_powerplant_mission"
PROFILES = load_manifest(Path(get_package_share_directory(PACKAGE)))
PX4_VERSIONED_TOPICS = [
    (base, f"{base}_v{message.MESSAGE_VERSION}")
    for base, message in (
        ("/fmu/out/vehicle_status", VehicleStatus),
        ("/fmu/out/vehicle_command_ack", VehicleCommandAck),
        ("/fmu/out/battery_status", BatteryStatus),
        ("/fmu/out/vehicle_local_position", VehicleLocalPosition),
    ) if message.MESSAGE_VERSION > 0
]


def _profile_default(key: str, profile, localization_profile) -> PythonExpression:
    """Build a launch default from the manifest, not from a second mapping."""
    simulation = PROFILES["simulation"].defaults[key]
    hardware = PROFILES["hardware"].defaults[key]
    return PythonExpression([
        f"'{simulation}' if (('", profile, "' == 'simulation') or ('", profile,
        "' == '' and '", localization_profile, "' == 'simulation')) else '",
        hardware, "'",
    ])


def _arg(context, name: str) -> str:
    return context.perform_substitution(LaunchConfiguration(name))


def _launch_setup(context, *args, **kwargs):
    share = Path(get_package_share_directory(PACKAGE))
    profiles = PROFILES
    selected = resolve_profile(_arg(context, "profile"), _arg(context, "localization_profile"), profiles)
    defaults = resolve_defaults(
        selected,
        {key: _arg(context, key) for key in DEFAULT_KEYS},
    )

    config_file = _arg(context, "config_file") or str(share / "config/powerplant.yaml")
    localization_config = selected.config_path(share, "localization")
    motion_config = _arg(context, "motion_config_file") or selected.config_path(share, "motion")
    for path in (localization_config, motion_config):
        if not path.is_file():
            raise RuntimeError(f"runtime profile {selected.name!r} references missing config: {path}")

    use_sim_time = defaults["use_sim_time"]
    use_uwb = defaults["use_uwb"]
    use_gazebo_truth_bridge = defaults["use_gazebo_truth_bridge"]
    use_gazebo_clock_bridge = defaults["use_gazebo_clock_bridge"]
    use_localization = _arg(context, "use_localization")
    use_external_vision = _arg(context, "use_external_vision")
    use_mapping = _arg(context, "use_mapping")
    use_yolo = _arg(context, "use_yolo")
    use_control = _arg(context, "use_control")
    yolo_python = _arg(context, "yolo_python")

    return [
        Node(
            package=PACKAGE,
            executable="localization_node",
            name="powerplant_localization",
            output="screen",
            remappings=PX4_VERSIONED_TOPICS,
            parameters=[config_file, str(localization_config), {"use_sim_time": use_sim_time}],
            condition=IfCondition(use_localization),
        ),
        Node(
            package=PACKAGE,
            executable="external_vision_bridge_node",
            name="powerplant_external_vision_bridge",
            output="screen",
            parameters=[config_file, str(localization_config), {"use_sim_time": use_sim_time}],
            condition=IfCondition(use_external_vision),
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
            condition=IfCondition(use_gazebo_clock_bridge),
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
            condition=IfCondition(use_gazebo_truth_bridge),
        ),
        Node(
            package=PACKAGE,
            executable="uwb_simulator_node",
            name="powerplant_uwb_simulator",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            condition=IfCondition(use_uwb),
        ),
        Node(
            package="px4_powerplant_mission_cpp",
            executable="voxel_mapper_node",
            name="powerplant_voxel_mapper",
            output="screen",
            parameters=[config_file, str(motion_config), {"use_sim_time": use_sim_time}],
            condition=IfCondition(use_mapping),
        ),
        Node(
            package=PACKAGE,
            executable="yolo_detector_node",
            name="powerplant_yolo_detector",
            output="screen",
            prefix=yolo_python,
            parameters=[config_file, {"use_sim_time": use_sim_time}],
            condition=IfCondition(use_yolo),
        ),
        Node(
            package="px4_powerplant_mission_cpp",
            executable="offboard_mission_node",
            name="powerplant_offboard_mission",
            output="screen",
            remappings=PX4_VERSIONED_TOPICS,
            parameters=[config_file, str(motion_config), {"use_sim_time": use_sim_time}],
            condition=IfCondition(use_control),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    profile = LaunchConfiguration("profile")
    localization_profile = LaunchConfiguration("localization_profile")
    default_config = PathJoinSubstitution(
        [FindPackageShare(PACKAGE), "config", "powerplant.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="",
                choices=["", "simulation", "hardware"],
                description="Preferred runtime profile; overrides localization_profile when set.",
            ),
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument(
                "localization_profile",
                default_value="simulation",
                choices=["simulation", "hardware"],
                description="Legacy profile selector; kept for compatibility.",
            ),
            DeclareLaunchArgument(
                "motion_config_file",
                default_value="",
                description="Optional motion config override; empty uses the profile manifest.",
            ),
            DeclareLaunchArgument("use_localization", default_value="true"),
            DeclareLaunchArgument("use_external_vision", default_value="true"),
            DeclareLaunchArgument("use_uwb", default_value=_profile_default("use_uwb", profile, localization_profile)),
            DeclareLaunchArgument("use_gazebo_truth_bridge", default_value=_profile_default("use_gazebo_truth_bridge", profile, localization_profile)),
            DeclareLaunchArgument("use_gazebo_clock_bridge", default_value=_profile_default("use_gazebo_clock_bridge", profile, localization_profile)),
            DeclareLaunchArgument("use_sim_time", default_value=_profile_default("use_sim_time", profile, localization_profile)),
            DeclareLaunchArgument("use_mapping", default_value="true"),
            DeclareLaunchArgument("use_yolo", default_value="false"),
            DeclareLaunchArgument("use_control", default_value="false"),
            DeclareLaunchArgument(
                "yolo_python",
                default_value="/home/pawn/px4_ros_ws/.venv/bin/python",
                description="Python interpreter used to run the YOLO detector.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
