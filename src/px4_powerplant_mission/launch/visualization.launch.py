"""Start RViz with the powerplant mission visualization layout."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare("px4_powerplant_mission"), "rviz", "powerplant_visualization.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            Node(
                package="rviz2",
                executable="rviz2",
                name="powerplant_rviz",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
