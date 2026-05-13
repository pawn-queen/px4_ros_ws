from glob import glob
from setuptools import find_packages, setup

package_name = "px4_powerplant_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "requirements-yolo.txt"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/models/yolo", glob("models/yolo/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pawn",
    maintainer_email="pawn@example.com",
    description=(
        "PX4 Gazebo powerplant simulation stack with localization, mapping, "
        "YOLO perception, and offboard mission control."
    ),
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localization_node = px4_powerplant_mission.nodes.localization_node:main",
            "uwb_simulator_node = px4_powerplant_mission.nodes.uwb_simulator_node:main",
            "voxel_mapper_node = px4_powerplant_mission.nodes.voxel_mapper_node:main",
            "yolo_detector_node = px4_powerplant_mission.nodes.yolo_detector_node:main",
            "offboard_mission_node = px4_powerplant_mission.nodes.offboard_mission_node:main",
        ],
    },
)
