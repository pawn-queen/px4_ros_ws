# Source after /opt/ros/jazzy/setup.bash and the workspace install/setup.bash.
# Scope is this shell and its children; no machine-wide ROS/DDS settings change.
P0_WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
P0_ROS_PREFIX="${P0_ROS_PREFIX:-${P0_WORKSPACE}/.p0_deps/opt/ros/jazzy}"
if [ -f "${P0_ROS_PREFIX}/lib/librmw_cyclonedds_cpp.so" ]; then
  export AMENT_PREFIX_PATH="${P0_ROS_PREFIX}:${AMENT_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${P0_ROS_PREFIX}/lib:${P0_ROS_PREFIX}/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
elif [ ! -f /opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so ]; then
  echo "Cyclone DDS is missing; run scripts/bootstrap_p0_ros.py or install ros-jazzy-rmw-cyclonedds-cpp." >&2
  return 1
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML ROS_DISCOVERY_SERVER
unset ROS_STATIC_PEERS CYCLONEDDS_URI
