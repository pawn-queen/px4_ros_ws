#!/usr/bin/env bash
set -eo pipefail
P0_PACKAGE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
P0_WORKSPACE="$(cd "${P0_PACKAGE}/../.." && pwd)"
cd "${P0_WORKSPACE}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source "${P0_PACKAGE}/scripts/p0_ros_env.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CTEST_PARALLEL_LEVEL=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="${P0_WORKSPACE}/src/px4_powerplant_mission:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=173
mkdir -p log/p0_baseline
/usr/bin/python3 "${P0_PACKAGE}/test/ros_transport_probe.py" --report log/p0_baseline/transport.json
colcon test --packages-select px4_powerplant_mission_cpp --executor sequential \
  --ctest-args -j1 --output-on-failure
colcon test-result --verbose --test-result-base build/px4_powerplant_mission_cpp
/usr/bin/python3 -m pytest -p no:cacheprovider -q src/px4_powerplant_mission/test \
  --junitxml=log/p0_baseline/python.xunit.xml
