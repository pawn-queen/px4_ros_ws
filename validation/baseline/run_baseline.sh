#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source "${ROOT}/tools/env/ros_env.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CTEST_PARALLEL_LEVEL=1
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="${ROOT}/src/px4_powerplant_mission:${PYTHONPATH:-}"
export POWERPLANT_CPP_BUILD="${ROOT}/build/px4_powerplant_mission_cpp"
export ROS_DOMAIN_ID=173
mkdir -p log/p0_baseline
/usr/bin/python3 "${ROOT}/validation/probes/transport_probe.py" --report log/p0_baseline/transport.json
colcon test --packages-select px4_powerplant_mission_cpp --executor sequential \
  --ctest-args -j1 --output-on-failure
colcon test-result --verbose --test-result-base build/px4_powerplant_mission_cpp
/usr/bin/python3 -m pytest -p no:cacheprovider -q src/px4_powerplant_mission/test \
  --junitxml=log/p0_baseline/python.xunit.xml
/usr/bin/python3 -m pytest -p no:cacheprovider -q validation/tests \
  --junitxml=log/p0_baseline/validation.xunit.xml
