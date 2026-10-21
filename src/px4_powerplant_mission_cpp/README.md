# Native powerplant mapping and mission control

ROS 2 Jazzy C++17 implementation of the retained `px4_powerplant_mission`
voxel mapper, conservative 3D planner and Offboard controller. It does not embed
Python or execute the Python nodes. Localization, UWB simulation and YOLO remain
in the original Python package.

Current results and remaining integration checks: [migration status](MIGRATION_STATUS.md).

## Build with bounded resource use

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 colcon build --symlink-install \
  --packages-up-to px4_powerplant_mission --executor sequential --parallel-workers 1 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
```

Both limits are intentional: colcon's package worker limit alone does not limit
Make's compilation processes. The ROS mission transport library uses `-O1` to
bound compiler memory on WSL; the numeric core retains the selected build type's
optimization. Do not run compilation, tests and Gazebo simultaneously on a
memory-constrained machine.

Dependencies are declared in `package.xml`: rclcpp, the workspace's px4_msgs,
ROS message packages, cv_bridge/OpenCV, Eigen3 and JsonCpp. No new Python inference
dependencies are needed.

## Modules

- `core.hpp`, `voxel_grid.cpp`: probability updates, 2D projection, stamped pose
  history, initial yaw scan and setpoint slew limits.
- `frontier_planner.cpp`: six-neighbor paths, known-free clearance, frontier view
  scoring, local path length and known-free return budget.
- `voxel_mapper_node.cpp`: bounded sensor queue, camera calibration matching,
  per-beam lidar motion compensation and map publications.
- `map_snapshot.cpp`: exact cloud/status timestamp pairing, field/count checks,
  observed-state classification and freshness tracking.
- `offboard_node.cpp`, `offboard_subscriptions_*.cpp`: ROS interfaces and status.
- `mission_state_machine.cpp`: explicit mission stages and transition conditions.
- `mission_planning.cpp`, `worker.hpp`: immutable planning jobs, bounded work queue,
  cooperative cancellation and stale-result rejection.
- `offboard_control.cpp`: PX4 heartbeat, NED setpoints, commands, preflight/arm
  sequencing, slew and obstacle guards.
- `inspection_recorder.cpp`: image encoding and disk writes outside the ROS timer.

The mission's ROS callbacks have a single state writer. Planner workers operate
on copied map snapshots; map copies are not made by each control-path safety
check. Cancellation never waits for a planning thread in a ROS callback. Worker
threads are joined during orderly node shutdown.

## Interface compatibility

The executables remain `voxel_mapper_node` and `offboard_mission_node`; node names
remain `powerplant_voxel_mapper` and `powerplant_offboard_mission`. Existing YAML,
ROS topics/types, service names, QoS, ENU/FLU to NED conventions, JSON fields,
map field layout and phase strings are retained. The ROS contract suite compares
all 151 native parameter defaults with the original Python declarations; see the
migration status for the current cross-process communication blocker.

`/powerplant/map/observed_voxels` still uses four float32 fields (`x,y,z,state`).
Unknown/missing and uncertain (`state=-1`) cells cannot carry a flight path.
Truncated snapshots never authorize exploration. The mapper uses ROS simulation
time when requested by launch; the controller retains the original receipt-time
freshness behavior and default wall-clock domain.

Algorithm tie-breaking is deterministic in C++ (lexicographic voxel keys), so
exactly equal candidate priorities may choose a different equivalent viewpoint
than a Python set iteration. Trajectories are checked for known-free clearance
rather than requiring byte-identical path messages. Voxel publication order is
not an API guarantee. Numeric calculations use doubles and wire map fields use
float32, matching the Python pipeline.

Image saving now completes asynchronously; photo events and counters follow the
write result on a later control cycle. The `require_image_for_photo` fallback and
landing-timeout-to-`COMPLETE` behaviors remain compatible. Invalid input is
rejected rather than permitted to produce a non-finite control setpoint.

## Validation

Run sequentially after compilation:

```bash
source install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CTEST_PARALLEL_LEVEL=1 \
  colcon test --packages-select px4_powerplant_mission_cpp --executor sequential \
  --ctest-args -j1 --output-on-failure
colcon test-result --verbose
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  PYTHONPATH=src/px4_powerplant_mission:$PYTHONPATH /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q src/px4_powerplant_mission/test
```

Native tests cover geometry, time history, map contracts, scan/state transitions,
setpoint limits and cancellation. The test-only `core_probe` performs JSON
input/output to compare the unchanged Python reference with C++ on original
planning scenarios, random obstacle routes and random ray updates. The ROS
contract tests start only one native process at a time, inject synthetic sensor
and PX4 messages, and verify parameter defaults, publications and services.
These tests do not replace a full PX4/Gazebo flight validation.
