# Native powerplant mapping and mission control

ROS 2 Jazzy C++17 implementation of the retained `px4_powerplant_mission`
voxel mapper, conservative 3D planner and Offboard controller. It does not embed
Python or execute the Python nodes. Localization, UWB simulation and YOLO remain
in the original Python package.

Current results and remaining integration checks: [migration status](MIGRATION_STATUS.md).

P0 input health, takeoff configuration, landing confirmation and reproducible ROS
checks: [P0 validation](P0_VALIDATION.md).

Research and staged improvements for indoor mapping, exploration and inspection:
[research.md](research.md) (Chinese; historical code audit and literature review).
Implemented P1 algorithms, parameters, changed behavior and limitations:
[P1 implementation](P1_IMPLEMENTATION.md).
P2 motion certificates, region guidance, registered target surfaces and photo
evidence: [P2 implementation](P2_IMPLEMENTATION.md). P3 remains future work.

The P2 software audit now checks sampling-time reference history, fresh HOLD
confirmation, shared planner/executor uncertainty budgets, executable local
prefixes, exact stop/time accounting, region identity and exposure-bound photo
coverage. The latest build and 316 direct regression cases pass; see
[P2 validation](P2_VALIDATION.md) for evidence and scope. Flight parameters and
physical acceptance remain separate from these software checks. Configure
safety/calibration parameters before the mission and restart while disarmed;
in-flight hot-swapping of physical safety models is outside this interface.

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

- `input_validation.hpp`: ROS sample timestamps and full pose covariance checks.
- `core.hpp`, `voxel_grid.cpp`: probability updates, 2D projection, stamped pose
  history, initial yaw scan and setpoint slew limits.
- `raycast.hpp`, `depth_sampling.hpp`: positive-length voxel traversal and
  depth/edge-adaptive measured-pixel sampling with a per-frame ray budget.
- `planning_index.cpp`: immutable incremental frontier and clearance caches.
- `motion.cpp`, `motion_runtime.cpp`: bounded quintic trajectories, continuous
  stopping envelopes and an independent 50 Hz position/velocity/acceleration stream.
- `topology.cpp`: 3D known-safe region/portal decomposition and regional quotas.
- `inspection.cpp`: registered RGB/depth observations, stable surface patches,
  quality assessment and transactional photo coverage.
- `frontier_planner.cpp`: six-neighbor paths, known-free clearance, frontier view
  scoring with calibrated rectangular camera bounds, turn/dwell time and
  known-free return budget; explicit failure reasons.
- `voxel_mapper_node.cpp`: bounded sensor queue, camera calibration matching,
  per-beam lidar motion compensation and map publications.
- `map_snapshot.cpp`: exact cloud/status timestamp pairing, field/count checks,
  observed-state classification, bounded classification-change history and
  freshness tracking.
- `offboard_node.cpp`, `offboard_subscriptions_*.cpp`: ROS interfaces and status.
- `mission_state_machine.cpp`: explicit mission stages and transition conditions.
- `scan_coverage.hpp`: actual fused-depth heading coverage and camera geometry;
  nominal yaw-step completion alone does not authorize exploration.
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
remain `powerplant_voxel_mapper` and `powerplant_offboard_mission`. The core ROS topics/types, service names, QoS, ENU/FLU to NED conventions and
four-field map layout are retained. P0 adds strict localization/map health
metadata, VehicleLandDetected, a latched FAULT phase and explicit takeoff-region
parameters. P1 adds successful-depth camera/observation metadata to fusion_status
and coverage/planning-reason diagnostics to mission_status. The contract suite
checks retained defaults and the intentional require_image_for_photo=true change.
See P0_VALIDATION.md for the start conditions and middleware profile, and
P1_IMPLEMENTATION.md for the new parameters. Actual matched CameraInfo is required
for normal exploration by default. P2 also requires real RGB calibration and
YOLO source exposures for inspection; hardware motion parameters must be
configured explicitly. The motion stream now carries finite P/V/A feedforward;
quality and coverage, rather than the last waypoint or photo count, determine
the inspection result.

`/powerplant/map/observed_voxels` still uses four float32 fields (`x,y,z,state`).
Unknown/missing and uncertain (`state=-1`) cells cannot carry a flight path.
Truncated snapshots never authorize exploration. Launch gives localization, mapping, YOLO and control the same use_sim_time
setting. Header samples use ROS time; wall timers and steady receipt deadlines
also detect a frozen simulation clock.

Algorithm tie-breaking is deterministic in C++ (lexicographic voxel keys), so
exactly equal candidate priorities may choose a different equivalent viewpoint
than a Python set iteration. Trajectories are checked for known-free clearance
rather than requiring byte-identical path messages. Voxel publication order is
not an API guarantee. Numeric calculations use doubles and wire map fields use
float32, matching the Python pipeline.

Image saving now completes asynchronously; photo events and counters follow the
write result on a later control cycle. A photo is counted only after a fresh image is written successfully. Landing
completion requires fresh PX4 landed and disarmed states; timeout stays
unconfirmed. Invalid navigation latches FAULT and stops heartbeat/setpoints,
allowing the configured PX4 Offboard-loss behavior to take over. Recovery never
automatically rearms.

## Validation

Run the complete reproducible baseline after compilation:

```bash
bash src/px4_powerplant_mission_cpp/scripts/run_p0_baseline.sh
```

For individual checks, source the same middleware environment first:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source src/px4_powerplant_mission_cpp/scripts/p0_ros_env.sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 CTEST_PARALLEL_LEVEL=1 \
  colcon test --packages-select px4_powerplant_mission_cpp --executor sequential \
  --ctest-args -j1 --output-on-failure
colcon test-result --verbose
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  PYTHONPATH=src/px4_powerplant_mission:$PYTHONPATH /usr/bin/python3 -m pytest \
  -p no:cacheprovider -q src/px4_powerplant_mission/test
```

Native tests cover geometry, time history, map contracts, incremental indexes,
adaptive sampling, calibrated visibility, turn/dwell budgets, scan coverage,
view confirmation, planning dependencies, setpoint limits and cancellation.
The test-only `core_probe` performs JSON input/output for original planning
invariants and random obstacle routes. Legacy planning comparisons explicitly
neutralize new turn/dwell costs; native P1 tests exercise the production model.
Exact ray updates intentionally differ from the old Python fixed-step sampler
and are checked against independent segment geometry oracles. The ROS
contract tests start only one native process at a time, inject synthetic sensor
and PX4 messages, and verify parameter defaults, publications and services;
localization/EV tests also start separate Python processes for both profiles.
These tests do not replace a full PX4/Gazebo flight validation.
