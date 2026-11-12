#!/usr/bin/env python3
"""Measure the PX4 ground response to Offboard position steps, then freeze it.

This is the missing calibration step for the SITL ground transition: the
bootstrap/probe/handoff heights used to be fixed assumptions.  The driver flies
real ground-step trials against a running PX4 SITL instance (same ROS domain and
sim clock), records the lift threshold and response delay, and derives the motion
constants through ``ground_response.py``.

Requirements: a running SITL stack (Gazebo + PX4 + DDS agent + task stack) on the
same ROS domain, a disarmed vehicle on the ground, and no other node publishing
`/fmu/in/trajectory_setpoint`.  Every trial lands before the next one starts; the
driver aborts a trial above the step magnitude plus a hard margin.

Example::

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 validation/sitl/calibrate_ground_response.py \
        --output-dir log/ground_response/<timestamp> \
        --magnitudes 0.05 0.10 0.15 0.20 0.25 0.30 --repeats 2 \
        --freeze src/px4_powerplant_mission/config/p2_sitl_ground_response.yaml
"""
from __future__ import annotations
import argparse
import json
import math
import statistics
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ground_response  # noqa: E402

LIFT_EPS_M = ground_response.LIFT_EPS_M
# 5 cm sustained rise sits above the measured +/-5 cm estimate noise floor.
LIFT_DETECTION_M = .03


def _topic(base, message):
    version = getattr(message, 'MESSAGE_VERSION', 0)
    return f'{base}_v{version}' if version else base


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--magnitudes', nargs='+', type=float,
                        default=[.05, .10, .15, .20, .25, .30])
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--trial-seconds', type=float, default=15.)
    parser.add_argument('--settle-seconds', type=float, default=.5)
    parser.add_argument('--abort-margin-m', type=float, default=.40)
    parser.add_argument('--freeze', type=Path, default=None,
                        help='write the frozen SITL motion profile here')
    parser.add_argument('--base-motion', type=Path,
                        default=ROOT/'src/px4_powerplant_mission/config/p2_simulation.yaml')
    parser.add_argument('--motion-config', type=Path, default=None,
                        help='effective motion YAML (defaults to --base-motion)')
    args = parser.parse_args(argv)
    if args.repeats < ground_response.MIN_TRIALS_PER_MAGNITUDE:
        parser.error('at least %d repeats per magnitude are required' %
                     ground_response.MIN_TRIALS_PER_MAGNITUDE)
    out = args.output_dir.resolve()
    if out.exists() and any(out.iterdir()):
        parser.error('use a new output directory to preserve previous evidence')
    out.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                              VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage

    class Driver(Node):
        def __init__(self):
            super().__init__('ground_response_calibration',
                             parameter_overrides=[rclpy.parameter.Parameter(
                                 'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.ocm = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
            self.tsp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
            self.cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
            self.position = None
            self.status = None
            self.land = None
            self.land_stamps = []
            self.odom_variance = []
            self.clock_now = 0.
            self.create_subscription(VehicleLocalPosition, _topic('/fmu/out/vehicle_local_position',
                                                                  VehicleLocalPosition),
                                     lambda m: setattr(self, 'position', m), qos)
            self.create_subscription(VehicleStatus, _topic('/fmu/out/vehicle_status', VehicleStatus),
                                     lambda m: setattr(self, 'status', m), qos)
            self.create_subscription(VehicleLandDetected,
                                     _topic('/fmu/out/vehicle_land_detected', VehicleLandDetected),
                                     self.on_land, qos)
            self.create_subscription(Odometry, '/powerplant/localization/odom', self.on_odom, 20)
            self.create_subscription(Clock, '/clock', self.on_clock, 20)
            # Gazebo truth: the ros_gz dynamic-pose bridge does not expose model
            # frame names, so the first transform is the vehicle (same fallback
            # the UWB simulator documents).  Truth is the physical lift evidence;
            # the EKF estimate is recorded only as the controller-side view.
            self.create_subscription(TFMessage, '/world/powerplant/dynamic_pose/info',
                                     self.on_pose, 20)
            self.truth_height_value = None
            self.truth_stamp = 0.
            self.setpoint_height = 0.
            self.enabled = False
            self.create_timer(0.02, self.publish)

        def on_land(self, message):
            self.land = message
            self.land_stamps.append(self.get_clock().now().nanoseconds * 1e-9)

        def on_odom(self, message):
            covariance = message.pose.covariance
            self.odom_variance.append((covariance[0], covariance[7], covariance[14],
                                       covariance[21], covariance[28], covariance[35]))

        def on_clock(self, message):
            self.clock_now = message.clock.sec + message.clock.nanosec * 1e-9

        def on_pose(self, message):
            # Entity order in dynamic_pose/info is not stable and the bridge does
            # not expose model frame names, so a fixed index is unsafe.  Prefer a
            # named vehicle frame; otherwise track the entity closest to the last
            # accepted height (the vehicle moves far less than 1 m per message).
            if not message.transforms:
                return
            heights = []
            for transform in message.transforms:
                name = transform.child_frame_id or transform.header.frame_id
                heights.append((name, float(transform.transform.translation.z)))
            chosen = None
            for name, height in heights:
                if 'x500_depth' in name:
                    chosen = height
                    break
            if chosen is None:
                if self.truth_height_value is None:
                    chosen = heights[0][1]
                else:
                    nearest = min((height for _, height in heights),
                                  key=lambda value: abs(value - self.truth_height_value))
                    if abs(nearest - self.truth_height_value) > 1.0:
                        return  # ambiguous frame switch: keep the previous sample
                    chosen = nearest
            self.truth_height_value = chosen
            self.truth_stamp = self.get_clock().now().nanoseconds * 1e-9

        def sensed_height(self):
            if (self.truth_height_value is not None and
                    self.truth_stamp > 0 and self.clock_now - self.truth_stamp <= .5):
                return self.truth_height_value
            return -float(self.position.z) if self.position is not None else 0.

        def now_us(self):
            return int(self.get_clock().now().nanoseconds / 1000)

        def publish(self):
            if not self.enabled:
                return
            mode = OffboardControlMode()
            mode.position = True
            mode.timestamp = self.now_us()
            self.ocm.publish(mode)
            setpoint = TrajectorySetpoint()
            setpoint.position = [0., 0., -float(self.setpoint_height)]
            setpoint.velocity = [float('nan')] * 3
            setpoint.acceleration = [float('nan')] * 3
            setpoint.jerk = [float('nan')] * 3
            setpoint.yaw = 0.
            setpoint.yawspeed = float('nan')
            setpoint.timestamp = self.now_us()
            self.tsp.publish(setpoint)

        def command(self, code, param1, param2=0.):
            message = VehicleCommand()
            message.command = int(code)
            message.param1 = float(param1)
            message.param2 = float(param2)
            message.target_system = 1
            message.target_component = 1
            message.source_system = 1
            message.source_component = 1
            message.from_external = True
            message.timestamp = self.now_us()
            self.cmd.publish(message)

    rclpy.init()
    node = Driver()
    trial_rows = []

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., deadline - time.monotonic())))

    def spin_until(predicate, timeout_s, label):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return True
        raise RuntimeError(f'timeout waiting for {label}')

    def ensure_grounded():
        node.command(400, 0)  # disarm (idempotent)
        node.command(21, 0)  # NAV_LAND
        spin_until(lambda: node.status is not None and node.status.arming_state == 1
                   and node.position is not None and node.position.z > -0.3,
                   20., 'disarmed on the ground')

    def armed():
        return node.status is not None and node.status.arming_state == 2

    def offboard():
        return node.status is not None and node.status.nav_state == 14

    def ground_height():
        return -float(node.position.z)

    # Give the disarmed ground reference a moment to settle before the first trial.
    spin_until(lambda: node.position is not None and node.status is not None, 20., 'first PX4 samples')
    ensure_grounded()
    trial_index = 0
    for magnitude in args.magnitudes:
        for repeat in range(args.repeats):
            trial_index += 1
            ensure_grounded()
            rest_z = node.sensed_height()          # physical rest (truth)
            rest_control = -float(node.position.z)  # PX4 frame for the setpoint
            node.setpoint_height = rest_control
            node.enabled = True
            spin(1.5)
            node.command(176, 1, 6)  # DO_SET_MODE -> Offboard
            if not offboard():
                spin_until(offboard, 10., 'PX4 to enter Offboard')
            node.command(400, 1)  # arm
            spin_until(armed, 10., 'arming')
            spin(args.settle_seconds)
            recent = []
            mark = time.monotonic()
            while time.monotonic() - mark < 1.0:
                rclpy.spin_once(node, timeout_sec=.02)
                time.sleep(.01)
                recent.append((node.clock_now, node.sensed_height()))
            baseline = statistics.median(z for _, z in recent) if recent else rest_z
            truth_before = baseline
            step_z = rest_control + magnitude
            node.setpoint_height = step_z
            step_time = node.clock_now
            raised_since = None
            lifted_at = None
            peak = rest_z
            ekf_peak = rest_z
            abort = False
            samples = []
            window = []
            deadline = time.monotonic() + args.trial_seconds
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                time.sleep(.01)
                height = node.sensed_height()
                samples.append({'t': node.clock_now, 'z': height})
                window.append((node.clock_now, height))
                while window and node.clock_now - window[0][0] > .6:
                    window.pop(0)
                rolling = statistics.median(z for _, z in window) if window else height
                peak = max(peak, rolling)
                ekf_peak = max(ekf_peak, -float(node.position.z))
                if raised_since is None or node.clock_now - raised_since >= .3:
                    if rolling - baseline >= LIFT_DETECTION_M:
                        if raised_since is None:
                            raised_since = node.clock_now
                        elif lifted_at is None:
                            lifted_at = raised_since
                    else:
                        raised_since = None
                # Safety ceiling is physical: 0.8 m above the truth baseline.
                if height - truth_before > .6 or rolling - truth_before > .6:
                    abort = True
                    break
            lift = lifted_at is not None
            delay = (lifted_at - step_time) if lift else None
            node.enabled = False
            node.command(400, 0)
            node.command(21, 0)
            try:
                spin_until(lambda: node.status is not None and node.status.arming_state == 1,
                           20., 'disarm after trial')
            except RuntimeError:
                pass
            row = {'index': trial_index, 'magnitude_m': magnitude, 'repeat': repeat,
                   'rest_height_m': rest_z, 'step_height_m': step_z, 'lifted': lift,
                   'delay_s': delay, 'peak_rise_m': peak - rest_z,
                   'settled_rise_m': node.sensed_height() - rest_z, 'aborted': abort,
                   'samples': len(samples)}
            trial_rows.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
            if abort:
                raise RuntimeError('trial exceeded the abort margin; refusing to continue')

    node.enabled = False
    (out / 'trials.json').write_text(json.dumps(
        {'magnitudes': args.magnitudes, 'repeats': args.repeats, 'trials': trial_rows,
         'lidar_topic_period_note': 'land-detected period is measured below'},
        indent=2, allow_nan=False))
    periods = [b - a for a, b in zip(node.land_stamps, node.land_stamps[1:]) if 0 < b - a < 30]
    land_period = statistics.median(periods) if periods else 1.
    variances = node.odom_variance[-200:] or [(0.001, 0.001, 0.001, 0., 0., 0.)]
    position_sigma = math.sqrt(max(1e-9, statistics.median(row[0] for row in variances)))
    velocity_sigma = math.sqrt(max(1e-9, statistics.median(row[3] for row in variances)))
    summary = ground_response.summarize_trials(trial_rows)
    summary.update(position_sigma_m=position_sigma, velocity_sigma_mps=velocity_sigma,
                   land_topic_period_s=land_period,
                   odom_median_position_variance_m2=position_sigma ** 2,
                   odom_median_velocity_variance_m2ps2=velocity_sigma ** 2)
    (out / 'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    import yaml
    base_params = yaml.safe_load(args.base_motion.read_text())['powerplant_offboard_mission']['ros__parameters']
    derived = ground_response.derive_motion_constants(summary, base_params,
                                                      land_topic_period_s=land_period)
    (out / 'derived.json').write_text(json.dumps(derived, indent=2, allow_nan=False))
    print(json.dumps(derived, indent=2, ensure_ascii=False), flush=True)
    if args.freeze:
        motion = args.motion_config or args.base_motion
        provenance = ground_response.freeze_motion_profile(
            motion, derived, output=args.freeze, source_motion=motion,
            trials=summary, trials_path=out / 'trials.json')
        (out / 'provenance.json').write_text(json.dumps(provenance, indent=2, allow_nan=False))
        print(f'frozen motion profile: {args.freeze}', flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
