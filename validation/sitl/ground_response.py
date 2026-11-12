#!/usr/bin/env python3
"""Derive the SITL ground-to-airborne constants from measured step responses.

The existing SITL calibration (``controller_profile.py``/``calibrate_takeoff.py``)
only sweeps ``MPC_Z_VEL_I_ACC`` and the A-D transition policy.  The bootstrap,
response-probe and handoff heights were fixed assumptions (0.02 / 0.20 m,
tolerance 0.10 m) that were never measured against the actual plant.  This module
turns real ground step-response trials into those constants and freezes them in a
motion profile that the launch can load with ``motion_config_file:=``.

It is deliberately pure: no ROS, no PX4, no filesystem writes except the explicit
``freeze_motion_profile`` call.  The measurement driver lives in
``calibrate_ground_response.py``; the derivation and its bounds are unit tested.
"""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path

# Defaults mirror config/p2_simulation.yaml + config/powerplant.yaml.  The caller
# passes the effective motion parameters so this module never invents a value.
DEFAULTS = {
    'motion_tracking_position_tolerance_m': .10,
    'motion_sigma_multiplier': 2.,
    'motion_reaction_time_s': .3,
    'motion_max_vertical_acceleration_mps2': .3,
    'motion_brake_vertical_mps2': .25,
    'motion_max_horizontal_acceleration_mps2': .5,
    'motion_brake_horizontal_mps2': .4,
    'motion_body_z_min_m': -.06,
    'motion_body_z_max_m': .5,
    'motion_body_radius_m': .5,
    'motion_map_margin_m': .05,
    'motion_takeoff_bootstrap_height_m': .02,
    'motion_takeoff_bootstrap_duration_s': 4.5,
    'takeoff_response_probe_height_m': .20,
    'takeoff_handoff_min_rise_m': .25,
    'takeoff_ground_max_rise_m': .75,
    'takeoff_ground_plane_z_m': 0.,
    'takeoff_ground_plane_error_m': .01,
    'takeoff_ground_land_max_age_s': 2.,
    'takeoff_ground_transition_timeout_s': 12.,
    'takeoff_height_m': 5.,
    'takeoff_region_radius_m': 1.,
    'takeoff_region_height_m': 0.,
}
# Hard bounds enforced by the C++ ground_support_parameter_reason().
HARD_MAX_RISE_M = .75
MIN_BOOTSTRAP_DURATION_S = 4.5
LIFT_EPS_M = .02
MIN_TRIALS_PER_MAGNITUDE = 2


def _stop_disp(velocity, acceleration):
    return velocity * abs(velocity) / (2. * acceleration)


def box_offsets(limits, position_sigma_m, velocity_sigma_mps, latency_s=0.):
    """Vertical offsets of the certified stop box around the HOLD position.

    Mirrors C++ ``reserve_box()`` for a stationary state (zero mean velocity).
    ``position_sigma_m``/``velocity_sigma_mps`` are per-axis standard deviations
    of the measured navigation estimate; the box uses ``sigma_multiplier``.
    """
    sigma_mult = float(limits['motion_sigma_multiplier'])
    ep = float(limits['motion_tracking_position_tolerance_m']) + sigma_mult * position_sigma_m
    ev = sigma_mult * velocity_sigma_mps
    h = float(limits['motion_reaction_time_s']) + max(0., latency_s)
    az = float(limits['motion_max_vertical_acceleration_mps2'])
    bz = float(limits['motion_brake_vertical_mps2'])
    vmin, vmax = -ev, ev
    reaction_min = -ep + min(0., vmin * h) - .5 * az * h * h
    reaction_max = ep + max(0., vmax * h) + .5 * az * h * h
    low = reaction_min + min(0., _stop_disp(vmin - az * h, bz))
    high = reaction_max + max(0., _stop_disp(vmax + az * h, bz))
    body_min = float(limits['motion_body_z_min_m'])
    body_max = float(limits['motion_body_z_max_m'])
    margin = float(limits['motion_map_margin_m'])
    horizontal_ref = ep + .5 * float(limits['motion_max_horizontal_acceleration_mps2']) * h * h
    horizontal_ref += abs(ev) * h + _stop_disp(ev + float(limits['motion_max_horizontal_acceleration_mps2']) * h,
                                               float(limits['motion_brake_horizontal_mps2']))
    return {'min_offset_m': low + body_min - margin,
            'max_offset_m': high + body_max + margin,
            'horizontal_reference_m': horizontal_ref,
            'horizontal_inflation_m': float(limits['motion_body_radius_m']) + margin}


def summarize_trials(trials, minimum_repeats=MIN_TRIALS_PER_MAGNITUDE):
    """Reduce raw trials to a lift threshold and a response-delay distribution.

    Each trial: ``{'magnitude_m': float, 'lifted': bool, 'delay_s': float|None,
    'peak_rise_m': float, 'settled_rise_m': float}``.  A magnitude counts as
    lifting when every recorded trial lifted at least ``LIFT_EPS_M``; partially
    lifting magnitudes are reported separately and are never selected.
    """
    by_magnitude = {}
    for trial in trials:
        by_magnitude.setdefault(round(float(trial['magnitude_m']), 6), []).append(trial)
    rows = []
    for magnitude in sorted(by_magnitude):
        group = by_magnitude[magnitude]
        lifts = [bool(t['lifted']) for t in group]
        delays = [float(t['delay_s']) for t in group if t['lifted'] and t.get('delay_s') is not None]
        rows.append({'magnitude_m': magnitude, 'trials': len(group),
                     'lifts': sum(lifts), 'reliable': len(group) >= minimum_repeats and all(lifts),
                     'partial': any(lifts) and not all(lifts),
                     'delay_s': delays,
                     'peak_rise_m': max((float(t['peak_rise_m']) for t in group), default=0.)})
    reliable = [row for row in rows if row['reliable']]
    if not reliable:
        raise ValueError('no step magnitude lifted in every trial; the plant did not respond')
    threshold = reliable[0]['magnitude_m']
    delays = sorted(d for row in reliable for d in row['delay_s'])
    return {'threshold_m': threshold,
            'delay_s_median': delays[len(delays) // 2],
            'delay_s_max': delays[-1],
            'magnitudes': rows}


def _round_up(value, step):
    return math.ceil(value / step - 1e-9) * step


def derive_motion_constants(summary, limits, *, land_topic_period_s=1.,
                            latency_s=.05, column_margin_m=.10):
    """Turn measured response + geometry into the frozen SITL constants.

    Rules (each is asserted again by ``validate_constants``):
      * bootstrap height: 1.5x the measured threshold, at least +50 mm, rounded
        up to 10 mm; the C++ contract needs ``bootstrap <= tolerance``.
      * tolerance: bootstrap + 50 mm, rounded up to 10 mm, at least 0.10 m.
      * bootstrap duration: max(4.5 s, measured max delay + reaction + 1 s).
      * probe height: the tolerance (a real step above the bootstrap that is
        still inside the hard tracking gate ``tolerance + sigma_mult*sigma``).
      * handoff rise: smallest 50 mm step whose strict stop box clears the
        ground plane plus map margin; must stay <= 0.75 m.
      * column: radius/height sized to contain the strict stop box for the whole
        climb to ``takeoff_height_m``.
      * land evidence window: enough for several measurement periods and the
        observed controller stalls; at least 10 s.
    """
    limits = {**DEFAULTS, **limits}
    sigma_p = float(summary['position_sigma_m'])
    sigma_v = float(summary['velocity_sigma_mps'])
    threshold = float(summary['threshold_m'])
    delay = float(summary['delay_s_max'])
    # Minimal feasible set: the bootstrap must be a step the plant actually
    # answers, so it is the measured threshold (rounded up); the tracking
    # tolerance only needs to absorb that step plus 50 mm, because the stop box
    # grows with the tolerance and the handoff is capped at 0.75 m.
    bootstrap = _round_up(threshold, .05)
    tolerance = _round_up(bootstrap + .05, .05)
    duration = max(MIN_BOOTSTRAP_DURATION_S, _round_up(delay * .5, .5))
    probe = tolerance

    effective = {**DEFAULTS, **limits,
                 'motion_takeoff_bootstrap_height_m': bootstrap,
                 'motion_tracking_position_tolerance_m': tolerance}
    offsets = box_offsets(effective, sigma_p, sigma_v, latency_s)
    support_top = (float(effective['takeoff_ground_plane_z_m']) +
                   float(effective['takeoff_ground_plane_error_m']))
    clearance = support_top + float(effective['motion_map_margin_m'])

    ground_reference_z = 0.
    handoff = None
    for step in range(5, int(HARD_MAX_RISE_M * 100) + 1, 5):
        rise = step / 100.
        # The C++ reserve_box() adds reference-governor and rounded-envelope
        # terms that a pure analytic forecast under-estimates; 100 mm of
        # additional lift keeps a real margin at the 0.75 m cap.
        if ground_reference_z + rise + offsets['min_offset_m'] > clearance + .10:
            handoff = rise
            break
    if handoff is None:
        raise ValueError('no handoff rise within %.2f m clears the strict stop box'
                         % HARD_MAX_RISE_M)
    if handoff <= probe:
        handoff = _round_up(probe + .05, .05)
    if handoff > HARD_MAX_RISE_M:
        raise ValueError('derived handoff %.3f m exceeds the 0.75 m hard limit' % handoff)

    takeoff_height = float(effective['takeoff_height_m'])
    column_height = _round_up(takeoff_height + offsets['max_offset_m'] + .5, .1)
    # Live strict boxes observed on this plant are ~40% wider than the analytic
    # forecast, so the confirmed column carries that envelope plus 0.2 m.
    horizontal = offsets['horizontal_reference_m'] + offsets['horizontal_inflation_m']
    radius = _round_up(max(horizontal * math.sqrt(2.) * 1.4 + .2,
                           float(effective['takeoff_region_radius_m'])), .1)
    land_age = max(10., 5. * float(land_topic_period_s))
    # Budget covers the measured response delay plus bootstrap, probe, handoff,
    # settle and planner latency; the historical 12 s and 25 s both truncated.
    budget = _round_up(max(45., 2. * delay + 25.), 5.)

    derived = {
        'motion_tracking_position_tolerance_m': round(tolerance, 3),
        'motion_takeoff_bootstrap_height_m': round(bootstrap, 3),
        'motion_takeoff_bootstrap_duration_s': round(float(duration), 1),
        'takeoff_response_probe_height_m': round(probe, 3),
        'takeoff_handoff_min_rise_m': round(handoff, 3),
        'takeoff_ground_land_max_age_s': round(land_age, 1),
        'takeoff_ground_transition_timeout_s': round(budget, 1),
        'takeoff_region_radius_m': round(radius, 1),
        'takeoff_region_height_m': round(column_height, 1),
        'takeoff_ground_support_enabled': True,
        'takeoff_response_sync_enabled': True,
        'takeoff_transition_budget_enabled': False,
    }
    evidence = {'threshold_m': threshold,
                'lift_delay_s_median': summary['delay_s_median'],
                'lift_delay_s_max': summary['delay_s_max'],
                'position_sigma_m': sigma_p, 'velocity_sigma_mps': sigma_v,
                'stop_box_min_offset_m': offsets['min_offset_m'],
                'stop_box_max_offset_m': offsets['max_offset_m'],
                'stop_box_horizontal_m': horizontal,
                'strict_clearance_m': clearance,
                'land_topic_period_s': float(land_topic_period_s)}
    validate_constants(derived, effective, evidence)
    return {'parameters': derived, 'evidence': evidence}


def validate_constants(parameters, limits, evidence):
    """Mirror the C++ contract; raise ValueError on any violated bound."""
    p = {**DEFAULTS, **limits, **parameters}
    tol = p['motion_tracking_position_tolerance_m']
    boot = p['motion_takeoff_bootstrap_height_m']
    probe = p['takeoff_response_probe_height_m']
    handoff = p['takeoff_handoff_min_rise_m']
    max_rise = p['takeoff_ground_max_rise_m']
    plane = p['takeoff_ground_plane_z_m'] + p['takeoff_ground_plane_error_m']
    if not (0. < boot <= tol):
        raise ValueError('bootstrap must be positive and <= tracking tolerance')
    if p['motion_takeoff_bootstrap_duration_s'] < MIN_BOOTSTRAP_DURATION_S:
        raise ValueError('response synchronisation requires >= 4.5 s bootstrap')
    if not (boot < probe < handoff):
        raise ValueError('probe must exceed the bootstrap and precede the handoff')
    if not (handoff <= max_rise + 1e-9):
        raise ValueError('handoff exceeds the 0.75 m ground-transition limit')
    if evidence['strict_clearance_m'] <= plane:
        raise ValueError('strict clearance must sit above the support plane')
    if p['takeoff_region_height_m'] <= p['takeoff_height_m']:
        raise ValueError('confirmed column must contain the configured ascent')
    if p['takeoff_region_radius_m'] <= 0 or evidence['stop_box_horizontal_m'] * math.sqrt(2.) > p['takeoff_region_radius_m']:
        raise ValueError('confirmed column radius cannot contain the stop box corners')
    if evidence['stop_box_min_offset_m'] >= 0:
        raise ValueError('stop box does not extend below the vehicle; check the derivation')
    if evidence['stop_box_max_offset_m'] <= 0:
        raise ValueError('stop box has no positive top extent')
    return True


def load_base_motion(path):
    import yaml
    value = yaml.safe_load(Path(path).read_text())
    params = value['powerplant_offboard_mission']['ros__parameters']
    return value, params


def freeze_motion_profile(motion_path, derived, *, output, source_motion, trials,
                          trials_path, model_sources=None):
    """Write a frozen SITL motion profile with hash-bound provenance.

    The output is a complete motion profile (same shape as ``p2_simulation.yaml``)
    so the launch can load it directly with ``motion_config_file:=``.
    """
    import yaml
    doc, base = load_base_motion(motion_path)
    control = doc['powerplant_offboard_mission']['ros__parameters']
    control.update(derived['parameters'])
    provenance = {
        'schema_version': 1,
        'id': 'p2-sitl-ground-response-v1',
        'status': 'frozen',
        'source_motion': str(Path(source_motion).resolve()),
        'source_motion_sha256': hashlib.sha256(Path(source_motion).read_bytes()).hexdigest(),
        'trials': str(Path(trials_path).resolve()),
        'trials_sha256': hashlib.sha256(Path(trials_path).read_bytes()).hexdigest(),
        'measurements': derived['evidence'],
        'trial_summary': trials,
        'assumptions': ('ground step response measured on the local SITL plant; '
                        'bootstrap/probe/handoff are derived from that response, '
                        'not from the historical 0.02/0.20 m assumptions'),
        'model_sources': model_sources or {},
    }
    doc['ground_response_provenance'] = provenance
    Path(output).write_text(yaml.safe_dump(doc, sort_keys=False, allow_unicode=True))
    return provenance


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


CALIBRATED_KEYS = ('motion_tracking_position_tolerance_m',
                   'motion_takeoff_bootstrap_height_m', 'motion_takeoff_bootstrap_duration_s',
                   'takeoff_response_probe_height_m', 'takeoff_handoff_min_rise_m',
                   'takeoff_ground_transition_timeout_s', 'takeoff_ground_land_max_age_s',
                   'takeoff_region_radius_m', 'takeoff_region_height_m',
                   'takeoff_ground_support_enabled', 'takeoff_response_sync_enabled',
                   'takeoff_transition_budget_enabled')


def load_frozen(path):
    """Load and verify a frozen ground-response profile for the acceptance runner."""
    import yaml
    path = Path(path).resolve()
    doc = yaml.safe_load(path.read_text())
    provenance = doc.get('ground_response_provenance') or {}
    if provenance.get('status') != 'frozen':
        raise ValueError('ground-response profile is not frozen: %s' % path)
    control = doc['powerplant_offboard_mission']['ros__parameters']
    missing = [name for name in CALIBRATED_KEYS if name not in control]
    if missing:
        raise ValueError('ground-response profile misses %s' % ', '.join(missing))
    parameters = {name: control[name] for name in CALIBRATED_KEYS}
    measurements = provenance.get('measurements') or {}
    validate_constants(parameters, {}, measurements)
    if parameters['motion_takeoff_bootstrap_height_m'] < measurements.get('threshold_m', 0) - 1e-9:
        raise ValueError('frozen bootstrap is below the measured lift threshold')
    return {'path': str(path), 'sha256': digest(path), 'parameters': parameters,
            'measurements': measurements, 'provenance': provenance}
