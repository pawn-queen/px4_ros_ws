"""Ground step-response calibration: derivation bounds and frozen profile shape."""
import importlib.util
import json
import math
from pathlib import Path
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'validation/sitl'
MOTION = ROOT / 'src/px4_powerplant_mission/config/p2_simulation.yaml'


def module(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / (name + '.py'))
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


response = module('ground_response')
BASE = yaml.safe_load(MOTION.read_text())['powerplant_offboard_mission']['ros__parameters']


def trials(threshold=0.20, delay=1.2, repeats=2):
    rows = []
    for magnitude in (0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
        for _ in range(repeats):
            lifted = magnitude >= threshold
            rows.append({'magnitude_m': magnitude, 'lifted': lifted,
                         'delay_s': delay if lifted else None,
                         'peak_rise_m': magnitude if lifted else 0.01,
                         'settled_rise_m': magnitude if lifted else 0.0})
    return rows


def summary(threshold=0.20, delay=1.2):
    value = response.summarize_trials(trials(threshold, delay))
    value.update(position_sigma_m=.032, velocity_sigma_mps=.043, land_topic_period_s=1.0)
    return value


def test_threshold_is_the_smallest_magnitude_that_lifts_in_every_trial():
    value = response.summarize_trials(trials(threshold=.20))
    assert value['threshold_m'] == .20
    assert value['delay_s_max'] == pytest.approx(1.2)
    rows = {row['magnitude_m']: row for row in value['magnitudes']}
    assert rows[.15]['reliable'] is False
    assert rows[.15]['lifts'] == 0
    assert rows[.25]['reliable'] is True


def test_partial_or_thin_evidence_never_sets_the_threshold():
    rows = trials(threshold=.20)
    # Make .20 lift only once out of two: partial, so .25 becomes the threshold.
    rows[6]['lifted'] = False
    value = response.summarize_trials(rows)
    assert value['threshold_m'] == .25
    thin = [row for row in trials(threshold=.05) if row['magnitude_m'] == .05][:1]
    with pytest.raises(ValueError, match='no step magnitude'):
        response.summarize_trials(thin)


def test_stop_box_extends_below_and_above_the_hold_position():
    offsets = response.box_offsets({**response.DEFAULTS, **BASE}, .032, .043, .05)
    assert offsets['min_offset_m'] < 0 < offsets['max_offset_m']
    assert offsets['horizontal_inflation_m'] > 0
    bigger = response.box_offsets({**response.DEFAULTS, **BASE,
                                   'motion_tracking_position_tolerance_m': .35}, .032, .043, .05)
    assert bigger['min_offset_m'] < offsets['min_offset_m']


def test_derived_constants_satisfy_the_cpp_contract():
    value = response.derive_motion_constants(summary(), BASE, land_topic_period_s=1.0)
    params = value['parameters']
    assert params['motion_takeoff_bootstrap_height_m'] <= params['motion_tracking_position_tolerance_m']
    assert params['takeoff_response_probe_height_m'] > params['motion_takeoff_bootstrap_height_m']
    assert params['takeoff_handoff_min_rise_m'] > params['takeoff_response_probe_height_m']
    assert params['takeoff_handoff_min_rise_m'] <= response.HARD_MAX_RISE_M
    assert params['takeoff_region_height_m'] > response.DEFAULTS['takeoff_height_m']
    corner = value['evidence']['stop_box_horizontal_m'] * math.sqrt(2.)
    assert params['takeoff_region_radius_m'] >= corner
    assert params['takeoff_ground_land_max_age_s'] >= 10.
    assert response.validate_constants(params, BASE, value['evidence'])


def test_impossible_plant_is_rejected_instead_of_silently_relaxed():
    # A plant whose box is huge cannot hand off inside 0.75 m.
    bad = {**response.DEFAULTS, **BASE, 'motion_tracking_position_tolerance_m': 3.0}
    tiny = {'threshold_m': .05, 'delay_s_median': .5, 'delay_s_max': 1.0,
            'position_sigma_m': .5, 'velocity_sigma_mps': .1}
    with pytest.raises(ValueError, match='no handoff rise'):
        response.derive_motion_constants(tiny, bad)


def test_freeze_writes_a_loadable_motion_profile_with_provenance(tmp_path):
    value = response.derive_motion_constants(summary(), BASE, land_topic_period_s=1.0)
    trials_path = tmp_path / 'trials.json'
    trials_path.write_text(json.dumps({'trials': trials()}))
    output = tmp_path / 'p2_sitl_ground_response.yaml'
    provenance = response.freeze_motion_profile(
        MOTION, value, output=output, source_motion=MOTION,
        trials=summary(), trials_path=trials_path)
    doc = yaml.safe_load(output.read_text())
    control = doc['powerplant_offboard_mission']['ros__parameters']
    for name, expected in value['parameters'].items():
        assert control[name] == expected, name
    assert doc['ground_response_provenance']['status'] == 'frozen'
    assert doc['ground_response_provenance']['trials_sha256'] == provenance['trials_sha256']
    assert 'powerplant_voxel_mapper' in doc


FROZEN = ROOT / 'src/px4_powerplant_mission/config/p2_sitl_ground_response.yaml'
CONTROL_KEYS = ('motion_tracking_position_tolerance_m', 'motion_takeoff_bootstrap_height_m',
                'motion_takeoff_bootstrap_duration_s', 'takeoff_response_probe_height_m',
                'takeoff_handoff_min_rise_m', 'takeoff_ground_land_max_age_s',
                'takeoff_ground_transition_timeout_s', 'takeoff_region_radius_m',
                'takeoff_region_height_m')


def test_frozen_profile_is_committed_and_satisfies_the_contract():
    doc = yaml.safe_load(FROZEN.read_text())
    provenance = doc['ground_response_provenance']
    assert provenance['status'] == 'frozen'
    assert provenance['measurements']['threshold_m'] > 0
    control = doc['powerplant_offboard_mission']['ros__parameters']
    parameters = {key: control[key] for key in CONTROL_KEYS}
    limits = {**BASE, **yaml.safe_load(
        (ROOT / 'src/px4_powerplant_mission/config/powerplant.yaml').read_text()
    )['powerplant_offboard_mission']['ros__parameters']}
    assert response.validate_constants(parameters, limits, provenance['measurements'])
    assert parameters['motion_takeoff_bootstrap_height_m'] >= \
        provenance['measurements']['threshold_m'] - 1e-9


def test_acceptance_policy_accepts_the_frozen_calibration_only_with_it():
    controller = module('controller_profile')
    frozen = module('ground_response').load_frozen(FROZEN)
    mission = yaml.safe_load(
        (ROOT / 'src/px4_powerplant_mission/config/powerplant.yaml').read_text()
    )['powerplant_offboard_mission']['ros__parameters']
    simulation = yaml.safe_load(
        (ROOT / 'src/px4_powerplant_mission/config/p2_simulation.yaml').read_text()
    )['powerplant_offboard_mission']['ros__parameters']
    # Same effective profile the runner assembles: simulation motion profile,
    # audited support flags, then the frozen ground-response constants.
    parameters = {**mission, **simulation, **frozen['parameters'],
                  'takeoff_transition_budget_enabled': True}
    passed = controller.validate_acceptance_policy(parameters, ground_response=frozen)
    assert passed['passed'] is True
    calibrated = passed['ground_response_calibration']
    assert calibrated['parameters']['motion_takeoff_bootstrap_height_m'] == \
        frozen['parameters']['motion_takeoff_bootstrap_height_m']
    # The historical fixed table (0.02 / 0.10 / 0.20) must reject the calibrated
    # profile: the frozen file is the only source that can authorise it.
    with pytest.raises(ValueError, match='acceptance policy differs'):
        controller.validate_acceptance_policy(parameters)
