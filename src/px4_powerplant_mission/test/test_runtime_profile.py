from pathlib import Path

import pytest

from px4_powerplant_mission.runtime_profile import (
    DEFAULT_KEYS,
    load_manifest,
    resolve_defaults,
    resolve_profile,
)


SHARE = Path(__file__).resolve().parents[1]


def profiles():
    return load_manifest(SHARE)


def test_manifest_loads_both_profiles_and_config_owners():
    loaded = profiles()
    assert set(loaded) == {"simulation", "hardware"}
    assert loaded["simulation"].localization_config == "localization_simulation.yaml"
    assert loaded["simulation"].motion_config == "p2_simulation.yaml"
    assert loaded["hardware"].localization_config == "localization_hardware.yaml"
    assert loaded["hardware"].motion_config == "p2_hardware.yaml"


def test_profile_prefers_new_argument_then_legacy_argument():
    loaded = profiles()
    assert resolve_profile("hardware", "simulation", loaded).name == "hardware"
    assert resolve_profile("", "simulation", loaded).name == "simulation"
    assert resolve_profile("", "hardware", loaded).name == "hardware"


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError, match="unknown runtime profile"):
        resolve_profile("missing", "", profiles())


def test_profile_defaults_and_explicit_overrides():
    loaded = profiles()
    sim = resolve_defaults(loaded["simulation"])
    hw = resolve_defaults(loaded["hardware"])
    assert sim == {key: "true" for key in DEFAULT_KEYS}
    assert hw == {key: "false" for key in DEFAULT_KEYS}
    overridden = resolve_defaults(loaded["simulation"], {"use_sim_time": "false", "use_uwb": ""})
    assert overridden["use_sim_time"] == "false"
    assert overridden["use_uwb"] == "true"


def test_invalid_boolean_is_rejected():
    with pytest.raises(ValueError, match="must be true or false"):
        resolve_defaults(profiles()["simulation"], {"use_uwb": "yes"})
