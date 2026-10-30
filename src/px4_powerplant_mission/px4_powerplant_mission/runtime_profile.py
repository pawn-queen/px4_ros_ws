"""Resolve launch/runtime profiles from one manifest.

The manifest owns the profile-to-config mapping and the defaults that used to
be embedded in launch expressions. The launch file keeps its old arguments as a
compatibility layer and adds a preferred ``profile`` argument.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping


MANIFEST_NAME = "runtime_profiles.json"
SCHEMA_VERSION = 1
DEFAULT_KEYS = (
    "use_uwb",
    "use_gazebo_truth_bridge",
    "use_gazebo_clock_bridge",
    "use_sim_time",
)


@dataclass(frozen=True)
class RuntimeProfile:
    """A named runtime profile and its config ownership."""

    name: str
    localization_config: str
    motion_config: str
    defaults: Mapping[str, str]

    def config_path(self, share_dir: Path | str, kind: str) -> Path:
        if kind == "localization":
            filename = self.localization_config
        elif kind == "motion":
            filename = self.motion_config
        else:
            raise ValueError(f"unknown profile config kind: {kind}")
        return Path(share_dir) / "config" / filename


def _read_manifest(share_dir: Path | str) -> Mapping[str, object]:
    path = Path(share_dir) / "config" / MANIFEST_NAME
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError as error:
        raise RuntimeError(f"runtime profile manifest is missing: {path}") from error
    if not isinstance(data, dict) or data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported runtime profile manifest schema: {path}")
    profiles = data.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError(f"runtime profile manifest has no profiles: {path}")
    return data


def load_manifest(share_dir: Path | str) -> dict[str, RuntimeProfile]:
    """Load and validate every profile from ``config/runtime_profiles.json``."""
    data = _read_manifest(share_dir)
    result: dict[str, RuntimeProfile] = {}
    for name, entry in data["profiles"].items():
        if not isinstance(name, str) or not name:
            raise ValueError("runtime profile names must be non-empty strings")
        if not isinstance(entry, dict):
            raise ValueError(f"runtime profile {name!r} must be an object")
        localization = entry.get("localization_config")
        motion = entry.get("motion_config")
        defaults = entry.get("defaults")
        if not isinstance(localization, str) or not localization:
            raise ValueError(f"runtime profile {name!r} lacks localization_config")
        if not isinstance(motion, str) or not motion:
            raise ValueError(f"runtime profile {name!r} lacks motion_config")
        if not isinstance(defaults, dict):
            raise ValueError(f"runtime profile {name!r} lacks defaults")
        for key in DEFAULT_KEYS:
            value = defaults.get(key)
            if not isinstance(value, str) or value not in ("true", "false"):
                raise ValueError(f"runtime profile {name!r} has invalid {key}: {value!r}")
        result[name] = RuntimeProfile(name, localization, motion, dict(defaults))
    return result


def resolve_profile(
    profile_name: str | None,
    localization_profile: str | None,
    profiles: Mapping[str, RuntimeProfile],
) -> RuntimeProfile:
    """Prefer ``profile``; fall back to the legacy ``localization_profile``."""
    selected = (profile_name or "").strip() or (localization_profile or "").strip() or "simulation"
    try:
        return profiles[selected]
    except KeyError as error:
        raise ValueError(f"unknown runtime profile: {selected!r}") from error


def resolve_defaults(
    profile: RuntimeProfile,
    requested: Mapping[str, str | None] | None = None,
) -> dict[str, str]:
    """Resolve explicit launch values, then fall back to profile defaults."""
    requested = requested or {}
    result: dict[str, str] = {}
    for key in DEFAULT_KEYS:
        value = (requested.get(key) or "").strip()
        if value and value not in ("true", "false"):
            raise ValueError(f"{key} must be true or false, got {value!r}")
        result[key] = value or profile.defaults[key]
    return result
