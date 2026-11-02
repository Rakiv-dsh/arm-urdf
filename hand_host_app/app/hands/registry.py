"""Hand profile registry backed only by installed hand packages."""

from __future__ import annotations

import copy
import json
import shutil
import time
from pathlib import Path

from app.hands.base import (
    HANDS_DIR,
    NUM_JOINTS,
    FingerConfig,
    HandProfile,
    JointInfo,
    apply_calibration_file,
)
from app.hands.manifest import HandManifestError, discover_manifest_paths, load_profile_from_manifest


STATE_PATH = HANDS_DIR / ".registry_state.json"
TRASH_DIR = HANDS_DIR / ".trash"
EMPTY_HAND_ID = "__empty__"


def _make_empty_profile() -> HandProfile:
    return HandProfile(
        hand_id=EMPTY_HAND_ID,
        display_name="未添加手型",
        finger_configs=[],
        model_path=HANDS_DIR / ".empty" / "empty.xml",
        calibration_path=HANDS_DIR / ".empty" / "calibration.json",
        gestures_dir=HANDS_DIR / ".empty" / "gestures",
        motor_kind="none",
        protocols=("none",),
        default_protocol="none",
        default_baudrates={"none": 0},
        actuator_order=(),
        force_control_supported=False,
        tactile_kind="none",
        tactile_default_baudrate=0,
        teleop_kind="none",
        default_gestures={},
        package_dir=None,
        driver_kind="none",
        driver_name="none",
        removable=False,
    )


EMPTY_PROFILE = _make_empty_profile()


def _load_state() -> dict:
    if not STATE_PATH.exists():
        return {}
    try:
        payload = json.loads(STATE_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _save_state(state: dict) -> None:
    HANDS_DIR.mkdir(parents=True, exist_ok=True)
    STATE_PATH.write_text(
        json.dumps(state, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _build_profiles() -> dict[str, HandProfile]:
    profiles: dict[str, HandProfile] = {}
    for manifest_path in discover_manifest_paths(HANDS_DIR):
        try:
            profile = load_profile_from_manifest(manifest_path)
        except (OSError, json.JSONDecodeError, HandManifestError) as exc:
            print(f"[hand registry] ignored invalid manifest {manifest_path}: {exc}")
            continue
        profiles[profile.hand_id] = profile

    for profile in profiles.values():
        apply_calibration_file(profile)
    return profiles


_PROFILES = _build_profiles()
_active_profile = next(iter(_PROFILES.values()), EMPTY_PROFILE)

# Legacy active-profile views. Keep the objects stable so old imports continue
# to observe active hand changes.
FINGER_CONFIGS: list[FingerConfig] = []
ALL_JOINT_NAMES: list[str] = []
MOTOR_ID_TO_JOINT_INDEX: dict[int, int] = {}
JOINT_INDEX_TO_MOTOR_ID: dict[int, int] = {}


def _sync_legacy_views(profile: HandProfile) -> None:
    FINGER_CONFIGS[:] = profile.finger_configs
    ALL_JOINT_NAMES[:] = profile.joint_names
    MOTOR_ID_TO_JOINT_INDEX.clear()
    MOTOR_ID_TO_JOINT_INDEX.update(profile.motor_id_to_joint_index)
    JOINT_INDEX_TO_MOTOR_ID.clear()
    JOINT_INDEX_TO_MOTOR_ID.update(profile.joint_index_to_motor_id)


def get_profiles() -> dict[str, HandProfile]:
    return _PROFILES


def get_profile(hand_id: str) -> HandProfile:
    if hand_id == EMPTY_HAND_ID:
        return EMPTY_PROFILE
    if hand_id in _PROFILES:
        return _PROFILES[hand_id]
    return next(iter(_PROFILES.values()), EMPTY_PROFILE)


def get_active_profile() -> HandProfile:
    return _active_profile


def set_active_profile(hand_id: str) -> HandProfile:
    global _active_profile
    profile = get_profile(hand_id)
    _active_profile = profile
    _sync_legacy_views(profile)
    return profile


def clone_profile(hand_id: str) -> HandProfile:
    return copy.deepcopy(get_profile(hand_id))


def reload_profiles(preferred_hand_id: str | None = None) -> dict[str, HandProfile]:
    """Reload installed hand manifests and keep legacy active views in sync."""
    global _PROFILES, _active_profile
    _PROFILES = _build_profiles()
    target = preferred_hand_id or getattr(_active_profile, "hand_id", None)
    _active_profile = get_profile(target) if target else next(iter(_PROFILES.values()), EMPTY_PROFILE)
    _sync_legacy_views(_active_profile)
    return _PROFILES


def delete_profile(hand_id: str) -> Path:
    if hand_id not in _PROFILES:
        raise ValueError(f"{hand_id} is not an installed hand profile")
    profile = get_profile(hand_id)
    if not profile.removable:
        raise ValueError(f"{hand_id} cannot be removed")
    package_dir = profile.package_dir
    if package_dir is None or not package_dir.exists():
        raise ValueError(f"{hand_id} has no removable package directory")
    TRASH_DIR.mkdir(parents=True, exist_ok=True)
    target = TRASH_DIR / f"{hand_id}_{time.strftime('%Y%m%d_%H%M%S')}"
    shutil.move(str(package_dir), str(target))
    reload_profiles()
    return target


_sync_legacy_views(_active_profile)


__all__ = [
    "ALL_JOINT_NAMES",
    "EMPTY_HAND_ID",
    "EMPTY_PROFILE",
    "FINGER_CONFIGS",
    "JOINT_INDEX_TO_MOTOR_ID",
    "MOTOR_ID_TO_JOINT_INDEX",
    "NUM_JOINTS",
    "FingerConfig",
    "HandProfile",
    "JointInfo",
    "clone_profile",
    "get_active_profile",
    "get_profile",
    "get_profiles",
    "reload_profiles",
    "delete_profile",
    "set_active_profile",
]
