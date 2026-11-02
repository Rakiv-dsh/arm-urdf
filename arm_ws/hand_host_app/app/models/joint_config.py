"""Compatibility import for the hand profile registry.

New code should import from :mod:`app.hands`. This module stays as a stable
bridge for older widgets and scripts.
"""

from app.hands import (  # noqa: F401
    ALL_JOINT_NAMES,
    FINGER_CONFIGS,
    JOINT_INDEX_TO_MOTOR_ID,
    MOTOR_ID_TO_JOINT_INDEX,
    NUM_JOINTS,
    FingerConfig,
    HandProfile,
    JointInfo,
    clone_profile,
    delete_profile,
    get_active_profile,
    get_profile,
    get_profiles,
    reload_profiles,
    set_active_profile,
)

__all__ = [
    "ALL_JOINT_NAMES",
    "FINGER_CONFIGS",
    "JOINT_INDEX_TO_MOTOR_ID",
    "MOTOR_ID_TO_JOINT_INDEX",
    "NUM_JOINTS",
    "FingerConfig",
    "HandProfile",
    "JointInfo",
    "clone_profile",
    "delete_profile",
    "get_active_profile",
    "get_profile",
    "get_profiles",
    "reload_profiles",
    "set_active_profile",
]
