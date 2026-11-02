"""Manifest loading for package-based dexterous hand definitions."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

from app.hands.base import FingerConfig, HANDS_DIR, HandProfile, JointInfo, TactilePatchInfo


MANIFEST_SCHEMA = "dexhand.hand.v1"
HAND_ID_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{1,63}$")


class HandManifestError(ValueError):
    """Raised when a hand package manifest is invalid."""


def _as_path(package_dir: Path, value: Any, default: str) -> Path:
    rel = value if isinstance(value, str) and value.strip() else default
    return package_dir / rel


def _as_optional_path(package_dir: Path, value: Any) -> Path | None:
    if isinstance(value, str) and value.strip():
        return package_dir / value
    return None


def _string_list(value: Any) -> tuple[str, ...]:
    if not isinstance(value, list):
        return ()
    return tuple(str(item) for item in value if isinstance(item, str) and item)


def validate_manifest_payload(payload: dict[str, Any], package_dir: Path) -> list[str]:
    """Return a list of validation errors for a manifest payload."""
    errors: list[str] = []
    if payload.get("schema") != MANIFEST_SCHEMA:
        errors.append(f"schema must be {MANIFEST_SCHEMA}")

    hand_id = payload.get("hand_id")
    if not isinstance(hand_id, str) or not HAND_ID_RE.match(hand_id):
        errors.append("hand_id must start with a letter and contain only letters, digits, '_' or '-'")

    if not isinstance(payload.get("display_name"), str) or not payload["display_name"].strip():
        errors.append("display_name is required")

    fingers = payload.get("fingers")
    if not isinstance(fingers, list) or not fingers:
        errors.append("fingers must be a non-empty list")
    else:
        seen_motor_ids: set[int] = set()
        joint_count = 0
        for fidx, finger in enumerate(fingers):
            if not isinstance(finger, dict):
                errors.append(f"fingers[{fidx}] must be an object")
                continue
            joints = finger.get("joints")
            if not isinstance(joints, list) or not joints:
                errors.append(f"fingers[{fidx}].joints must be a non-empty list")
                continue
            for jidx, joint in enumerate(joints):
                joint_count += 1
                if not isinstance(joint, dict):
                    errors.append(f"fingers[{fidx}].joints[{jidx}] must be an object")
                    continue
                motor_id = joint.get("motor_id")
                if not isinstance(motor_id, int) or motor_id <= 0:
                    errors.append(f"joint {fidx}:{jidx} motor_id must be a positive integer")
                elif motor_id in seen_motor_ids:
                    errors.append(f"motor_id {motor_id} is duplicated")
                else:
                    seen_motor_ids.add(motor_id)
                for key in ("name", "display_name"):
                    if not isinstance(joint.get(key), str) or not joint[key].strip():
                        errors.append(f"joint {fidx}:{jidx} {key} is required")
        if joint_count <= 0:
            errors.append("at least one joint is required")

    driver = payload.get("driver", {})
    if not isinstance(driver, dict):
        errors.append("driver must be an object")
    else:
        kind = driver.get("kind", "python")
        if kind != "python":
            errors.append("driver.kind must be 'python'")
        rel_path = driver.get("path", "driver.py")
        if not isinstance(rel_path, str) or not rel_path.strip():
            errors.append("python driver.path is required")
        elif not (package_dir / rel_path).exists():
            errors.append(f"python driver file does not exist: {rel_path}")

    tactile = payload.get("tactile", {})
    if isinstance(tactile, dict):
        if tactile.get("kind") == "external_python":
            rel_path = tactile.get("adapter_path", "tactile_adapter.py")
            if not isinstance(rel_path, str) or not rel_path.strip():
                errors.append("external tactile adapter_path is required")
            elif not (package_dir / rel_path).exists():
                errors.append(f"tactile adapter file does not exist: {rel_path}")
        layout = tactile.get("layout")
        if isinstance(layout, dict) and "patches" in layout:
            patches = layout.get("patches")
            if not isinstance(patches, list):
                errors.append("tactile.layout.patches must be a list")
            else:
                seen_patch_names: set[str] = set()
                for pidx, patch in enumerate(patches):
                    if not isinstance(patch, dict):
                        errors.append(f"tactile.layout.patches[{pidx}] must be an object")
                        continue
                    name = patch.get("name")
                    if not isinstance(name, str) or not name.strip():
                        errors.append(f"tactile.layout.patches[{pidx}].name is required")
                    elif name in seen_patch_names:
                        errors.append(f"tactile patch name duplicated: {name}")
                    else:
                        seen_patch_names.add(name)
                    for key in ("rows", "cols"):
                        value = patch.get(key)
                        if not isinstance(value, int) or value <= 0:
                            errors.append(f"tactile.layout.patches[{pidx}].{key} must be a positive integer")
                    for key in ("display_transform", "site_transform"):
                        value = patch.get(key, "identity")
                        if not isinstance(value, str):
                            errors.append(f"tactile.layout.patches[{pidx}].{key} must be a string")
    elif "tactile" in payload:
        errors.append("tactile must be an object")

    teleop = payload.get("teleop", {})
    if isinstance(teleop, dict):
        if teleop.get("kind") == "external_python":
            rel_path = teleop.get("adapter_path", "teleop_adapter.py")
            if not isinstance(rel_path, str) or not rel_path.strip():
                errors.append("external teleop adapter_path is required")
            elif not (package_dir / rel_path).exists():
                errors.append(f"teleop adapter file does not exist: {rel_path}")
    elif "teleop" in payload:
        errors.append("teleop must be an object")

    return errors


def validate_manifest_file(path: Path) -> list[str]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return [f"manifest cannot be read: {exc}"]
    if not isinstance(payload, dict):
        return ["manifest root must be an object"]
    return validate_manifest_payload(payload, path.parent)


def load_profile_from_manifest(path: Path) -> HandProfile:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise HandManifestError("manifest root must be an object")

    errors = validate_manifest_payload(payload, path.parent)
    if errors:
        raise HandManifestError("; ".join(errors))

    package_dir = path.parent
    motor = payload.get("motor", {})
    if not isinstance(motor, dict):
        motor = {}
    tactile = payload.get("tactile", {})
    if not isinstance(tactile, dict):
        tactile = {}
    teleop = payload.get("teleop", {})
    if not isinstance(teleop, dict):
        teleop = {}
    resources = payload.get("resources", {})
    if not isinstance(resources, dict):
        resources = {}
    driver = payload.get("driver", {})
    if not isinstance(driver, dict):
        driver = {}

    finger_configs: list[FingerConfig] = []
    for finger in payload["fingers"]:
        joints: list[JointInfo] = []
        for joint in finger["joints"]:
            joints.append(
                JointInfo(
                    name=joint["name"],
                    display_name=joint["display_name"],
                    motor_id=int(joint["motor_id"]),
                    min_angle=float(joint.get("min_angle", -200.0)),
                    max_angle=float(joint.get("max_angle", 200.0)),
                    default_angle=float(joint.get("default_angle", 0.0)),
                    torque_limit=float(joint.get("torque_limit", 150.0)),
                    actuator_name=joint.get("actuator_name"),
                )
            )
        finger_configs.append(
            FingerConfig(
                name=finger.get("name", f"finger_{len(finger_configs) + 1}"),
                display_name=finger.get("display_name", finger.get("name", "Finger")),
                joints=joints,
            )
        )

    protocols = _string_list(motor.get("protocols")) or ("adapter",)
    default_protocol = str(motor.get("default_protocol") or protocols[0])
    default_baudrates = motor.get("default_baudrates")
    if not isinstance(default_baudrates, dict):
        default_baudrates = {default_protocol: 115200}
    default_baudrates = {str(k): int(v) for k, v in default_baudrates.items()}

    actuator_order = _string_list(motor.get("actuator_order"))
    if not actuator_order:
        actuator_order = tuple(
            j.actuator_name or j.name
            for f in finger_configs
            for j in f.joints
        )

    gestures = payload.get("gestures", {})
    default_gestures = {}
    if isinstance(gestures, dict) and isinstance(gestures.get("builtin"), dict):
        for name, values in gestures["builtin"].items():
            if isinstance(name, str) and isinstance(values, list):
                default_gestures[name] = [float(v) for v in values]

    teleop_joint_names = _string_list(teleop.get("joint_names")) or None
    teleop_tip_link_names = _string_list(teleop.get("tip_link_names")) or None
    motor_adapter_config = motor.get("adapter_config")
    if not isinstance(motor_adapter_config, dict):
        motor_adapter_config = {}
    tactile_adapter_config = tactile.get("adapter_config")
    if not isinstance(tactile_adapter_config, dict):
        tactile_adapter_config = {}
    teleop_adapter_config = teleop.get("adapter_config")
    if not isinstance(teleop_adapter_config, dict):
        teleop_adapter_config = {}
    tactile_layout_payload = tactile.get("layout")
    if not isinstance(tactile_layout_payload, dict):
        tactile_layout_payload = {}
    tactile_patches_payload = tactile_layout_payload.get("patches")
    tactile_layout: list[TactilePatchInfo] = []
    if isinstance(tactile_patches_payload, list):
        for index, patch in enumerate(tactile_patches_payload, start=1):
            if not isinstance(patch, dict):
                continue
            name = str(patch.get("name") or f"patch_{index}").strip()
            if not name:
                continue
            try:
                rows = max(1, int(patch.get("rows", 1)))
                cols = max(1, int(patch.get("cols", 1)))
            except (TypeError, ValueError):
                continue
            site_prefix = patch.get("site_prefix")
            if not isinstance(site_prefix, str) or not site_prefix.strip():
                site_prefix = None
            tactile_layout.append(
                TactilePatchInfo(
                    name=name,
                    display_name=str(patch.get("display_name") or name),
                    rows=rows,
                    cols=cols,
                    group=str(patch.get("group") or "触觉"),
                    site_prefix=site_prefix,
                    display_transform=str(patch.get("display_transform") or "identity"),
                    site_transform=str(patch.get("site_transform") or "identity"),
                )
            )

    driver_kind = str(driver.get("kind", "python"))
    driver_name = driver.get("name")
    if not isinstance(driver_name, str) or not driver_name.strip():
        driver_name = str(motor.get("kind", "adapter"))
    driver_path = None
    driver_path = _as_path(package_dir, driver.get("path"), "driver.py")

    return HandProfile(
        hand_id=payload["hand_id"],
        display_name=payload["display_name"],
        finger_configs=finger_configs,
        model_path=_as_path(package_dir, resources.get("model"), "model/1.xml"),
        calibration_path=_as_path(package_dir, resources.get("calibration"), "calibration.json"),
        gestures_dir=_as_path(package_dir, resources.get("gestures"), "gestures"),
        motor_kind=str(motor.get("kind", driver_name or "adapter")),
        protocols=protocols,
        default_protocol=default_protocol,
        default_baudrates=default_baudrates,
        actuator_order=actuator_order,
        force_control_supported=bool(motor.get("force_control_supported", True)),
        motor_adapter_config=dict(motor_adapter_config),
        tactile_kind=str(tactile.get("kind", "none")),
        tactile_default_baudrate=int(tactile.get("default_baudrate", 115200)),
        tactile_adapter_path=_as_optional_path(package_dir, tactile.get("adapter_path")),
        tactile_adapter_class=str(tactile.get("adapter_class", "TactileAdapter")),
        tactile_adapter_config=dict(tactile_adapter_config),
        tactile_layout=tuple(tactile_layout),
        teleop_kind=str(teleop.get("kind", "none")),
        teleop_config_path=_as_optional_path(package_dir, teleop.get("config")),
        teleop_pair_calibration_path=_as_optional_path(package_dir, teleop.get("pair_calibration")),
        motor_to_sim_calibration_path=_as_optional_path(package_dir, teleop.get("motor_to_sim_calibration")),
        teleop_adapter_path=_as_optional_path(package_dir, teleop.get("adapter_path")),
        teleop_adapter_class=str(teleop.get("adapter_class", "TeleopAdapter")),
        teleop_adapter_config=dict(teleop_adapter_config),
        teleop_joint_names=teleop_joint_names,
        teleop_tip_link_names=teleop_tip_link_names,
        teleop_wrist_link_name=str(teleop.get("wrist_link_name", "base_link")),
        default_gestures=default_gestures,
        package_dir=package_dir,
        driver_kind=driver_kind,
        driver_name=driver_name,
        driver_path=driver_path,
        driver_class=str(driver.get("class", "HandDriver")),
        preset=bool(payload.get("preset", False)),
        removable=bool(payload.get("removable", not payload.get("preset", False))),
    )


def discover_manifest_paths(root: Path = HANDS_DIR) -> list[Path]:
    if not root.exists():
        return []
    paths: list[Path] = []
    for child in sorted(root.iterdir(), key=lambda p: p.name.lower()):
        if not child.is_dir() or child.name.startswith("."):
            continue
        manifest = child / "manifest.json"
        if manifest.exists():
            paths.append(manifest)
    return paths
