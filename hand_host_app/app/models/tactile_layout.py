"""Shared tactile patch layout helpers for package-defined sensors."""

from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np


VALID_TRANSFORMS = {"identity", "flip_horizontal", "flip_vertical", "rotate_180"}

# Kept for legacy imports. The core no longer provides a built-in hand layout.
PATCH_ORDER: tuple[str, ...] = ()
PATCH_SHAPES: dict[str, tuple[int, int]] = {}
DISPLAY_TRANSFORMS: dict[str, str] = {}
DISPLAY_TO_SITE_TRANSFORMS: dict[str, str] = {}


def _layout(profile=None):
    if profile is not None and getattr(profile, "tactile_kind", "") == "none":
        return ()
    return tuple(getattr(profile, "tactile_layout", ()) or ()) if profile is not None else ()


def _clean_transform(value: str | None) -> str:
    value = value or "identity"
    return value if value in VALID_TRANSFORMS else "identity"


def empty_tactile_patches(dtype=np.float64) -> dict[str, np.ndarray]:
    return {}


def patch_shapes_for_profile(profile=None) -> dict[str, tuple[int, int]]:
    return {
        patch.name: (int(patch.rows), int(patch.cols))
        for patch in _layout(profile)
    }


def patch_order_for_profile(profile=None) -> tuple[str, ...]:
    return tuple(patch.name for patch in _layout(profile))


def patch_labels_for_profile(profile=None) -> dict[str, str]:
    return {patch.name: patch.display_name for patch in _layout(profile)}


def patch_site_prefixes_for_profile(profile=None) -> dict[str, str]:
    return {
        patch.name: (patch.site_prefix or patch.name)
        for patch in _layout(profile)
    }


def patch_display_transforms_for_profile(profile=None) -> dict[str, str]:
    return {
        patch.name: _clean_transform(getattr(patch, "display_transform", "identity"))
        for patch in _layout(profile)
    }


def patch_site_transforms_for_profile(profile=None) -> dict[str, str]:
    return {
        patch.name: _clean_transform(getattr(patch, "site_transform", "identity"))
        for patch in _layout(profile)
    }


def patch_groups_for_profile(profile=None) -> list[tuple[str, tuple[str, ...]]]:
    groups: dict[str, list[str]] = {}
    for patch in _layout(profile):
        groups.setdefault(patch.group or "触觉", []).append(patch.name)
    return [(name, tuple(patches)) for name, patches in groups.items()]


def empty_tactile_patches_for_profile(profile=None, dtype=np.float64) -> dict[str, np.ndarray]:
    return {
        name: np.zeros(shape, dtype=dtype)
        for name, shape in patch_shapes_for_profile(profile).items()
    }


def apply_display_transform(
    matrix: np.ndarray,
    patch_name: str,
    profile=None,
    transform: str | None = None,
) -> np.ndarray:
    transform = _clean_transform(
        transform
        or patch_display_transforms_for_profile(profile).get(patch_name)
        or DISPLAY_TRANSFORMS.get(patch_name)
    )
    if transform == "flip_horizontal":
        return np.fliplr(matrix)
    if transform == "flip_vertical":
        return np.flipud(matrix)
    if transform == "rotate_180":
        return np.rot90(matrix, 2)
    return matrix


def display_to_site_indices(
    display_row: int,
    display_col: int,
    rows: int,
    cols: int,
    transform: str,
) -> tuple[int, int]:
    transform = _clean_transform(transform)
    if transform == "identity":
        return display_row, display_col
    if transform == "flip_vertical":
        return rows - 1 - display_row, display_col
    if transform == "flip_horizontal":
        return display_row, cols - 1 - display_col
    if transform == "rotate_180":
        return rows - 1 - display_row, cols - 1 - display_col
    raise ValueError(f"Unsupported tactile display transform: {transform}")


def normalized_display_patches(
    patches: Mapping[str, np.ndarray],
    profile=None,
) -> dict[str, np.ndarray]:
    shapes = patch_shapes_for_profile(profile)
    order = patch_order_for_profile(profile)
    result: dict[str, np.ndarray] = {}

    if order:
        for name in order:
            expected = shapes[name]
            arr = np.asarray(patches.get(name, np.zeros(expected)), dtype=np.float32)
            if arr.ndim != 2 or arr.shape != expected:
                arr = np.zeros(expected, dtype=np.float32)
            result[name] = np.clip(
                apply_display_transform(arr, name, profile),
                0.0,
                1.0,
            )
        return result

    for name, value in patches.items():
        arr = np.asarray(value, dtype=np.float32)
        if arr.ndim == 2:
            result[str(name)] = np.clip(arr, 0.0, 1.0)
    return result


def patch_dict_to_vector(patches: Mapping[str, np.ndarray], profile=None) -> np.ndarray:
    order = patch_order_for_profile(profile) or tuple(PATCH_ORDER)
    if not order:
        return np.zeros(0, dtype=np.float32)
    return np.concatenate(
        [np.asarray(patches[name], dtype=np.float32).reshape(-1) for name in order]
    )


def vector_to_patch_dict(values: Sequence[float], profile=None) -> dict[str, np.ndarray]:
    shapes = patch_shapes_for_profile(profile) or dict(PATCH_SHAPES)
    order = patch_order_for_profile(profile) or tuple(PATCH_ORDER)
    arr = np.asarray(values, dtype=np.float32).reshape(-1)
    result: dict[str, np.ndarray] = {}
    offset = 0
    for name in order:
        rows, cols = shapes[name]
        count = rows * cols
        result[name] = arr[offset:offset + count].reshape(rows, cols).copy()
        offset += count
    if offset != arr.size:
        raise ValueError(f"Tactile vector length {arr.size} does not match expected {offset}")
    return result


def patch_vector_mask(patch_names: Sequence[str], profile=None) -> np.ndarray:
    shapes = patch_shapes_for_profile(profile) or dict(PATCH_SHAPES)
    order = patch_order_for_profile(profile) or tuple(PATCH_ORDER)
    if not order:
        return np.zeros(0, dtype=bool)
    selected = set(patch_names)
    unknown = selected.difference(shapes)
    if unknown:
        raise ValueError(f"Unknown tactile patches: {sorted(unknown)}")
    return np.concatenate(
        [
            np.full(shapes[name][0] * shapes[name][1], name in selected, dtype=bool)
            for name in order
        ]
    )


def reduce_patches_to_finger_palm(
    patches: Mapping[str, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    finger = np.zeros((4, 3), dtype=np.float64)
    groups = (
        ("th_pad", "th_tip", "th_tip"),
        ("if_mid_pad", "if_distal_pad", "if_tip"),
        ("mf_mid_pad", "mf_distal_pad", "mf_tip"),
        ("rf_mid_pad", "rf_distal_pad", "rf_tip"),
    )
    for row, names in enumerate(groups):
        for col, name in enumerate(names):
            if name in patches:
                finger[row, col] = float(np.mean(np.asarray(patches[name], dtype=np.float32)))

    palm_blocks = np.zeros(8, dtype=np.float64)
    if "palm" in patches:
        palm = np.asarray(patches["palm"], dtype=np.float32)
        blocks = []
        if palm.ndim == 2 and palm.size:
            for rr in np.array_split(palm, 2, axis=0):
                for block in np.array_split(rr, 4, axis=1):
                    blocks.append(float(np.mean(block)))
        palm_blocks[: min(8, len(blocks))] = blocks[:8]
    return np.clip(finger, 0.0, 1.0), np.clip(palm_blocks, 0.0, 1.0)


def patches_to_legacy_matrix(patches: Mapping[str, np.ndarray]) -> np.ndarray:
    finger, palm = reduce_patches_to_finger_palm(patches)
    mat = np.zeros((6, 8), dtype=np.float64)
    for row in range(4):
        mat[row, 0:3] = finger[row, 0]
        mat[row, 3:5] = finger[row, 1]
        mat[row, 5:8] = finger[row, 2]
    mat[4, :] = palm
    mat[5, :] = palm
    return np.clip(mat, 0.0, 1.0)
