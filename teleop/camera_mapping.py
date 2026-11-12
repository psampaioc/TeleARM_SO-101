"""Measured hand-camera coordinates; never opens a camera or robot connection."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from teleop.hand_geometry import PALM_INDICES


PROFILE_VERSION = 1


@dataclass(frozen=True)
class MetricProfile:
    device: str
    width_px: int
    height_px: int
    focus_zoom_mode: str
    K: np.ndarray
    distortion: np.ndarray
    hand_identity: str
    depth_model: str
    effective_scale: float
    axial_offset_m: float
    valid_z_m: tuple[float, float]
    min_facing_cos: float
    camera_delta_to_base: np.ndarray
    tool_offset_gripper_m: np.ndarray


def _finite_array(value, shape, name):
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} is missing or non-numeric") from exc
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite {shape} array")
    return array


def load_metric_profile(path: Path, *, device: str, width_px: int, height_px: int) -> MetricProfile:
    """Reject incomplete or mismatched measured data before metric live control."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("version") != PROFILE_VERSION:
        raise ValueError("unsupported hand profile version")
    camera, hand, control = (data.get(key) or {} for key in ("camera", "hand", "control"))
    if (camera.get("device"), camera.get("width_px"), camera.get("height_px")) != (device, width_px, height_px):
        raise ValueError("hand profile camera or capture resolution does not match")
    if not isinstance(camera.get("focus_zoom_mode"), str) or not camera["focus_zoom_mode"]:
        raise ValueError("hand profile needs measured focus/zoom mode")
    if not isinstance(hand.get("identity"), str) or not hand["identity"]:
        raise ValueError("hand profile needs the calibrated operator identity")
    K = _finite_array(camera.get("K"), (3, 3), "camera K")
    if K[0, 0] <= 0 or K[1, 1] <= 0 or abs(np.linalg.det(K)) < 1e-9:
        raise ValueError("camera K has invalid focal lengths")
    distortion = np.asarray(camera.get("distortion"), dtype=float) if camera.get("distortion") is not None else None
    if distortion is None or distortion.ndim != 1 or distortion.size not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
        raise ValueError("camera distortion coefficients are missing or invalid")
    model = hand.get("depth_model")
    if model not in ("span", "area"):
        raise ValueError("hand depth model must be selected from measured samples")
    scale_key = "effective_span_m" if model == "span" else "effective_area_m2"
    scale = hand.get(scale_key)
    if not isinstance(scale, (float, int)) or not math.isfinite(scale) or scale <= 0:
        raise ValueError(f"hand {scale_key} needs a positive fitted value")
    offset = hand.get("axial_offset_m")
    if not isinstance(offset, (float, int)) or not math.isfinite(offset):
        raise ValueError("hand axial_offset_m needs a measured value")
    bounds = _finite_array(hand.get("valid_z_m"), (2,), "hand valid_z_m")
    if bounds[0] <= 0 or bounds[1] <= bounds[0]:
        raise ValueError("hand valid_z_m must be increasing positive metres")
    facing = hand.get("min_facing_cos")
    if not isinstance(facing, (float, int)) or not math.isfinite(facing) or not 0 <= facing <= 1:
        raise ValueError("hand min_facing_cos needs a validated value between 0 and 1")
    axes = _finite_array(control.get("camera_delta_to_base"), (3, 3), "control axis map")
    if not (np.all(np.isin(axes, (-1, 0, 1))) and np.all(np.sum(np.abs(axes), axis=0) == 1)
            and np.all(np.sum(np.abs(axes), axis=1) == 1)):
        raise ValueError("control axis map must be a signed permutation")
    tool_offset = _finite_array(control.get("tool_offset_gripper_m"), (3,), "gripper tool offset")
    return MetricProfile(device, width_px, height_px, camera["focus_zoom_mode"], K, distortion,
                         hand["identity"], model, float(scale), float(offset),
                         (float(bounds[0]), float(bounds[1])), float(facing), axes, tool_offset)


def estimate_camera_xyz(image_landmarks, profile: MetricProfile, palm_normal):
    """Return camera XYZ metres from unmirrored normalized image landmarks."""
    import cv2

    points = np.asarray(image_landmarks, dtype=float)
    if points.shape != (21, 2) or not np.all(np.isfinite(points)):
        raise ValueError("image landmarks must be finite 21x2 coordinates")
    normal = _finite_array(palm_normal, (3,), "palm normal")
    if abs(float(normal[2])) < profile.min_facing_cos:
        raise ValueError("palm is too edge-on for the measured depth model")
    pixels = points * np.array([profile.width_px, profile.height_px])
    rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), profile.K, profile.distortion).reshape(21, 2)
    palm = rays[list(PALM_INDICES)]
    center = palm.mean(axis=0)
    if profile.depth_model == "span":
        feature = float(np.linalg.norm(rays[5] - rays[17]))
        if feature < 1e-8:
            raise ValueError("palm span is too small")
        z = profile.effective_scale / feature + profile.axial_offset_m
    else:
        x, y = palm[:, 0], palm[:, 1]
        area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
        if area < 1e-8:
            raise ValueError("palm polygon is degenerate")
        z = math.sqrt(profile.effective_scale / area) + profile.axial_offset_m
    if not profile.valid_z_m[0] <= z <= profile.valid_z_m[1]:
        raise ValueError("hand depth is outside the measured working volume")
    return np.array([center[0] * z, center[1] * z, z], dtype=float)


def hand_delta_to_base(current_xyz, anchor_xyz, profile: MetricProfile):
    current = _finite_array(current_xyz, (3,), "current camera XYZ")
    anchor = _finite_array(anchor_xyz, (3,), "anchor camera XYZ")
    return profile.camera_delta_to_base @ (current - anchor)
