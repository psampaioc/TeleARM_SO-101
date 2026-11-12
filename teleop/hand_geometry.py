"""Geometry of one MediaPipe hand; no camera, robot, or display ownership."""

import math
from dataclasses import dataclass

import numpy as np


FIST_RATIO_MAX = 0.7
PALM_RATIO_MIN = 1.1
PALM_INDICES = (0, 5, 9, 13, 17)


@dataclass(frozen=True)
class HandSample:
    # Legacy wrist/span coordinates remain until measured XYZ replaces them.
    hx: float
    hy: float
    size: float
    gesture: str | None
    confidence: float
    landmarks: np.ndarray | None = None
    palm_ratio: float | None = None
    jaw_angle_deg: float | None = None
    palm_normal: np.ndarray | None = None
    palm_center: tuple[float, float] | None = None  # unmirrored normalized image
    palm_area: float | None = None  # normalized image area, not silhouette area
    palm_axis: np.ndarray | None = None  # MediaPipe hand-world direction


def _points3(landmarks):
    if isinstance(landmarks, np.ndarray):
        points = np.asarray(landmarks, dtype=float)
    else:
        points = np.asarray([[p.x, p.y, p.z] for p in landmarks], dtype=float)
    if points.shape != (21, 3) or not np.all(np.isfinite(points)):
        raise ValueError("hand landmarks must be a finite 21x3 array")
    return points


def _palm_fist_ratio(points):
    points = _points3(points)
    palm_width = float(np.linalg.norm(points[5] - points[17]))
    if palm_width < 1e-6:
        return None
    return float(np.linalg.norm(points[2] - points[12]) / palm_width)


def _jaw_angle(points):
    points = _points3(points)
    thumb_to_joint = float(np.linalg.norm(points[4] - points[9]))
    joint_to_middle = float(np.linalg.norm(points[12] - points[9]))
    thumb_to_middle = float(np.linalg.norm(points[4] - points[12]))
    if thumb_to_joint < 1e-6 or joint_to_middle < 1e-6:
        return None
    cosine = (thumb_to_joint**2 + joint_to_middle**2 - thumb_to_middle**2) / (2 * thumb_to_joint * joint_to_middle)
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def _palm_normal(points):
    points = _points3(points)
    normal = np.cross(points[5] - points[0], points[17] - points[0])
    norm = float(np.linalg.norm(normal))
    return normal / norm if norm >= 1e-8 else None


def _palm_image_geometry(image_points):
    points = np.asarray(image_points, dtype=float)
    if points.shape != (21, 2) or not np.all(np.isfinite(points)):
        raise ValueError("image landmarks must be a finite 21x2 array")
    palm = points[list(PALM_INDICES)]
    center = palm.mean(axis=0)
    x, y = palm[:, 0], palm[:, 1]
    area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
    if not np.all((center >= 0) & (center <= 1)):
        return None
    return (float(center[0]), float(center[1])), area if area >= 1e-8 else None


def classify_gesture(world_landmarks):
    ratio = _palm_fist_ratio(world_landmarks)
    if ratio is None:
        return None
    if ratio <= FIST_RATIO_MAX:
        return "FIST"
    if ratio >= PALM_RATIO_MIN:
        return "PALM"
    return "HAND"


def _sample(result):
    if not result.hand_landmarks or not result.hand_world_landmarks:
        return None
    image_points = np.asarray([[p.x, p.y] for p in result.hand_landmarks[0]], dtype=float)
    if image_points.shape != (21, 2) or not np.all(np.isfinite(image_points)):
        return None
    wrist = image_points[0]
    size = float(np.linalg.norm(wrist - image_points[9]))
    if not (0 <= wrist[0] <= 1 and 0 <= wrist[1] <= 1 and 0 < size <= 1):
        return None
    try:
        world = _points3(result.hand_world_landmarks[0])
        gesture = classify_gesture(world)
        ratio = _palm_fist_ratio(world)
        image_geometry = _palm_image_geometry(image_points)
        if gesture is None or ratio is None or image_geometry is None:
            return None
        jaw_angle = _jaw_angle(world)
        normal = _palm_normal(world)
        axis = world[9] - world[0]
        axis_norm = float(np.linalg.norm(axis))
        axis = axis / axis_norm if axis_norm >= 1e-8 else None
    except ValueError:
        return None
    confidence = 0.0
    if result.handedness and result.handedness[0]:
        confidence = float(result.handedness[0][0].score)
    center, area = image_geometry
    return HandSample(float(wrist[0]), float(wrist[1]), size, gesture, confidence,
                      image_points, ratio, jaw_angle, normal, center, area, axis)
