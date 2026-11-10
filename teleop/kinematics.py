import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from lerobot.model.kinematics import RobotKinematics


JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
URDF_PATH = Path(__file__).parent / "models/so101/so101_new_calib.urdf"
GRIPPER_FRAME = "gripper_frame_link"
FEETECH_MAX_RESOLUTION = 4095
ENCODER_STEP_DEG = 360 / FEETECH_MAX_RESOLUTION
READBACK_TOLERANCE_DEG = 1.1 * ENCODER_STEP_DEG


@dataclass(frozen=True)
class IKResult:
    ok: bool
    q_deg: np.ndarray
    residual_m: float
    reason: str
    at_limit: bool = False
    solver_seed_deg: np.ndarray | None = None


class SO101Kinematics:
    def __init__(self, urdf_path=URDF_PATH, calibration_path=None):
        self.urdf_path = Path(urdf_path)
        self.joint_names = JOINT_NAMES
        self.target_frame_name = GRIPPER_FRAME
        urdf_lower, urdf_upper = self._urdf_limits(self.urdf_path)
        lower, upper = urdf_lower.copy(), urdf_upper.copy()
        self.ik_lower, self.ik_upper = urdf_lower.copy(), urdf_upper.copy()
        if calibration_path is not None:
            cal_lower, cal_upper = self._calibration_limits(Path(calibration_path))
            lower, upper = cal_lower, cal_upper
            self.ik_lower, self.ik_upper = np.maximum(urdf_lower, cal_lower), np.minimum(urdf_upper, cal_upper)
        # LeRobot calibration bounds the motor; the URDF bounds IK targets.
        lower, upper = lower + ENCODER_STEP_DEG, upper - ENCODER_STEP_DEG
        self.ik_lower += ENCODER_STEP_DEG
        self.ik_upper -= ENCODER_STEP_DEG
        if np.any(lower >= upper) or np.any(self.ik_lower >= self.ik_upper):
            raise ValueError("calibration and URDF joint limits do not overlap")
        self.lower = lower
        self.upper = upper
        self.robot = RobotKinematics(
            urdf_path=str(self.urdf_path),
            target_frame_name=self.target_frame_name,
            joint_names=list(self.joint_names),
        )

    @staticmethod
    def _urdf_limits(path):
        root = ET.parse(path).getroot()
        found = {}
        for joint in root.findall("joint"):
            name = joint.get("name")
            if name in JOINT_NAMES:
                limit = joint.find("limit")
                if joint.get("type") != "revolute" or limit is None:
                    raise ValueError(f"URDF joint {name} must have revolute limits")
                found[name] = (float(limit.get("lower")), float(limit.get("upper")))
        if set(found) != set(JOINT_NAMES):
            raise ValueError("URDF is missing one or more named SO-101 joints")
        lower = np.array([math.degrees(found[name][0]) for name in JOINT_NAMES])
        upper = np.array([math.degrees(found[name][1]) for name in JOINT_NAMES])
        if not np.all(np.isfinite(lower)) or not np.all(np.isfinite(upper)) or np.any(lower >= upper):
            raise ValueError("URDF contains invalid SO-101 joint limits")
        return lower, upper

    @staticmethod
    def _calibration_limits(path):
        data = json.loads(path.read_text())
        if not isinstance(data, dict):
            raise ValueError("LeRobot calibration must be a JSON object")
        lower, upper = [], []
        for name in JOINT_NAMES:
            motor = data.get(name)
            if not isinstance(motor, dict):
                raise ValueError(f"LeRobot calibration is missing {name}")
            raw_min, raw_max = motor.get("range_min"), motor.get("range_max")
            if (not isinstance(raw_min, int) or isinstance(raw_min, bool)
                    or not isinstance(raw_max, int) or isinstance(raw_max, bool)
                    or not 0 <= raw_min < raw_max <= FEETECH_MAX_RESOLUTION):
                raise ValueError(f"LeRobot calibration has invalid encoder limits for {name}")
            # Matches LeRobot 0.7 Feetech MotorNormMode.DEGREES: (raw - midpoint) * 360 / 4095.
            midpoint = (raw_min + raw_max) / 2
            lower.append((raw_min - midpoint) * 360 / FEETECH_MAX_RESOLUTION)
            upper.append((raw_max - midpoint) * 360 / FEETECH_MAX_RESOLUTION)
        return np.asarray(lower), np.asarray(upper)

    @staticmethod
    def _vector(value, size, name):
        result = np.asarray(value, dtype=float)
        if result.shape != (size,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{name} must contain {size} finite values")
        return result

    @staticmethod
    def _pose(value):
        pose = np.asarray(value, dtype=float)
        if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
            raise ValueError("end-effector pose must be a finite 4x4 matrix")
        if not np.allclose(pose[3], (0, 0, 0, 1), atol=1e-7):
            raise ValueError("end-effector pose must use homogeneous 4x4 coordinates")
        return pose

    def clamp(self, q_deg):
        q = self._vector(q_deg, len(JOINT_NAMES), "joint positions")
        bounded = np.clip(q, self.lower, self.upper)
        outside = np.any(q < self.lower - READBACK_TOLERANCE_DEG) or np.any(q > self.upper + READBACK_TOLERANCE_DEG)
        return bounded, bool(outside)

    def outside_ik(self, q_deg):
        q = self._vector(q_deg, len(JOINT_NAMES), "joint positions")
        return bool(np.any(q < self.ik_lower - READBACK_TOLERANCE_DEG)
                    or np.any(q > self.ik_upper + READBACK_TOLERANCE_DEG))

    def forward_pose(self, q_deg):
        q = self._vector(q_deg, len(JOINT_NAMES), "joint positions")
        return self._pose(self.robot.forward_kinematics(q))

    def forward_position(self, q_deg):
        return self.forward_pose(q_deg)[:3, 3].copy()

    def solve_position(self, current_joint_pos, desired_ee_pose, max_step_deg=None):
        current = self._vector(current_joint_pos, len(JOINT_NAMES), "current joint positions")
        target = self._pose(desired_ee_pose)
        bounded, outside_calibration = self.clamp(current)
        if outside_calibration:
            return IKResult(False, bounded, math.inf, "READBACK_OUTSIDE_LIMITS", True)
        current = bounded
        seed = np.clip(current, self.ik_lower, self.ik_upper)
        before = float(np.linalg.norm(target[:3, 3] - self.forward_position(seed)))
        raw = self._vector(
            self.robot.inverse_kinematics(
                current_joint_pos=seed,
                desired_ee_pose=target,
                position_weight=1.0,
                orientation_weight=0.01,
                max_iters=8,
            ),
            len(JOINT_NAMES),
            "IK result",
        )
        candidate = np.clip(raw, self.ik_lower, self.ik_upper)
        if max_step_deg is not None:
            step = float(max_step_deg)
            if not math.isfinite(step) or step <= 0:
                raise ValueError("max_step_deg must be positive and finite")
            direction = candidate - seed
            largest_step = float(np.max(np.abs(direction)))
            if largest_step > step:
                candidate = seed + direction * (step / largest_step)
        residual = float(np.linalg.norm(target[:3, 3] - self.forward_position(candidate)))
        at_limit = bool(np.any(np.isclose(candidate, self.ik_lower, atol=1e-6))
                        or np.any(np.isclose(candidate, self.ik_upper, atol=1e-6)))
        reached = residual <= 0.005
        progressed = before - residual > 1e-7
        if reached or progressed:
            reason = "LIMIT" if at_limit and not reached else "OK"
            return IKResult(True, candidate, residual, reason, at_limit, seed.copy())
        return IKResult(False, candidate, residual, "NO_PROGRESS", at_limit, seed.copy())
