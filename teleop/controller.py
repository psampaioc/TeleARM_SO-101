import math
import time
from collections import deque

import numpy as np

from teleop.kinematics import ENCODER_STEP_DEG, JOINT_NAMES, READBACK_TOLERANCE_DEG


PALM_FRAMES = 3
HAND_LOST_HOLD_S = 0.5
XYZ_DEADBAND_M = 0.001
XYZ_HALF_RANGE_M = 0.10
DEPTH_SCALE_M = 0.40
IMAGE_YZ_MAP = np.array([[0.50, 0.0], [0.0, -0.50]], dtype=float)
JOINT_RATE_RAD_S = 0.5
MAX_STEP_DEG = 1.0
LIMIT_POLL_S = 0.1
RECOVERY_TIMEOUT_S = 2.0
RECOVERY_INSET_DEG = 7 * ENCODER_STEP_DEG
GRIPPER_CLOSED = 100.0
GRIPPER_OPEN = 0.0


class GestureDebouncer:
    def __init__(self, stable_frames=PALM_FRAMES):
        self.stable_frames = stable_frames
        self.candidate = None
        self.stable = None
        self.count = 0

    def reset(self):
        self.candidate = self.stable = None
        self.count = 0

    def update(self, gesture):
        if gesture == "FIST":
            self.candidate = self.stable = "FIST"
            self.count = self.stable_frames
            return "FIST"
        if gesture != "PALM":
            self.reset()
            return None
        if gesture == self.candidate:
            self.count = min(self.count + 1, self.stable_frames)
        else:
            self.candidate, self.count = gesture, 1
        self.stable = gesture if self.count >= self.stable_frames else None
        return self.stable


class TeleopController:
    """Single-process hand-to-SO-101 state machine; LeRobot owns the serial bus."""

    def __init__(self, adapter, kinematics, image_yz_map=None):
        self.adapter = adapter
        self.kinematics = kinematics
        self.image_yz_map = np.asarray(IMAGE_YZ_MAP if image_yz_map is None else image_yz_map, dtype=float)
        if self.image_yz_map.shape != (2, 2) or not np.all(np.isfinite(self.image_yz_map)):
            raise ValueError("image-to-base YZ map must be a finite 2x2 matrix")
        if abs(float(np.linalg.det(self.image_yz_map))) < 1e-8:
            raise ValueError("image-to-base YZ map must have independent axes")
        self.state = "READY"
        self.feedback = "READY"
        self.debouncer = GestureDebouncer()
        self.hand_anchor = None
        self.arm_anchor = None
        self.last_seen = None
        self.last_step = None
        self.gripper_target = None
        self.hand_missing = False
        self.wait_check = 0.0
        self.pending_palms = deque(maxlen=PALM_FRAMES)
        self.deadband_xyz = np.full(3, XYZ_DEADBAND_M)
        self.limit_kind = None
        self.recovery_started = None
        self.last_progress_at = None
        self.last_recovery_distance = math.inf
        self.recovery_stalled = False
        self.latest_timings = {}
        self.last_measured_q = None
        self.last_read_at = None
        self.last_measured_gripper = None
        self.last_command_gripper = None
        self.measured_pose = None
        self.last_command_q = None
        self.requested_target = None
        self.hand_delta = None
        self.base_delta = None
        self.last_ik_result = None
        self.boundary_joints = ()
        self.saturated_axes = ()

    @property
    def palm_count(self):
        return self.debouncer.count if self.debouncer.candidate == "PALM" else 0

    def _read(self):
        started = time.perf_counter_ns()
        observation = self.adapter.read_positions()
        self.latest_timings["read"] = (time.perf_counter_ns() - started) / 1e6
        q = np.asarray([observation[f"{name}.pos"] for name in JOINT_NAMES], dtype=float)
        if q.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(q)):
            raise ValueError("LeRobot readback must contain five finite joint positions")
        grip = float(observation["gripper.pos"])
        if not math.isfinite(grip) or not 0 <= grip <= 100:
            raise ValueError("LeRobot gripper readback is outside 0-100")
        self.last_measured_q = q.copy()
        self.last_measured_gripper = grip
        self.measured_pose = self._forward_pose(q)
        self.last_read_at = time.monotonic()
        return observation, q, grip

    def _send(self, action):
        started = time.perf_counter_ns()
        result = self.adapter.send_action(action)
        self.latest_timings["write"] = (time.perf_counter_ns() - started) / 1e6
        for index, name in enumerate(JOINT_NAMES):
            key = f"{name}.pos"
            if key in action:
                if self.last_command_q is None:
                    self.last_command_q = np.full(len(JOINT_NAMES), np.nan)
                self.last_command_q[index] = float(action[key])
        if "gripper.pos" in action:
            self.last_command_gripper = float(action["gripper.pos"])
        return result

    def _forward_pose(self, q):
        started = time.perf_counter_ns()
        pose = self.kinematics.forward_pose(q)
        self.latest_timings["fk"] = (time.perf_counter_ns() - started) / 1e6
        return pose

    def _clear_anchor(self):
        self.hand_anchor = self.arm_anchor = None
        self.last_seen = None
        self.last_step = None
        self.gripper_target = None
        self.hand_missing = False
        self.pending_palms.clear()
        self.deadband_xyz = np.full(3, XYZ_DEADBAND_M)
        self.saturated_axes = ()

    def _fail(self, reason):
        self._clear_anchor()
        self.debouncer.reset()
        self.state, self.limit_kind = "BLOCKED", "READ"
        self.wait_check = 0.0
        self.feedback = f"BLOCKED: {reason}; retrying readback"

    def _start_calibration_recovery(self, q, now):
        bounded = np.clip(q, self.kinematics.lower + RECOVERY_INSET_DEG,
                          self.kinematics.upper - RECOVERY_INSET_DEG)
        action = {
            f"{name}.pos": float(bound)
            for name, value, bound in zip(JOINT_NAMES, q, bounded)
            if abs(float(value) - float(bound)) > READBACK_TOLERANCE_DEG
        }
        if not action:
            self._fail("calibration boundary did not produce an inward target")
            return
        self._clear_anchor()
        self.debouncer.reset()
        self.state, self.limit_kind = "LIMIT", "CALIBRATION"
        self.recovery_started = self.last_progress_at = now
        self.recovery_stalled = False
        safe_boundary, _ = self.kinematics.clamp(q)
        self.last_recovery_distance = float(np.linalg.norm(q - safe_boundary))
        self.boundary_joints = tuple(key.removesuffix(".pos") for key in action)
        self.feedback = "LIMIT: calibrated boundary recovery"
        try:
            self._send(action)
        except Exception as exc:
            self._fail(f"calibration recovery write failed: {exc}")
            return
        self.wait_check = now + LIMIT_POLL_S

    def _begin_ik_recovery(self, q, now):
        self._clear_anchor()
        self.debouncer.reset()
        self.state, self.limit_kind = "LIMIT", "IK"
        self.recovery_started = self.last_progress_at = now
        self.recovery_stalled = False
        self.last_recovery_distance = float(np.linalg.norm(
            q - np.clip(q, self.kinematics.ik_lower, self.kinematics.ik_upper)
        ))
        self.boundary_joints = tuple(
            name for name, value, low, high in zip(
                JOINT_NAMES, q, self.kinematics.ik_lower, self.kinematics.ik_upper
            ) if value < low or value > high
        )
        self.feedback = "LIMIT: returning inside IK bounds"
        self._send_ik_recovery(q, now)
        self.wait_check = now + LIMIT_POLL_S

    def _send_ik_recovery(self, q, now):
        dt = max(0.001, min(now - (self.last_step if self.last_step is not None else now - LIMIT_POLL_S), 0.05))
        max_step = min(math.degrees(JOINT_RATE_RAD_S * dt), MAX_STEP_DEG)
        target = np.clip(q, self.kinematics.ik_lower, self.kinematics.ik_upper)
        command = q + np.clip(target - q, -max_step, max_step)
        command = np.clip(command, self.kinematics.lower, self.kinematics.upper)
        action = {
            f"{name}.pos": float(value)
            for name, q_value, value in zip(JOINT_NAMES, q, command)
            if abs(float(q_value) - float(value)) > 1e-7
        }
        if action:
            self._send(action)
        self.last_command_q = command.copy()
        self.last_step = now

    def _finish_limit(self):
        self._clear_anchor()
        self.debouncer.reset()
        self.state, self.limit_kind = "READY", None
        self.recovery_stalled = False
        self.boundary_joints = ()
        self.feedback = "READY: boundary readback confirmed; show PALM"

    def _check_limit(self, now):
        if now < self.wait_check:
            return
        self.wait_check = now + LIMIT_POLL_S
        try:
            _, q, _ = self._read()
        except Exception as exc:
            self.feedback = f"BLOCKED: readback failed ({exc}); retrying"
            return
        if self.limit_kind == "READ":
            _, clipped = self.kinematics.clamp(q)
            if clipped:
                self.boundary_joints = tuple(
                    name for name, value, low, high in zip(JOINT_NAMES, q, self.kinematics.lower, self.kinematics.upper)
                    if value < low - READBACK_TOLERANCE_DEG or value > high + READBACK_TOLERANCE_DEG
                )
                self.feedback = "BLOCKED: readback outside calibration; waiting for valid position"
            else:
                self._finish_limit()
            return
        if self.limit_kind == "CALIBRATION":
            bounded, clipped = self.kinematics.clamp(q)
            if not clipped:
                self._finish_limit()
                return
            distance = float(np.linalg.norm(q - bounded))
            if distance < self.last_recovery_distance - ENCODER_STEP_DEG * 0.5:
                self.last_progress_at = now
                self.last_recovery_distance = distance
                self.recovery_stalled = False
            if now - self.last_progress_at >= RECOVERY_TIMEOUT_S:
                self.recovery_stalled = True
                self.feedback = f"BLOCKED: {','.join(self.boundary_joints)} made no readback progress; still polling"
            return
        if self.limit_kind == "IK":
            if not self.kinematics.outside_ik(q):
                self._finish_limit()
                return
            distance = float(np.linalg.norm(q - np.clip(q, self.kinematics.ik_lower, self.kinematics.ik_upper)))
            if distance < self.last_recovery_distance - ENCODER_STEP_DEG * 0.5:
                self.last_progress_at = now
                self.last_recovery_distance = distance
                self.recovery_stalled = False
            if now - self.last_progress_at >= RECOVERY_TIMEOUT_S:
                self.recovery_stalled = True
                self.feedback = f"BLOCKED: {','.join(self.boundary_joints)} made no IK readback progress; still polling"
                return
            if not self.recovery_stalled:
                self._send_ik_recovery(q, now)

    def _measure_activation_noise(self):
        if len(self.pending_palms) < 2:
            return
        samples = np.asarray(self.pending_palms, dtype=float)
        uv_noise = np.std(samples[:, :2], axis=0)
        yz_noise = np.abs(self.image_yz_map) @ uv_noise
        size = max(float(np.mean(samples[:, 2])), 1e-6)
        depth_noise = DEPTH_SCALE_M * float(np.std(samples[:, 2])) / size
        self.deadband_xyz = np.maximum(XYZ_DEADBAND_M, 3 * np.array([depth_noise, *yz_noise]))

    def _activate(self, sample, now):
        try:
            _, q, _ = self._read()
            bounded, clipped = self.kinematics.clamp(q)
            if clipped:
                self._start_calibration_recovery(q, now)
                return
            self.arm_anchor = self._forward_pose(bounded)
            self.hand_anchor = (float(sample.hx), float(sample.hy), float(sample.size))
            self._measure_activation_noise()
            self.last_seen = self.last_step = now
            self.requested_target = self.arm_anchor.copy()
            self.target_pose = self.arm_anchor.copy()
            self.hand_delta = self.base_delta = (0.0, 0.0, 0.0)
            self.limited_delta = (0.0, 0.0, 0.0)
            jaw_angle = getattr(sample, "jaw_angle_deg", None)
            self.gripper_target = (self._map_gripper(jaw_angle)
                                   if jaw_angle is not None and math.isfinite(float(jaw_angle)) else None)
            self.state, self.feedback = "ACTIVE", "ACTIVE: anchored; no activation movement"
        except Exception as exc:
            self._fail(f"activation failed: {exc}")

    def _reanchor(self, sample, now):
        try:
            _, q, _ = self._read()
            bounded, clipped = self.kinematics.clamp(q)
            if clipped:
                self._start_calibration_recovery(q, now)
                return
            self.hand_anchor = (float(sample.hx), float(sample.hy), float(sample.size))
            self.arm_anchor = self._forward_pose(bounded)
            self.requested_target = self.arm_anchor.copy()
            self.target_pose = self.arm_anchor.copy()
            self.last_step = now
            self.hand_missing = False
            self.feedback = "ACTIVE: re-anchored; no movement"
        except Exception as exc:
            self._fail(f"re-anchor failed: {exc}")

    @staticmethod
    def _map_gripper(angle_deg):
        fraction = np.clip((60.0 - float(angle_deg)) / 50.0, 0.0, 1.0)
        return float(GRIPPER_OPEN + fraction * (GRIPPER_CLOSED - GRIPPER_OPEN))

    def _move(self, sample, now):
        try:
            _, q, grip = self._read()
            bounded, clipped = self.kinematics.clamp(q)
            if clipped:
                self._start_calibration_recovery(q, now)
                return
            q = bounded
            if self.kinematics.outside_ik(q):
                self._begin_ik_recovery(q, now)
                return

            du = float(sample.hx) - self.hand_anchor[0]
            dv = float(sample.hy) - self.hand_anchor[1]
            size_ratio = float(sample.size) / self.hand_anchor[2]
            yz = self.image_yz_map @ np.array([du, dv])
            depth = DEPTH_SCALE_M * (size_ratio - 1.0)
            raw_delta = np.array([depth, yz[0], yz[1]], dtype=float)
            axes = ("X", "Y", "Z")
            self.saturated_axes = tuple(
                axis for axis, value in zip(axes, raw_delta)
                if abs(float(value)) > XYZ_HALF_RANGE_M
            )
            delta = np.clip(raw_delta, -XYZ_HALF_RANGE_M, XYZ_HALF_RANGE_M)
            raw_target = self.arm_anchor.copy()
            raw_target[:3, 3] += raw_delta
            target = self.arm_anchor.copy()
            target[:3, 3] += delta
            self.hand_delta = (du, dv, size_ratio - 1.0)
            self.base_delta = tuple(float(value) for value in raw_delta)
            self.limited_delta = tuple(float(value) for value in delta)
            self.requested_target = raw_target
            self.target_pose = target.copy()

            jaw_angle = getattr(sample, "jaw_angle_deg", None)
            grip_target = self.gripper_target
            if jaw_angle is not None and math.isfinite(float(jaw_angle)):
                grip_target = self._map_gripper(jaw_angle)
                self.gripper_target = grip_target
            grip_action = ({"gripper.pos": grip_target}
                           if grip_target is not None and abs(grip - grip_target) > 1.0 else {})

            position_delta = target[:3, 3] - self.arm_anchor[:3, 3]
            if np.all(np.abs(position_delta) <= self.deadband_xyz):
                if grip_action:
                    self._send(grip_action)
                self.feedback = "axis cap: " + ",".join(self.saturated_axes) if self.saturated_axes else "at anchor"
                return

            dt = max(0.001, min(now - self.last_step, 0.05))
            max_step = min(math.degrees(JOINT_RATE_RAD_S * dt), MAX_STEP_DEG)
            started = time.perf_counter_ns()
            result = self.kinematics.solve_position(q, target, max_step_deg=max_step)
            self.latest_timings["ik"] = (time.perf_counter_ns() - started) / 1e6
            self.last_ik_result = result
            if not result.ok:
                if result.at_limit:
                    if grip_action:
                        self._send(grip_action)
                    self.feedback = "LIMIT: target saturated at reachable boundary"
                    return
                self._write_hold(f"IK {result.reason}")
                return
            q_result = np.asarray(result.q_deg, dtype=float)
            if (q_result.shape != (len(JOINT_NAMES),) or not np.all(np.isfinite(q_result))
                    or np.any(q_result < self.kinematics.ik_lower - 1e-7)
                    or np.any(q_result > self.kinematics.ik_upper + 1e-7)):
                self._write_hold("IK returned an invalid or out-of-limit vector")
                return
            action = {f"{name}.pos": float(value) for name, value in zip(JOINT_NAMES, q_result)}
            action.update(grip_action)
            self._send(action)
            self.last_step = now
            self.feedback = ("IK boundary" if result.reason == "LIMIT" else
                             "axis cap: " + ",".join(self.saturated_axes) if self.saturated_axes else "moving")
        except Exception as exc:
            self._fail(f"movement failed: {exc}")

    def _write_hold(self, reason):
        try:
            _, q, grip = self._read()
            bounded, clipped = self.kinematics.clamp(q)
            if clipped:
                self._start_calibration_recovery(q, time.monotonic())
                return
            self._send({**{f"{name}.pos": float(value) for name, value in zip(JOINT_NAMES, bounded)},
                        "gripper.pos": grip})
        except Exception as exc:
            self._fail(f"HOLD not confirmed ({reason}): {exc}")
            return
        self._clear_anchor()
        self.debouncer.reset()
        self.state, self.limit_kind = "READY", None
        self.feedback = "stopped; show PALM" if reason == "FIST" else f"held: {reason}"

    def hold(self, reason="stop"):
        if self.state == "ACTIVE":
            self._write_hold(reason)
        elif self.state == "LIMIT" and self.limit_kind == "IK":
            self._write_hold(reason)
        elif self.state == "LIMIT" and self.limit_kind == "CALIBRATION":
            if not self.feedback.startswith("BLOCKED"):
                self.feedback = "LIMIT: calibrated boundary recovery; awaiting readback"
        elif self.state == "BLOCKED":
            self.feedback = "BLOCKED: control stopped; still polling readback"
        else:
            self._clear_anchor()
            self.debouncer.reset()
            self.state, self.limit_kind = "READY", None
            self.feedback = f"READY: {reason}"

    def update(self, sample, now=None):
        now = time.monotonic() if now is None else float(now)
        self.latest_timings = {}
        if self.state == "BLOCKED":
            self._check_limit(now)
            return
        raw_gesture = getattr(sample, "gesture", None) if sample is not None else None
        if raw_gesture is None:
            sample = None
        if self.state == "LIMIT" and self.limit_kind == "CALIBRATION":
            self._check_limit(now)
            return
        if raw_gesture == "FIST":
            if self.state in ("ACTIVE", "LIMIT"):
                self.hold("FIST")
            else:
                self.feedback = "show PALM"
            return
        if self.state == "LIMIT":
            self._check_limit(now)
            return
        if sample is None:
            self.debouncer.update(None)
            self.pending_palms.clear()
            if self.state == "ACTIVE" and self.last_seen is not None:
                if not self.hand_missing:
                    self.hand_missing = True
                    self.hand_anchor = self.arm_anchor = None
                    self.requested_target = None
                if now - self.last_seen >= HAND_LOST_HOLD_S:
                    self.hold("hand-lost")
            return

        self.last_seen = now
        if self.state == "ACTIVE" and self.hand_missing:
            if raw_gesture == "PALM":
                self.pending_palms.append((sample.hx, sample.hy, sample.size))
            else:
                self.pending_palms.clear()
            if self.debouncer.update(raw_gesture) == "PALM":
                self._reanchor(sample, now)
            else:
                self.feedback = "re-anchor"
            return

        if raw_gesture == "PALM":
            self.pending_palms.append((sample.hx, sample.hy, sample.size))
        else:
            self.pending_palms.clear()
        stable = self.debouncer.update(raw_gesture)
        if self.state == "READY":
            if stable == "PALM":
                self._activate(sample, now)
            return
        if self.state == "ACTIVE":
            self._move(sample, now)
