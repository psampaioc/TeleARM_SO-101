"""Presentation and timing for hand teleoperation."""
from collections import defaultdict, deque
import math
import time
import numpy as np
from teleop.controller import PALM_FRAMES
from teleop.lerobot_adapter import JOINT_NAMES as MOTOR_NAMES
TIMING_KEYS = ("camera", "convert", "infer", "feature", "read", "fk", "ik", "write", "controller", "display", "vision", "loop")

class TimingWindow:
    def __init__(self, warmup=30, window=300):
        self.warmup = int(warmup)
        self.values = defaultdict(lambda: deque(maxlen=int(window)))
        self.frames = 0

    def add(self, values):
        self.frames += 1
        if self.frames <= self.warmup:
            return
        for key, value in values.items():
            value = float(value)
            if math.isfinite(value):
                self.values[key].append(value)

    def percentile(self, key, percentile):
        values = self.values[key]
        return float(np.percentile(values, percentile)) if values else None

    def line(self, keys, labels):
        parts = []
        for key, label in zip(keys, labels):
            p50, p95 = self.percentile(key, 50), self.percentile(key, 95)
            parts.append(f"{label} {p50:.1f}/{p95:.1f}" if p50 is not None else f"{label} --/--")
        return "ms p50/p95 " + "  ".join(parts)


def _fmt_vector(values):
    if values is None:
        return "—"
    values = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(values)):
        return "—"
    return "[" + ",".join(f"{value:.2f}" for value in values) + "]"


def _motor_lines(controller):
    ids = getattr(getattr(controller, "adapter", None), "motor_ids", {})
    if not ids:
        return ["motors 1-3 read/cmd —", "motors 4-6 read/cmd —"]
    measured = getattr(controller, "last_measured_q", None)
    commanded = getattr(controller, "last_command_q", None)
    read = list(measured) if measured is not None else [None] * 5
    cmd = list(commanded) if commanded is not None else [None] * 5
    read.append(getattr(controller, "last_measured_gripper", None))
    cmd.append(getattr(controller, "last_command_gripper", None))

    def value(number):
        return f"{number:.2f}" if number is not None and math.isfinite(number) else "—"

    parts = [f"M{ids[name]} {value(read[index])}/{value(cmd[index])}"
             for index, name in enumerate(MOTOR_NAMES)]
    return ["motors[°/%] read/cmd " + " | ".join(parts[:3]),
            "motors[°/%] read/cmd " + " | ".join(parts[3:])]


def _motor_feedback(controller, feedback):
    ids = getattr(getattr(controller, "adapter", None), "motor_ids", {})
    for name, id_ in ids.items():
        feedback = feedback.replace(name, f"Motor {id_}")
    return feedback


def _hud_lines(mode, state, sample, controller, timings, count=0):
    gesture = _gesture_status(sample.gesture if sample is not None else None, count, sample is not None)
    feedback = _motor_feedback(controller, getattr(controller, "feedback", "")) if controller is not None else ""
    ids = getattr(getattr(controller, "adapter", None), "motor_ids", {})
    boundary = ",".join(f"Motor {ids.get(name, '?')}" for name in getattr(controller, "boundary_joints", ())) or "-"
    caps = ",".join(getattr(controller, "saturated_axes", ())) or "-"
    status = f" | {state}" if state else ""
    lines = [f"{mode}{status} | joints {boundary} | axes {caps}"]

    lines.extend(_motor_lines(controller))
    last_read = getattr(controller, "last_read_at", None) if controller is not None else None
    if controller is not None:
        age = f"{time.monotonic() - last_read:.1f}s" if last_read is not None else "never"
        freshness = "STALE" if last_read is None or time.monotonic() - last_read > 0.5 else "fresh"
        control = "NOT FOLLOWING HAND" if state == "BLOCKED" or feedback.startswith("BLOCKED") else "following" if state == "ACTIVE" else "waiting"
        lines.append(f"motor read {freshness} ({age}) | {control}")
    anchor = getattr(controller, "arm_anchor", None) if controller is not None else None
    measured_pose = getattr(controller, "measured_pose", None) if controller is not None else None
    lines.append(f"XYZ[m] read {_fmt_vector(measured_pose[:3, 3] if measured_pose is not None else None)} | anchor {_fmt_vector(anchor[:3, 3] if anchor is not None else None)}")

    ratio = getattr(sample, "palm_ratio", None) if sample is not None else None
    jaw = getattr(sample, "jaw_angle_deg", None) if sample is not None else None
    lines.append(f"hand {gesture} | d(2,12)/span(5,17) {ratio:.2f}" if ratio is not None else f"hand {gesture} | ratio —")
    target = getattr(controller, "gripper_target", None) if controller is not None else None
    jaw_text = f"{jaw:.0f}°" if jaw is not None else "—"
    target_text = f"{target:.0f}" if target is not None else "—"
    measured_grip = getattr(controller, "last_measured_gripper", None) if controller is not None else None
    grip_read = f"{measured_grip:.0f}" if measured_grip is not None else "—"
    lines.append(f"jaw ∠4-9-12 (opposite 4-12) {jaw_text} | grip read/target {grip_read}/{target_text}")

    hand_delta = getattr(controller, "hand_delta", None) if controller is not None else None
    base_delta = getattr(controller, "base_delta", None) if controller is not None else None
    limited_delta = getattr(controller, "limited_delta", None) if controller is not None else None
    lines.append(f"hand Δnorm {_fmt_vector(hand_delta)} | base Δ[m] raw {_fmt_vector(base_delta)} → cap {_fmt_vector(limited_delta)}")
    normal = getattr(sample, "palm_normal", None) if sample is not None else None
    lines.append(f"palm normal n(0,5,17) {_fmt_vector(normal)}")
    requested = getattr(controller, "requested_target", None) if controller is not None else None
    target = getattr(controller, "target_pose", None) if controller is not None else None
    lines.append(f"XYZ[m] raw {_fmt_vector(requested[:3, 3] if requested is not None else None)} → target {_fmt_vector(target[:3, 3] if target is not None else None)}")
    result = getattr(controller, "last_ik_result", None) if controller is not None else None
    ik = f"{result.reason} {result.residual_m * 1000:.1f}mm" if result is not None and math.isfinite(result.residual_m) else (result.reason if result is not None else "—")
    lines.append(f"feedback {feedback or '—'} | IK {ik}")
    lines.append(timings.line(("camera", "convert", "infer", "feature", "vision"), ("cam", "cvt", "inf", "feat", "vis")))
    lines.append(timings.line(("read", "fk", "ik", "write", "controller", "display", "loop"), ("read", "fk", "ik", "write", "ctrl", "ui", "loop")))
    return lines


def _draw_overlay(cv2, frame, sample, mode, state, controller, timings, count=0):
    display = cv2.flip(cv2.resize(frame, (800, 600), interpolation=cv2.INTER_LINEAR), 1)
    blocked = state == "BLOCKED" or getattr(controller, "feedback", "").startswith("BLOCKED")
    gesture = _gesture_status(sample.gesture if sample is not None else None, count, sample is not None)
    control = state or "PREVIEW"
    if blocked:
        control = "BLOCKED"
    elif getattr(controller, "feedback", "").startswith("INVALID"):
        control = "INVALID"
    elif state == "ACTIVE" and sample is None:
        control = "HOLD"
    color = (100, 100, 255) if control in ("BLOCKED", "INVALID") else (220, 255, 220)
    cv2.putText(display, f"{gesture} | {control}", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    latency = timings.percentile("loop", 95)
    if latency is not None:
        cv2.putText(display, f"loop p95 {latency:.0f} ms", (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
    center = getattr(sample, "palm_center", None) if sample is not None else None
    if center is not None:
        cv2.circle(display, (799 - int(center[0] * 800), int(center[1] * 600)), 5, (0, 0, 255), -1)
    return display


def _diagnostic_frame(cv2, display, sample):
    annotated = display.copy()
    if sample is not None and sample.landmarks is not None:
        for index, (x, y) in enumerate(sample.landmarks):
            px, py = 799 - int(x * 800), int(y * 600)
            cv2.circle(annotated, (px, py), 3, (40, 230, 40), -1)
            cv2.putText(annotated, str(index), (px + 3, py - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 255, 255), 1)
    return annotated


def _terminal_status(state, sample, count, controller, timings):
    gesture = sample.gesture if sample is not None else "NO_HAND"
    palm = f"{count}/3" if gesture == "PALM" else "-"
    ratio = f"{sample.palm_ratio:.2f}" if sample is not None and sample.palm_ratio is not None else "-"
    jaw = f"{sample.jaw_angle_deg:.0f}deg" if sample is not None and sample.jaw_angle_deg is not None else "-"
    grip_cmd = getattr(controller, "gripper_target", None) if controller is not None else None
    grip_read = getattr(controller, "last_measured_gripper", None) if controller is not None else None
    grip = f"{grip_read:.0f}/{grip_cmd:.0f}" if grip_read is not None and grip_cmd is not None else "-/-"
    motors = "\n".join(_motor_lines(controller))
    feedback = _motor_feedback(controller, getattr(controller, "feedback", "-")) if controller is not None else "-"
    delta = _fmt_vector(np.asarray(getattr(controller, "limited_delta", ()), dtype=float) * 1000) if controller is not None and getattr(controller, "limited_delta", None) is not None else "-"
    ik_result = getattr(controller, "last_ik_result", None) if controller is not None else None
    ik = f"{ik_result.reason}:{ik_result.residual_m * 1000:.1f}mm" if ik_result is not None and math.isfinite(ik_result.residual_m) else "-"
    vision_p95 = timings.percentile("vision", 95)
    loop_p95 = timings.percentile("loop", 95)
    perf = f"vis/loop p95={vision_p95:.0f}/{loop_p95:.0f}ms" if vision_p95 is not None and loop_p95 is not None else "timing-warmup"
    palm_center = _fmt_vector(getattr(sample, "palm_center", None))
    palm_area = getattr(sample, "palm_area", None)
    area = f"{palm_area:.5f}" if palm_area is not None else "—"
    hand_coords = getattr(controller, "current_hand_coords", None)
    hand_units = "C[m]" if getattr(controller, "metric_profile", None) is not None else "legacy[u,v,span]"
    measured_pose = getattr(controller, "measured_pose", None)
    target_pose = getattr(controller, "target_pose", None)
    return (f"{state or 'PREVIEW'} raw={gesture} palm={palm} d2-12/span5-17={ratio} "
            f"jaw4-9-12={jaw} grip read/target={grip}\n"
            f"palm_I={palm_center} area_norm={area} hand_{hand_units}={_fmt_vector(hand_coords)} "
            f"tool_B_read[m]={_fmt_vector(measured_pose[:3, 3] if measured_pose is not None else None)} "
            f"tool_B_target[m]={_fmt_vector(target_pose[:3, 3] if target_pose is not None else None)}\n"
            f"{motors} dXYZmm={delta} IK={ik} "
            f"feedback={feedback} {perf}")


def _gesture_status(gesture, count, detected):
    if gesture == "PALM":
        return f"PALM {count}/3"
    return gesture or ("HAND" if detected else "NO HAND")


