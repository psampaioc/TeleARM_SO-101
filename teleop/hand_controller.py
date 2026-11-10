import argparse
from collections import defaultdict, deque
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from teleop.controller import GestureDebouncer, PALM_FRAMES, TeleopController
from teleop.kinematics import SO101Kinematics
from teleop.lerobot_adapter import JOINT_NAMES as MOTOR_NAMES, RobotAdapter


@dataclass(frozen=True)
class HandSample:
    hx: float
    hy: float
    size: float
    gesture: str | None
    confidence: float
    landmarks: np.ndarray | None = None
    palm_ratio: float | None = None
    jaw_angle_deg: float | None = None
    palm_normal: np.ndarray | None = None


FIST_RATIO_MAX = 0.7
PALM_RATIO_MIN = 1.1
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
    if sample is not None and sample.landmarks is not None:
        for index, (x, y) in enumerate(sample.landmarks):
            px, py = 799 - int(x * 800), int(y * 600)
            cv2.circle(display, (px, py), 3, (40, 230, 40), -1)
            cv2.putText(display, str(index), (px + 3, py - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.28, (255, 255, 255), 1)
    blocked = state == "BLOCKED" or getattr(controller, "feedback", "").startswith("BLOCKED")
    color = (100, 100, 255) if blocked else (220, 255, 220)
    lines = _hud_lines(mode, state, sample, controller, timings, count)
    panel = display.copy()
    cv2.rectangle(panel, (0, 0), (800, 285 if controller is not None else 245), (15, 15, 15), -1)
    cv2.addWeighted(panel, 0.78, display, 0.22, 0, display)
    for row, line in enumerate(lines):
        cv2.putText(display, line, (10, 20 + row * 21), cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1)
    return display


def _points3(landmarks):
    if isinstance(landmarks, np.ndarray):
        points = np.asarray(landmarks, dtype=float)
    else:
        points = np.asarray([[p.x, p.y, p.z] for p in landmarks], dtype=float)
    if points.shape != (21, 3) or not np.all(np.isfinite(points)):
        raise ValueError("hand landmarks must be a finite 21x3 array")
    return points


def _unit(vector):
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        raise ValueError("degenerate hand geometry")
    return vector / norm


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
    cosine = float(np.clip(cosine, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _palm_normal(points):
    points = _points3(points)
    normal = np.cross(points[5] - points[0], points[17] - points[0])
    norm = float(np.linalg.norm(normal))
    return normal / norm if norm >= 1e-8 else None


def classify_gesture(world_landmarks):
    points = _points3(world_landmarks)
    palm_ratio = _palm_fist_ratio(points)
    if palm_ratio is None:
        return None
    if palm_ratio <= FIST_RATIO_MAX:
        return "FIST"
    if palm_ratio >= PALM_RATIO_MIN:
        return "PALM"
    return "HAND"


def _sample(result):
    if not result.hand_landmarks or not result.hand_world_landmarks:
        return None
    image_points = np.asarray([[p.x, p.y] for p in result.hand_landmarks[0]], dtype=float)
    if image_points.shape != (21, 2) or not np.all(np.isfinite(image_points)):
        return None
    center = image_points[0]
    size = float(np.linalg.norm(image_points[0] - image_points[9]))
    if not (0 <= center[0] <= 1 and 0 <= center[1] <= 1 and 0 < size <= 1):
        return None
    try:
        world_points = _points3(result.hand_world_landmarks[0])
        gesture = classify_gesture(world_points)
        palm_ratio = _palm_fist_ratio(world_points)
        if gesture is None or palm_ratio is None:
            return None
        jaw_angle = _jaw_angle(world_points)
        palm_normal = _palm_normal(world_points)
    except ValueError:
        return None
    confidence = 0.0
    if result.handedness and result.handedness[0]:
        confidence = float(result.handedness[0][0].score)
    return HandSample(float(center[0]), float(center[1]), size, gesture, confidence,
                      image_points, palm_ratio, jaw_angle, palm_normal)


def _save_snapshot(cv2, image, label, directory=Path("/workspace/data")):
    if not directory.is_dir():
        return None
    path = directory / f"hand_tracking_{label.lower()}_{time.time_ns()}.png"
    if path.exists():
        return None
    try:
        if not cv2.imwrite(str(path), image):
            raise OSError("OpenCV could not write the image")
    except Exception as exc:
        print(f"SNAPSHOT failed: {exc}", flush=True)
        return None
    return path


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
    return (f"{state or 'PREVIEW'} raw={gesture} palm={palm} d2-12/span5-17={ratio} "
            f"jaw4-9-12={jaw} grip read/target={grip}\n"
            f"{motors} dXYZmm={delta} IK={ik} "
            f"feedback={feedback} {perf}")


def _gesture_status(gesture, count, detected):
    if gesture == "PALM":
        return f"PALM {count}/3"
    return gesture or ("HAND" if detected else "NO HAND")


def _build_live_control(args):
    if not args.live:
        return None, None
    adapter = RobotAdapter(args.port, args.robot_id, live=True)
    kinematics = SO101Kinematics(calibration_path=adapter.calibration_file)
    return adapter, TeleopController(adapter, kinematics)


def run(args):
    import cv2
    import mediapipe as mp

    model_path = Path(args.model)
    if not model_path.is_file():
        raise FileNotFoundError(f"MediaPipe hand model not found: {model_path}")
    camera = None
    adapter = controller = None
    preview_gestures = GestureDebouncer()
    attempted_snapshots = set()
    last_video_timestamp = 0
    timing_window = TimingWindow()
    last_log = 0.0
    try:
        camera = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        if not camera.isOpened():
            raise RuntimeError(f"cannot open camera {args.camera}")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 30)
        options = mp.tasks.vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=mp.tasks.vision.RunningMode.VIDEO,
            num_hands=1,
            min_hand_detection_confidence=0.6,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        with mp.tasks.vision.HandLandmarker.create_from_options(options) as landmarker:
            adapter, controller = _build_live_control(args)
            if adapter is not None:
                adapter.connect()
                print(f"LIVE — robot={args.robot_id} serial={args.port}; MediaPipe CPU; SPACE holds, q exits")
            else:
                print("PREVIEW — serial OFF; MediaPipe CPU; q exits")
            cv2.namedWindow("hand_tracking", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("hand_tracking", 800, 600)
            while True:
                loop_start = time.perf_counter_ns()
                stages = {}
                started = time.perf_counter_ns()
                ok, frame = camera.read()
                stages["camera"] = (time.perf_counter_ns() - started) / 1e6
                if not ok or frame is None:
                    if controller is not None:
                        controller.hold("camera-frame-lost")
                    raise RuntimeError(f"camera {args.camera} stopped returning frames")
                timestamp_ms = time.monotonic_ns() // 1_000_000
                timestamp_ms = max(timestamp_ms, last_video_timestamp + 1)
                last_video_timestamp = timestamp_ms
                started = time.perf_counter_ns()
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                stages["convert"] = (time.perf_counter_ns() - started) / 1e6
                started = time.perf_counter_ns()
                result = landmarker.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), timestamp_ms
                )
                stages["infer"] = (time.perf_counter_ns() - started) / 1e6
                vision_ms = (time.perf_counter_ns() - loop_start) / 1e6
                stages["vision"] = vision_ms
                started = time.perf_counter_ns()
                stale = vision_ms > 250.0
                sample = None if stale else _sample(result)
                raw_gesture = sample.gesture if sample is not None else None
                stages["feature"] = (time.perf_counter_ns() - started) / 1e6
                now = time.monotonic()
                if controller is not None:
                    started = time.perf_counter_ns()
                    controller.update(sample, now)
                    stages["controller"] = (time.perf_counter_ns() - started) / 1e6
                    stages.update(controller.latest_timings)
                    state = controller.state
                    count = controller.palm_count if raw_gesture == "PALM" else 0
                    mode = f"LIVE {args.robot_id} {args.port}"
                else:
                    preview_gestures.update(raw_gesture)
                    count = preview_gestures.count if preview_gestures.candidate == raw_gesture else 0
                    state, mode = None, "PREVIEW | serial OFF"

                started = time.perf_counter_ns()
                display = _draw_overlay(cv2, frame, sample, mode, state, controller, timing_window, count)
                cv2.imshow("hand_tracking", display)
                key = cv2.waitKey(1) & 0xFF
                stages["display"] = (time.perf_counter_ns() - started) / 1e6
                stages["loop"] = (time.perf_counter_ns() - loop_start) / 1e6
                timing_window.add(stages)
                snapshot_labels = ["camera"]
                if sample is not None:
                    snapshot_labels.append("tracking")
                if sample is not None and raw_gesture == "PALM" and count == PALM_FRAMES:
                    snapshot_labels.append("palm")
                if sample is not None and state == "ACTIVE" and sample.jaw_angle_deg is not None:
                    if 10 <= sample.jaw_angle_deg <= 60:
                        snapshot_labels.append("jaw")
                    if sample.jaw_angle_deg <= 10:
                        snapshot_labels.append("jaw_closed")
                if sample is not None and raw_gesture == "FIST":
                    snapshot_labels.append("fist")
                for label in snapshot_labels:
                    if label not in attempted_snapshots:
                        attempted_snapshots.add(label)
                        path = _save_snapshot(cv2, display, label)
                        if path is not None:
                            print(f"SNAPSHOT {path}", flush=True)
                        else:
                            print(f"SNAPSHOT unavailable: {label}", flush=True)
                if now - last_log >= 1.0:
                    print(_terminal_status(state, sample, count, controller, timing_window), flush=True)
                    last_log = now
                if key == ord(" ") and controller is not None:
                    controller.hold("SPACE")
                elif key == ord("q"):
                    recovering = (controller is not None and controller.state == "LIMIT"
                                  and controller.limit_kind == "CALIBRATION")
                    if recovering:
                        print("WARNING: leaving during calibration recovery; torque will turn off.", flush=True)
                    break
    finally:
        if controller is not None and controller.state in ("ACTIVE", "LIMIT"):
            controller.hold("exit")
            if controller.state == "LIMIT" and controller.limit_kind == "CALIBRATION":
                print("WARNING: boundary readback was not confirmed before disconnect; the motor may still execute its last calibrated target.", flush=True)
        if camera is not None:
            camera.release()
        try:
            cv2.destroyAllWindows()
        except UnboundLocalError:
            pass
        if adapter is not None:
            adapter.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Single-process SO-101 hand tracking teleoperation")
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--model", default="/workspace/data/models/hand_landmarker.task")
    parser.add_argument("--port", default="/dev/ttyACM0", help="follower serial port; used only with --live")
    parser.add_argument("--robot-id", default="teleArm_follower", help="LeRobot calibration ID; used only with --live")
    parser.add_argument("--live", action="store_true", help="connect to and command the follower; preview is the default")
    try:
        run(parser.parse_args())
    except KeyboardInterrupt:
        print("\nInterrupted; camera and robot connection closed.")


if __name__ == "__main__":
    main()
