import argparse
import json
import math
import re
import time
from pathlib import Path

from teleop.controller import GestureDebouncer, PALM_FRAMES, TeleopController
from teleop.camera_mapping import load_metric_profile
from teleop.hand_geometry import (HandSample, FIST_RATIO_MAX, PALM_RATIO_MIN, _jaw_angle,
    _palm_fist_ratio, _palm_normal, _sample, classify_gesture)
from teleop.overlay import (TimingWindow, TIMING_KEYS, _draw_overlay,
    _diagnostic_frame, _gesture_status, _hud_lines, _motor_lines, _terminal_status)
from teleop.kinematics import SO101Kinematics
from teleop.lerobot_adapter import RobotAdapter

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


def _load_sample_label(path):
    label = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("session_id", "split", "hand_identity", "pose", "focus_zoom_mode", "measured_depth_m"):
        if label.get(key) is None:
            raise ValueError(f"sample label needs {key}")
    if not isinstance(label["session_id"], str) or not re.fullmatch(r"[A-Za-z0-9_-]+", label["session_id"]):
        raise ValueError("session_id must use letters, digits, _ or -")
    if label["split"] not in ("fit", "held_out"):
        raise ValueError("sample split must be fit or held_out")
    for key in ("hand_identity", "pose", "focus_zoom_mode"):
        if not isinstance(label[key], str) or not label[key].strip():
            raise ValueError(f"{key} must be a nonempty label")
    if not isinstance(label.get("use_for_metric"), bool):
        raise ValueError("use_for_metric must explicitly be true or false")
    for key in ("measured_depth_m", "measured_dx_m", "measured_dy_m", "uncertainty_m", "palm_to_grid_gap_m"):
        value = label.get(key)
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                  or not math.isfinite(value)):
            raise ValueError(f"{key} must be finite metres or null")
    if label["measured_depth_m"] <= 0:
        raise ValueError("measured_depth_m must be positive metres")
    return label


def _write_sample_capture(directory, label, frames, camera, frame_shape):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if not directory.is_dir():
        raise NotADirectoryError(directory)
    path = directory / f"hand_{label['session_id']}_{time.time_ns()}.json"
    if path.exists():
        raise FileExistsError(path)
    data = {"version": 1, "label": label, "camera": camera,
            "width_px": frame_shape[1], "height_px": frame_shape[0], "frames": frames}
    partial = path.with_suffix(".partial")
    if not directory.is_dir() or partial.exists():
        raise RuntimeError("sample destination missing or partial capture already exists")
    partial.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    if not directory.is_dir():
        raise NotADirectoryError(directory)
    partial.replace(path)
    return path


def _build_live_control(args, camera_mode=None):
    if not args.live:
        return None, None
    profile_path = getattr(args, "metric_profile", None)
    profile = None
    if profile_path:
        if camera_mode is None:
            raise ValueError("metric profile requires the actual camera capture mode")
        profile = load_metric_profile(profile_path, device=args.camera,
                                      width_px=camera_mode[0], height_px=camera_mode[1])
    adapter = RobotAdapter(args.port, args.robot_id, live=True)
    kinematics = SO101Kinematics(calibration_path=adapter.calibration_file)
    return adapter, TeleopController(adapter, kinematics, metric_profile=profile)


def run(args):
    import cv2
    import mediapipe as mp

    model_path = Path(args.model)
    if not model_path.is_file():
        raise FileNotFoundError(f"MediaPipe hand model not found: {model_path}")
    camera = None
    sample_label = _load_sample_label(args.sample_label) if args.sample_label else None
    if sample_label is not None and (args.live or args.sample_frames <= 0):
        raise ValueError("hand sample capture requires preview and positive --sample-frames")
    capture_remaining = 0
    capture_records = []
    adapter = controller = None
    preview_gestures = GestureDebouncer()
    attempted_snapshots = set()
    last_video_timestamp = 0
    timing_window = TimingWindow()
    last_log = 0.0
    last_state = None
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
            camera_mode = (int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
                           int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            adapter, controller = _build_live_control(args, camera_mode)
            if adapter is not None:
                adapter.connect()
                method = "metric measured profile" if controller.metric_profile is not None else "legacy empirical XYZ"
                print(f"LIVE — {method}; robot={args.robot_id} serial={args.port}; MediaPipe CPU; SPACE holds, q exits")
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
                if key == ord("s") and sample_label is not None and capture_remaining == 0:
                    capture_remaining = args.sample_frames
                    capture_records = []
                    print(f"CAPTURE {capture_remaining} valid frames; hold hand still", flush=True)
                if capture_remaining and sample is not None:
                    capture_records.append({
                        "timestamp_ns": time.time_ns(), "landmarks_normalized": sample.landmarks.tolist(),
                        "palm_center_normalized": list(sample.palm_center), "palm_area_normalized": sample.palm_area,
                        "gesture": sample.gesture, "jaw_angle_deg": sample.jaw_angle_deg,
                        "palm_normal": sample.palm_normal.tolist() if sample.palm_normal is not None else None,
                    })
                    capture_remaining -= 1
                    if capture_remaining == 0:
                        path = _write_sample_capture(args.samples_dir, sample_label, capture_records, args.camera, frame.shape)
                        print(f"CAPTURE saved {path}", flush=True)
                stages["display"] = (time.perf_counter_ns() - started) / 1e6
                snapshot_labels = ["camera"] if args.live else []
                if args.live and sample is not None:
                    snapshot_labels.append("tracking")
                if args.live and sample is not None and raw_gesture == "PALM" and count == PALM_FRAMES:
                    snapshot_labels.append("palm")
                if args.live and sample is not None and state == "ACTIVE" and sample.jaw_angle_deg is not None:
                    if 10 <= sample.jaw_angle_deg <= 60:
                        snapshot_labels.append("jaw")
                    if sample.jaw_angle_deg <= 10:
                        snapshot_labels.append("jaw_closed")
                if args.live and sample is not None and raw_gesture == "FIST":
                    snapshot_labels.append("fist")
                diagnostic = _diagnostic_frame(cv2, display, sample) if snapshot_labels else None
                for label in snapshot_labels:
                    if label not in attempted_snapshots:
                        attempted_snapshots.add(label)
                        path = _save_snapshot(cv2, diagnostic, label)
                        if path is not None:
                            print(f"SNAPSHOT {path}", flush=True)
                        else:
                            print(f"SNAPSHOT unavailable: {label}", flush=True)
                if state != last_state or now - last_log >= (0.2 if args.live else 1.0):
                    print(_terminal_status(state, sample, count, controller, timing_window), flush=True)
                    last_log = now
                    last_state = state
                stages["loop"] = (time.perf_counter_ns() - loop_start) / 1e6
                timing_window.add(stages)
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
    parser.add_argument("--metric-profile", help="measured profile under data/; absent means legacy empirical XYZ")
    parser.add_argument("--sample-label", help="preview-only JSON with independently measured depth and session labels")
    parser.add_argument("--sample-frames", type=int, default=30)
    parser.add_argument("--samples-dir", default="/workspace/data/hand_samples")
    try:
        run(parser.parse_args())
    except KeyboardInterrupt:
        print("\nInterrupted; camera and robot connection closed.")


if __name__ == "__main__":
    main()
