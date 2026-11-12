"""Capture and fit a measured chessboard calibration in the project container.

Capture with f=fit image, h=held-out image, q=quit. The board is 9x7 internal
corners. The measured printed square size is required; nominal SVG size is not
treated as a physical measurement.
"""

import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np


BOARD_CORNERS = (9, 7)
CAPTURE_WIDTH_PX = 640
CAPTURE_HEIGHT_PX = 480
CAPTURE_FPS = 30
MIN_FIT_IMAGES = 12
MIN_HELD_OUT_IMAGES = 3


def _output_directory(path):
    path = Path(path)
    path.mkdir(parents=True, exist_ok=True)
    if not path.is_dir():
        raise NotADirectoryError(path)
    return path


def _write_json(path, data):
    if not path.parent.is_dir():
        raise NotADirectoryError(path.parent)
    if path.exists():
        raise FileExistsError(path)
    partial = path.with_suffix(".partial")
    if partial.exists():
        raise FileExistsError(partial)
    partial.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    if not path.parent.is_dir():
        raise NotADirectoryError(path.parent)
    partial.replace(path)


def _find_corners(cv2, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, BOARD_CORNERS)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def capture(args):
    import cv2

    if not np.isfinite(args.square_mm) or args.square_mm <= 0:
        raise ValueError("--square-mm must be the measured printed square size")
    if not args.focus_zoom_mode:
        raise ValueError("--focus-zoom-mode must describe the locked camera focus/zoom")
    directory = _output_directory(args.output_dir)
    camera = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise RuntimeError(f"cannot open camera {args.camera}")
    try:
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_WIDTH_PX)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT_PX)
        camera.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
        cv2.namedWindow("camera_calibration", cv2.WINDOW_NORMAL)
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("camera stopped returning frames")
            height, width = frame.shape[:2]
            if (width, height) != (CAPTURE_WIDTH_PX, CAPTURE_HEIGHT_PX):
                raise RuntimeError(f"camera returned {width}x{height}, expected 640x480")
            corners = _find_corners(cv2, frame)
            display = frame.copy()
            if corners is not None:
                cv2.drawChessboardCorners(display, BOARD_CORNERS, corners, True)
            label = "f=fit h=holdout q=quit" if corners is not None else "show full 9x7 inner-corner board"
            cv2.putText(display, label, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 230, 40), 2)
            cv2.imshow("camera_calibration", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key not in (ord("f"), ord("h")) or corners is None:
                continue
            split = "fit" if key == ord("f") else "held_out"
            stem = f"board_{split}_{time.time_ns()}"
            image_path = directory / f"{stem}.png"
            metadata_path = directory / f"{stem}.json"
            if not directory.is_dir() or image_path.exists() or metadata_path.exists():
                raise RuntimeError("calibration destination missing or capture name already exists")
            if not cv2.imwrite(str(image_path), frame):
                raise OSError(f"could not save {image_path}")
            _write_json(metadata_path, {
                "version": 1,
                "image": image_path.name, "split": split, "camera": args.camera,
                "width_px": width, "height_px": height,
                "square_size_measured_mm": args.square_mm,
                "focus_zoom_mode": args.focus_zoom_mode,
                "board_corners": list(BOARD_CORNERS),
                "corners_px": corners.reshape(-1, 2).tolist(),
                "captured_unix_ns": time.time_ns(),
            })
            print(f"saved {split}: {image_path}", flush=True)
    finally:
        camera.release()
        cv2.destroyAllWindows()


def fit(args):
    import cv2

    directory = Path(args.output_dir)
    if not directory.is_dir():
        raise NotADirectoryError(directory)
    records = [json.loads(path.read_text(encoding="utf-8")) for path in sorted(directory.glob("board_*.json"))]
    fit_records = [record for record in records if record.get("split") == "fit"]
    held_records = [record for record in records if record.get("split") == "held_out"]
    if len(fit_records) < MIN_FIT_IMAGES or len(held_records) < MIN_HELD_OUT_IMAGES:
        raise ValueError(f"need at least {MIN_FIT_IMAGES} fit and {MIN_HELD_OUT_IMAGES} held-out board views")
    mode = (fit_records[0]["camera"], fit_records[0]["width_px"], fit_records[0]["height_px"],
            fit_records[0]["square_size_measured_mm"], fit_records[0]["focus_zoom_mode"])
    if not np.isfinite(mode[3]) or mode[3] <= 0:
        raise ValueError("printed square size must be measured in positive millimetres")
    grid = np.zeros((BOARD_CORNERS[0] * BOARD_CORNERS[1], 3), np.float32)
    grid[:, :2] = np.mgrid[:BOARD_CORNERS[0], :BOARD_CORNERS[1]].T.reshape(-1, 2)
    grid *= float(mode[3]) / 1000.0
    observed = []
    for record in fit_records + held_records:
        image_name = record.get("image")
        if record.get("version") != 1 or not isinstance(image_name, str):
            raise ValueError("board metadata version or image name is missing")
        image_path = directory / image_name
        if image_path.parent != directory or not image_path.is_file():
            raise ValueError("board metadata lacks its original image")
        image = cv2.imread(str(image_path))
        if image is None or image.shape[:2] != (mode[2], mode[1]):
            raise ValueError("board image cannot be decoded at the recorded resolution")
        current_mode = (record["camera"], record["width_px"], record["height_px"],
                        record["square_size_measured_mm"], record["focus_zoom_mode"])
        if current_mode != mode or tuple(record.get("board_corners", ())) != BOARD_CORNERS:
            raise ValueError("calibration captures mix camera modes or measured board size")
        points = np.asarray(record["corners_px"], dtype=np.float32)
        if points.shape != (len(grid), 2) or not np.all(np.isfinite(points)):
            raise ValueError("invalid board corner coordinates")
        observed.append(points.reshape(-1, 1, 2))
    rms, K, distortion, rvecs, tvecs = cv2.calibrateCamera(
        [grid] * len(fit_records), observed[:len(fit_records)], (mode[1], mode[2]), None, None
    )
    errors = []
    for index, (record, corners) in enumerate(zip(fit_records + held_records, observed)):
        if record["split"] == "fit":
            rotation, translation = rvecs[index], tvecs[index]
        else:
            solved, rotation, translation = cv2.solvePnP(grid, corners, K, distortion)
            if not solved:
                raise ValueError("held-out board pose could not be estimated")
        projected, _ = cv2.projectPoints(grid, rotation, translation, K, distortion)
        errors.append((np.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1), record["split"]))
    fit_error = np.concatenate([error for error, split in errors if split == "fit"])
    held_error = np.concatenate([error for error, split in errors if split == "held_out"])
    report = {
        "version": 1, "camera": mode[0], "width_px": mode[1], "height_px": mode[2],
        "focus_zoom_mode": mode[4],
        "square_size_measured_mm": mode[3], "fit_views": len(fit_records),
        "held_out_views": len(held_records), "fit_rms_px": float(rms),
        "fit_reprojection_p95_px": float(np.percentile(fit_error, 95)),
        "held_out_reprojection_p95_px": float(np.percentile(held_error, 95)),
        "K": K.tolist(), "distortion": distortion.ravel().tolist(),
    }
    if rms > 1.0 or report["held_out_reprojection_p95_px"] > 2.0:
        raise ValueError(f"calibration quality gate failed: {report}")
    output = directory / f"intrinsics_{time.time_ns()}.json"
    _write_json(output, report)
    print(f"intrinsics saved: {output}", flush=True)


def fit_hand(args):
    import cv2

    intrinsics_bytes = Path(args.intrinsics).read_bytes()
    intrinsics = json.loads(intrinsics_bytes)
    K = np.asarray(intrinsics["K"], dtype=float)
    distortion = np.asarray(intrinsics["distortion"], dtype=float)
    directory = Path(args.samples_dir)
    if not directory.is_dir():
        raise NotADirectoryError(directory)
    captures = [json.loads(path.read_text(encoding="utf-8")) for path in sorted(directory.glob("hand_*.json"))]
    if not captures:
        raise ValueError("no hand captures were found")
    sessions = {}
    rows = []
    hand_identity = None
    negative_captures = 0
    for capture_index, capture in enumerate(captures):
        if capture.get("version") != 1:
            raise ValueError("unsupported hand capture version")
        label = capture["label"]
        if not isinstance(label.get("hand_identity"), str) or not label["hand_identity"].strip():
            raise ValueError("hand capture needs a nonempty operator identity")
        if hand_identity is None:
            hand_identity = label["hand_identity"]
        if label["hand_identity"] != hand_identity:
            raise ValueError("hand captures mix different operator identities")
        split = label["split"]
        if split not in ("fit", "held_out"):
            raise ValueError("unknown sample split")
        session_id = label["session_id"]
        if session_id in sessions and sessions[session_id] != split:
            raise ValueError("a session cannot be split across fit and held-out data")
        sessions[session_id] = split
        mode = (capture["camera"], capture["width_px"], capture["height_px"], label["focus_zoom_mode"])
        expected = (intrinsics["camera"], intrinsics["width_px"], intrinsics["height_px"],
                    intrinsics["focus_zoom_mode"])
        if mode != expected:
            raise ValueError("hand samples and intrinsics use different camera modes")
        if not isinstance(label.get("use_for_metric"), bool):
            raise ValueError("each pose must explicitly opt into or out of metric fitting")
        if not label["use_for_metric"]:
            negative_captures += 1
            continue
        depth = float(label["measured_depth_m"])
        if not np.isfinite(depth) or depth <= 0:
            raise ValueError("hand depth label must be measured positive metres")
        landmarks = np.asarray([frame["landmarks_normalized"] for frame in capture["frames"]], dtype=float)
        if landmarks.ndim != 3 or landmarks.shape[1:] != (21, 2) or not np.all(np.isfinite(landmarks)):
            raise ValueError("hand capture has invalid 21-point frames")
        pixels = landmarks * np.array([mode[1], mode[2]])
        rays = cv2.undistortPoints(pixels.reshape(-1, 1, 2), K, distortion).reshape(-1, 21, 2)
        palm = rays[:, [0, 5, 9, 13, 17], :]
        center = palm.mean(axis=1)
        area = 0.5 * np.abs(np.sum(palm[:, :, 0] * np.roll(palm[:, :, 1], -1, axis=1)
                                    - palm[:, :, 1] * np.roll(palm[:, :, 0], -1, axis=1), axis=1))
        span = np.linalg.norm(rays[:, 5] - rays[:, 17], axis=1)
        if np.any(span <= 1e-8) or np.any(area <= 1e-8):
            raise ValueError("capture contains degenerate palm geometry; recapture it")
        for xy, s, a in zip(center, span, area):
            rows.append({"session": session_id, "capture": capture_index, "split": split, "pose": label["pose"],
                         "depth_m": depth, "dx_m": label.get("measured_dx_m"),
                         "dy_m": label.get("measured_dy_m"), "center": xy,
                         "span": float(s), "area": float(a)})
    if len({row["session"] for row in rows if row["split"] == "fit"}) < 3 or not any(
            row["split"] == "held_out" for row in rows):
        raise ValueError("need at least three fit sessions and one held-out session")
    fit_poses = {row["pose"] for row in rows if row["split"] == "fit"}
    held_poses = {row["pose"] for row in rows if row["split"] == "held_out"}
    if fit_poses != held_poses:
        raise ValueError("each metric pose needs both fit and held-out captures")

    candidates = {}
    for model in ("span", "area"):
        feature = lambda row: 1.0 / (row["span"] if model == "span" else np.sqrt(row["area"]))
        fit_rows = [row for row in rows if row["split"] == "fit"]
        held_rows = [row for row in rows if row["split"] == "held_out"]
        design = np.array([[feature(row), 1.0] for row in fit_rows])
        if len({row["depth_m"] for row in fit_rows}) < 2 or np.linalg.matrix_rank(design) < 2:
            candidates[model] = {"passed": False, "reason": "need two measured fit distances"}
            continue
        coefficient, offset = np.linalg.lstsq(design, [row["depth_m"] for row in fit_rows], rcond=None)[0]
        if coefficient <= 0 or not np.isfinite(coefficient) or not np.isfinite(offset):
            candidates[model] = {"passed": False, "reason": "invalid fitted scale"}
            continue
        for row in rows:
            z = coefficient * feature(row) + offset
            row[f"{model}_xyz"] = np.array([row["center"][0] * z, row["center"][1] * z, z])
        depth_errors = np.array([abs(row[f"{model}_xyz"][2] - row["depth_m"]) for row in held_rows])
        shift_errors = []
        drift = []
        for capture_index in {row["capture"] for row in held_rows}:
            xyz = np.array([row[f"{model}_xyz"] for row in held_rows if row["capture"] == capture_index])
            drift.append(float(np.percentile(np.linalg.norm(xyz - np.median(xyz, axis=0), axis=1), 95)))
        for session in {row["session"] for row in held_rows}:
            samples = [row for row in held_rows if row["session"] == session]
            reference = [row for row in samples if row["dx_m"] == 0 and row["dy_m"] == 0]
            if reference:
                origin = np.mean([row[f"{model}_xyz"][:2] for row in reference], axis=0)
                for row in samples:
                    if row["dx_m"] is not None and row["dy_m"] is not None:
                        observed = row[f"{model}_xyz"][:2] - origin
                        if row["dx_m"] != 0 or row["dy_m"] != 0:
                            shift_errors.append((row["pose"], observed - [row["dx_m"], row["dy_m"]]))
        depth_p95 = float(np.percentile(depth_errors, 95))
        shift_array = np.array([error for _, error in shift_errors]) if shift_errors else None
        shift_p95 = np.percentile(np.abs(shift_array), 95, axis=0).tolist() if shift_array is not None else None
        drift_p95 = max(drift)
        pose_errors = {
            pose: [row[f"{model}_xyz"][2] - row["depth_m"]
                   for row in held_rows if row["pose"] == pose]
            for pose in {row["pose"] for row in held_rows}
        }
        pose_depth = {
            pose: {"median_abs_m": float(np.median(np.abs(errors))),
                   "p95_abs_m": float(np.percentile(np.abs(errors), 95)),
                   "bias_m": float(np.mean(errors))}
            for pose, errors in pose_errors.items()
        }
        pose_shift = {
            pose: np.percentile(np.abs([error for name, error in shift_errors if name == pose]),
                                95, axis=0).tolist()
            for pose in {name for name, _ in shift_errors}
        }
        passed = (depth_p95 <= 0.03 and all(value["p95_abs_m"] <= 0.03 for value in pose_depth.values())
                  and drift_p95 <= 0.01 and shift_p95 is not None
                  and set(pose_shift) == held_poses
                  and all(value <= 0.02 for value in shift_p95)
                  and all(all(axis <= 0.02 for axis in values) for values in pose_shift.values()))
        candidates[model] = {
            "passed": passed, "effective_span_m" if model == "span" else "effective_area_m2":
                float(coefficient if model == "span" else coefficient**2),
            "axial_offset_m": float(offset), "held_out_depth_p95_m": depth_p95,
            "held_out_xy_shift_p95_m": shift_p95, "held_out_stationary_jitter_p95_m": drift_p95,
            "held_out_depth_by_pose_m": pose_depth, "held_out_xy_shift_p95_by_pose_m": pose_shift,
        }
    passed = [name for name, result in candidates.items() if result["passed"]]
    selected = min(passed, key=lambda name: candidates[name]["held_out_depth_p95_m"]) if passed else None
    report = {"version": 1, "selected_model": selected, "candidates": candidates,
              "hand_identity": hand_identity,
              "camera": {"device": intrinsics["camera"], "width_px": intrinsics["width_px"],
                         "height_px": intrinsics["height_px"], "focus_zoom_mode": intrinsics["focus_zoom_mode"]},
              "intrinsics_sha256": hashlib.sha256(intrinsics_bytes).hexdigest(),
              "negative_quality_captures": negative_captures,
              "valid_z_m": [min(row["depth_m"] for row in rows if row["split"] == "fit"),
                            max(row["depth_m"] for row in rows if row["split"] == "fit")],
              "fit_sessions": len({row["session"] for row in rows if row["split"] == "fit"}),
              "held_out_sessions": len({row["session"] for row in rows if row["split"] == "held_out"})}
    output = directory / f"scale_fit_{time.time_ns()}.json"
    _write_json(output, report)
    print(f"hand model report: {output}; selected={selected or 'none'}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="SO-101 webcam chessboard calibration; no robot access")
    parser.add_argument("mode", choices=("capture", "fit", "fit-hand"))
    parser.add_argument("--camera", default="/dev/video0")
    parser.add_argument("--square-mm", type=float, help="required only for capture: measured printed square side")
    parser.add_argument("--focus-zoom-mode", help="required only for capture: fixed focus and zoom setting")
    parser.add_argument("--output-dir", default="/workspace/data/hand_camera")
    parser.add_argument("--samples-dir", default="/workspace/data/hand_samples")
    parser.add_argument("--intrinsics", help="intrinsics JSON generated by fit; required for fit-hand")
    args = parser.parse_args()
    if args.mode == "capture":
        if args.square_mm is None:
            parser.error("capture requires --square-mm measured from the printed board")
        capture(args)
    elif args.mode == "fit":
        fit(args)
    else:
        if not args.intrinsics:
            parser.error("fit-hand requires --intrinsics")
        fit_hand(args)


if __name__ == "__main__":
    main()
