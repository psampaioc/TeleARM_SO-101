# !/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specif

import copy
import time

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    GripperVelocityToJoint,
    InverseKinematicsEEToJoints,
)
from lerobot.robots.so_follower.so_follower import SO101Follower
from camera_processor import MapCameraHandActionToRobotAction

import cv2
import mediapipe as mp
import threading
import queue
import numpy as np

class FixedJointKinematics(RobotKinematics):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.fixed_targets = None
        self.base_index = self.joint_names.index("shoulder_pan")
        self.pitch_index = self.joint_names.index("wrist_flex")
        self.roll_index = self.joint_names.index("wrist_roll")
        self.solver.mask_dof("shoulder_pan")
        self.solver.mask_dof("wrist_flex")
        self.solver.mask_dof("wrist_roll")

    def inverse_kinematics(self, current_joint_pos, desired_ee_pose, *args, **kwargs):
        joints = np.array(current_joint_pos, dtype=float, copy=True)
        if self.fixed_targets is not None:
            joints[self.base_index], joints[self.pitch_index], joints[self.roll_index] = self.fixed_targets
        return super().inverse_kinematics(joints, desired_ee_pose, *args, **kwargs)

def calibrated_fixed_bounds(bus):
    bounds = {}
    for name in ("shoulder_pan", "wrist_flex", "wrist_roll"):
        motor = bus.motors[name]
        cal = bus.calibration[name]
        if cal.id != motor.id or cal.range_min >= cal.range_max:
            raise ValueError(f"calibração inválida: {name}")
        low = float(bus._normalize({motor.id: cal.range_min})[motor.id])
        high = float(bus._normalize({motor.id: cal.range_max})[motor.id])
        if not np.isfinite([low, high]).all() or low >= high:
            raise ValueError(f"limites calibrados inválidos: {name}")
        bounds[name] = (low, high)
    return bounds

def fixed_targets_in_bounds(targets, bounds):
    return len(targets) == 3 and all(np.isfinite(value) and bounds[name][0] <= value <= bounds[name][1]
               for name, value in zip(("shoulder_pan", "wrist_flex", "wrist_roll"), targets))

FPS = 45

# Initialize the robot and teleoperator
robot_config = SO101FollowerConfig(
    port="/dev/ttyACM0", id="teleArm_follower", use_degrees=True
)

# Initialize the robot and teleoperator
robot = SO101Follower(robot_config)

# NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
kinematics_solver = FixedJointKinematics(
    urdf_path="/workspace/teleop/models/so101/so101_new_calib.urdf",
    target_frame_name="wrist_link",
    joint_names=list(robot.bus.motors.keys()),
)

# Build the LeRobot pipeline from camera-hand deltas to end-effector and joint actions.
hand_to_robot_joints_processor = RobotProcessorPipeline[tuple[RobotAction, RobotObservation], RobotAction](
    steps=[
        MapCameraHandActionToRobotAction(),
        EEReferenceAndDelta(
            kinematics=kinematics_solver,
            end_effector_step_sizes={"x": 0.5, "y": 0.5, "z": 0.5},
            motor_names=list(robot.bus.motors.keys()),
            use_latched_reference=True,
        ),
        EEBoundsAndSafety(
            end_effector_bounds={"min": [-1.0, -1.0, -1.0], "max": [1.0, 1.0, 1.0]},
            max_ee_step_m=0.10,
        ),
        GripperVelocityToJoint(
            speed_factor=20.0,
        ),
        InverseKinematicsEEToJoints(
            kinematics=kinematics_solver,
            motor_names=list(robot.bus.motors.keys()),
            initial_guess_current_joints=True,
            orientation_weight=0.0,
        ),
    ],
    to_transition=robot_action_observation_to_transition,
    to_output=transition_to_robot_action,
)

# Initialize MediaPipe Tasks Hand Landmarker
hands = mp.tasks.vision.HandLandmarker.create_from_options(
    mp.tasks.vision.HandLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(
            model_asset_path="/workspace/data/models/hand_landmarker.task"
        ),
        running_mode=mp.tasks.vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
)

# Open webcam
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30.0)

if not cap.isOpened():
    hands.close()
    raise RuntimeError("Could not open camera /dev/video0")

# Connect only after the local model and camera have initialized.
robot.connect()
if not robot.is_connected:
    cap.release()
    hands.close()
    raise RuntimeError("SO-101 follower did not connect")

try:
    fixed_bounds = calibrated_fixed_bounds(robot.bus)
except Exception:
    cap.release()
    hands.close()
    robot.disconnect()
    raise

print("Starting Brosnan hand teleoperation on SO-101. Press Esc to exit.", flush=True)

frame_queue = queue.Queue(maxsize=5)
stop_event = threading.Event()
vision_error = threading.Event()
action_queue = queue.Queue(maxsize=100)
tracking_lock = threading.Lock()
tracking_state = {"visible": False, "epoch": 0, "captured_at": 0.0, "status": "PAUSED"}
MAX_FRAME_AGE = 0.5
MAX_IK_ERROR_M = 0.005

def frame_capture():
    while not stop_event.is_set():
        success, frame = cap.read()
        if success:
            if not frame_queue.full():
                frame_queue.put((time.monotonic(), frame))
        else:
            print("Ignoring empty camera frame.")

def to_vec(landmark):
    return np.array([landmark.x, landmark.y])

def angle_delta(angle, reference):
    return (angle - reference + 180.0) % 360.0 - 180.0

def direction_angles(world_landmarks):
    points = np.array([[world_landmarks[i].x, world_landmarks[i].y, world_landmarks[i].z]
                       for i in (0, 2, 4, 5)], dtype=float)
    if not np.isfinite(points).all():
        return None
    forward = points[2] - points[1]
    length = np.linalg.norm(forward)
    if length < 1e-6:
        return None
    x, y, z = forward / length
    if np.hypot(x, z) < 0.1:
        return None
    forward = forward / length
    side = points[3] - points[0]
    side_length = np.linalg.norm(side)
    side = side - np.dot(side, forward) * forward
    if side_length < 1e-6 or np.linalg.norm(side) < 0.1 * side_length:
        return None
    up = np.array([0., -1., 0.])
    up -= np.dot(up, forward) * forward
    up /= np.linalg.norm(up)
    right = np.cross(up, forward)
    roll = np.degrees(np.arctan2(np.dot(side, right), np.dot(side, up)))
    return (float(np.degrees(np.arctan2(x, z))),
            float(np.degrees(np.arctan2(-y, np.hypot(x, z)))), float(roll))

def angle_between_vectors(v1, v2):
    # Normalize the vectors
    v1_u = v1 / np.linalg.norm(v1)
    v2_u = v2 / np.linalg.norm(v2)
    
    # Compute the dot product
    dot_product = np.dot(v1_u, v2_u)
    
    # Clip the dot product to avoid numerical issues with arccos
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # Compute the angle in radians and convert to degrees
    angle_rad = np.arccos(dot_product)
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg

def depth_estimate(hand_landmarks):
    diff1 = to_vec(hand_landmarks[4]) - to_vec(hand_landmarks[3])
    diff2 = to_vec(hand_landmarks[3]) - to_vec(hand_landmarks[2])
    diff3 = to_vec(hand_landmarks[2]) - to_vec(hand_landmarks[1])
    diff4 = to_vec(hand_landmarks[1]) - to_vec(hand_landmarks[0])

    diff5 = to_vec(hand_landmarks[8]) - to_vec(hand_landmarks[7])
    diff6 = to_vec(hand_landmarks[7]) - to_vec(hand_landmarks[6])
    diff7 = to_vec(hand_landmarks[6]) - to_vec(hand_landmarks[5])
    diff8 = to_vec(hand_landmarks[5]) - to_vec(hand_landmarks[0])
    return np.linalg.norm(diff1) + np.linalg.norm(diff2) + np.linalg.norm(diff3) + np.linalg.norm(diff4) + np.linalg.norm(diff5) + np.linalg.norm(diff6) + np.linalg.norm(diff7) + np.linalg.norm(diff8)


def frame_display():
    filtered_pos = filtered_depth = filtered_grip = None
    filtered_yaw = filtered_pitch = filtered_roll = None
    filter_epoch = None
    last_video_timestamp = 0
    while not stop_event.is_set():
        try:
            captured_at, frame = frame_queue.get(timeout=0.05)
        except queue.Empty:
            if cv2.waitKey(5) & 0xFF == 27:
                stop_event.set()
            continue

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        timestamp_ms = max(time.monotonic_ns() // 1_000_000, last_video_timestamp + 1)
        last_video_timestamp = timestamp_ms
        results = hands.detect_for_video(
            mp.Image(image_format=mp.ImageFormat.SRGB, data=image), timestamp_ms
        )
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        height, width = image.shape[:2]
        landmarks = results.hand_landmarks[0] if results.hand_landmarks else None
        world = results.hand_world_landmarks[0] if results.hand_world_landmarks else None
        direction = direction_angles(world) if world else None
        valid = landmarks is not None and direction is not None and 0 <= time.monotonic() - captured_at <= MAX_FRAME_AGE

        if landmarks is not None:
            for connection in mp.tasks.vision.HandLandmarksConnections.HAND_CONNECTIONS:
                start, end = landmarks[connection.start], landmarks[connection.end]
                cv2.line(image, (int(start.x * width), int(start.y * height)),
                         (int(end.x * width), int(end.y * height)), (0, 255, 0), 2)
            for index, landmark in enumerate(landmarks):
                point = (int(landmark.x * width), int(landmark.y * height))
                cv2.circle(image, point, 3, (0, 255, 0), -1)
                cv2.putText(image, str(index), (point[0] + 4, point[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)

        if valid:
            yaw, pitch, roll_angle = direction
            position = to_vec(landmarks[0])
            depth = depth_estimate(landmarks)
            thumb = to_vec(landmarks[4]) - to_vec(landmarks[0])
            finger = to_vec(landmarks[8]) - to_vec(landmarks[0])
            if np.linalg.norm(thumb) == 0 or np.linalg.norm(finger) == 0:
                valid = False
            else:
                grip = angle_between_vectors(thumb, finger)
                valid = bool(np.isfinite(position).all() and
                             np.isfinite([depth, grip, yaw, pitch, roll_angle]).all())

        if valid:
            with tracking_lock:
                if not tracking_state["visible"]:
                    tracking_state["epoch"] += 1
                tracking_state["visible"] = True
                tracking_state["captured_at"] = captured_at
                epoch = tracking_state["epoch"]
            if filter_epoch != epoch:
                filtered_pos, filtered_depth, filtered_grip = position, depth, grip
                filtered_yaw, filtered_pitch, filtered_roll = yaw, pitch, roll_angle
                filter_epoch = epoch
            else:
                filtered_pos = 0.9 * filtered_pos + 0.1 * position
                filtered_depth = 0.9 * filtered_depth + 0.1 * depth
                filtered_grip = 0.9 * filtered_grip + 0.1 * grip
                filtered_yaw += 0.1 * angle_delta(yaw, filtered_yaw)
                filtered_pitch += 0.1 * angle_delta(pitch, filtered_pitch)
                filtered_roll += 0.1 * angle_delta(roll_angle, filtered_roll)

            origin = np.array([landmarks[0].x * width, landmarks[0].y * height])
            thumb_direction = np.array([(landmarks[4].x - landmarks[2].x) * width,
                                        (landmarks[4].y - landmarks[2].y) * height])
            cv2.arrowedLine(image, tuple(origin.astype(int)),
                            tuple((origin + thumb_direction).astype(int)), (0, 0, 255), 2)
            action = {"epoch": epoch, "captured_at": captured_at,
                      "hand_pos": filtered_pos.copy(), "hand_depth": float(filtered_depth),
                      "yaw": filtered_yaw, "pitch": filtered_pitch, "roll_angle": filtered_roll,
                      "gripper_angle": float(filtered_grip)}
            try:
                action_queue.put_nowait(action)
            except queue.Full:
                pass
        else:
            with tracking_lock:
                if tracking_state["visible"]:
                    tracking_state["visible"] = False
                    tracking_state["epoch"] += 1

        with tracking_lock:
            status = tracking_state["status"]
        label = "HAND - WAIT" if landmarks is not None and not valid else status
        cv2.putText(image, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0) if label == "ACTIVE" else (0, 0, 255), 2)
        cv2.imshow("SO-101 Hand Tracking", image)
        if cv2.waitKey(5) & 0xFF == 27:
            stop_event.set()

def robot_process():
    pipeline = hand_to_robot_joints_processor
    names = list(robot.bus.motors)
    anchor = None
    held = False
    control_error = False
    try:
        while not stop_event.is_set():
            start = time.monotonic()
            with tracking_lock:
                if (tracking_state["visible"] and
                        start - tracking_state["captured_at"] > MAX_FRAME_AGE):
                    tracking_state["visible"] = False
                    tracking_state["epoch"] += 1
                visible = tracking_state["visible"]
                epoch = tracking_state["epoch"]

            # Uma perda pode ter ocorrido mesmo que a mão já tenha voltado.
            if anchor is not None and anchor["epoch"] != epoch:
                reading = robot.get_observation()
                hold = {f"{name}.pos": reading[f"{name}.pos"] for name in names}
                with tracking_lock:
                    if tracking_state["epoch"] == epoch and not stop_event.is_set():
                        fixed = (hold["shoulder_pan.pos"], hold["wrist_flex.pos"], hold["wrist_roll.pos"])
                        if fixed_targets_in_bounds(fixed, fixed_bounds):
                            robot.send_action(hold)
                            anchor, held = None, True
                            tracking_state["status"] = "PAUSED"
                            print("PAUSED: referência anterior invalidada", flush=True)
                        else:
                            anchor, held = None, False
                            tracking_state["status"] = "WAIT MOTOR LIMIT"
                            print("WAIT MOTOR LIMIT: último objetivo pode permanecer ativo", flush=True)
                continue

            if not visible:
                with tracking_lock:
                    if tracking_state["status"] != "WAIT MOTOR LIMIT":
                        tracking_state["status"] = "PAUSED"
                if not held:
                    reading = robot.get_observation()
                    hold = {f"{name}.pos": reading[f"{name}.pos"] for name in names}
                    with tracking_lock:
                        if not tracking_state["visible"] and not stop_event.is_set():
                            fixed = (hold["shoulder_pan.pos"], hold["wrist_flex.pos"], hold["wrist_roll.pos"])
                            if fixed_targets_in_bounds(fixed, fixed_bounds):
                                robot.send_action(hold)
                                anchor, held = None, True
                                tracking_state["status"] = "PAUSED"
                                print("PAUSED: mão indisponível", flush=True)
                            else:
                                if tracking_state["status"] != "WAIT MOTOR LIMIT":
                                    print("WAIT MOTOR LIMIT: último objetivo pode permanecer ativo", flush=True)
                                tracking_state["status"] = "WAIT MOTOR LIMIT"
                else:
                    anchor = None
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue

            sample = None
            while True:
                try:
                    sample = action_queue.get_nowait()
                except queue.Empty:
                    break
            if (sample is None or sample["epoch"] != epoch or
                    time.monotonic() - sample["captured_at"] > MAX_FRAME_AGE):
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue

            reading = robot.get_observation()
            obs = {f"{name}.pos": reading[f"{name}.pos"] for name in names}
            current_fixed = (obs["shoulder_pan.pos"], obs["wrist_flex.pos"], obs["wrist_roll.pos"])
            if not fixed_targets_in_bounds(current_fixed, fixed_bounds):
                with tracking_lock:
                    if tracking_state["status"] != "WAIT MOTOR LIMIT":
                        print(f"WAIT MOTOR LIMIT: leitura M1/M4/M5 fora da calibração: {current_fixed}; "
                              "último objetivo pode permanecer ativo", flush=True)
                    tracking_state["status"] = "WAIT MOTOR LIMIT"
                anchor, held = None, False
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue
            trial = copy.copy(pipeline)
            trial.steps = [copy.copy(step) for step in pipeline.steps]
            fresh = anchor is None or anchor["epoch"] != epoch
            if fresh:
                proposed = {"epoch": epoch, "pos": sample["hand_pos"].copy(),
                            "depth": sample["hand_depth"], "yaw": sample["yaw"], "pitch": sample["pitch"],
                            "roll": sample["roll_angle"], "grip": sample["gripper_angle"],
                            "m1": obs["shoulder_pan.pos"], "m4": obs["wrist_flex.pos"], "m5": obs["wrist_roll.pos"],
                            "joints": np.array([obs[f"{name}.pos"] for name in names]),
                            "advance": 0.0, "last_pos": sample["hand_pos"].copy(),
                            "last_depth": sample["hand_depth"]}
                initial_pose = kinematics_solver.forward_kinematics(proposed["joints"])
                initial_shoulder = kinematics_solver.robot.get_T_world_frame("shoulder_link")
                radial_side = np.dot(initial_shoulder[:2, 0], initial_pose[:2, 3] - initial_shoulder[:2, 3])
                proposed["radial_sign"] = 1.0 if radial_side >= 0 else -1.0
                trial.steps[1].reset()
                with tracking_lock:
                    valid = (tracking_state["visible"] and tracking_state["epoch"] == epoch and
                             0 <= time.monotonic() - sample["captured_at"] <= MAX_FRAME_AGE)
                    if valid and not stop_event.is_set():
                        if not held:
                            sent = robot.send_action(obs)
                            if sent != obs:
                                raise RuntimeError("o comando enviado difere do calculado")
                        anchor, pipeline, held = proposed, trial, False
                        tracking_state["status"] = "ACTIVE"
                        print("ACTIVE: nova referência", flush=True)
                continue
            else:
                proposed = anchor.copy()

            targets = (
                proposed["m1"] + angle_delta(sample["yaw"], proposed["yaw"]),
                obs["wrist_flex.pos"],
                proposed["m5"] + angle_delta(sample["roll_angle"], proposed["roll"]),
            )
            if not fixed_targets_in_bounds(targets, fixed_bounds):
                with tracking_lock:
                    valid = (tracking_state["visible"] and tracking_state["epoch"] == epoch and
                             0 <= time.monotonic() - sample["captured_at"] <= MAX_FRAME_AGE)
                    if valid and not stop_event.is_set():
                        if not held:
                            robot.send_action(obs)
                            held = True
                        anchor["grip"] = sample["gripper_angle"]
                        if tracking_state["status"] != "WAIT HAND LIMIT":
                            print(f"WAIT HAND LIMIT: alvo M1/M4/M5 fora da calibração: {targets}", flush=True)
                        tracking_state["status"] = "WAIT HAND LIMIT"
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue
            kinematics_solver.fixed_targets = targets
            reference_joints = proposed["joints"].copy()
            reference_joints[kinematics_solver.base_index] = targets[0]
            rotating_reference = kinematics_solver.forward_kinematics(reference_joints)
            shoulder = kinematics_solver.robot.get_T_world_frame("shoulder_link")
            radial = shoulder[:2, 0].copy()
            radial_norm = np.linalg.norm(radial)
            if not np.isfinite(radial_norm) or radial_norm < 1e-6:
                raise ValueError("eixo radial inválido")
            radial *= proposed["radial_sign"] / radial_norm
            hand_x = sample["hand_pos"][0] - proposed["last_pos"][0]
            hand_z = sample["hand_depth"] - proposed["last_depth"]
            yaw = np.deg2rad(sample["yaw"])
            # shortcut: apparent size changes with pose; upgrade to measured depth when available.
            proposed["advance"] += hand_x * np.sin(yaw) + hand_z
            proposed["last_pos"] = sample["hand_pos"].copy()
            proposed["last_depth"] = sample["hand_depth"]
            desired_position = rotating_reference[:3, 3].copy()
            desired_position[:2] += 0.5 * proposed["advance"] * radial
            desired_position[2] += 0.5 * -(sample["hand_pos"][1] - proposed["pos"][1])
            reference_pose = trial.steps[1].reference_ee_pose
            if reference_pose is None or not trial.steps[1]._prev_enabled:
                current_joints = np.array([obs[f"{name}.pos"] for name in names])
                reference_pose = kinematics_solver.forward_kinematics(current_joints)
            delta = (desired_position - reference_pose[:3, 3]) / 0.5
            camera_action = {
                "enabled": True,
                "target_x": delta[0],
                "target_y": delta[1],
                "target_z": delta[2],
                "target_wx": 0.0, "target_wy": 0.0, "target_wz": 0.0,
                "gripper_vel": (sample["gripper_angle"] - proposed["grip"]) * 0.5,
            }
            try:
                joint_action = trial((camera_action, obs))
            except (ValueError, RuntimeError) as exc:
                is_qp_error = str(exc).startswith("QPError: Problem: Infeasible QP")
                if not is_qp_error and not str(exc).startswith("EE jump "):
                    raise
                with tracking_lock:
                    valid = (tracking_state["visible"] and tracking_state["epoch"] == epoch and
                             0 <= time.monotonic() - sample["captured_at"] <= MAX_FRAME_AGE)
                    if valid and not stop_event.is_set():
                        if not held:
                            robot.send_action(obs)
                            held = True
                        anchor["grip"] = sample["gripper_angle"]
                        status = "WAIT IK" if is_qp_error else "WAIT MOVE LIMIT"
                        if tracking_state["status"] != status:
                            print(f"{status}: {exc}; motores={obs}; alvos M1/M4/M5={targets}", flush=True)
                        tracking_state["status"] = status
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue
            m4 = (proposed["m4"] - angle_delta(sample["pitch"], proposed["pitch"])
                  - (joint_action["shoulder_lift.pos"] - proposed["joints"][names.index("shoulder_lift")])
                  - (joint_action["elbow_flex.pos"] - proposed["joints"][names.index("elbow_flex")]))
            urdf_low, urdf_high = np.degrees(kinematics_solver.robot.get_joint_limits("wrist_flex"))
            if not (fixed_targets_in_bounds((targets[0], m4, targets[2]), fixed_bounds)
                    and urdf_low <= m4 <= urdf_high):
                with tracking_lock:
                    valid = (tracking_state["visible"] and tracking_state["epoch"] == epoch and
                             0 <= time.monotonic() - sample["captured_at"] <= MAX_FRAME_AGE)
                    if valid and not stop_event.is_set():
                        if not held:
                            robot.send_action(obs)
                            held = True
                        anchor["grip"] = sample["gripper_angle"]
                        if tracking_state["status"] != "WAIT HAND LIMIT":
                            print(f"WAIT HAND LIMIT: alvo M4 compensado fora dos limites: {m4}", flush=True)
                        tracking_state["status"] = "WAIT HAND LIMIT"
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue
            joint_action["wrist_flex.pos"] = m4
            trial.steps[4].q_curr[kinematics_solver.pitch_index] = m4
            desired = trial.steps[2]._last_pos
            actual = kinematics_solver.forward_kinematics(trial.steps[4].q_curr)[:3, 3]
            if not np.isfinite(actual).all() or not np.isfinite(desired).all():
                raise ValueError("resultado IK inválido")
            error = float(np.linalg.norm(actual - desired))
            if error > MAX_IK_ERROR_M:
                with tracking_lock:
                    valid = (tracking_state["visible"] and tracking_state["epoch"] == epoch and
                             0 <= time.monotonic() - sample["captured_at"] <= MAX_FRAME_AGE)
                    if valid and not stop_event.is_set():
                        if not held:
                            robot.send_action(obs)
                            held = True
                        anchor["grip"] = sample["gripper_angle"]
                        if tracking_state["status"] != "WAIT IK":
                            print(f"WAIT IK: erro {error:.3f}m", flush=True)
                        tracking_state["status"] = "WAIT IK"
                stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
                continue
            proposed["grip"] = sample["gripper_angle"]
            with tracking_lock:
                valid = (tracking_state["visible"] and tracking_state["epoch"] == epoch and
                         0 <= time.monotonic() - sample["captured_at"] <= MAX_FRAME_AGE)
                if valid and not stop_event.is_set():
                    sent = robot.send_action(joint_action)
                    if sent != joint_action:
                        raise RuntimeError("o comando enviado difere do calculado")
                    anchor, pipeline, held = proposed, trial, False
                    tracking_state["status"] = "ACTIVE"
            stop_event.wait(max(1 / FPS - (time.monotonic() - start), 0))
    except Exception as exc:
        control_error = True
        print(f"Erro no controlo do braço: {exc}", flush=True)
        stop_event.set()
    finally:
        if vision_error.is_set() or control_error:
            try:
                reading = robot.get_observation()
                hold = {f"{name}.pos": reading[f"{name}.pos"] for name in names}
                robot.send_action(hold)
                print("Erro no controlo ou visão: manutenção pedida", flush=True)
            except Exception as exc:
                print(f"Erro ao pedir manutenção: {exc}", flush=True)

def run_vision(target):
    try:
        target()
    except Exception as exc:
        print(f"Erro na visão: {exc}", flush=True)
        vision_error.set()
        stop_event.set()

# Start threads
capture_thread = threading.Thread(target=run_vision, args=(frame_capture,))
display_thread = threading.Thread(target=run_vision, args=(frame_display,))
robot_thread = threading.Thread(target=robot_process)

capture_thread.start()
display_thread.start()
robot_thread.start()

capture_thread.join()
display_thread.join()
robot_thread.join()

cap.release()
hands.close()
cv2.destroyAllWindows()
robot.disconnect()


