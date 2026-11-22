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

FPS = 45

# Initialize the robot and teleoperator
robot_config = SO101FollowerConfig(
    port="/dev/ttyACM0", id="teleArm_follower", use_degrees=True
)

# Initialize the robot and teleoperator
robot = SO101Follower(robot_config)

# NOTE: It is highly recommended to use the urdf in the SO-ARM100 repo: https://github.com/TheRobotStudio/SO-ARM100/blob/main/Simulation/SO101/so101_new_calib.urdf
kinematics_solver = RobotKinematics(
    urdf_path="/workspace/teleop/models/so101/so101_new_calib.urdf",
    target_frame_name="gripper_frame_link",
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

print("Starting Brosnan hand teleoperation on SO-101. Press Esc to exit.", flush=True)

frame_queue = queue.Queue(maxsize=5)
stop_event = threading.Event()
action_queue = queue.Queue(maxsize=100)

def frame_capture():
    while not stop_event.is_set():
        success, frame = cap.read()
        if success:
            if not frame_queue.full():
                frame_queue.put(frame)
        else:
            print("Ignoring empty camera frame.")

def to_vec(landmark):
    return np.array([landmark.x, landmark.y])

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

downvec =  np.array([0.0, 1.0])

def angle_between_vectors2(v1, v2):
    angle_deg = angle_between_vectors(v1, v2)

    down_angle = np.dot(v2, downvec)
    if (down_angle > 0.0):
        angle_deg = np.abs(angle_deg)
    else:
        angle_deg = -np.abs(angle_deg)
    
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
    hand_depth = -10000.0
    initial_hand_depth = -10000.0

    hand_pos = np.array([-10000.0, -10000.0])
    initial_hand_pos = np.array([-10000.0, -10000.0])

    thumb = np.array([-10000.0, -10000.0])
    initial_thumb = np.array([-10000.0, -10000.0])

    gripper_angle = -10000.0
    prev_gripper_angle = -10000.0

    target_wy = -10000.0
    last_video_timestamp = 0
    while not stop_event.is_set():
        if not frame_queue.empty():
            frame = frame_queue.get()

            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            timestamp_ms = time.monotonic_ns() // 1_000_000
            timestamp_ms = max(timestamp_ms, last_video_timestamp + 1)
            last_video_timestamp = timestamp_ms
            results = hands.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=image), timestamp_ms
            )

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.hand_landmarks:
                for hand_landmarks in results.hand_landmarks:
                    height, width = image.shape[:2]
                    for connection in mp.tasks.vision.HandLandmarksConnections.HAND_CONNECTIONS:
                        start = hand_landmarks[connection.start]
                        end = hand_landmarks[connection.end]
                        cv2.line(
                            image,
                            (int(start.x * width), int(start.y * height)),
                            (int(end.x * width), int(end.y * height)),
                            (0, 255, 0),
                            2,
                        )
                    for index, landmark in enumerate(hand_landmarks):
                        cv2.circle(
                            image,
                            (int(landmark.x * width), int(landmark.y * height)),
                            3,
                            (0, 255, 0),
                            -1,
                        )
                        cv2.putText(
                            image, str(index),
                            (int(landmark.x * width) + 4, int(landmark.y * height) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA,
                        )

                    # Estimate apparent hand scale from normalized image landmarks.
                    if hand_depth == -10000.0:
                        hand_depth = depth_estimate(hand_landmarks)
                    else:
                        hand_depth = 0.9*hand_depth + 0.1*depth_estimate(hand_landmarks)

                    if initial_hand_depth == -10000.0:
                        initial_hand_depth = hand_depth

                    # Track the normalized image position of landmark 2.
                    if hand_pos[0] == -10000.0:
                        hand_pos = to_vec(hand_landmarks[2])
                    else:
                        hand_pos = 0.9*hand_pos + 0.1*to_vec(hand_landmarks[2])
                    
                    if initial_hand_pos[0] == -10000.0:
                        initial_hand_pos = hand_pos

                    # Compute gripper angle
                    thumb = to_vec(hand_landmarks[4]) - to_vec(hand_landmarks[0])
                    finger = to_vec(hand_landmarks[8]) - to_vec(hand_landmarks[0])

                    if gripper_angle == -10000.0:
                        gripper_angle = angle_between_vectors(thumb, finger)
                    else:
                        gripper_angle = 0.9*gripper_angle + 0.1*angle_between_vectors(thumb, finger)
                    if prev_gripper_angle == -10000.0:
                        prev_gripper_angle = gripper_angle

                    # Estimate a 2D thumb-angle change for the wrist-y target.
                    if initial_thumb[0] == -10000.0:
                        initial_thumb = thumb

                    if target_wy == -10000.0:
                        target_wy = angle_between_vectors2(initial_thumb, thumb)
                    else:
                        target_wy = 0.9*target_wy + 0.1*angle_between_vectors2(initial_thumb, thumb)
                    # Send Action
                    if not action_queue.full():
                        action = {
                            "target_x": hand_pos[0] - initial_hand_pos[0],
                            "target_y": hand_pos[1] - initial_hand_pos[1],
                            "target_z": hand_depth - initial_hand_depth,
                            "gripper_vel": gripper_angle - prev_gripper_angle,
                            "target_wy": target_wy,
                        }
                        action_queue.put(action)
                        prev_gripper_angle = gripper_angle
                    
            cv2.imshow('SO-101 Hand Tracking', image)
            if cv2.waitKey(5) & 0xFF == 27:
                stop_event.set()

def robot_process():
    camera_action = {
        "enabled": True,
        "target_x": 0.0,
        "target_y": 0.0,
        "target_z": 0.0,
        "target_wx": 0.0,
        "target_wy": 0.0,
        "target_wz": 0.0,   #Wrist rotate
        "gripper_vel": 0.0,
    }

    while not stop_event.is_set():
        t0 = time.perf_counter()

        # Get robot observation
        robot_obs = robot.get_observation()

        #Set Robot Action
        if not action_queue.empty():
            action = action_queue.get()

            camera_action["target_x"] = action["target_x"]
            camera_action["target_z"] = -action["target_y"]
            camera_action["target_y"] = -action["target_z"]
            camera_action["gripper_vel"] = action["gripper_vel"]*0.5
            camera_action["target_wy"] = action["target_wy"]*0.01745

        # Camera-hand deltas -> EE pose -> joints.
        joint_action = hand_to_robot_joints_processor((camera_action, robot_obs))

        # Send action to robot
        _ = robot.send_action(joint_action)

        time.sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))

# Start threads
capture_thread = threading.Thread(target=frame_capture)
display_thread = threading.Thread(target=frame_display)
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


