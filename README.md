# Desk Arm — SO-101 hand teleoperation

One Python process inside the project container captures the webcam, detects a hand with MediaPipe, solves XYZ with LeRobot `RobotKinematics`/PlaCo, and commands the calibrated SO-101 follower through LeRobot. No leader arm, ROS 2 teleop node, TCP receiver, or MuJoCo solver is part of this runtime.

## Current state

- Compose selects the existing `desk-arm-lerobot:0.7.0-single-process` image and names its service container `so101-hand`. Recheck the live Docker state before using it; this snapshot may be stale.
- The current offline suite has 62 tests; all passed in the existing container after the temporary URDF, axis mapping and IK orientation changes. The 2026-11-10 supervised trials reached `ACTIVE` and moved multiple motors, but the operator reported reversed lateral motion and inconclusive vertical/gripper response. The revised lateral sign, wrist-based tracking, depth proxy and IK orientation weight are awaiting physical validation.
- In the shared container, `pip check` passed; LeRobot 0.7.0, MediaPipe 0.10.35 and OpenCV 5.0.0 import; PyTorch is `2.15.0+cpu` (`cuda=False`). `/dev/video0`, the hand model and the existing LeRobot calibration file are present. These checks did not open the serial port or move the arm.
- Do not use a temporary Compose container or build a replacement image for routine work. Use the existing project container and image; if the service is stopped or points at the old image, inspect that exact state before switching it with the already available image and `--no-build`.
- The old project container and its `desk-arm-lerobot:jazzy-0.7.0` image have been removed. The ROS/MuJoCo source archive and user data remain in the repository.

## Active implementation plan

The current 3D hand-control scope is tracked in the [separate SO-101 3D hand teleoperation plan](docs/plans/2026-11-09-2218-feat-so101-3d-hand-teleoperation-plan.md). It preserves the November 8 single-process plan and gates gripper tilt on successful physical XYZ acceptance.

The plan appendix records its pre-fix baseline; this README records the latest implementation and verification state.

For a professor-facing explanation of the camera, landmarks, gesture rules, XYZ mapping, LeRobot path and known limitations, see [SO-101 hand teleoperation explained](docs/hand-teleop-explained.md).

## Enter the project container

In the shared project terminal, enter the existing service:

```bash
desk-arm exec lerobot bash
```

If it is already the correct container shell, continue there. From `/workspace`, preview without connecting to the robot:

```bash
cd /workspace
python -m teleop.hand_controller --camera /dev/video0
```

This opens `hand_tracking` and does not construct the robot adapter, read calibration, open serial or send servo commands. The expected camera is `/dev/video0`, 640×480 at up to 30 FPS. `SPACE` only matters in live mode and when the preview window has keyboard focus.

## Gestures and first XYZ mode

- Three consecutive valid PALM frames activate live mode. The third frame anchors the hand to the current measured arm pose and sends no movement.
- PALM/FIST uses the dimensionless ratio `distance(2,12) / distance(5,17)` from MediaPipe world landmarks: up to `0.7` is FIST, `1.1` or more is PALM, and the middle band is HAND. Three consecutive PALM frames activate live mode; FIST stops on its first classified frame. The thresholds have synthetic tests only and need validation from the live camera.
- There is no discrete PINCH label. After PALM is accepted, the angle at point 9 in triangle `4–9–12` (opposite side `4–12`) is measured continuously. The software maps 10° to gripper target 100 (closed) and 60° to target 0 (open), clamping outside that range. Activation seeds the target but sends no movement on that frame; subsequent valid frames update the gripper. The physical open/close direction is not verified.
- In live mode, the HUD and terminal identify physical Motors 1–6 using the IDs in the active LeRobot calibration (1 base, 6 gripper). Read/command values are shown separately, including sparse recovery commands; the HUD shows readback age and whether hand motion is blocked. The terminal also logs hand geometry, capped XYZ delta, IK feedback and p95 timing once per second. Annotated frames are saved in existing `data/` at startup and key gestures; preview does not create snapshots.
- Hand motion is anchored at the accepted PALM. The image position follows wrist landmark 0, so finger opening does not shift the XY reference. Image horizontal maps to base Y with gain +0.50 m per normalized image width, image vertical maps to base Z with gain −0.50 m per normalized image height, and apparent wrist-to-middle-knuckle distance (landmarks 0 and 9) maps to base X with gain 0.40 m per relative size change. Raw and ±100 mm capped deltas and targets appear separately in the HUD. The camera preview and drawn landmarks are mirrored for a selfie view; tracking coordinates and robot motion are unchanged. The operator confirmed the horizontal direction; vertical motion still drifts and the other gains need physical confirmation. Palm size is a relative proxy, not metric depth; the earlier palm-width proxy (5–17) saturated depth when the hand rotated. Without a measured webcam/hand scale, 10 cm of hand travel cannot yet be guaranteed to mean 10 cm of gripper travel.
- A palm normal from landmarks `[0,5,17]` is displayed for later tilt work; it is not yet anchored/compared or sent to IK. FIST and tracking loss stop new Cartesian targets. A brief loss requires three PALM frames to re-anchor; 0.5 seconds without a hand sends one HOLD attempt and requires a new activation.
- The capture remains 640×480 at up to 30 FPS; the display is 800×600. MediaPipe uses CPU. The HUD reports p50/p95 for camera, inference, controller/robot, display, and complete loop after 30 warm-up frames using a rolling 300-frame window. In the preview, camera read was about 28/33 ms and inference 12/19 ms p50/p95; their vision path measured 40/46 ms. The 40 ms p95 target is not met. Setting OpenCV's capture buffer to one frame increased camera-read time to about 52/52 ms, so that setting was rejected.
- The IK solves all five arm joints together with LeRobot's 0.01 orientation weight to keep the gripper's anchored pointing direction; Motor 6 controls jaw opening independently. The joint displacement vector is scaled uniformly to `min(0.5 rad/s × dt, 1°)` per joint, preserving its Cartesian direction; commands are clipped one encoder tick inside the intersection of LeRobot calibration and local runtime URDF ranges. Readback up to 1.1 ticks outside is treated as integer encoder quantization and projected inside for IK. A larger excursion issues one correction eight ticks inside the saved calibration, then polls fresh readback. The earlier one-tick correction did not move Motor 3 from raw 3153; an isolated raw goal of 3140 moved it to 3144, inside the saved maximum 3148.
- For the 2026-11-10 supervised trial, the local runtime URDF widened only Motor 2 to ±1.94 rad and Motor 3 to ±1.71 rad. The saved LeRobot calibration still provides the tighter bound. This is a temporary model-limit trial, not a recalibration; see [URDF provenance](teleop/models/so101/PROVENANCE.md).
- The `XYZ measured` HUD value is recalculated from each latest synchronous joint read. A PALM anchor uses the calibration-clamped readback so a one-tick encoder rounding error cannot become an out-of-calibration IK seed.
- During calibrated-boundary recovery, FIST and `SPACE` do not cancel the already-issued inward command. If the servo does not progress, the display says `BLOCKED` and serial polling continues without blindly resending the same correction. A fresh in-range read returns to `READY` and requires another PALM. Serial read errors also remain visible and are retried. `q` exits with a warning and disconnects, which disables motor torque; support the arm first. IK recovery stops writing after no progress but continues polling.
- HOLD reads the current positions and sends one hold action. It is not an emergency power cut. A USB failure or failed write means the physical stop is unconfirmed.

## Supervised live mode

Use this only when an operator is beside the arm, has supported it, cleared the workspace, and can reach the power cut-off. Run it from the correct project container:

```bash
python -m teleop.hand_controller \
  --camera /dev/video0 \
  --port /dev/ttyACM0 \
  --robot-id teleArm_follower \
  --live
```

`--live` is the only switch that constructs the LeRobot adapter and opens the follower. The app does not run motor setup or calibration. It uses the existing calibration at `data/lerobot-home/calibration/robots/so_follower/teleArm_follower.json`. Support the arm when exiting because LeRobot disconnect behavior can change motor torque.

Before hand teleoperation, follow [the bring-up checklist](project/bring-up-checklist.md). Jog and physical XYZ remain one supervised joint/axis at a time; this repository has not yet passed those physical gates.

## Files and provenance

- `teleop/hand_controller.py`: webcam, MediaPipe, preview and CLI.
- `teleop/controller.py`: local gesture state, anchoring, XYZ/gripper actions and HOLD.
- `teleop/kinematics.py`: official LeRobot FK/IK API, calibration-to-degree bounds, URDF intersection and output validation.
- `teleop/lerobot_adapter.py`: only owner of the serial connection and `.pos` actions.
- `teleop/models/so101/`: official SO-101 URDF, referenced meshes, Apache-2.0 license and source hashes.
- `data/`: local calibration and MediaPipe model; not part of the image or Git.
- `upstream/`: SO-ARM100 source checkout and local print assets; not imported by the active runtime.

## Repository sources and references

These links document what is actually in the current runtime versus what was only studied. The hand-teleoperation examples below were not copied into the runtime; the current MediaPipe classifier, controller, and overlay are project code. In particular, the current `hand_tracking` window is OpenCV with 21 green landmark dots and status text; it does not use the Brosnan UI or Rerun.

| Repository | Role in this project | Current use |
|---|---|---|
| [huggingface/lerobot](https://github.com/huggingface/lerobot/tree/v0.7.0) | Main hardware and kinematics library | Installed as LeRobot 0.7.0; runtime uses `SO101Follower`, Feetech support, calibrated `.pos` conversion and `RobotKinematics`/PlaCo. No project-local LeRobot fork. |
| [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/7629d2ad9853d10fb903093a33ef6114099d97e5) | SO-101 mechanical model | URDF and meshes in `teleop/models/so101/` are from this pinned commit, Apache-2.0. The same upstream contains the preserved legacy MuJoCo/XML reference. |
| [psampaioc/SO-ARM100](https://github.com/psampaioc/SO-ARM100) | User fork that is the `origin` remote of the nested `upstream/SO-ARM100` checkout | Retained for local print assets and source checkout; runtime URDF provenance remains the pinned official TheRobotStudio commit above. |
| [google-ai-edge/mediapipe](https://github.com/google-ai-edge/mediapipe) | Hand landmark inference | Installed Python dependency; the `.task` model is downloaded from Google's MediaPipe model storage into ignored `data/`. No project code copied. |
| [AFK-Kireeti/Lerobot-SO101](https://github.com/AFK-Kireeti/Lerobot-SO101/tree/32c561abd05354fb51aae2bdf0e74df8b0df9ff2) | Studied three alternatives: ROS/webcam, Pi with IMU, and ArUco | Reference only; no runtime code copied. |
| [ABMI-software/hand_controlLerobot](https://github.com/ABMI-software/hand_controlLerobot/tree/52c7f0d926d0f66107ae91b94ceb900766f3009d) | Studied hand geometry and re-anchoring | Reference only; no runtime code copied. |
| [sruthihsr/SO101-Gesture-Teleoperation](https://github.com/sruthihsr/SO101-Gesture-Teleoperation/tree/ac5468b603a65c556634893ef8564873206734c4) | Studied gesture state, Jacobian and TCP architecture | Reference only; no runtime code copied. |
| [BrosnanYuen/hand_pose_lerobot_teleop](https://github.com/BrosnanYuen/hand_pose_lerobot_teleop/tree/425a4fba62b6c7c07fc3968a18d00da99bc193f2) | Compared the hand-pose LeRobot pipeline and UI | Reference only; not integrated. The current green-dot UI and classifier are not its pipeline. |
| [ros2/ros2_documentation](https://github.com/ros2/ros2_documentation) | Earlier ROS 2 architecture research | Historical reference only; ROS 2 is not installed in the active image. |
| [NVIDIA/Isaac-GR00T](https://github.com/NVIDIA/Isaac-GR00T) | Earlier ACT/VLA research | Historical reference only; no ACT/VLA dependency or model in this runtime. |

The exact pins and license notes for copied model assets and the examined hand-teleoperation repositories are in [source and license details](teleop/SOURCES.md). The simple host has an AMD Radeon 780M; the installed PyTorch build is CPU-only and MediaPipe logs XNNPACK CPU inference, so `gpus: all` does not mean hand inference runs on a GPU.

The ROS/MuJoCo files are preserved as an archive; they are not required by the current runtime. No unrelated Docker containers, images, volumes, or build cache were removed.
