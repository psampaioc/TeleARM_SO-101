# Desk Arm — SO-101 hand teleoperation

One Python process inside the project container captures the webcam, detects a hand with MediaPipe, solves XYZ with LeRobot `RobotKinematics`/PlaCo, and commands the calibrated SO-101 follower through LeRobot. No leader arm, ROS 2 teleop node, TCP receiver, or MuJoCo solver is part of this runtime.

## Current state

- Compose selects the existing `desk-arm-lerobot:0.7.0-single-process` image and names its service container `so101-hand`. Recheck the live Docker state before using it; this snapshot may be stale.
- The current offline suite has 62 tests; all passed in the existing container after the temporary URDF, axis mapping and IK orientation changes. In supervised trials the arm reached `ACTIVE` and moved multiple motors. The operator later confirmed the horizontal direction, but observed vertical drift with a stationary hand; depth scale, gripper response and complete XYZ movement remain unvalidated.
- In the shared container, `pip check` passed; LeRobot 0.7.0, MediaPipe 0.10.35 and OpenCV 5.0.0 import; PyTorch is `2.15.0+cpu` (`cuda=False`). `/dev/video0`, the hand model and the existing LeRobot calibration file are present. These checks did not open the serial port or move the arm.
- Do not use a temporary Compose container or build a replacement image for routine work. Use the existing project container and image; if the service is stopped or points at the old image, inspect that exact state before switching it with the already available image and `--no-build`.
- The old project container and its `desk-arm-lerobot:jazzy-0.7.0` image have been removed. The ROS/MuJoCo source archive and user data remain in the repository.

## Active implementation plan

The [calibrated RGB hand teleoperation plan](docs/plans/2026-11-10-1200-refactor-calibrated-hand-teleop-plan.md) is being implemented. Geometry, presentation, calibration capture, a measured-profile loader and an optional metric XYZ path have been prepared offline. The measured profile is incomplete until H03–H06; the default `--live` path remains the bounded empirical controller. Do not claim metric XYZ or physical success before the measured and supervised gates. Gripper inclination remains gated on physical XYZ acceptance.
The final design has **one** hand-control behavior: PALM anchors the measured hand position to the current robot tool-center pose, and subsequent relative hand translation drives the reachable tool position. The current empirical path and `--metric-profile` switch exist only while that replacement is being measured and validated; they will be removed after acceptance. There is no requirement for camera-to-robot positioning or an absolute hand starting location.

The operator's numbered, parallel calibration and live-test tasks are in [TarefasHumanas](docs/TarefasHumanas.md); the printable [A4 checkerboard](docs/calibration-board-9x7-18mm.svg) is ready for H02.

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
- The camera window shows a compact gesture/control state, red palm reference point and loop p95 latency. Motor IDs 1–6, read/command values, hand geometry, capped XYZ delta, IK feedback and timing appear in the terminal during live mode. Numbered landmark images are saved under `data/` at key live gestures; ordinary preview does not create snapshots.
- In the default legacy live path, hand motion is anchored at the accepted PALM. Control still follows wrist landmark 0 and apparent wrist-to-middle-knuckle span 0–9 through the empirical gains `IMAGE_YZ_MAP` and `DEPTH_SCALE_M` at the top of `teleop/controller.py`; ±100 mm caps remain. The red display point is instead the mean of stable palm landmarks 0/5/9/13/17, prepared for measured control. The preview is mirrored for a selfie view; tracking coordinates are unmirrored. The operator confirmed horizontal direction; vertical drift and depth still need physical validation. The optional metric path requires a complete measured profile and is not yet accepted.
- **Requested mapping, pending implementation:** ±20 cm of *human hand* movement from PALM on each camera-control axis should act like a normalized control input. The robot tool center should use the available continuous reach in that direction from its anchored pose, bounded by LeRobot-calibrated motor and URDF joint limits, including coupled 3D movement. This is not a ±20 cm robot travel limit. The present ±100 mm software cap is a legacy temporary restriction and also remains in the prepared metric path; neither current path implements the requested workspace mapping yet. Motor 6 jaw opening remains a separate calibrated action.
- **Z refinement, also pending implementation:** X/Y keep measured hand translation. For toward/away, record apparent palm span at PALM and map its later **ratio** to a virtual ±20 cm input that enters the same 3D reach mapping. A 1.5× near ratio and reciprocal 1/1.5× far ratio are preview candidates, not measured or accepted thresholds. This does not claim the hand physically moved 20 cm in depth. The current code has not switched to this ratio mapping.
- Palm normal and a longitudinal palm axis are calculated for later tilt work; neither is yet sent to IK. FIST and tracking loss stop new Cartesian targets. A brief loss requires three PALM frames to re-anchor; 0.5 seconds without a hand sends one HOLD attempt and requires a new activation.
- The capture remains 640×480 at up to 30 FPS; the display is 800×600. MediaPipe uses CPU. The terminal keeps p50/p95 timing for camera, inference, controller/robot, display and complete loop after 30 warm-up frames using a rolling 300-frame window. Historical preview measurements were camera read about 28/33 ms, inference 12/19 ms and vision 40/46 ms p50/p95. These are pre-refactor numbers; measure the same settings again before claiming a speed improvement. Setting OpenCV's capture buffer to one frame had increased camera-read time to about 52/52 ms, so that setting was rejected.
- The IK solves all five arm joints together with LeRobot's 0.01 orientation weight to keep the gripper's anchored pointing direction; Motor 6 controls jaw opening independently. The joint displacement vector is scaled uniformly to `min(0.5 rad/s × dt, 1°)` per joint, preserving its Cartesian direction; commands are clipped one encoder tick inside the intersection of LeRobot calibration and local runtime URDF ranges. Readback up to 1.1 ticks outside is treated as integer encoder quantization and projected inside for IK. A larger excursion issues one correction eight ticks inside the saved calibration, then polls fresh readback. The earlier one-tick correction did not move Motor 3 from raw 3153; an isolated raw goal of 3140 moved it to 3144, inside the saved maximum 3148.
- For the 2026-11-10 supervised trial, the local runtime URDF widened only Motor 2 to ±1.94 rad and Motor 3 to ±1.71 rad. The saved LeRobot calibration still provides the tighter bound. This is a temporary model-limit trial, not a recalibration; see [URDF provenance](teleop/models/so101/PROVENANCE.md).
- Terminal `tool_B_read` is recalculated from each latest synchronous joint read. In metric mode it includes the measured gripper-to-tool-center offset; in legacy mode it remains the URDF gripper frame. A PALM anchor uses the calibration-clamped readback so a one-tick encoder rounding error cannot become an out-of-calibration IK seed.
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

For measured XYZ later, add `--metric-profile /workspace/data/hand_profile.json` to the live command only after H03–H06 and held-out validation pass. The [tracked example profile](teleop/hand_profile.example.json) contains `null` for unknown physical values; the loader rejects incomplete or camera-mismatched profiles. The operator must also confirm that the current focus/zoom setting and hand identity match the capture: these cannot be inferred from the camera frames. The signed `camera_delta_to_base` matrix maps unmirrored camera X right, Y down, Z away to chosen robot-base axes. The `tool_offset_gripper_m` vector is in the URDF gripper frame, in metres. Saved LeRobot servo calibration is separate and unchanged.

## Prepare measured hand data (preview only)

In the existing container, once H01/H02 are ready, measure the actual printed square side and run:

```bash
cd /workspace
python scripts/calibrate_hand_camera.py capture --camera /dev/video0 --square-mm <MEASURED_MM> --focus-zoom-mode <LOCKED_MODE>
```

Show the whole 9×7 internal-corner board; press `f` for a fit view, `h` for a reserved view and `q` to finish. The script saves each original frame and its corner/measurement metadata under ignored `data/hand_camera/`; it requires at least 12 fit and 3 held-out views. Then run `python scripts/calibrate_hand_camera.py fit`, which writes an intrinsics report only when the reprojection quality gates pass.

For H04, copy [the hand sample label template](teleop/hand_sample_label.example.json) into ignored `data/`, fill its session, split, same-hand identity, pose, focus/zoom mode and independently measured palm depth in **metres**. Set `use_for_metric=true` only for poses to be used by the depth estimator; set it to `false` for negative quality examples such as fist or edge-on views. Every metric pose needs fit and held-out captures, including a measured nonzero XY shift in held-out data. `measured_dx_m` and `measured_dy_m` are optional for pure depth holds; for shift sessions they are relative to a 0/0 baseline in the same session, positive right/down in the unmirrored camera frame. Press `s` in preview to record 30 valid hand frames for that label:

```bash
python -m teleop.hand_controller --camera /dev/video0 --sample-label /workspace/data/hand_sample_label.json
```

Repeat with a new label for each hold; assign complete sessions to either `fit` or `held_out`. The samples are local under `data/hand_samples/`. After H03/H04, run `python scripts/calibrate_hand_camera.py fit-hand --intrinsics /workspace/data/hand_camera/<INTRINSICS_FILE>.json`; it compares palm span and polygon area on reserved sessions and writes a report. A selected model is evidence for further review, not automatic live activation. The angle/visibility threshold, robot axis signs and physical gripper offset still need H04–H06 confirmation.

Before hand teleoperation, follow [the bring-up checklist](project/bring-up-checklist.md). Jog and physical XYZ remain one supervised joint/axis at a time; this repository has not yet passed those physical gates.

## Files and provenance

- `teleop/hand_controller.py`: webcam, MediaPipe, preview and CLI.
- `teleop/hand_geometry.py`: MediaPipe result to hand geometry and gestures; no serial or camera ownership.
- `teleop/camera_mapping.py`: versioned measured profile, camera XYZ and relative control-axis mapping.
- `teleop/overlay.py`: compact camera display, diagnostic frames and terminal timing/status.
- `scripts/calibrate_hand_camera.py`: one-time board and hand-scale capture/fit; never opens the robot serial port.
- `teleop/controller.py`: local gesture state, anchoring, XYZ/gripper actions and HOLD.
- `teleop/kinematics.py`: official LeRobot FK/IK API, calibration-to-degree bounds, URDF intersection and output validation.
- `teleop/lerobot_adapter.py`: only owner of the serial connection and `.pos` actions.
- `teleop/models/so101/`: official SO-101 URDF, referenced meshes, Apache-2.0 license and source hashes.
- `data/`: local calibration and MediaPipe model; not part of the image or Git.
- `upstream/`: SO-ARM100 source checkout and local print assets; not imported by the active runtime.

## Repository sources and references

These links document what is actually in the current runtime versus what was only studied. The hand-teleoperation examples below were not copied into the runtime; the MediaPipe classifier, controller and compact OpenCV overlay are project code. Numbered landmarks appear in saved diagnostics, not the normal video window. It does not use the Brosnan UI or Rerun.

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
