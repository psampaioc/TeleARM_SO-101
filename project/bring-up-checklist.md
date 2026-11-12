# SO-101 hand teleoperation bring-up

Status: supervised trials reached ACTIVE and moved the arm, but full XYZ accuracy has not passed physical acceptance. The follower calibration and MediaPipe model remain in `data/` and must not be edited or recalibrated as part of this checklist.

## 1. Enter and identify the existing container

Use the shared project terminal and the existing service (`desk-arm exec lerobot bash`). The persistent service container is named `so101-hand` and uses `desk-arm-lerobot:0.7.0-single-process`; verify current state before proceeding. Do not run a temporary `docker compose run` container or build another image for preview. If stopped or mismatched, do not run project Python on the host: inspect the exact service and reuse the already-present single-process image with `--no-build` only when the switch is needed. Do not prune Docker data.

## 2. Preview without connecting to the arm

From `/workspace` inside the correct container:

```bash
cd /workspace
python -m teleop.hand_controller --camera /dev/video0
```

Confirm the `hand_tracking` window shows PALM for an open hand, FIST for a closed hand, and HAND for intermediate poses. There is no discrete PINCH label: the 4–9–12 jaw angle is measured continuously after PALM activation. PALM needs 3/3 consecutive frames to activate live mode; FIST requests HOLD immediately. With no hand it must say `NO HAND`. Preview must not open `/dev/ttyACM0`, read calibration, connect LeRobot, or send servo commands. Press `q` to close it.

Correct room lighting, camera framing, and the `/dev/video0` selection here. The current monocular mapping is relative; it does not estimate absolute depth.

## 3. Physical preconditions before any live command

- The operator is physically at the arm; no remote live command.
- Base fixed, arm supported, no payload, motion path clear, cables slack, and power cut-off within reach.
- Confirm the power supply and controller are connected correctly. Do not change servo wiring while powered; switch off both the supply and USB before moving servo connectors.
- Confirm no other process owns the follower serial port.
- Verify the existing LeRobot calibration for ID `teleArm_follower` loads. Do not call `setup_motors`, `lerobot-setup-motors`, or `lerobot-calibrate` here.
- Begin with the separate one-joint jog, one joint and at most 1° per run. From an interactive shell in `/workspace`, run `python -m teleop.jog_joint --joint shoulder_pan --delta 1 --live`. It connects first, prints the measured and target positions, then waits for Enter before the one-joint move. Observe direction, readback, cable clearance, and whether the other joints remain still; press Enter at the return prompt only after visually confirming the arm is clear. Ctrl+C cancels the pending action and leaves the joint where it is.
- Check gripper direction with a small supervised action before relying on the hand gesture labels.

## 4. Live hand tracking, first session

Only after the preconditions and joint/gripper checks pass:

```bash
python -m teleop.hand_controller \
  --camera /dev/video0 \
  --port /dev/ttyACM0 \
  --robot-id teleArm_follower \
  --live
```

Read the on-screen `LIVE`, port, and robot ID before presenting a gesture. PALM must be 3/3 before the session becomes ACTIVE; that frame only anchors and must not move the arm. Start with a small horizontal hand displacement, then isolate vertical and depth movements. The current software permits XYZ and caps each axis at ±10 cm from the anchor; this is a command bound, not evidence of accurate physical travel. The operator confirmed horizontal direction, while vertical drift and depth/gripper response still require physical verification. FIST or SPACE requests a one-time HOLD; `q` exits. HOLD is not a power cut, and disconnect may change torque, so keep supporting the arm.

The software keeps commands one encoder tick inside the calibration/URDF intersection. Readback within 1.1 ticks is projected to the boundary for IK; a larger excursion commands the inner boundary and waits for readback before rearming. This tolerance is offline-tested only. If serial/readback/write fails, assume the physical stop is unconfirmed and use the power cut-off.

## Deferred acceptance

The following have not been proven by offline tests: live camera/hand visibility, servo direction, each joint's response, gripper direction, XYZ axis/sign/scale, repeated boundary behavior, or safe torque after disconnect. Hand inclination remains disabled until XYZ passes supervised acceptance; when added, it must share one end-effector pose and one IK solve for all five arm joints.
