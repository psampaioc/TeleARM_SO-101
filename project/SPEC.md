# Project specification - SO-101 follower

> **PROJECT ADDITION.** This file is not upstream documentation. It records only our decisions and requirements. The upstream baseline is in `../upstream/`.

## Baseline

- Upstream project: SO-101.
- Variant: follower only.
- Source commit: see `../upstream/source-manifest.md`.
- Mechanical files: use the upstream SO-101 follower STL/STEP without redesign for the first build.

## Our current scope

- Operate the assembled SO-101 follower; there is no leader arm.
- Use the upstream follower kinematics and assembly.
- Keep the installed motor/controller power configuration and the existing LeRobot calibration.
- Control the follower by USB from the workstation.
- Use the workstation webcam for experimental hand teleoperation after the stock follower moves safely. Defer the ESP32-mounted camera, Wi-Fi/Bluetooth bridge, OCR, SLM and ROS 2 integration.

## Explicit future features

- Wireless workstation connection through ESP32 UART bridge.
- ESP32 camera, potentially eye-in-hand, after mechanical and electrical validation.
- Pre-programmed skills and validated natural-language intents.
- OCR and object detection.

## Initial assembly acceptance gates (historical)

1. All six motors and the controller match the selected voltage and bus protocol.
2. The upstream assembly passes visual and mechanical inspection.
3. Power polarity, current limit and emergency cut-off are verified before motion.
4. Motor IDs and calibration complete without collision.
5. Payload and repeatability are measured rather than inferred from stall torque.
