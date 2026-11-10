import argparse
import logging
import math
import time

from teleop.lerobot_adapter import JOINT_NAMES, RobotAdapter


def make_action(joint, current, delta, limits):
    if joint not in JOINT_NAMES:
        raise ValueError(f"unknown joint: {joint}")
    current, delta = float(current), float(delta)
    if not math.isfinite(current) or not math.isfinite(delta):
        raise ValueError("current position and delta must be finite")
    if abs(delta) > 1.0:
        raise ValueError("delta must not exceed 1 degree or gripper point")
    lower, upper = limits
    target = current + delta
    if not lower <= current <= upper or not lower <= target <= upper:
        raise ValueError(f"{joint} target exceeds model limits [{lower}, {upper}]")
    return {f"{joint}.pos": target}


def wait_for_target(adapter, joint, target, timeout=3.0):
    key = f"{joint}.pos"
    tolerance = 1.0 if joint == "gripper" else 0.5
    end = time.monotonic() + timeout
    position = None
    while time.monotonic() < end:
        position = adapter.read_positions()[key]
        if abs(position - target) <= tolerance:
            return position, True
        time.sleep(0.1)
    return position, False


def main():
    parser = argparse.ArgumentParser(description="Move exactly one SO-101 joint by a small amount")
    parser.add_argument("--joint", choices=JOINT_NAMES, required=True)
    parser.add_argument("--delta", type=float, required=True, help="degrees; gripper uses LeRobot points 0-100")
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--robot-id", default="teleArm_follower")
    parser.add_argument("--live", action="store_true", help="send one-joint Goal_Position after confirmation")
    parser.add_argument("--dry-run", action="store_true", help="explicitly do not send Goal_Position")
    args = parser.parse_args()
    if args.live and args.dry_run:
        parser.error("choose only one of --live and --dry-run")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")
    from teleop.kinematics import SO101Kinematics

    adapter = RobotAdapter(args.serial_port, args.robot_id, live=args.live)
    kinematics = SO101Kinematics(calibration_path=adapter.calibration_file)
    limits = {
        name: (float(low), float(high))
        for name, low, high in zip(kinematics.joint_names, kinematics.lower, kinematics.upper)
    }
    limits["gripper"] = (0.0, 100.0)
    try:
        adapter.connect()
        initial = adapter.read_positions()[f"{args.joint}.pos"]
        action = make_action(args.joint, initial, args.delta, limits[args.joint])
        print(f"Selected joint: {args.joint}; measured={initial:.2f}; target={next(iter(action.values())):.2f}")
        if not args.live:
            print("DRY RUN: no Goal_Position was sent. A serial connection was opened for readback.")
            return

        input("Support the arm, clear its path, and press Enter for this single-joint move; Ctrl+C cancels: ")
        adapter.send_action(action)
        reached, stable = wait_for_target(adapter, args.joint, next(iter(action.values())))
        print(f"Readback={reached}; within tolerance={stable}")
        input(f"Press Enter to return only {args.joint} to {initial:.2f}; Ctrl+C leaves it in place: ")
        adapter.send_action({f"{args.joint}.pos": initial})
        returned, stable = wait_for_target(adapter, args.joint, initial)
        print(f"Return readback={returned}; within tolerance={stable}")
    finally:
        adapter.disconnect()


if __name__ == "__main__":
    main()
