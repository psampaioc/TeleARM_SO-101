import argparse

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_ROBOT_ID = "teleArm_follower"


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate an SO-101 follower arm.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Motor controller serial port.")
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID, help="LeRobot name for this follower.")
    args = parser.parse_args()

    follower = SO101Follower(SO101FollowerConfig(port=args.port, id=args.robot_id))
    follower.connect(calibrate=False)
    try:
        follower.calibrate()
    finally:
        follower.disconnect()


if __name__ == "__main__":
    main()
