import argparse

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_ROBOT_ID = "teleArm_follower"


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure the SO-101 follower motor IDs.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Motor controller serial port.")
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID, help="LeRobot name for this follower.")
    args = parser.parse_args()

    config = SO101FollowerConfig(port=args.port, id=args.robot_id)
    follower = SO101Follower(config)
    follower.setup_motors()


if __name__ == "__main__":
    main()
