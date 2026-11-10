import fcntl
import json
import logging
import math
import os
import re
import stat
from pathlib import Path


logger = logging.getLogger("desk_arm.adapter")
JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class RobotAdapter:
    def __init__(self, serial_port, robot_id, live=False, lerobot_home=None, follower_factory=None):
        if not re.fullmatch(r"[A-Za-z0-9_-]+", robot_id):
            raise ValueError("robot ID may contain only letters, numbers, _ and -")
        self.serial_port = str(serial_port)
        self.robot_id = robot_id
        self.live = bool(live)
        self.lerobot_home = Path(lerobot_home or os.environ.get("HF_LEROBOT_HOME", "/workspace/data/lerobot-home"))
        self.follower_factory = follower_factory or self._make_follower
        self.follower = None
        self.lock_file = None
        self.connected = False
        self.motor_ids = {}

    def _make_follower(self, port, robot_id):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        return SO101Follower(SO101FollowerConfig(port=port, id=robot_id, use_degrees=True))

    @property
    def calibration_file(self) -> Path:
        return self.lerobot_home / "calibration/robots/so_follower" / f"{self.robot_id}.json"

    def _acquire_lock(self):
        lock_path = self.lerobot_home.parent / "teleop-serial.lock"
        if not lock_path.parent.is_dir():
            raise FileNotFoundError(f"data directory not found for serial lock: {lock_path.parent}")
        self.lock_file = lock_path.open("a+")
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.lock_file.close()
            self.lock_file = None
            raise RuntimeError(f"serial lock is busy for {self.serial_port}: {lock_path}") from exc

    def _release_lock(self):
        if self.lock_file is not None:
            fcntl.flock(self.lock_file, fcntl.LOCK_UN)
            self.lock_file.close()
            self.lock_file = None

    def connect(self):
        if self.connected:
            raise RuntimeError("robot adapter already connected")
        if not self.calibration_file.is_file():
            raise FileNotFoundError(f"calibration not found for robot ID {self.robot_id}: {self.calibration_file}")
        calibration = json.loads(self.calibration_file.read_text())
        ids = [calibration.get(name, {}).get("id") for name in JOINT_NAMES]
        if any(type(value) is not int for value in ids) or ids != list(range(1, 7)):
            raise ValueError("LeRobot calibration motor IDs must map base through gripper to 1-6")
        self.motor_ids = dict(zip(JOINT_NAMES, ids))
        port = Path(self.serial_port)
        try:
            mode = port.stat().st_mode
        except OSError as exc:
            raise FileNotFoundError(f"serial port is unavailable: {self.serial_port}") from exc
        if not stat.S_ISCHR(mode):
            raise ValueError(f"serial path is not a character device: {self.serial_port}")

        self._acquire_lock()
        try:
            self.follower = self.follower_factory(self.serial_port, self.robot_id)
            self.follower.connect(calibrate=False)
            self.connected = True
            if not self.follower.is_calibrated:
                raise RuntimeError(f"LeRobot reports robot {self.robot_id} is not calibrated")
            self.read_positions()
            logger.info("connected to calibrated follower %s on %s", self.robot_id, self.serial_port)
        except Exception:
            if self.follower is not None:
                try:
                    self.follower.disconnect()
                except Exception:
                    pass
            self.connected = False
            self._release_lock()
            raise

    def read_positions(self) -> dict[str, float]:
        if not self.connected or self.follower is None:
            raise RuntimeError("robot adapter is not connected")
        observation = self.follower.get_observation()
        positions = {}
        for joint in JOINT_NAMES:
            key = f"{joint}.pos"
            try:
                value = float(observation[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(f"LeRobot observation is missing a numeric {key}") from exc
            if not math.isfinite(value):
                raise RuntimeError(f"LeRobot observation has non-finite {key}")
            positions[key] = value
        return positions

    def send_action(self, action: dict[str, float]):
        if not self.connected or self.follower is None:
            raise RuntimeError("robot adapter is not connected")
        if not action or any(key not in {f"{joint}.pos" for joint in JOINT_NAMES} for key in action):
            raise ValueError("action must contain one or more named SO-101 .pos values")
        checked = {key: float(value) for key, value in action.items()}
        if not all(math.isfinite(value) for value in checked.values()):
            raise ValueError("action contains non-finite positions")
        if "gripper.pos" in checked and not 0 <= checked["gripper.pos"] <= 100:
            raise ValueError("gripper action must be in the LeRobot 0-100 range")
        if not self.live:
            logger.info("DRY RUN; no Goal_Position sent: %s", checked)
            return checked
        return self.follower.send_action(checked)

    def disconnect(self):
        try:
            if self.connected and self.follower is not None:
                self.follower.disconnect()
        finally:
            self.connected = False
            self._release_lock()
