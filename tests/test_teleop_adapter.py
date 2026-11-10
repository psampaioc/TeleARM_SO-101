import tempfile
import unittest
import json
from pathlib import Path

from teleop.lerobot_adapter import RobotAdapter


class FakeFollower:
    is_calibrated = True

    def __init__(self):
        self.actions = []
        self.connected = False

    def connect(self, calibrate=True):
        self.connected = True
        self.calibrate_called = calibrate

    def get_observation(self):
        return {
            "shoulder_pan.pos": 1.0,
            "shoulder_lift.pos": -2.0,
            "elbow_flex.pos": 3.0,
            "wrist_flex.pos": -4.0,
            "wrist_roll.pos": 5.0,
            "gripper.pos": 50.0,
        }

    def send_action(self, action):
        self.actions.append(action.copy())
        return action

    def disconnect(self):
        self.connected = False

    def calibrate(self):
        raise AssertionError("adapter must never recalibrate")


class AdapterTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.home = Path(self.tmp.name) / "lerobot-home"
        calibration = self.home / "calibration/robots/so_follower"
        calibration.mkdir(parents=True)
        (calibration / "teleArm_follower.json").write_text(json.dumps({
            name: {"id": index} for index, name in enumerate((
                "shoulder_pan", "shoulder_lift", "elbow_flex",
                "wrist_flex", "wrist_roll", "gripper",
            ), 1)
        }))
        self.follower = FakeFollower()

    def tearDown(self):
        self.tmp.cleanup()

    def adapter(self, live=False):
        return RobotAdapter(
            "/dev/null", "teleArm_follower", live=live,
            lerobot_home=self.home, follower_factory=lambda *_: self.follower,
        )

    def test_connect_never_recalibrates_and_dry_run_never_writes(self):
        adapter = self.adapter()
        adapter.connect()
        self.assertEqual(adapter.motor_ids["elbow_flex"], 3)
        self.assertFalse(self.follower.calibrate_called)
        self.assertEqual(adapter.read_positions()["gripper.pos"], 50.0)
        adapter.send_action({"shoulder_pan.pos": 2.0})
        self.assertEqual(self.follower.actions, [])
        adapter.disconnect()

    def test_duplicate_calibration_motor_id_is_rejected(self):
        path = self.home / "calibration/robots/so_follower/teleArm_follower.json"
        data = json.loads(path.read_text())
        data["elbow_flex"]["id"] = 2
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "motor IDs"):
            self.adapter().connect()

    def test_live_action_reaches_follower(self):
        adapter = self.adapter(live=True)
        adapter.connect()
        action = {"wrist_roll.pos": 1.0}
        adapter.send_action(action)
        self.assertEqual(self.follower.actions, [action])
        adapter.disconnect()

    def test_nonblocking_lock_rejects_second_owner(self):
        first, second = self.adapter(), self.adapter()
        first.connect()
        with self.assertRaisesRegex(RuntimeError, "lock"):
            second.connect()
        first.disconnect()


if __name__ == "__main__":
    unittest.main()
