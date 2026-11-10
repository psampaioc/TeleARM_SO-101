import unittest

from teleop.jog_joint import make_action


class JogTests(unittest.TestCase):
    def test_jog_contains_only_the_selected_joint_and_small_delta(self):
        action = make_action("wrist_roll", 5.0, -1.0, (-157.0, 163.0))
        self.assertEqual(action, {"wrist_roll.pos": 4.0})

    def test_jog_rejects_large_or_out_of_range_delta(self):
        with self.assertRaisesRegex(ValueError, "1 degree"):
            make_action("shoulder_pan", 0.0, 2.0, (-110.0, 110.0))
        with self.assertRaisesRegex(ValueError, "model limits"):
            make_action("shoulder_pan", 110.0, 1.0, (-110.0, 110.0))

    def test_gripper_uses_le_robot_zero_to_100_scale(self):
        self.assertEqual(make_action("gripper", 50.0, 1.0, (0.0, 100.0)), {"gripper.pos": 51.0})


if __name__ == "__main__":
    unittest.main()
