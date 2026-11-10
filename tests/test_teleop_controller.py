import unittest
from types import SimpleNamespace

import numpy as np

from teleop.controller import RECOVERY_TIMEOUT_S, TeleopController
from teleop.kinematics import ENCODER_STEP_DEG, IKResult, JOINT_NAMES, READBACK_TOLERANCE_DEG


class FakeKinematics:
    last_solve_count = 0
    lower = np.full(5, -10.0)
    upper = np.full(5, 10.0)
    ik_lower = np.full(5, -8.0)
    ik_upper = np.full(5, 8.0)

    def clamp(self, q):
        q = np.asarray(q, dtype=float)
        bounded = np.clip(q, self.lower, self.upper)
        outside = np.any(q < self.lower - READBACK_TOLERANCE_DEG) or np.any(q > self.upper + READBACK_TOLERANCE_DEG)
        return bounded, bool(outside)

    def forward_pose(self, q):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(q[:3], dtype=float) * 0.001
        return pose

    def forward_position(self, q):
        return self.forward_pose(q)[:3, 3]

    def outside_ik(self, q):
        q = np.asarray(q, dtype=float)
        return bool(np.any(q < self.ik_lower - READBACK_TOLERANCE_DEG)
                    or np.any(q > self.ik_upper + READBACK_TOLERANCE_DEG))

    def solve_position(self, q, target, max_step_deg):
        self.last_solve_count = getattr(self, "last_solve_count", 0) + 1
        q = np.asarray(q, dtype=float)
        before = float(np.linalg.norm(target[:3, 3] - self.forward_position(q)))
        goal = q.copy()
        goal[:3] = target[:3, 3] * 1000
        result = q + np.clip(goal - q, -max_step_deg, max_step_deg)
        result = np.clip(result, self.ik_lower, self.ik_upper)
        residual = float(np.linalg.norm(target[:3, 3] - self.forward_position(result)))
        at_limit = bool(np.any(np.isclose(result, self.lower)) or np.any(np.isclose(result, self.upper)))
        ok = residual <= 0.005 or residual < before - 1e-7
        reason = "LIMIT" if at_limit and residual > 0.005 else "OK"
        return IKResult(ok, result, residual, reason if ok else "NO_PROGRESS", at_limit)


class FakeAdapter:
    def __init__(self):
        self.positions = {f"{name}.pos": 0.0 for name in JOINT_NAMES}
        self.positions["gripper.pos"] = 50.0
        self.actions = []
        self.apply_actions = True
        self.read_count = 0

    def read_positions(self):
        self.read_count += 1
        return self.positions.copy()

    def send_action(self, action):
        self.actions.append(action.copy())
        if self.apply_actions:
            self.positions.update(action)
        return action


def sample(hx=0.5, hy=0.5, size=0.2, gesture="PALM", jaw_angle_deg=35.0, palm_ratio=1.4):
    return SimpleNamespace(hx=hx, hy=hy, size=size, gesture=gesture,
                           jaw_angle_deg=jaw_angle_deg, palm_ratio=palm_ratio,
                           palm_normal=np.array([0.0, 0.0, 1.0]))


def activate(controller):
    for frame, now in enumerate((0.0, 0.01, 0.02)):
        controller.update(sample(), now)


class LocalControllerTests(unittest.TestCase):
    def setUp(self):
        self.adapter = FakeAdapter()
        self.kin = FakeKinematics()
        self.controller = TeleopController(self.adapter, self.kin)

    def test_three_palms_anchor_without_sending_an_action(self):
        self.controller.update(sample(), 0.0)
        self.controller.update(sample(), 0.01)
        self.assertEqual(self.controller.state, "READY")
        self.assertEqual(self.controller.palm_count, 2)
        self.controller.update(sample(), 0.02)
        self.assertEqual(self.controller.state, "ACTIVE")
        self.assertEqual(self.controller.gripper_target, 50.0)
        self.assertEqual(self.controller.last_measured_gripper, 50.0)
        self.assertEqual(self.adapter.actions, [])
        np.testing.assert_allclose(self.controller.arm_anchor, np.eye(4))

    def test_initial_readback_outside_calibration_recovers_to_boundary(self):
        self.adapter.positions["shoulder_lift.pos"] = -10.2
        activate(self.controller)
        self.assertEqual(self.controller.state, "LIMIT")
        self.assertAlmostEqual(self.adapter.positions["shoulder_lift.pos"], -10.0 + 7 * ENCODER_STEP_DEG)
        self.assertIsNone(self.controller.arm_anchor)
        self.assertEqual(self.controller.feedback, "LIMIT: calibrated boundary recovery")

    def test_motion_keeps_advancing_until_readback_reaches_deadband(self):
        activate(self.controller)
        self.controller.update(sample(hx=0.56), 0.053)
        first = self.adapter.positions["shoulder_lift.pos"]
        self.controller.update(sample(hx=0.56), 0.086)
        second = self.adapter.positions["shoulder_lift.pos"]
        self.assertGreater(first, 0.0)
        self.assertGreater(second, first)
        self.assertLessEqual(second - first, 1.0)

    def test_image_yz_matrix_and_palm_size_drive_all_three_cartesian_axes(self):
        self.controller = TeleopController(self.adapter, self.kin,
                                           image_yz_map=np.array([[0.1, 0.05], [-0.02, -0.1]]))
        activate(self.controller)
        anchor = self.controller.arm_anchor[:3, 3].copy()
        self.controller.update(sample(hx=0.6, hy=0.4, size=0.22), 0.053)
        target = self.controller.target_pose[:3, 3]
        np.testing.assert_allclose(target - anchor, [0.04, 0.005, 0.008], atol=1e-8)

    def test_raw_target_and_axis_caps_are_reported_separately(self):
        activate(self.controller)
        anchor = self.controller.arm_anchor[:3, 3].copy()
        self.controller.update(sample(hx=0.8), 0.053)
        np.testing.assert_allclose(self.controller.requested_target[:3, 3] - anchor, [0.0, 0.15, 0.0])
        np.testing.assert_allclose(self.controller.target_pose[:3, 3] - anchor, [0.0, 0.10, 0.0])
        self.assertEqual(self.controller.saturated_axes, ("Y",))

    def test_fist_stops_immediately_and_holds_the_measured_state(self):
        activate(self.controller)
        self.controller.update(sample(gesture="FIST"), 0.03)
        self.assertEqual(self.controller.state, "READY")
        self.assertNotIn("FIST", self.controller.feedback)
        self.assertEqual(len(self.adapter.actions), 1)
        self.assertEqual(self.adapter.actions[0]["shoulder_pan.pos"], 0.0)

    def test_fist_in_ready_does_not_activate(self):
        self.controller.update(sample(gesture="FIST"), 0.0)
        self.assertEqual(self.controller.state, "READY")
        self.assertEqual(self.adapter.actions, [])

    def test_unclassified_sample_clears_anchor_without_moving(self):
        activate(self.controller)
        self.controller.update(sample(hx=0.8, gesture=None), 0.053)
        self.assertIsNone(self.controller.hand_anchor)
        self.assertEqual(self.adapter.actions, [])

    def test_palm_counter_resets_after_invalid_frame(self):
        self.controller.update(sample(), 0.0)
        self.controller.update(sample(), 0.01)
        self.controller.update(None, 0.02)
        self.assertEqual(self.controller.palm_count, 0)
        self.controller.update(sample(), 0.03)
        self.assertEqual(self.controller.state, "READY")

    def test_crocodile_angle_maps_continuously_to_gripper_position(self):
        activate(self.controller)
        self.controller.update(sample(jaw_angle_deg=10), 0.03)
        self.assertEqual(self.adapter.positions["gripper.pos"], 100.0)
        self.controller.update(sample(jaw_angle_deg=35), 0.04)
        self.assertEqual(self.adapter.positions["gripper.pos"], 50.0)
        self.controller.update(sample(jaw_angle_deg=60), 0.05)
        self.assertEqual(self.adapter.positions["gripper.pos"], 0.0)
        self.assertTrue(all(set(action) == {"gripper.pos"} for action in self.adapter.actions))

    def test_jaw_keeps_updating_when_arm_ik_reaches_a_boundary(self):
        class AtLimitKinematics(FakeKinematics):
            def solve_position(self, q, target, max_step_deg):
                return IKResult(False, np.asarray(q), 0.01, "LIMIT", True)

        self.controller = TeleopController(self.adapter, AtLimitKinematics())
        activate(self.controller)
        self.controller.update(sample(hx=0.7, jaw_angle_deg=10), 0.053)
        self.assertEqual(self.controller.state, "ACTIVE")
        self.assertEqual(self.adapter.actions[-1], {"gripper.pos": 100.0})

    def test_brief_tracking_loss_reanchors_without_a_position_jump(self):
        activate(self.controller)
        self.controller.update(None, 0.03)
        self.controller.update(sample(hx=0.8), 0.04)
        self.assertIsNone(self.controller.hand_anchor)
        self.controller.update(sample(hx=0.8), 0.05)
        self.assertIsNone(self.controller.hand_anchor)
        self.controller.update(sample(hx=0.8), 0.06)
        self.assertEqual(self.controller.state, "ACTIVE")
        self.assertEqual(self.adapter.actions, [])
        self.controller.update(sample(hx=0.8), 0.07)
        self.assertEqual(self.adapter.actions, [])
        self.controller.update(sample(hx=0.81), 0.08)
        self.assertEqual(len(self.adapter.actions), 1)

    def test_half_second_tracking_loss_holds_once_and_requires_new_palm(self):
        activate(self.controller)
        self.controller.update(None, 0.03)
        self.controller.update(None, 0.53)
        self.assertEqual(self.controller.state, "READY")
        self.assertEqual(len(self.adapter.actions), 1)
        self.controller.update(sample(), 0.54)
        self.assertEqual(self.controller.state, "READY")

    def test_outside_calibration_readback_is_corrected_to_safe_boundary(self):
        self.adapter.positions["elbow_flex.pos"] = 10.2
        activate(self.controller)
        self.assertEqual(self.controller.state, "LIMIT")
        self.assertAlmostEqual(self.adapter.positions["elbow_flex.pos"], 10.0 - 7 * ENCODER_STEP_DEG)
        self.assertEqual(len(self.adapter.actions), 1)
        self.controller.update(sample(), 0.2)
        self.assertEqual(self.controller.state, "READY")

    def test_fist_and_space_do_not_cancel_calibration_recovery(self):
        self.adapter.positions["elbow_flex.pos"] = 10.2
        activate(self.controller)
        self.controller.update(sample(gesture="FIST"), 0.03)
        self.assertEqual((self.controller.state, self.controller.limit_kind), ("LIMIT", "CALIBRATION"))
        self.controller.hold("SPACE")
        self.assertEqual((self.controller.state, self.controller.limit_kind), ("LIMIT", "CALIBRATION"))
        self.controller.update(sample(gesture="HAND"), 0.2)
        self.assertEqual(self.controller.state, "READY")

    def test_ik_recovery_requires_distance_to_boundary_to_decrease(self):
        self.adapter.apply_actions = False
        q = np.zeros(5)
        q[0] = 9.0
        self.controller._begin_ik_recovery(q, 0.0)
        self.adapter.positions["shoulder_pan.pos"] = 9.5
        self.controller.update(sample(gesture="HAND"), 0.11)
        self.assertEqual(self.controller.last_progress_at, 0.0)
        self.assertEqual(self.controller.state, "LIMIT")
        self.controller.update(sample(gesture="HAND"), RECOVERY_TIMEOUT_S + 0.2)
        self.assertEqual(self.controller.state, "LIMIT")
        self.assertIn("BLOCKED", self.controller.feedback)
        writes = len(self.adapter.actions)
        self.controller.update(sample(gesture="HAND"), RECOVERY_TIMEOUT_S + 0.4)
        self.assertEqual(len(self.adapter.actions), writes)

    def test_stalled_calibration_keeps_reading_and_rearms_after_valid_feedback(self):
        self.adapter.apply_actions = False
        self.adapter.positions["elbow_flex.pos"] = 10.2
        activate(self.controller)
        self.controller.update(sample(), RECOVERY_TIMEOUT_S + 0.2)
        self.assertEqual(self.controller.state, "LIMIT")
        self.assertIn("BLOCKED", self.controller.feedback)
        reads = self.adapter.read_count
        writes = len(self.adapter.actions)
        self.controller.update(sample(), RECOVERY_TIMEOUT_S + 0.35)
        self.assertGreater(self.adapter.read_count, reads)
        self.assertEqual(len(self.adapter.actions), writes)
        self.adapter.positions["elbow_flex.pos"] = 9.9
        self.controller.update(sample(), RECOVERY_TIMEOUT_S + 0.5)
        self.assertEqual(self.controller.state, "READY")
        self.assertEqual(self.controller.palm_count, 0)

    def test_read_failure_retries_without_accepting_hand_motion(self):
        self.adapter.positions["elbow_flex.pos"] = 10.2
        activate(self.controller)
        original = self.adapter.read_positions
        self.adapter.read_positions = lambda: (_ for _ in ()).throw(ConnectionError("serial timeout"))
        self.controller.update(sample(), 0.2)
        self.assertEqual(self.controller.state, "LIMIT")
        self.assertIn("BLOCKED", self.controller.feedback)
        self.adapter.read_positions = original
        self.controller.update(sample(), 0.4)
        self.assertEqual(self.controller.state, "READY")

    def test_ik_recovery_accepts_late_progress_toward_boundary(self):
        q = np.zeros(5)
        q[0] = 9.0
        self.controller._begin_ik_recovery(q, 0.0)
        self.adapter.positions["shoulder_pan.pos"] = 8.5
        now = RECOVERY_TIMEOUT_S + 0.1
        self.controller.update(sample(gesture="HAND"), now)
        self.assertEqual(self.controller.state, "LIMIT")
        self.assertEqual(self.controller.last_progress_at, now)

    def test_measured_fk_tracks_each_latest_joint_read(self):
        self.adapter.positions["shoulder_pan.pos"] = 4.0
        self.controller._read()
        np.testing.assert_allclose(self.controller.measured_pose[:3, 3], [0.004, 0.0, 0.0])
        self.adapter.positions["shoulder_pan.pos"] = 5.0
        self.controller._read()
        np.testing.assert_allclose(self.controller.measured_pose[:3, 3], [0.005, 0.0, 0.0])

    def test_calibration_valid_pose_outside_ik_returns_inward_without_solving(self):
        self.adapter.positions["elbow_flex.pos"] = 8.5
        activate(self.controller)
        self.assertEqual(self.controller.state, "ACTIVE")
        self.controller.update(sample(hx=0.7), 0.053)
        self.assertEqual(self.controller.state, "LIMIT")
        self.assertLess(self.adapter.positions["elbow_flex.pos"], 8.5)
        self.assertGreaterEqual(self.adapter.positions["elbow_flex.pos"], 8.0)
        self.assertEqual(self.controller.kinematics.last_solve_count, 0)

    def test_one_encoder_count_outside_boundary_is_treated_as_quantized_readback(self):
        self.adapter.positions["elbow_flex.pos"] = 10.0 + 360 / 4095
        activate(self.controller)
        self.assertEqual(self.controller.state, "ACTIVE")
        self.assertAlmostEqual(self.controller.arm_anchor[2, 3], 0.01)


if __name__ == "__main__":
    unittest.main()
