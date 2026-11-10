import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from teleop.kinematics import ENCODER_STEP_DEG, JOINT_NAMES, READBACK_TOLERANCE_DEG, SO101Kinematics


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kin = SO101Kinematics()

    def test_named_so101_joints_and_gripper_frame(self):
        self.assertEqual(self.kin.joint_names, JOINT_NAMES)
        self.assertEqual(self.kin.target_frame_name, "gripper_frame_link")
        self.assertEqual(self.kin.lower.shape, (5,))
        self.assertTrue(np.all(self.kin.lower < self.kin.upper))

    def test_forward_pose_is_finite_homogeneous_matrix(self):
        q = np.clip(np.zeros(5), self.kin.lower, self.kin.upper)
        pose = self.kin.forward_pose(q)
        self.assertEqual(pose.shape, (4, 4))
        self.assertTrue(np.all(np.isfinite(pose)))
        np.testing.assert_allclose(pose[3], [0, 0, 0, 1], atol=1e-7)

    def test_current_fk_pose_is_zero_error_ik(self):
        q = np.clip(np.zeros(5), self.kin.lower, self.kin.upper)
        result = self.kin.solve_position(q, self.kin.forward_pose(q))
        self.assertTrue(result.ok, result.reason)
        np.testing.assert_allclose(result.q_deg, q, atol=1e-4)
        self.assertLess(result.residual_m, 1e-6)

    def test_reachable_fk_target_round_trips_through_lerobot_ik(self):
        q = (self.kin.lower + self.kin.upper) / 2
        q_goal = q + np.array([0.5, -0.5, 0.5, -0.25, 0.25])
        target = self.kin.forward_pose(q_goal)
        result = self.kin.solve_position(q, target)
        self.assertTrue(result.ok, result.reason)
        self.assertLess(result.residual_m, 0.005)
        self.assertTrue(np.all(result.q_deg >= self.kin.lower))
        self.assertTrue(np.all(result.q_deg <= self.kin.upper))

    def test_rate_limit_is_applied_in_degrees(self):
        q = (self.kin.lower + self.kin.upper) / 2
        target = self.kin.forward_pose(q + np.array([3.0, -2.0, 1.0, 0, 0]))
        result = self.kin.solve_position(q, target, max_step_deg=0.4)
        self.assertTrue(result.ok, result.reason)
        self.assertLessEqual(float(np.max(np.abs(result.q_deg - q))), 0.400001)

    def test_calibration_bounds_readback_while_ik_targets_stay_inside_intersection(self):
        calibration = {
            "shoulder_pan": {"range_min": 784, "range_max": 3479},
            "shoulder_lift": {"range_min": 708, "range_max": 3231},
            "elbow_flex": {"range_min": 931, "range_max": 3148},
            "wrist_flex": {"range_min": 757, "range_max": 3168},
            "wrist_roll": {"range_min": 0, "range_max": 4095},
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration.json"
            path.write_text(json.dumps(calibration))
            calibrated = SO101Kinematics(calibration_path=path)
        self.assertTrue(np.all(calibrated.ik_lower >= calibrated.lower))
        self.assertTrue(np.all(calibrated.ik_upper <= calibrated.upper))
        self.assertTrue(np.all(calibrated.ik_lower >= self.kin.lower))
        self.assertTrue(np.all(calibrated.ik_upper <= self.kin.upper))
        q = np.array([-2.593, -109.495, 97.099, 28.967, 2.945])
        bounded, clipped = calibrated.clamp(q)
        self.assertFalse(clipped)
        np.testing.assert_allclose(bounded, q, atol=1e-8)

    def test_solver_projects_a_calibration_valid_seed_into_ik_bounds(self):
        calibration = {
            "shoulder_pan": {"range_min": 784, "range_max": 3479},
            "shoulder_lift": {"range_min": 708, "range_max": 3231},
            "elbow_flex": {"range_min": 931, "range_max": 3148},
            "wrist_flex": {"range_min": 757, "range_max": 3168},
            "wrist_roll": {"range_min": 0, "range_max": 4095},
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration.json"
            path.write_text(json.dumps(calibration))
            calibrated = SO101Kinematics(calibration_path=path)
        current = np.zeros(5)
        current[0] = calibrated.ik_lower[0] - 1.0
        target = calibrated.forward_pose(np.clip(current, calibrated.ik_lower, calibrated.ik_upper))
        result = calibrated.solve_position(current, target, max_step_deg=0.4)
        self.assertTrue(result.ok, result.reason)
        self.assertTrue(np.all(result.solver_seed_deg >= calibrated.ik_lower))
        self.assertTrue(np.all(result.solver_seed_deg <= calibrated.ik_upper))
        self.assertTrue(np.all(result.q_deg >= calibrated.ik_lower))
        self.assertTrue(np.all(result.q_deg <= calibrated.ik_upper))
        self.assertLess(result.residual_m, 0.005)

    def test_rate_limited_ik_keeps_cartesian_progress_near_starting_pose(self):
        q = np.array([-2.95, -109.85, 97.01, 81.63, 2.95])
        target = self.kin.forward_pose(q)
        target[0, 3] += 0.01
        before = np.linalg.norm(target[:3, 3] - self.kin.forward_position(q))
        result = self.kin.solve_position(q, target, max_step_deg=1.0)
        self.assertTrue(result.ok, result.reason)
        self.assertLess(result.residual_m, before)
        self.assertLessEqual(np.max(np.abs(result.q_deg - q)), 1.000001)

    def test_vertical_translation_preserves_anchored_gripper_orientation(self):
        q = np.array([-21.58, -93.05, 96.22, 64.66, 2.86])
        target = self.kin.forward_pose(q)
        target[2, 3] += 0.05
        result = self.kin.solve_position(q, target)
        self.assertTrue(result.ok, result.reason)
        self.assertLess(result.residual_m, 0.005)
        self.assertLess(abs(result.q_deg[4] - q[4]), 10.0)

    def test_clamp_projects_encoder_quantization_to_safe_boundary(self):
        q = (self.kin.lower + self.kin.upper) / 2
        q[0] = self.kin.lower[0] - ENCODER_STEP_DEG
        bounded, clipped = self.kin.clamp(q)
        self.assertFalse(clipped)
        self.assertEqual(bounded[0], self.kin.lower[0])
        q[0] = self.kin.lower[0] - 2 * READBACK_TOLERANCE_DEG
        _, clipped = self.kin.clamp(q)
        self.assertTrue(clipped)

    def test_invalid_calibration_and_pose_are_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "calibration.json"
            path.write_text(json.dumps({"shoulder_pan": {"range_min": 1, "range_max": 2}}))
            with self.assertRaisesRegex(ValueError, "missing"):
                SO101Kinematics(calibration_path=path)
        with self.assertRaisesRegex(ValueError, "4x4"):
            self.kin.solve_position(np.zeros(5), np.zeros((3, 3)))

    def test_out_of_range_readback_cannot_be_used_as_ik_seed(self):
        q = (self.kin.lower + self.kin.upper) / 2
        q[0] = self.kin.upper[0] + 0.1
        result = self.kin.solve_position(q, np.eye(4))
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "READBACK_OUTSIDE_LIMITS")
        self.assertLessEqual(result.q_deg[0], self.kin.upper[0])

    def test_nonfinite_joint_values_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "finite"):
            self.kin.forward_pose([0, 0, np.nan, 0, 0])


if __name__ == "__main__":
    unittest.main()
