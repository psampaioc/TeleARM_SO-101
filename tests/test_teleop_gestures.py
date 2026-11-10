import unittest
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from teleop.controller import GestureDebouncer
from teleop.hand_controller import (
    PALM_FRAMES,
    _build_live_control,
    _gesture_status,
    _jaw_angle,
    _hud_lines,
    _palm_fist_ratio,
    _palm_normal,
    _save_snapshot,
    _sample,
    _terminal_status,
    TimingWindow,
    classify_gesture,
)


def hand_landmarks(extended=True, fist=False, crocodile=False, jaw_degrees=35):
    points = np.zeros((21, 3), dtype=float)
    points[0] = [0.0, 0.0, 0.0]
    fingers = ((5, -0.04), (9, -0.015), (13, 0.015), (17, 0.04))
    for i, (mcp, x) in enumerate(fingers):
        points[mcp] = [x, 0.04, 0.0]
        finger_extended = extended or (crocodile and i == 1)
        if finger_extended:
            points[mcp + 1] = [x, 0.07, 0.0]
            points[mcp + 2] = [x, 0.10, 0.0]
            points[mcp + 3] = [x, 0.13, 0.0]
        else:
            points[mcp + 1] = [x, 0.07, 0.0]
            points[mcp + 2] = [x, 0.04, 0.0]
            points[mcp + 3] = [x + 0.02, 0.04, 0.0]
    points[1:5] = [[-0.02, 0.01, 0.0], [-0.04, 0.02, 0.0], [-0.06, 0.04, 0.0], [-0.08, 0.06, 0.0]]
    points[4] = points[[0, 5, 9, 13, 17]].mean(axis=0) + np.array([0.06, 0.0, 0.0])
    if fist:
        points[12] = points[0] + np.array([0.0, 0.03, 0.0])
    elif crocodile:
        points[4] = points[9] + np.array([0.05, 0.0, 0.0])
        points[12] = points[9] + np.array([
            0.08 * np.cos(np.deg2rad(jaw_degrees)),
            0.08 * np.sin(np.deg2rad(jaw_degrees)),
            0.0,
        ])
    return points


class GestureTests(unittest.TestCase):
    def test_hand_controller_uses_the_debouncer_palm_frame_count(self):
        self.assertEqual(PALM_FRAMES, 3)

    def test_overlay_reports_detected_gesture_instead_of_labeling_every_pose_palm(self):
        self.assertEqual(_gesture_status("FIST", 3, True), "FIST")
        self.assertEqual(_gesture_status("PALM", 1, True), "PALM 1/3")
        self.assertEqual(_gesture_status("PALM", 3, True), "PALM 3/3")
        self.assertEqual(_gesture_status("HAND", 0, True), "HAND")
        self.assertEqual(_gesture_status(None, 0, False), "NO HAND")

    def test_open_palm_crocodile_and_fist_classification(self):
        self.assertEqual(classify_gesture(hand_landmarks()), "PALM")
        self.assertEqual(classify_gesture(hand_landmarks(fist=True)), "FIST")
        self.assertEqual(classify_gesture(hand_landmarks(extended=False, crocodile=True)), "PALM")

    def test_fist_and_palm_use_thumb_base_to_middle_dip_distance_over_palm_width(self):
        palm = hand_landmarks()
        fist = hand_landmarks(fist=True)
        self.assertLessEqual(_palm_fist_ratio(fist), 0.7)
        self.assertGreaterEqual(_palm_fist_ratio(palm), 1.1)
        self.assertEqual(classify_gesture(palm), "PALM")
        self.assertEqual(classify_gesture(fist), "FIST")

    def test_ambiguous_thumb_base_to_middle_dip_distance_is_hand(self):
        points = hand_landmarks()
        points[12] = [0.03, 0.02, 0.0]
        self.assertEqual(classify_gesture(points), "HAND")

    def test_crocodile_angle_uses_triangle_at_landmark_9_and_opposite_side_4_12(self):
        for degrees in (10, 35, 60):
            points = hand_landmarks(extended=False, crocodile=True, jaw_degrees=degrees)
            self.assertAlmostEqual(_jaw_angle(points), degrees, delta=0.01)

    def test_crocodile_angle_rejects_zero_length_triangle_leg(self):
        points = hand_landmarks()
        points[4] = points[9]
        self.assertIsNone(_jaw_angle(points))

    def test_palm_normal_is_unit_vector_and_degenerate_plane_is_rejected(self):
        points = hand_landmarks()
        normal = _palm_normal(points)
        self.assertAlmostEqual(float(np.linalg.norm(normal)), 1.0, places=6)
        points[17] = points[5]
        self.assertIsNone(_palm_normal(points))

    def test_activation_uses_three_consecutive_palms_and_resets_on_invalid(self):
        gate = GestureDebouncer()
        self.assertIsNone(gate.update("PALM"))
        self.assertEqual(gate.count, 1)
        self.assertIsNone(gate.update("PALM"))
        self.assertIsNone(gate.update(None))
        self.assertEqual(gate.count, 0)
        self.assertIsNone(gate.update("PALM"))
        self.assertIsNone(gate.update("PALM"))
        self.assertEqual(gate.update("PALM"), "PALM")
        for _ in range(10):
            self.assertEqual(gate.update("PALM"), "PALM")
        self.assertEqual(gate.count, 3)

    def test_fist_is_immediate_and_not_counted_as_palm(self):
        gate = GestureDebouncer()
        self.assertIsNone(gate.update("PALM"))
        self.assertEqual(gate.update("FIST"), "FIST")
        self.assertEqual(gate.palm_count if hasattr(gate, "palm_count") else gate.count, 3)
        self.assertEqual(gate.candidate, "FIST")
        self.assertIsNone(gate.update("PALM"))
        self.assertEqual(gate.count, 1)

    def test_non_palm_hand_breaks_activation_count(self):
        gate = GestureDebouncer()
        self.assertIsNone(gate.update("PALM"))
        self.assertIsNone(gate.update("HAND"))
        self.assertEqual(gate.count, 0)
        self.assertIsNone(gate.update("PALM"))
        self.assertIsNone(gate.update("PALM"))
        self.assertEqual(gate.update("PALM"), "PALM")

    def test_preview_does_not_construct_robot_or_read_calibration(self):
        args = SimpleNamespace(live=False)
        with patch("teleop.hand_controller.RobotAdapter") as adapter, patch(
            "teleop.hand_controller.SO101Kinematics"
        ) as kinematics:
            self.assertEqual(_build_live_control(args), (None, None))
        adapter.assert_not_called()
        kinematics.assert_not_called()

    def test_missing_or_invalid_hand_landmarks_return_no_sample(self):
        self.assertIsNone(_sample(SimpleNamespace(hand_landmarks=[], hand_world_landmarks=[])))

        points = hand_landmarks()
        points[17] = points[5]
        image = [SimpleNamespace(x=float(i / 21), y=0.5) for i in range(21)]
        world = [SimpleNamespace(x=x, y=y, z=z) for x, y, z in points]
        result = SimpleNamespace(hand_landmarks=[image], hand_world_landmarks=[world], handedness=[])
        self.assertIsNone(_sample(result))

    def test_sample_keeps_numbered_points_and_live_geometry(self):
        points = hand_landmarks()
        image = [SimpleNamespace(x=float(i / 21), y=0.5) for i in range(21)]
        world = [SimpleNamespace(x=x, y=y, z=z) for x, y, z in points]
        result = SimpleNamespace(hand_landmarks=[image], hand_world_landmarks=[world], handedness=[])
        sample = _sample(result)
        self.assertEqual(sample.landmarks.shape, (21, 2))
        self.assertEqual((sample.hx, sample.hy), (0.0, 0.5))
        self.assertAlmostEqual(sample.size, 9 / 21)
        self.assertAlmostEqual(sample.palm_ratio, _palm_fist_ratio(points))
        self.assertAlmostEqual(sample.jaw_angle_deg, _jaw_angle(points))
        self.assertIsNotNone(sample.palm_normal)

    def test_timing_window_discards_warmup_and_rolls_over(self):
        window = TimingWindow(warmup=2, window=3)
        for value in (100, 200, 1, 2, 3):
            window.add({"infer": value})
        self.assertEqual(list(window.values["infer"]), [1, 2, 3])
        self.assertEqual(window.percentile("infer", 50), 2.0)
        self.assertAlmostEqual(window.percentile("infer", 95), 2.9)

    def test_hud_prints_fist_once_and_keeps_state_separate(self):
        fist = SimpleNamespace(gesture="FIST", palm_ratio=0.8, jaw_angle_deg=12.0)
        window = TimingWindow(warmup=0)
        ids = dict(zip(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"), range(1, 7)))
        controller = SimpleNamespace(adapter=SimpleNamespace(motor_ids=ids), feedback="stopped; show PALM", boundary_joints=(), saturated_axes=(),
                                     gripper_target=None, hand_delta=None, base_delta=None, limited_delta=None,
                                     last_measured_gripper=None,
                                     arm_anchor=None, requested_target=None, target_pose=None, measured_pose=None,
                                     last_measured_q=None, last_command_q=None, last_ik_result=None)
        lines = _hud_lines("LIVE", "READY", fist, controller, window)
        rendered = "\n".join(lines)
        self.assertEqual(rendered.count("FIST"), 1)
        self.assertIn("READY", lines[0])
        self.assertIn("M1", lines[1])
        self.assertIn("M6", lines[2])
        self.assertIn("XYZ[m] read", rendered)
        self.assertIn("hand FIST", rendered)
        self.assertIn("palm normal", rendered)
        self.assertIn("grip read/target", rendered)

    def test_terminal_log_reports_geometry_state_joint_readback_and_timing(self):
        sample = SimpleNamespace(gesture="PALM", palm_ratio=1.25, jaw_angle_deg=45.0)
        ids = dict(zip(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"), range(1, 7)))
        controller = SimpleNamespace(adapter=SimpleNamespace(motor_ids=ids), gripper_target=30.0, last_measured_gripper=45.0,
                                     last_measured_q=np.zeros(5), limited_delta=(0.001, 0.0, -0.002),
                                     last_ik_result=None,
                                     last_command_q=np.ones(5), feedback="ACTIVE: anchored")
        timing = TimingWindow(warmup=0)
        timing.add({"vision": 40.0, "loop": 50.0})
        line = _terminal_status("ACTIVE", sample, 3, controller, timing)
        self.assertIn("raw=PALM palm=3/3", line)
        self.assertIn("d2-12/span5-17=1.25", line)
        self.assertIn("jaw4-9-12=45deg grip read/target=45/30", line)
        self.assertIn("dXYZmm=[1.00,0.00,-2.00]", line)
        self.assertIn("M3 0.00/1.00", line)
        self.assertIn("M6 45.00/—", line)
        self.assertIn("vis/loop p95=40/50ms", line)

    def test_partial_motor_command_is_visible(self):
        ids = dict(zip(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"), range(1, 7)))
        controller = SimpleNamespace(adapter=SimpleNamespace(motor_ids=ids), last_measured_q=np.zeros(5),
                                     last_command_q=np.array([np.nan, np.nan, 97.0, np.nan, np.nan]),
                                     last_measured_gripper=0.0, last_command_gripper=None)
        from teleop.hand_controller import _motor_lines
        lines = " ".join(_motor_lines(controller))
        self.assertIn("M3 0.00/97.00", lines)
        self.assertIn("M2 0.00/—", lines)

    def test_snapshot_saves_only_in_an_existing_directory(self):
        class FakeCV2:
            @staticmethod
            def imwrite(path, _image):
                Path(path).write_bytes(b"image")
                return True

        with tempfile.TemporaryDirectory() as temp_dir:
            directory = Path(temp_dir)
            path = _save_snapshot(FakeCV2, np.zeros((1, 1)), "PALM", directory)
            self.assertTrue(path.is_file())
            missing = directory / "missing"
            self.assertIsNone(_save_snapshot(FakeCV2, np.zeros((1, 1)), "FIST", missing))
            self.assertFalse(missing.exists())

    def test_preview_hud_does_not_repeat_preview_state(self):
        lines = _hud_lines("PREVIEW | serial OFF", None, None, None, TimingWindow(warmup=0))
        self.assertEqual(lines[0].count("PREVIEW"), 1)


if __name__ == "__main__":
    unittest.main()
