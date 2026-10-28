"""Check wheel frames, Euler reconstruction and exported coordinate bases."""

import csv
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from suspension_geo.car import SuspensionF
from suspension_geo.we3D.sus_3Dwe_pose import (
    calculate_wheel_pose, create_pose_animation, export_pose_csv, kingpin_axis,
    load_pose_csv, main, sweep_wheel_poses,
)


class WheelPoseTests(unittest.TestCase):
    def test_zero_steering_is_reference_frame(self):
        pose = calculate_wheel_pose(0.0)
        np.testing.assert_array_equal(pose["center_m"], np.zeros(3))
        np.testing.assert_array_equal(pose["rotation_matrix"], np.eye(3))
        np.testing.assert_array_equal(pose["euler_xyz_deg"], np.zeros(3))

    def test_vertical_kingpin_gives_only_z_rotation(self):
        suspension = SuspensionF()
        suspension.kpi = 0.0
        suspension.caster_sv_deg = 0.0
        for angle in (-30.0, -15.0, 0.0, 15.0, 30.0):
            with self.subTest(angle=angle):
                pose = calculate_wheel_pose(angle, suspension=suspension)
                np.testing.assert_allclose(pose["euler_xyz_deg"], [0.0, 0.0, angle], atol=1e-12)
                expected_x = [np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle)), 0.0]
                np.testing.assert_allclose(pose["rotation_matrix"][:, 0], expected_x, atol=1e-12)

    def test_default_sweep_preserves_rigid_right_handed_frame_and_kingpin(self):
        suspension = SuspensionF()
        axis = kingpin_axis(suspension)
        poses = sweep_wheel_poses(suspension=suspension)
        self.assertEqual(len(poses), 121)
        for pose in poses:
            matrix = pose["rotation_matrix"]
            np.testing.assert_array_equal(pose["center_m"], np.zeros(3))
            np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(matrix), 1.0)
            np.testing.assert_allclose(np.cross(matrix[:, 0], matrix[:, 1]), matrix[:, 2], atol=1e-12)
            np.testing.assert_allclose(matrix @ axis, axis, atol=1e-12)
            inverse = calculate_wheel_pose(-pose["kingpin_rotation_deg"], suspension=suspension)
            np.testing.assert_allclose(inverse["rotation_matrix"], matrix.T, atol=1e-12)

    def test_fixed_xyz_euler_angles_reconstruct_full_orientation(self):
        for kpi, caster in ((12.0, 5.0), (-16.0, 8.0), (0.0, -10.0), (20.0, 0.0)):
            suspension = SuspensionF()
            suspension.kpi = np.deg2rad(kpi)
            suspension.caster_sv_deg = caster
            for angle in (-30.0, -18.0, 0.0, 9.0, 30.0):
                with self.subTest(kpi=kpi, caster=caster, angle=angle):
                    pose = calculate_wheel_pose(angle, suspension=suspension)
                    x, y, z = np.deg2rad(pose["euler_xyz_deg"])
                    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
                    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
                    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
                    np.testing.assert_allclose(rz @ ry @ rx, pose["rotation_matrix"], atol=1e-12)

    def test_nondivisible_sweep_includes_endpoints_zero_and_both_signs(self):
        angles = np.array([pose["kingpin_rotation_deg"] for pose in sweep_wheel_poses(7.0)])
        self.assertEqual(angles[0], -30.0)
        self.assertEqual(angles[-1], 30.0)
        self.assertEqual(np.count_nonzero(angles == 0.0), 1)
        self.assertTrue(np.all(np.diff(angles) > 0.0))
        self.assertLessEqual(np.max(np.diff(angles)), 7.0)
        np.testing.assert_array_equal(angles, -angles[::-1])

    def test_csv_axis_vectors_are_columns_in_reference_coordinates(self):
        poses = [calculate_wheel_pose(-30.0), calculate_wheel_pose(0.0), calculate_wheel_pose(30.0)]
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            path = Path(directory) / "pose.csv"
            export_pose_csv(path, poses)
            with path.open(encoding="utf-8-sig", newline="") as stream:
                rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), 3)
        for row, pose in zip(rows, poses):
            self.assertEqual(float(row["kingpin_rotation_deg"]), pose["kingpin_rotation_deg"])
            for index, axis in enumerate("xyz"):
                self.assertEqual(float(row[f"center_{axis}_mm"]), 0.0)
                self.assertAlmostEqual(float(row[f"r{axis}_deg"]), pose["euler_xyz_deg"][index])
                vector = [float(row[f"wheel_{axis}_in_ref_{ref}"]) for ref in "xyz"]
                np.testing.assert_allclose(vector, pose["rotation_matrix"][:, index], atol=1e-12)

    def test_csv_reload_preserves_positions_frames_and_sorts_playback(self):
        poses = [calculate_wheel_pose(30.0), calculate_wheel_pose(-30.0)]
        poses[0]["center_m"] = np.array([0.01, -0.02, 0.03])
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            path = Path(directory) / "poses.csv"
            export_pose_csv(path, poses)
            loaded = load_pose_csv(path)
        self.assertEqual([pose["kingpin_rotation_deg"] for pose in loaded], [-30.0, 30.0])
        for actual, expected in zip(loaded, reversed(poses)):
            for key in ("center_m", "euler_xyz_deg", "rotation_matrix"):
                np.testing.assert_allclose(actual[key], expected[key], atol=1e-12)

    def test_csv_reload_reports_missing_columns_and_invalid_frame(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            path = Path(directory) / "bad.csv"
            path.write_text("kingpin_rotation_deg\n0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "missing columns"):
                load_pose_csv(path)
            pose = calculate_wheel_pose(0.0)
            pose["rotation_matrix"][0, 0] = 2.0
            export_pose_csv(path, [pose])
            with self.assertRaisesRegex(ValueError, "right-handed unit frame"):
                load_pose_csv(path)
            export_pose_csv(path, [])
            with self.assertRaisesRegex(ValueError, "no wheel poses"):
                load_pose_csv(path)

    def test_animation_uses_csv_frames_without_recalculating_geometry(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        poses = [calculate_wheel_pose(-30.0), calculate_wheel_pose(0.0), calculate_wheel_pose(30.0)]
        poses[0]["center_m"] = np.array([0.01, -0.02, 0.03])
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            path = Path(directory) / "animation.csv"
            export_pose_csv(path, poses)
            with patch("suspension_geo.we3D.sus_3Dwe_pose.calculate_wheel_pose",
                       side_effect=AssertionError("Animation must read saved frames")), \
                 patch("suspension_geo.we3D.sus_3Dwe_pose.kingpin_axis",
                       side_effect=AssertionError("Animation must not use current geometry")):
                figure = create_pose_animation(path, interval_ms=80)
                figure.canvas.draw()
                frames = list(figure.wheel_animation.new_frame_seq())
                self.assertEqual([frame["kingpin_rotation_deg"] for frame in frames], [-30.0, 0.0, 30.0])
                np.testing.assert_allclose(frames[0]["center_m"], [0.01, -0.02, 0.03])
                labels = "\n".join(text.get_text() for text in figure.axes[0].texts)
                self.assertIn("Center [mm] = (10.000, -20.000, 30.000)", labels)
                self.assertIn("Kingpin = -30.00 deg", labels)
                plt.close(figure)

    def test_prompt_reads_generated_csv_on_y_and_no_show_skips_prompt(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            for answer in ("Y", " y ", "N"):
                csv_path = Path(directory) / "wheel_center_pose.csv"

                def reply(prompt):
                    self.assertTrue(csv_path.exists(), "CSV must exist before asking for Y")
                    self.assertEqual(len(load_pose_csv(csv_path)), 3)
                    return answer

                with self.subTest(answer=answer), contextlib.redirect_stdout(io.StringIO()), \
                     patch("builtins.input", side_effect=reply), \
                     patch("suspension_geo.we3D.sus_3Dwe_pose.create_pose_animation",
                           wraps=create_pose_animation) as animate, \
                     patch.object(plt, "show", side_effect=lambda: plt.gcf().canvas.draw()) as show:
                    main(["--step", "30", "--output-dir", directory])
                    self.assertEqual(animate.call_count, 0 if answer == "N" else 1)
                    self.assertEqual(show.call_count, 0 if answer == "N" else 1)
                    if answer != "N":
                        animate.assert_called_once_with(csv_path, interval_ms=50.0)
            with contextlib.redirect_stdout(io.StringIO()), patch("builtins.input") as prompt, \
                 patch("suspension_geo.we3D.sus_3Dwe_pose.create_pose_animation") as animate:
                main(["--no-show", "--step", "30", "--output-dir", directory])
                prompt.assert_not_called()
                animate.assert_not_called()
        plt.close("all")

    def test_rejects_invalid_angles_steps_and_nonfinite_geometry(self):
        for angle in (-30.01, 30.01, np.nan, np.inf):
            with self.subTest(angle=angle), self.assertRaises(ValueError):
                calculate_wheel_pose(angle)
        for step in (-1.0, 0.0, 30.01, np.nan, np.inf):
            with self.subTest(step=step), self.assertRaises(ValueError):
                sweep_wheel_poses(step)
        for field in ("kpi", "caster_sv_deg"):
            suspension = SuspensionF()
            setattr(suspension, field, np.nan)
            with self.subTest(field=field), self.assertRaises(ValueError):
                calculate_wheel_pose(0.0, suspension=suspension)


if __name__ == "__main__":
    unittest.main()
