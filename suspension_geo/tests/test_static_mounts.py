"""Check fixed chassis X coordinates, design geometry and moving-wheel behavior."""

import contextlib
import io
import unittest

import numpy as np

from suspension_geo import Geometric_Analysis as ga
from suspension_geo.car import Car, SuspensionF, SuspensionR


class StaticMountTests(unittest.TestCase):
    def build(self, suspension=None, car=None):
        suspension = SuspensionF() if suspension is None else suspension
        helper = ga.SusGeometryHelper(Car() if car is None else car, suspension)
        points = helper.calc_static_points()
        right = ga.SuspensionSide("Right")
        right.setup_right_side(points)
        left = ga.SuspensionSide("Left", is_left=True)
        left.setup_from_right_template(right)
        return helper, points, left, right

    def test_rear_mounts_are_symmetric_and_preserve_design_centers(self):
        suspension = SuspensionR()
        helper, points, left, right = self.build(suspension)
        for point_id, x in (("cpb_rl", 0.205), ("cpb_ru", 0.250)):
            with self.subTest(point=point_id):
                self.assertEqual(getattr(right, point_id).x, x)
                np.testing.assert_array_equal(
                    getattr(left, point_id).pos,
                    getattr(right, point_id).pos * [-1, 1],
                )
        np.testing.assert_allclose(points["IC"], points["ICo"], atol=1e-10)
        rc = helper.line_intersection(left.gnd, left.IC, right.gnd, right.IC)
        np.testing.assert_allclose(rc, [0, suspension.h_rc], atol=1e-10)

    def test_rear_geometry_uses_rear_track_and_loaded_radius(self):
        car = Car()
        car.tf, car.tr = 1.45, 1.16
        car.load_Radius = np.array([0.235, 0.171])
        suspension = SuspensionR()
        _, _, left, right = self.build(suspension, car)
        self.assertAlmostEqual(right.gnd.x - left.gnd.x, car.tr)
        self.assertAlmostEqual(right.kua.y - suspension.Au, car.load_Radius[1])
        self.assertAlmostEqual(right.lak.y + suspension.Al, car.load_Radius[1])

    def test_default_mounts_match_requested_csv_coordinates(self):
        _, _, left, right = self.build()
        expected = {
            "cpb_rl": [-208.33638495469026, 94.68009380063722],
            "cpb_ru": [-237.88313825217958, 224.68580830959013],
        }
        for point_id, position_mm in expected.items():
            with self.subTest(point=point_id):
                np.testing.assert_allclose(
                    getattr(left, point_id).pos * 1000.0, position_mm, atol=1e-9, rtol=0,
                )
                np.testing.assert_allclose(
                    getattr(right, point_id).pos * 1000.0,
                    np.array(position_mm) * [-1, 1], atol=1e-9, rtol=0,
                )

    def test_upper_and_lower_x_can_be_changed_independently(self):
        for suspension_class in (SuspensionF, SuspensionR):
            with self.subTest(axle=suspension_class.__name__):
                self.check_independent_x(suspension_class)

    def check_independent_x(self, suspension_class):
        _, baseline, _, _ = self.build(suspension_class())
        for setting, point_id, other_point in (
            ("lower_mount_x", "cpb_rl", "cpb_ru"),
            ("upper_mount_x", "cpb_ru", "cpb_rl"),
        ):
            with self.subTest(setting=setting):
                suspension = suspension_class()
                x = getattr(suspension, setting) + 0.025
                setattr(suspension, setting, x)
                _, points, _, _ = self.build(suspension)
                self.assertEqual(points[point_id][0], x)
                self.assertNotAlmostEqual(points[point_id][1], baseline[point_id][1])
                np.testing.assert_array_equal(points[other_point], baseline[other_point])
                np.testing.assert_allclose(points["IC"], baseline["ICo"], atol=1e-10)

    def test_design_instant_center_and_roll_center_are_preserved(self):
        for height, fvsa in ((-0.005, 0.75), (0.035, 1.25), (0.05, 2.5)):
            with self.subTest(height=height, fvsa=fvsa):
                suspension = SuspensionF()
                suspension.h_rc = height
                suspension.fvsa = fvsa
                suspension.lower_mount_x = 0.185
                suspension.upper_mount_x = 0.265
                helper, points, left, right = self.build(suspension)
                np.testing.assert_allclose(points["IC"], points["ICo"], atol=1e-10)
                rc = helper.line_intersection(left.gnd, left.IC, right.gnd, right.IC)
                np.testing.assert_allclose(rc, [0, height], atol=1e-10)
                self.assertEqual(right.cpb_rl.x, suspension.lower_mount_x)
                self.assertEqual(right.cpb_ru.x, suspension.upper_mount_x)

    def test_asymmetric_wheel_motion_leaves_chassis_mounts_fixed(self):
        for suspension_class in (SuspensionF, SuspensionR):
            with self.subTest(axle=suspension_class.__name__):
                self.check_fixed_mounts(suspension_class)

    def check_fixed_mounts(self, suspension_class):
        helper, _, left, right = self.build(suspension_class())
        initial_ground = left.gnd.pos.copy()
        mounts = [left.cpb_rl, left.cpb_ru, right.cpb_rl, right.cpb_ru]
        initial_mounts = [point.pos.copy() for point in mounts]
        low, high = ga.calculate_theta_limits(right)
        solver = ga.SuspensionDirectSolver(helper, left, right)
        with contextlib.redirect_stdout(io.StringIO()):
            state = solver.solve_by_thetas(high, low)
        self.assertTrue(np.isfinite(state["rc_pos"]).all())
        self.assertFalse(np.allclose(left.gnd.pos, initial_ground))
        for point, initial in zip(mounts, initial_mounts):
            np.testing.assert_array_equal(point.pos, initial)

    def test_invalid_mount_x_is_rejected(self):
        for setting in ("lower_mount_x", "upper_mount_x"):
            for value in (np.nan, np.inf, -0.2):
                with self.subTest(setting=setting, value=value):
                    suspension = SuspensionF()
                    setattr(suspension, setting, value)
                    with self.assertRaisesRegex(ValueError, "Mount X"):
                        self.build(suspension)

    def test_vertical_design_line_has_no_unique_mount_y(self):
        helper, points, _, _ = self.build()
        helper.fvsa = helper.t / 2 - points["lak"][0]
        with self.assertRaisesRegex(ValueError, "vertical"):
            helper.calc_static_points()


if __name__ == "__main__":
    unittest.main()
