"""Validate local camber gain against geometry and independent travel differences."""

import contextlib
import io
import unittest

import numpy as np

from suspension_geo import Geometric_Analysis as ga
from suspension_geo.car import Car, SuspensionF, SuspensionR


GAIN_KEYS = ("camber_gain_left_deg_per_mm", "camber_gain_right_deg_per_mm")


class CamberGainTests(unittest.TestCase):
    def build(self, suspension=None, car=None):
        helper = ga.SusGeometryHelper(
            Car() if car is None else car,
            SuspensionF() if suspension is None else suspension,
        )
        right = ga.SuspensionSide("Right")
        right.setup_right_side(helper.calc_static_points())
        left = ga.SuspensionSide("Left", is_left=True)
        left.setup_from_right_template(right)
        return ga.SuspensionDirectSolver(helper, left, right)

    def solve(self, solver, method, **kwargs):
        with contextlib.redirect_stdout(io.StringIO()):
            return getattr(solver, method)(**kwargs)

    def test_static_gain_matches_instant_center_geometry_in_deg_per_mm(self):
        # At static: tire rotation contributes -1/FVSA and the moving ground
        # contributes +1/track, both in rad/m. This also checks zero travel.
        for suspension_class in (SuspensionF, SuspensionR):
            with self.subTest(axle=suspension_class.__name__):
                car = Car()
                car.tf, car.tr = 1.45, 1.16
                suspension = suspension_class()
                suspension.fvsa = 0.9
                solver = self.build(suspension, car)
                state = self.solve(
                    solver, "solve_by_travels", l_travel_m=0.0, r_travel_m=0.0,
                )
                track = solver.sus_helper.t
                expected = np.degrees(1 / track - 1 / suspension.fvsa) / 1000
                for key in GAIN_KEYS:
                    self.assertLess(state[key], 0)
                    self.assertAlmostEqual(state[key], expected, places=8)

    def test_local_gain_matches_independent_wheel_travel_differences(self):
        for left_m, right_m in ((0.010, -0.005), (-0.018, 0.014)):
            solver = self.build()
            state = self.solve(
                solver, "solve_by_travels", l_travel_m=left_m, r_travel_m=right_m,
            )
            for index, side in enumerate(("left", "right")):
                with self.subTest(left_m=left_m, right_m=right_m, side=side):
                    references = []
                    for step_m in (0.0001, 0.00005):
                        samples = []
                        for sign in (-1, 1):
                            target = [left_m, right_m]
                            target[index] += sign * step_m
                            sample = self.solve(
                                solver, "solve_by_travels",
                                l_travel_m=target[0], r_travel_m=target[1],
                            )
                            other = "right" if side == "left" else "left"
                            self.assertAlmostEqual(
                                sample[f"travel_{other}_mm"],
                                [right_m, left_m][index] * 1000, places=6,
                            )
                            samples.append(sample[f"camber_{side}_deg"])
                        references.append((samples[1] - samples[0]) / (2 * step_m * 1000))
                    self.assertAlmostEqual(references[0], references[1], places=7)
                    self.assertAlmostEqual(state[GAIN_KEYS[index]], references[1], places=7)

    def test_heave_pose_and_theta_inputs_return_the_same_gains(self):
        solver = self.build()
        travel = self.solve(
            solver, "solve_by_travels", l_travel_m=0.012, r_travel_m=-0.004,
        )
        pose = self.solve(
            solver, "solve_by_pose", target_heave_m=travel["heave_mm"] / 1000,
            target_roll_deg=travel["roll_deg"],
        )
        angles = self.solve(
            solver, "solve_by_thetas",
            theta_l=travel["theta_left"], theta_r=travel["theta_right"],
        )
        for state in (pose, angles):
            for key in (*GAIN_KEYS, "camber_left_deg", "camber_right_deg"):
                with self.subTest(key=key):
                    self.assertAlmostEqual(state[key], travel[key], places=8)
            np.testing.assert_allclose(state["rc_pos"], travel["rc_pos"], atol=1e-9)

    def test_gain_sampling_preserves_requested_geometry_and_existing_results(self):
        solver = self.build()
        theta_l = solver.right_sus.lower_arm.angle + 0.035
        theta_r = solver.right_sus.lower_arm.angle - 0.025
        solver.left_sus.update_kinematics(theta_l, solver.sus_helper)
        solver.right_sus.update_kinematics(theta_r, solver.sus_helper)
        sides = (solver.left_sus, solver.right_sus)
        points = [
            point for side in sides for point in
            [*side.upright_body.points, side.IC, side.cpb_rl, side.cpb_ru]
        ]
        positions = [point.pos.copy() for point in points]
        ground = solver.right_sus.gnd.pos - solver.left_sus.gnd.pos
        expected_cambers = [side.calc_camber_angle(ground) for side in sides]
        expected_rc = solver.sus_helper.line_intersection(
            solver.left_sus.gnd, solver.left_sus.IC,
            solver.right_sus.gnd, solver.right_sus.IC,
        )
        state = self.solve(solver, "solve_by_thetas", theta_l=theta_l, theta_r=theta_r)
        for point, position in zip(points, positions):
            np.testing.assert_allclose(point.pos, position, rtol=0, atol=1e-14)
        for side, camber in zip(("left", "right"), expected_cambers):
            self.assertAlmostEqual(state[f"camber_{side}_deg"], camber, places=10)
        np.testing.assert_allclose(state["rc_pos"], expected_rc, rtol=0, atol=1e-14)

    def test_symmetric_compression_and_rebound_have_equal_left_right_gain(self):
        solver = self.build()
        for travel_m in (-0.020, 0.0, 0.020):
            with self.subTest(travel_m=travel_m):
                state = self.solve(
                    solver, "solve_by_travels",
                    l_travel_m=travel_m, r_travel_m=travel_m,
                )
                self.assertTrue(np.isfinite([state[key] for key in GAIN_KEYS]).all())
                self.assertAlmostEqual(state[GAIN_KEYS[0]], state[GAIN_KEYS[1]], places=8)


if __name__ == "__main__":
    unittest.main()
