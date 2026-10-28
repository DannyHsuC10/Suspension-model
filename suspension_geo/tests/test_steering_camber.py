"""Check steering-induced camber against independent geometric limits."""

import contextlib
import io
import unittest

import numpy as np

from suspension_geo.car import SuspensionF
from suspension_geo.we3D.steering_camber import calculate_steering_camber, main


class SteeringCamberTests(unittest.TestCase):
    def suspension(self, kpi_deg, caster_deg):
        suspension = SuspensionF()
        suspension.kpi = np.deg2rad(kpi_deg)
        suspension.caster_sv_deg = caster_deg
        return suspension

    def test_zero_steer_preserves_independent_static_cambers(self):
        state = calculate_steering_camber(
            0.0, 0.0, static_camber_left_deg=-2.5, static_camber_right_deg=-1.5,
        )
        for side, expected in (("left", -2.5), ("right", -1.5)):
            self.assertAlmostEqual(state[f"camber_{side}_deg"], expected, places=11)
            self.assertAlmostEqual(state[f"camber_change_{side}_deg"], 0.0, places=11)
            self.assertAlmostEqual(state[f"kingpin_rotation_{side}_deg"], 0.0, places=11)

    def test_vertical_kingpins_preserve_camber_at_every_heading(self):
        suspension = self.suspension(0.0, 0.0)
        for heading in (-80.0, -30.0, 0.0, 25.0, 80.0):
            with self.subTest(heading=heading):
                state = calculate_steering_camber(
                    heading, suspension=suspension,
                    static_camber_left_deg=-2.0, static_camber_right_deg=3.0,
                )
                for side, static in (("left", -2.0), ("right", 3.0)):
                    self.assertAlmostEqual(state[f"camber_{side}_deg"], static, places=11)
                    self.assertAlmostEqual(state[f"camber_gain_{side}_deg_per_deg"], 0.0)
                    self.assertAlmostEqual(state[f"kingpin_rotation_{side}_deg"], heading)

    def test_pure_caster_matches_exact_heading_formula_and_signs(self):
        # With KPI=0: tan(camber) = side * tan(caster) * sin(heading).
        caster = np.deg2rad(8.0)
        for heading_deg in (-60.0, -20.0, 0.0, 20.0, 60.0):
            state = calculate_steering_camber(
                heading_deg, suspension=self.suspension(0.0, 8.0),
            )
            heading = np.deg2rad(heading_deg)
            for side, sign in (("left", 1.0), ("right", -1.0)):
                with self.subTest(heading=heading_deg, side=side):
                    expected = np.rad2deg(np.arctan(sign * np.tan(caster) * np.sin(heading)))
                    gain = sign * np.tan(caster) * np.cos(heading) / (
                        1.0 + (np.tan(caster) * np.sin(heading))**2
                    )
                    self.assertAlmostEqual(state[f"camber_{side}_deg"], expected, places=10)
                    self.assertAlmostEqual(state[f"camber_gain_{side}_deg_per_deg"], gain, places=10)

    def test_pure_kpi_adds_positive_camber_symmetrically(self):
        suspension = self.suspension(12.0, 0.0)
        tangent = np.tan(suspension.kpi)
        for heading_deg in (-40.0, -15.0, 15.0, 40.0):
            state = calculate_steering_camber(heading_deg, suspension=suspension)
            horizontal = tangent * np.cos(np.deg2rad(heading_deg))
            expected = np.rad2deg(
                np.arcsin(tangent / np.hypot(1.0, horizontal)) - np.arctan(horizontal)
            )
            self.assertGreater(expected, 0.0)
            for side in ("left", "right"):
                with self.subTest(heading=heading_deg, side=side):
                    self.assertAlmostEqual(state[f"camber_{side}_deg"], expected, places=10)

    def test_reversing_turn_swaps_wheels_and_reverses_gain_sign(self):
        left_turn = calculate_steering_camber(27.0, 19.0)
        right_turn = calculate_steering_camber(-19.0, -27.0)
        for side, opposite in (("left", "right"), ("right", "left")):
            self.assertAlmostEqual(
                left_turn[f"camber_{side}_deg"], right_turn[f"camber_{opposite}_deg"],
            )
            self.assertAlmostEqual(
                left_turn[f"camber_gain_{side}_deg_per_deg"],
                -right_turn[f"camber_gain_{opposite}_deg_per_deg"],
            )

    def test_local_gain_matches_finite_heading_differences(self):
        for headings in ((0.0, 0.0), (28.0, 19.0), (-23.0, -31.0)):
            kwargs = {"static_camber_left_deg": -2.0, "static_camber_right_deg": -1.0}
            state = calculate_steering_camber(*headings, **kwargs)
            for index, side in enumerate(("left", "right")):
                with self.subTest(headings=headings, side=side):
                    samples = []
                    for offset in (-1e-4, 1e-4):
                        target = list(headings)
                        target[index] += offset
                        samples.append(calculate_steering_camber(*target, **kwargs))
                    expected = (samples[1][f"camber_{side}_deg"] - samples[0][f"camber_{side}_deg"]) / 2e-4
                    self.assertAlmostEqual(state[f"camber_gain_{side}_deg_per_deg"], expected, places=8)
                    other = "right" if side == "left" else "left"
                    self.assertEqual(samples[0][f"camber_{other}_deg"], samples[1][f"camber_{other}_deg"])

    def test_combined_geometry_preserves_kingpin_projection_of_normal(self):
        # Rotation around a fixed kingpin must preserve axis.normal. Construct
        # the final normal directly from the requested heading and output camber.
        suspension = self.suspension(16.0, 7.0)
        state = calculate_steering_camber(
            33.0, 21.0, suspension=suspension,
            static_camber_left_deg=-3.0, static_camber_right_deg=-2.0,
        )
        for side, sign in (("left", 1.0), ("right", -1.0)):
            axis = np.array([-np.tan(np.deg2rad(7.0)), -sign * np.tan(suspension.kpi), 1.0])
            axis /= np.linalg.norm(axis)
            self.assertAlmostEqual(np.rad2deg(np.arctan2(-axis[0], axis[2])), 7.0)
            self.assertAlmostEqual(np.rad2deg(np.arctan2(-sign * axis[1], axis[2])), 16.0)
            heading = np.deg2rad(state[f"steer_{side}_deg"])
            camber = np.deg2rad(state[f"camber_{side}_deg"])
            static = np.deg2rad(state[f"static_camber_{side}_deg"])
            initial = np.array([0.0, sign * np.cos(static), -np.sin(static)])
            final = np.array([
                -sign * np.cos(camber) * np.sin(heading),
                sign * np.cos(camber) * np.cos(heading), -np.sin(camber),
            ])
            self.assertAlmostEqual(np.dot(axis, initial), np.dot(axis, final), places=12)
            # The actual rotation is not the heading for an inclined kingpin.
            rotation = np.deg2rad(state[f"kingpin_rotation_{side}_deg"])
            sine = np.dot(axis, np.cross(initial, final))
            cosine = np.dot(initial, final) - np.dot(axis, initial)**2
            self.assertAlmostEqual(np.arctan2(sine, cosine), rotation, places=12)

    def test_single_input_means_parallel_steering(self):
        self.assertEqual(calculate_steering_camber(20.0), calculate_steering_camber(20.0, 20.0))

    def test_reads_current_suspension_parameters_without_modifying_them(self):
        suspension = self.suspension(5.0, 9.0)
        before = vars(suspension).copy()
        state = calculate_steering_camber(20.0, suspension=suspension)
        self.assertAlmostEqual(state["kpi_deg"], 5.0)
        self.assertAlmostEqual(state["caster_deg"], 9.0)
        self.assertEqual(vars(suspension), before)
        self.assertNotAlmostEqual(state["camber_left_deg"], calculate_steering_camber(20.0)["camber_left_deg"])

    def test_rejects_nonfinite_and_out_of_range_inputs(self):
        for invalid in (float("nan"), float("inf"), -float("inf"), -90.0, 90.0):
            for field in ("steer_left_deg", "steer_right_deg", "static_camber_left_deg", "static_camber_right_deg"):
                with self.subTest(field=field, invalid=invalid), self.assertRaises(ValueError):
                    kwargs = {"steer_left_deg": 0.0, field: invalid}
                    calculate_steering_camber(**kwargs)
            for field in ("kpi", "caster_sv_deg"):
                suspension = SuspensionF()
                setattr(suspension, field, np.deg2rad(invalid) if field == "kpi" else invalid)
                with self.subTest(field=field, invalid=invalid), self.assertRaises(ValueError):
                    calculate_steering_camber(0.0, suspension=suspension)

    def test_cli_supports_distinct_angles_and_parameter_overrides(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            state = main(["--left", "25", "--right", "18", "--kpi", "12", "--caster", "6",
                          "--static-camber-left", "-2", "--static-camber-right", "-1"])
        self.assertEqual(state["steer_left_deg"], 25.0)
        self.assertEqual(state["steer_right_deg"], 18.0)
        self.assertEqual(state["static_camber_left_deg"], -2.0)
        self.assertEqual(state["static_camber_right_deg"], -1.0)
        self.assertAlmostEqual(state["kpi_deg"], 12.0)
        self.assertAlmostEqual(state["caster_deg"], 6.0)
        self.assertIn("Local camber gain [deg/deg]", output.getvalue())

    def test_cli_rejects_incomplete_or_ambiguous_angle_inputs(self):
        for arguments in (["--left", "20"], ["--right", "20"], ["20", "--left", "20", "--right", "15"]):
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as caught:
                    main(arguments)
                self.assertEqual(caught.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
