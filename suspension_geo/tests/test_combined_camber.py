"""Check angle superposition against the standalone FV and steering models."""

import contextlib
import io
import unittest

import numpy as np

from suspension_geo.combined_camber import calculate_combined_camber, main
from suspension_geo.fv import fv_camber_rc
from suspension_geo.we3D.steering_camber import calculate_steering_camber


class CombinedCamberTests(unittest.TestCase):
    def fv_reference(self, left_mm, right_mm):
        # Use the actual solver exposed by the user's FV example. Reset it so
        # the integration must match the same static configuration each time.
        solver = fv_camber_rc.solver
        with contextlib.redirect_stdout(io.StringIO()):
            try:
                baseline = solver.solve_by_travels(0.0, 0.0)
                state = solver.solve_by_travels(left_mm / 1000.0, right_mm / 1000.0)
            finally:
                solver.solve_by_travels(0.0, 0.0)
        return baseline, state

    def test_zero_inputs_have_zero_angle_change(self):
        state = calculate_combined_camber(0.0, 0.0, 0.0, 0.0)
        for side in ("left", "right"):
            for quantity in ("fv_camber_change", "steering_camber_change", "total_camber_change", "camber"):
                self.assertAlmostEqual(state[f"{quantity}_{side}_deg"], 0.0, places=10)

    def test_zero_steer_recovers_fv_camber_and_gain(self):
        for travels in ((12.0, -4.0), (-18.0, 14.0), (15.0, 15.0)):
            baseline, fv = self.fv_reference(*travels)
            combined = calculate_combined_camber(0.0, 0.0, *travels)
            for side in ("left", "right"):
                with self.subTest(travels=travels, side=side):
                    expected = fv[f"camber_{side}_deg"] - baseline[f"camber_{side}_deg"]
                    self.assertAlmostEqual(combined[f"total_camber_change_{side}_deg"], expected, places=9)
                    self.assertAlmostEqual(combined[f"camber_{side}_deg"], expected, places=9)
                    self.assertAlmostEqual(
                        combined[f"fv_camber_gain_{side}_deg_per_mm"],
                        fv[f"camber_gain_{side}_deg_per_mm"], places=8,
                    )
            np.testing.assert_allclose(combined["rc_pos"], fv["rc_pos"], atol=1e-10)

    def test_zero_travel_recovers_steering_model_with_static_camber(self):
        kwargs = {"static_camber_left_deg": -2.0, "static_camber_right_deg": -1.0}
        for headings in ((25.0, 18.0), (-18.0, -25.0)):
            steering = calculate_steering_camber(*headings, **kwargs)
            combined = calculate_combined_camber(*headings, 0.0, 0.0, **kwargs)
            for side in ("left", "right"):
                self.assertAlmostEqual(combined[f"camber_{side}_deg"], steering[f"camber_{side}_deg"], places=10)
                self.assertAlmostEqual(
                    combined[f"total_camber_change_{side}_deg"], steering[f"camber_change_{side}_deg"], places=10,
                )
                self.assertAlmostEqual(
                    combined[f"steering_camber_gain_{side}_deg_per_deg"],
                    steering[f"camber_gain_{side}_deg_per_deg"], places=10,
                )

    def test_finite_angle_changes_add_and_static_camber_is_counted_once(self):
        baseline, fv = self.fv_reference(10.0, -5.0)
        kwargs = {"static_camber_left_deg": -2.0, "static_camber_right_deg": -1.5}
        steering = calculate_steering_camber(25.0, 18.0, **kwargs)
        combined = calculate_combined_camber(25.0, 18.0, 10.0, -5.0, **kwargs)
        for side in ("left", "right"):
            fv_change = fv[f"camber_{side}_deg"] - baseline[f"camber_{side}_deg"]
            steering_change = steering[f"camber_change_{side}_deg"]
            self.assertAlmostEqual(combined[f"total_camber_change_{side}_deg"], fv_change + steering_change, places=10)
            self.assertAlmostEqual(
                combined[f"camber_{side}_deg"], kwargs[f"static_camber_{side}_deg"] + fv_change + steering_change,
                places=10,
            )

    def test_partial_gains_match_finite_differences_of_total_camber(self):
        inputs = [25.0, 18.0, 10.0, -5.0]
        kwargs = {"static_camber_left_deg": -2.0, "static_camber_right_deg": -1.0}
        combined = calculate_combined_camber(*inputs, **kwargs)
        for side_index, side in enumerate(("left", "right")):
            for index, step, key in (
                (side_index, 1e-4, f"steering_camber_gain_{side}_deg_per_deg"),
                (side_index + 2, 0.01, f"fv_camber_gain_{side}_deg_per_mm"),
            ):
                with self.subTest(side=side, gain=key):
                    samples = []
                    for offset in (-step, step):
                        target = inputs.copy()
                        target[index] += offset
                        samples.append(calculate_combined_camber(*target, **kwargs)[f"camber_{side}_deg"])
                    self.assertAlmostEqual(combined[key], (samples[1] - samples[0]) / (2 * step), places=7)

    def test_mirror_turn_and_travels_swap_camber_results(self):
        left_turn = calculate_combined_camber(
            25.0, 18.0, 10.0, -5.0, static_camber_left_deg=-2.0, static_camber_right_deg=-1.0,
        )
        right_turn = calculate_combined_camber(
            -18.0, -25.0, -5.0, 10.0, static_camber_left_deg=-1.0, static_camber_right_deg=-2.0,
        )
        for side, opposite in (("left", "right"), ("right", "left")):
            for quantity in ("camber", "total_camber_change", "fv_camber_change", "steering_camber_change"):
                self.assertAlmostEqual(left_turn[f"{quantity}_{side}_deg"], right_turn[f"{quantity}_{opposite}_deg"], places=9)
        self.assertAlmostEqual(left_turn["rc_pos"][0], -right_turn["rc_pos"][0], places=10)
        self.assertAlmostEqual(left_turn["rc_pos"][1], right_turn["rc_pos"][1], places=10)

    def test_calls_are_quiet_and_do_not_share_mutable_solver_geometry(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            first = calculate_combined_camber(25.0, 18.0, 10.0, -5.0)
            calculate_combined_camber(-18.0, -25.0, -17.0, 11.0)
            repeated = calculate_combined_camber(25.0, 18.0, 10.0, -5.0)
        self.assertEqual(output.getvalue(), "")
        for key in first:
            np.testing.assert_allclose(first[key], repeated[key], atol=1e-12)

    def test_invalid_inputs_and_unreachable_travel_raise_errors(self):
        for index in range(4):
            for invalid in (float("nan"), float("inf"), -float("inf")):
                target = [20.0, 15.0, 10.0, -5.0]
                target[index] = invalid
                with self.subTest(index=index, invalid=invalid), self.assertRaises(ValueError):
                    calculate_combined_camber(*target)
        with self.assertRaises(RuntimeError):
            calculate_combined_camber(20.0, 15.0, 1e6, 0.0)

    def test_cli_accepts_all_four_inputs_and_reports_contributions(self):
        with contextlib.redirect_stdout(io.StringIO()) as output:
            state = main(["--steer-left", "25", "--steer-right", "18",
                          "--travel-left-mm", "10", "--travel-right-mm", "-5"])
        self.assertEqual(state["steer_left_deg"], 25.0)
        self.assertEqual(state["steer_right_deg"], 18.0)
        self.assertAlmostEqual(state["travel_left_mm"], 10.0, places=7)
        self.assertAlmostEqual(state["travel_right_mm"], -5.0, places=7)
        self.assertIn("TOTAL camber change [deg]", output.getvalue())
        self.assertIn("FV local camber gain [deg/mm]", output.getvalue())
        self.assertIn("Steering local camber gain [deg/deg]", output.getvalue())
        self.assertIn("FV roll center:", output.getvalue())

    def test_cli_reports_bad_inputs_without_traceback(self):
        for arguments in (["--travel-left-mm", "nan"], ["--steer-left", "90"], ["--travel-right-mm", "1000000"]):
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()) as output:
                with self.assertRaises(SystemExit) as caught:
                    main(arguments)
                self.assertEqual(caught.exception.code, 2)
                self.assertNotIn("Traceback", output.getvalue())


if __name__ == "__main__":
    unittest.main()
