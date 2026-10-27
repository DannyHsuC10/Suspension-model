"""Verify cost rules and parity with the existing full FV mapping."""

import unittest
from unittest.mock import patch

import numpy as np

from suspension_geo import Geometric_Analysis as ga
from suspension_geo.car import Car, SuspensionF
from suspension_geo.fv.optimize_fv_geometry import (
    CAMBER_LIMIT_DEG, GRID_SIZE, RC_LIMIT_MM, START_HEIGHT_MM, Y_BEST_MM,
    CostScales, GeometryObjective, Settings, build_mapping, normalize_cost,
    optimize, summarize_cost, summarize_raw_cost,
)


class CostTests(unittest.TestCase):
    def setUp(self):
        self.mapping = {
            "rc_x_mm": np.array([[0.0, RC_LIMIT_MM, RC_LIMIT_MM + 1.0]]),
            "rc_y_mm": np.full((1, 3), 33.0),
            "camber_left_deg": np.array([[0.0, 2.0, 3.0]]),
            "camber_right_deg": np.array([[0.0, -2.0, -3.0]]),
        }

    def test_rc_penalty_applies_only_outside_radius(self):
        result = summarize_raw_cost(self.mapping, Settings())
        expected = (RC_LIMIT_MM ** 2 + 3.0 * (RC_LIMIT_MM + 1.0) ** 2) / 3.0
        self.assertAlmostEqual(result["rc_cost_raw_mm2"], expected)
        self.assertEqual(result["rc_penalty_points"], 1)

    def test_one_camber_exceedance_triples_the_whole_term(self):
        self.mapping["camber_left_deg"] = np.array([[0.0, 0.0, 3.0]])
        self.mapping["camber_right_deg"] = np.array([[0.0, 1.0, 1.0]])
        result = summarize_raw_cost(self.mapping, Settings())
        self.assertAlmostEqual(result["camber_cost_raw_deg2"], 3.0 * (9.0 + 1.0 + 1.0) / 6.0)
        self.assertEqual(result["camber_penalty_wheel_points"], 1)

    def test_exactly_two_degrees_does_not_trigger_camber_penalty(self):
        self.mapping["camber_left_deg"] = np.full((1, 3), 2.0)
        self.mapping["camber_right_deg"] = np.full((1, 3), -2.0)
        result = summarize_raw_cost(self.mapping, Settings())
        self.assertAlmostEqual(result["camber_cost_raw_deg2"], 4.0)
        self.assertEqual(result["camber_penalty_wheel_points"], 0)

    def test_reference_is_fixed_at_33_mm(self):
        self.mapping["rc_x_mm"][:] = 0.0
        self.assertEqual(Y_BEST_MM, 33.0)
        for height in (33.0, 35.0, 50.0):
            with self.subTest(height=height):
                self.mapping["rc_y_mm"][:] = height
                result = summarize_raw_cost(self.mapping, Settings())
                distance = abs(height - 33.0)
                factor = 3.0 if distance > RC_LIMIT_MM else 1.0
                self.assertAlmostEqual(result["rc_rms_distance_mm"], distance)
                self.assertAlmostEqual(result["rc_cost_raw_mm2"], factor * distance ** 2)

    def test_reference_does_not_move_with_starting_height(self):
        self.assertEqual(START_HEIGHT_MM, 35)
        self.mapping["rc_x_mm"][:] = 0.0
        self.mapping["rc_y_mm"][:] = 35.0
        with patch("suspension_geo.fv.optimize_fv_geometry.START_HEIGHT_MM", -5):
            result = summarize_raw_cost(self.mapping, Settings())
        self.assertAlmostEqual(result["rc_cost_raw_mm2"], 4.0)

    def test_vertical_radius_penalty_is_centered_at_33_mm(self):
        self.mapping["rc_x_mm"][:] = 0.0
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.mapping["rc_y_mm"][:] = 33.0 + direction * np.array([0.0, RC_LIMIT_MM, RC_LIMIT_MM + 1.0])
                result = summarize_raw_cost(self.mapping, Settings())
                self.assertAlmostEqual(result["rc_cost_raw_mm2"], (RC_LIMIT_MM ** 2 + 3.0 * (RC_LIMIT_MM + 1.0) ** 2) / 3.0)
                self.assertEqual(result["rc_penalty_points"], 1)

    def test_weights_combine_the_two_normalized_costs(self):
        result = summarize_cost(self.mapping, Settings(rc_weight=2.0, camber_weight=0.5), CostScales(100.0, 25.0, 4.0))
        expected_x = (RC_LIMIT_MM ** 2 + 3.0 * (RC_LIMIT_MM + 1.0) ** 2) / 3.0 / 100.0
        expected_rc = expected_x / 6.0
        self.assertAlmostEqual(result["rc_cost"], expected_rc)
        self.assertAlmostEqual(result["camber_cost"], 13.0 / 4.0)
        self.assertAlmostEqual(result["total_cost"], 0.8 * expected_rc + 0.2 * 13.0 / 4.0)
        self.assertAlmostEqual(result["total_cost"], result["rc_contribution"] + result["camber_contribution"])

    def test_invalid_settings_are_rejected(self):
        for kwargs in ({"steps": 1}, {"rc_weight": -1}, {"rc_weight": 0, "camber_weight": 0},
                       {"rc_height_weight": -1}, {"rc_distance_weight": 0, "rc_height_weight": 0}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                Settings(**kwargs)


class NormalizationTests(unittest.TestCase):
    @staticmethod
    def raw(distance, height, camber):
        return {"rc_cost_raw_mm2": distance, "rc_height_cost_raw_mm2": height,
                "camber_cost_raw_deg2": camber}

    def test_initial_design_has_unit_terms_and_equal_contributions(self):
        objective = GeometryObjective(Settings(steps=3))
        self.assertAlmostEqual(objective([35, 20]), 1.0)
        initial = objective.cache[(35, 1000)]
        self.assertAlmostEqual(initial["rc_cost"], 1.0)
        self.assertAlmostEqual(initial["rc_distance_cost"], 1.0)
        self.assertAlmostEqual(initial["rc_height_cost"], 1.0)
        self.assertAlmostEqual(initial["camber_cost"], 1.0)
        self.assertAlmostEqual(initial["rc_contribution"], 0.5)
        self.assertAlmostEqual(initial["camber_contribution"], 0.5)
        self.assertAlmostEqual(initial["rc_distance_contribution"], 1.0 / 12.0)
        self.assertAlmostEqual(initial["rc_height_contribution"], 5.0 / 12.0)

    def test_scales_are_independent_of_evaluation_order_and_do_not_move(self):
        settings = Settings(steps=3)
        initial_raw = summarize_raw_cost(build_mapping(35, 1000, 3), settings)
        objective = GeometryObjective(settings)
        scales = objective.scales
        objective([50, 50])
        objective([-5, 15])
        self.assertIs(objective.scales, scales)
        self.assertAlmostEqual(scales.rc_distance_mm2, initial_raw["rc_cost_raw_mm2"])
        self.assertAlmostEqual(scales.rc_height_mm2, initial_raw["rc_height_cost_raw_mm2"])
        self.assertAlmostEqual(scales.camber_deg2, initial_raw["camber_cost_raw_deg2"])
        self.assertAlmostEqual(objective([35, 20]), 1.0)

    def test_normalization_preserves_improvements_and_degradations(self):
        scales = CostScales(100.0, 20.0, 4.0)
        settings = Settings()
        better = normalize_cost(self.raw(50.0, 10.0, 2.0), settings, scales)
        worse = normalize_cost(self.raw(200.0, 40.0, 8.0), settings, scales)
        self.assertAlmostEqual(better["total_cost"], 0.5)
        self.assertAlmostEqual(worse["total_cost"], 2.0)

    def test_units_cancel_against_the_same_fixed_baseline(self):
        settings = Settings()
        base = normalize_cost(self.raw(50.0, 10.0, 8.0), settings, CostScales(100.0, 20.0, 4.0))
        rescaled = normalize_cost(self.raw(50e6, 10e6, 8e3), settings, CostScales(100e6, 20e6, 4e3))
        self.assertAlmostEqual(base["total_cost"], rescaled["total_cost"])

    def test_zero_baseline_uses_physical_tolerance_without_division_by_zero(self):
        zero = self.raw(0.0, 0.0, 0.0)
        scales = CostScales.from_initial(zero)
        self.assertEqual(scales.rc_distance_mm2, RC_LIMIT_MM ** 2)
        self.assertEqual(scales.rc_height_mm2, RC_LIMIT_MM ** 2)
        self.assertEqual(scales.camber_deg2, CAMBER_LIMIT_DEG ** 2)
        self.assertEqual(normalize_cost(zero, Settings(), scales)["total_cost"], 0.0)

    def test_penalties_remain_active_after_normalization(self):
        mapping = {
            "rc_x_mm": np.array([[1.5 * RC_LIMIT_MM]]),
            "rc_y_mm": np.array([[33.0]]),
            "camber_left_deg": np.array([[3.0]]),
            "camber_right_deg": np.array([[-3.0]]),
        }
        metrics = summarize_cost(mapping, Settings(), CostScales(RC_LIMIT_MM ** 2, RC_LIMIT_MM ** 2, 4.0))
        self.assertAlmostEqual(metrics["rc_distance_cost"], 3.0 * 1.5 ** 2)
        self.assertAlmostEqual(metrics["rc_cost"], 3.0 * 1.5 ** 2 / 6.0)
        self.assertAlmostEqual(metrics["camber_cost"], 3.0 * 3.0 ** 2 / 4.0)

    def test_raising_camber_weight_changes_relative_contribution(self):
        metrics = normalize_cost(self.raw(100.0, 20.0, 4.0),
                                 Settings(rc_weight=1.0, camber_weight=2.0), CostScales(100.0, 20.0, 4.0))
        self.assertAlmostEqual(metrics["rc_contribution"], 1.0 / 3.0)
        self.assertAlmostEqual(metrics["camber_contribution"], 2.0 / 3.0)
        self.assertAlmostEqual(metrics["total_cost"], 1.0)

    def test_nonpositive_or_nonfinite_scales_are_rejected(self):
        for x, y, camber in ((0.0, 1.0, 1.0), (1.0, 0.0, 1.0), (1.0, 1.0, -1.0), (float("nan"), 1.0, 1.0)):
            with self.subTest(x=x, y=y, camber=camber), self.assertRaises(ValueError):
                CostScales(x, y, camber)

    def test_small_height_error_is_not_drowned_out_by_large_horizontal_units(self):
        scales = CostScales(1e6, 1.0, 4.0)
        low_x_bad_height = normalize_cost(self.raw(0.3e6, 4.0, 4.0), Settings(), scales)
        larger_x_good_height = normalize_cost(self.raw(1.2e6, 0.5, 4.0), Settings(), scales)
        self.assertLess(larger_x_good_height["total_cost"], low_x_bad_height["total_cost"])

    def test_increasing_height_weight_penalizes_height_error_more(self):
        raw = self.raw(100.0, 80.0, 4.0)
        scales = CostScales(100.0, 20.0, 4.0)
        equal = normalize_cost(raw, Settings(rc_height_weight=1.0), scales)
        prioritized = normalize_cost(raw, Settings(rc_height_weight=5.0), scales)
        self.assertGreater(prioritized["rc_cost"], equal["rc_cost"])

    def test_pure_height_target_is_independent_of_horizontal_penalty_trigger(self):
        base = {"rc_x_mm": np.array([[0.0]]), "rc_y_mm": np.array([[34.0]]),
                "camber_left_deg": np.array([[0.0]]), "camber_right_deg": np.array([[0.0]])}
        no_trigger = summarize_raw_cost(base, Settings())
        base["rc_x_mm"][:] = RC_LIMIT_MM * 2.0
        triggered = summarize_raw_cost(base, Settings())
        self.assertEqual(no_trigger["rc_height_cost_raw_mm2"], 1.0)
        self.assertEqual(triggered["rc_height_cost_raw_mm2"], 1.0)
        self.assertGreater(triggered["rc_cost_raw_mm2"], no_trigger["rc_cost_raw_mm2"])

    def test_high_rc_design_is_worse_than_near_target_under_default_objective(self):
        objective = GeometryObjective(Settings(steps=30))
        near_target = objective([35, 24])  # RC=35 mm, FVSA=1200 mm
        high = objective([50, 24])
        self.assertGreater(high, near_target)
        self.assertGreater(objective.cache[(50, 1200)]["rc_y_error_rms_mm"],
                           objective.cache[(35, 1200)]["rc_y_error_rms_mm"])


class MappingTests(unittest.TestCase):
    def test_broadcast_mapping_matches_existing_wheel_solver(self):
        for height, fvsa in ((35, 1000), (-5, 750), (50, 2500)):
            with self.subTest(height=height, fvsa=fvsa):
                steps = 5
                actual = build_mapping(height, fvsa, steps)
                suspension = SuspensionF()
                suspension.h_rc = height / 1000.0
                suspension.fvsa = fvsa / 1000.0
                helper = ga.SusGeometryHelper(Car(), suspension)
                right = ga.SuspensionSide("Right")
                right.setup_right_side(helper.calc_static_points())
                left = ga.SuspensionSide("Left", is_left=True)
                left.setup_from_right_template(right)
                limits = ga.calculate_theta_limits(right, suspension.front_view_target_travel)
                angles = np.linspace(*limits, steps)
                y0 = right.gnd.y
                for i, theta_l in enumerate(angles):
                    for j, theta_r in enumerate(angles):
                        left.update_kinematics(theta_l, helper)
                        right.update_kinematics(theta_r, helper)
                        ground = right.gnd.pos - left.gnd.pos
                        rc = helper.line_intersection(left.gnd, left.IC, right.gnd, right.IC) * 1000.0
                        expected = {
                            "rc_x_mm": rc[0], "rc_y_mm": rc[1],
                            "camber_left_deg": left.calc_camber_angle(ground),
                            "camber_right_deg": right.calc_camber_angle(ground),
                            "travel_left_mm": (left.gnd.y - y0) * 1000.0,
                            "travel_right_mm": (right.gnd.y - y0) * 1000.0,
                            "roll_deg": np.degrees(np.arctan2(ground[1], ground[0])),
                        }
                        for key, value in expected.items():
                            np.testing.assert_allclose(actual[key][i, j], value, atol=1e-7, rtol=1e-10, err_msg=key)

    def test_search_respects_lattice_and_includes_initial_design(self):
        objective, initial, result = optimize(Settings(steps=3), maxiter=1, verify_grid=False)
        self.assertEqual(GRID_SIZE, 2016)
        self.assertEqual((initial["height_mm"], initial["fvsa_mm"]), (35, 1000))
        self.assertEqual(objective.history[0]["phase"], "initial")
        for height, fvsa in objective.cache:
            self.assertIsInstance(height, int)
            self.assertTrue(-5 <= height <= 50)
            self.assertTrue(750 <= fvsa <= 2500)
            self.assertEqual(fvsa % 50, 0)
        self.assertLessEqual(objective.best["total_cost"], initial["total_cost"])
        self.assertEqual(result.nit, 1)


if __name__ == "__main__":
    unittest.main()
