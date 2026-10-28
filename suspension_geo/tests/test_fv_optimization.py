"""Verify cost rules and parity with the existing full FV mapping."""

import unittest
from unittest.mock import patch

import numpy as np

from suspension_geo import Geometric_Analysis as ga
from suspension_geo.car import Car, SuspensionF
from suspension_geo.analysis_settings import AnalysisSettings
from suspension_geo.fv.optimize_fv_geometry import (
    CAMBER_LIMIT_DEG, FVSA_STEP_MM, GRID_SIZE, RC_LIMIT_MM,
    START_FVSA_MM, START_HEIGHT_MM, Y_BEST_MM,
    CostScales, GeometryObjective, Settings, build_mapping, normalize_cost,
    optimize, summarize_cost, summarize_raw_cost,
)


class CostTests(unittest.TestCase):
    def setUp(self):
        self.mapping = {
            "rc_x_mm": np.array([[0.0, RC_LIMIT_MM, RC_LIMIT_MM + 1.0]]),
            "rc_y_mm": np.full((1, 3), Y_BEST_MM),
            "camber_left_deg": np.array([[0.0, 2.0, 3.0]]),
            "camber_right_deg": np.array([[0.0, -2.0, -3.0]]),
        }

    def test_raw_rc_errors_exclude_the_global_penalty(self):
        result = summarize_raw_cost(self.mapping, Settings())
        expected = (RC_LIMIT_MM ** 2 + (RC_LIMIT_MM + 1.0) ** 2) / 3.0
        self.assertAlmostEqual(result["rc_cost_raw_mm2"], expected)
        self.assertEqual(result["rc_penalty_points"], 1)
        self.assertEqual(result["rc_penalty_factor"], 3.0)

    def test_exactly_at_radius_does_not_trigger_rc_penalty(self):
        self.mapping["rc_x_mm"] = np.array([[0.0, 0.6 * RC_LIMIT_MM, RC_LIMIT_MM]])
        self.mapping["rc_y_mm"] = Y_BEST_MM + np.array([[0.0, 0.8 * RC_LIMIT_MM, 0.0]])
        result = summarize_cost(self.mapping, Settings(), CostScales(100.0, 25.0, 4.0))
        self.assertEqual(result["rc_penalty_points"], 0)
        self.assertEqual(result["rc_penalty_factor"], 1.0)
        self.assertAlmostEqual(result["rc_cost"], result["rc_cost_unpenalized"])

    def test_one_rc_exceedance_triples_distance_and_height_contributions(self):
        self.mapping["rc_x_mm"] = np.array([[0.0, 1.0, 0.0]])
        self.mapping["rc_y_mm"] = Y_BEST_MM + np.array([[2.0, 0.0, RC_LIMIT_MM + 1.0]])
        settings = Settings(rc_weight=2.0, camber_weight=0.5,
                            rc_distance_weight=2.0, rc_height_weight=3.0)
        result = summarize_cost(self.mapping, settings, CostScales(100.0, 25.0, 4.0))
        distance = (1.0 + 4.0 + (RC_LIMIT_MM + 1.0) ** 2) / 3.0 / 100.0
        height = (4.0 + (RC_LIMIT_MM + 1.0) ** 2) / 3.0 / 25.0
        unpenalized = 0.4 * distance + 0.6 * height
        self.assertEqual(result["rc_penalty_points"], 1)
        self.assertAlmostEqual(result["rc_cost_unpenalized"], unpenalized)
        self.assertAlmostEqual(result["rc_cost"], 3.0 * unpenalized)
        self.assertAlmostEqual(result["rc_distance_contribution"], 0.8 * 3.0 * 0.4 * distance)
        self.assertAlmostEqual(result["rc_height_contribution"], 0.8 * 3.0 * 0.6 * height)
        self.assertAlmostEqual(result["rc_contribution"],
                               result["rc_distance_contribution"] + result["rc_height_contribution"])
        self.assertAlmostEqual(result["camber_cost"], 13.0 / 4.0)
        self.assertAlmostEqual(result["total_cost"], 0.8 * 3.0 * unpenalized + 0.2 * 13.0 / 4.0)

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

    def test_reference_is_fixed_at_configured_target(self):
        self.mapping["rc_x_mm"][:] = 0.0
        for height in (Y_BEST_MM, Y_BEST_MM + 2.0, Y_BEST_MM + 20.0):
            with self.subTest(height=height):
                self.mapping["rc_y_mm"][:] = height
                result = summarize_raw_cost(self.mapping, Settings())
                distance = abs(height - Y_BEST_MM)
                self.assertAlmostEqual(result["rc_rms_distance_mm"], distance)
                self.assertAlmostEqual(result["rc_cost_raw_mm2"], distance ** 2)

    def test_reference_does_not_move_with_starting_height(self):
        self.mapping["rc_x_mm"][:] = 0.0
        self.mapping["rc_y_mm"][:] = Y_BEST_MM + 2.0
        with patch("suspension_geo.fv.optimize_fv_geometry.START_HEIGHT_MM", -5):
            result = summarize_raw_cost(self.mapping, Settings())
        self.assertAlmostEqual(result["rc_cost_raw_mm2"], 4.0)

    def test_vertical_radius_penalty_is_centered_at_fixed_target(self):
        self.mapping["rc_x_mm"][:] = 0.0
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.mapping["rc_y_mm"][:] = Y_BEST_MM + direction * np.array([0.0, RC_LIMIT_MM, RC_LIMIT_MM + 1.0])
                result = summarize_raw_cost(self.mapping, Settings())
                self.assertAlmostEqual(result["rc_cost_raw_mm2"], (RC_LIMIT_MM ** 2 + (RC_LIMIT_MM + 1.0) ** 2) / 3.0)
                self.assertEqual(result["rc_penalty_points"], 1)
                self.assertEqual(result["rc_penalty_factor"], 3.0)

    def test_weights_combine_the_two_normalized_costs(self):
        result = summarize_cost(self.mapping, Settings(rc_weight=2.0, camber_weight=0.5, rc_height_weight=5.0), CostScales(100.0, 25.0, 4.0))
        expected_x = (RC_LIMIT_MM ** 2 + (RC_LIMIT_MM + 1.0) ** 2) / 3.0 / 100.0
        expected_rc = 3.0 * expected_x / 6.0
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
    def raw(distance, height, camber, rc_penalty=1.0):
        return {"rc_cost_raw_mm2": distance, "rc_height_cost_raw_mm2": height,
                "camber_cost_raw_deg2": camber, "rc_penalty_factor": rc_penalty}

    def test_initial_design_has_unit_base_terms_and_retains_rc_penalty(self):
        settings = Settings(steps=3)
        objective = GeometryObjective(settings)
        factor = objective.initial_raw["rc_penalty_factor"]
        self.assertAlmostEqual(objective([START_HEIGHT_MM, START_FVSA_MM // FVSA_STEP_MM]), 0.5 * factor + 0.5)
        initial = objective.cache[(START_HEIGHT_MM, START_FVSA_MM)]
        self.assertAlmostEqual(initial["rc_cost"], factor)
        self.assertAlmostEqual(initial["rc_cost_unpenalized"], 1.0)
        self.assertAlmostEqual(initial["rc_distance_cost"], 1.0)
        self.assertAlmostEqual(initial["rc_height_cost"], 1.0)
        self.assertAlmostEqual(initial["camber_cost"], 1.0)
        self.assertAlmostEqual(initial["rc_contribution"], 0.5 * factor)
        self.assertAlmostEqual(initial["camber_contribution"], 0.5)
        distance_weight, height_weight = settings.normalized_rc_component_weights
        self.assertAlmostEqual(initial["rc_distance_contribution"], 0.5 * factor * distance_weight)
        self.assertAlmostEqual(initial["rc_height_contribution"], 0.5 * factor * height_weight)

    def test_violating_initial_design_does_not_normalize_away_rc_penalty(self):
        mapping = {"rc_x_mm": np.array([[RC_LIMIT_MM + 1.0, 1.0]]),
                   "rc_y_mm": np.full((1, 2), Y_BEST_MM + 2.0),
                   "camber_left_deg": np.ones((1, 2)), "camber_right_deg": np.ones((1, 2))}
        with patch("suspension_geo.fv.optimize_fv_geometry.build_mapping", return_value=mapping):
            objective = GeometryObjective(Settings(steps=2))
            total = objective([START_HEIGHT_MM, START_FVSA_MM // FVSA_STEP_MM])
        initial = objective.cache[(START_HEIGHT_MM, START_FVSA_MM)]
        self.assertAlmostEqual(objective.scales.rc_distance_mm2, ((RC_LIMIT_MM + 1.0) ** 2 + 1.0) / 2.0 + 4.0)
        self.assertAlmostEqual(objective.scales.rc_height_mm2, 4.0)
        self.assertAlmostEqual(initial["rc_distance_cost"], 1.0)
        self.assertAlmostEqual(initial["rc_height_cost"], 1.0)
        self.assertAlmostEqual(initial["rc_cost"], 3.0)
        self.assertAlmostEqual(total, 2.0)

    def test_scales_are_independent_of_evaluation_order_and_do_not_move(self):
        settings = Settings(steps=3)
        initial_raw = summarize_raw_cost(build_mapping(START_HEIGHT_MM, START_FVSA_MM, 3), settings)
        objective = GeometryObjective(settings)
        scales = objective.scales
        objective([50, 50])
        objective([-5, 15])
        self.assertIs(objective.scales, scales)
        self.assertAlmostEqual(scales.rc_distance_mm2, initial_raw["rc_cost_raw_mm2"])
        self.assertAlmostEqual(scales.rc_height_mm2, initial_raw["rc_height_cost_raw_mm2"])
        self.assertAlmostEqual(scales.camber_deg2, initial_raw["camber_cost_raw_deg2"])
        self.assertAlmostEqual(objective([START_HEIGHT_MM, START_FVSA_MM // FVSA_STEP_MM]),
                               0.5 * initial_raw["rc_penalty_factor"] + 0.5)

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
            "rc_y_mm": np.array([[Y_BEST_MM + 10.0]]),
            "camber_left_deg": np.array([[3.0]]),
            "camber_right_deg": np.array([[-3.0]]),
        }
        metrics = summarize_cost(mapping, Settings(rc_height_weight=5.0), CostScales(RC_LIMIT_MM ** 2, RC_LIMIT_MM ** 2, 4.0))
        distance = 1.5 ** 2 + 10.0 ** 2 / RC_LIMIT_MM ** 2
        height = 10.0 ** 2 / RC_LIMIT_MM ** 2
        self.assertAlmostEqual(metrics["rc_distance_cost"], distance)
        self.assertAlmostEqual(metrics["rc_cost"], 3.0 * (distance + 5.0 * height) / 6.0)
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
        base = {"rc_x_mm": np.array([[0.0]]), "rc_y_mm": np.array([[Y_BEST_MM + 1.0]]),
                "camber_left_deg": np.array([[0.0]]), "camber_right_deg": np.array([[0.0]])}
        no_trigger = summarize_raw_cost(base, Settings())
        base["rc_x_mm"][:] = RC_LIMIT_MM * 2.0
        triggered = summarize_raw_cost(base, Settings())
        self.assertEqual(no_trigger["rc_height_cost_raw_mm2"], 1.0)
        self.assertEqual(triggered["rc_height_cost_raw_mm2"], 1.0)
        self.assertGreater(triggered["rc_cost_raw_mm2"], no_trigger["rc_cost_raw_mm2"])

    def test_global_rc_penalty_changes_objective_ranking_with_one_bad_point(self):
        def mapping(x, y_error):
            return {"rc_x_mm": np.array([x], dtype=float),
                    "rc_y_mm": np.full((1, 2), Y_BEST_MM + y_error),
                    "camber_left_deg": np.ones((1, 2)), "camber_right_deg": np.ones((1, 2))}

        baseline = mapping([10.0, 10.0], 10.0)
        inside = mapping([40.0, 40.0], 10.0)
        outside = mapping([RC_LIMIT_MM + 1.0, 0.0], 5.0)
        with patch("suspension_geo.fv.optimize_fv_geometry.build_mapping",
                   side_effect=[baseline, inside, outside]):
            objective = GeometryObjective(Settings(steps=2, camber_weight=0.0))
            inside_cost = objective([1, 20])
            outside_cost = objective([2, 20])
        good = objective.cache[(1, 1000)]
        bad = objective.cache[(2, 1000)]
        self.assertEqual(bad["rc_penalty_points"], 1)
        self.assertLess(bad["rc_cost_unpenalized"], good["rc_cost_unpenalized"])
        self.assertGreater(outside_cost, inside_cost)
        self.assertEqual(objective.best["height_mm"], 1)

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
                limits = ga.calculate_theta_limits(right, AnalysisSettings().front_view_target_travel)
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
        self.assertEqual((initial["height_mm"], initial["fvsa_mm"]), (START_HEIGHT_MM, START_FVSA_MM))
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
