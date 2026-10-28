"""Check displacement signs, camber reference frames, axle setup and CSV output."""

import contextlib
import csv
import io
import tempfile
import unittest
from pathlib import Path

import numpy as np

from suspension_geo.car import Car, SuspensionF, SuspensionR
from suspension_geo.combined_camber import _build_fv_solver
from suspension_geo.data_signals_camber import (
    CamberSignalAnalyzer, WHEELS, analyze_csv, front_camber_comparison,
    front_comparison_summary,
)
from suspension_geo.we3D.steering_camber import calculate_steering_camber


def signal_row(**overrides):
    row = {"time_s": 0.0, "roll_front_deg": 0.0, "roll_rear_deg": 0.0}
    for wheel in WHEELS:
        row.update({f"zs_{wheel}_mm": 0.0, f"zu_{wheel}_mm": 0.0, f"steer_{wheel}_deg": 0.0})
    row.update(overrides)
    return row


class DataSignalsCamberTests(unittest.TestCase):
    def test_inside_outside_follows_acceleration_even_when_steering_disagrees(self):
        for ay, outside, inside, expected in (
            (4.0, "FR", "FL", (-2.5, 0.5, -3.0)),
            (-4.0, "FL", "FR", (0.5, -2.5, 3.0)),
        ):
            with self.subTest(ay=ay):
                result = front_camber_comparison({
                    "ay_m_s2": ay, "steer_FL_deg": -ay,
                    "camber_FL_deg": 0.5, "camber_FR_deg": -2.5,
                })
                self.assertEqual(result["front_outside_wheel"], outside)
                self.assertEqual(result["front_inside_wheel"], inside)
                for name, value in zip(("outside", "inside", "outside_minus_inside"), expected):
                    self.assertEqual(result[f"camber_front_{name}_deg"], value)

    def test_zero_missing_and_invalid_acceleration_leave_comparison_unclassified(self):
        for ay in (0.0, -0.0, "", "bad", None, np.nan, np.inf, -np.inf):
            with self.subTest(ay=ay):
                result = front_camber_comparison({"ay_m_s2": ay, "camber_FL_deg": 1, "camber_FR_deg": 2})
                self.assertEqual(result["front_outside_wheel"], "")
                self.assertEqual(result["front_inside_wheel"], "")
                self.assertTrue(np.isnan(result["camber_front_outside_deg"]))
                self.assertTrue(np.isnan(result["camber_front_inside_deg"]))
                self.assertTrue(np.isnan(result["camber_front_outside_minus_inside_deg"]))
        self.assertEqual(front_camber_comparison({})["front_outside_wheel"], "")

    def test_comparison_preserves_valid_tire_but_statistics_require_pairs(self):
        rows = [front_camber_comparison(row) for row in (
            {"ay_m_s2": 5, "camber_FL_deg": 1, "camber_FR_deg": -3},
            {"ay_m_s2": -5, "camber_FL_deg": -1, "camber_FR_deg": 3},
            {"ay_m_s2": 5, "camber_FL_deg": np.nan, "camber_FR_deg": -20},
            {"ay_m_s2": 0, "camber_FL_deg": 100, "camber_FR_deg": 100},
        )]
        self.assertEqual(rows[2]["camber_front_outside_deg"], -20)
        self.assertTrue(np.isnan(rows[2]["camber_front_outside_minus_inside_deg"]))
        summary = front_comparison_summary(rows)
        self.assertEqual(summary["paired_rows"], 2)
        self.assertEqual(summary["turning_rows"], 3)
        self.assertEqual(summary["unclassified_rows"], 1)
        self.assertEqual(summary["outside"]["mean_deg"], -2)
        self.assertEqual(summary["inside"]["mean_deg"], 2)
        self.assertEqual(summary["outside_minus_inside"]["mean_deg"], -4)

    def test_zero_motion_preserves_independent_static_cambers(self):
        static = (-2.0, -1.5, -1.0, -0.5)
        state = CamberSignalAnalyzer(static).analyze_row(signal_row())
        for wheel, camber in zip(WHEELS, static):
            self.assertAlmostEqual(state[f"camber_gain_{wheel}_deg"], 0.0, places=10)
            self.assertAlmostEqual(state[f"camber_{wheel}_deg"], camber, places=10)

    def test_sprung_minus_unsprung_is_negated_for_wheel_compression(self):
        row = signal_row(zs_FL_mm=-10.0, zu_FL_mm=2.0, zs_FR_mm=6.0, zu_FR_mm=-1.0,
                         zs_RL_mm=-8.0, zu_RL_mm=3.0, zs_RR_mm=4.0, zu_RR_mm=-2.0)
        state = CamberSignalAnalyzer().analyze_row(row)
        for wheel, expected in zip(WHEELS, (12.0, -7.0, 11.0, -6.0)):
            self.assertEqual(state[f"wheel_travel_{wheel}_mm"], expected)
            self.assertEqual(state[f"relative_displacement_{wheel}_mm"], -expected)
        self.assertLess(state["fv_camber_gain_FL_deg"], 0.0)
        self.assertGreater(state["fv_camber_gain_FR_deg"], 0.0)

    def test_fv_body_camber_matches_tire_measured_against_horizontal_chassis(self):
        row = signal_row(zs_FL_mm=-12.0, zs_FR_mm=7.0, zs_RL_mm=-9.0, zs_RR_mm=5.0)
        state = CamberSignalAnalyzer().analyze_row(row)
        for suspension, wheels in ((SuspensionF(), ("FL", "FR")), (SuspensionR(), ("RL", "RR"))):
            solver = _build_fv_solver(Car(), suspension)
            with contextlib.redirect_stdout(io.StringIO()):
                solver.solve_by_travels(*(state[f"wheel_travel_{wheel}_mm"] / 1000 for wheel in wheels))
            for geometry, wheel in zip((solver.left_sus, solver.right_sus), wheels):
                self.assertAlmostEqual(
                    state[f"fv_camber_gain_{wheel}_deg"], geometry.calc_camber_angle(np.array([1.0, 0.0])),
                    places=10,
                )

    def test_other_wheel_travel_does_not_add_contact_line_roll_to_body_gain(self):
        analyzer = CamberSignalAnalyzer()
        first = analyzer.analyze_row(signal_row(zs_FL_mm=-10.0, zs_FR_mm=15.0))
        second = analyzer.analyze_row(signal_row(zs_FL_mm=-10.0, zs_FR_mm=-15.0))
        self.assertNotAlmostEqual(first["fv_contact_line_front_deg"], second["fv_contact_line_front_deg"])
        self.assertAlmostEqual(first["fv_camber_gain_FL_deg"], second["fv_camber_gain_FL_deg"], places=9)

    def test_roll_applies_once_with_opposite_signs_and_separate_axles(self):
        state = CamberSignalAnalyzer().analyze_row(signal_row(roll_front_deg=2.0, roll_rear_deg=-3.0))
        for axle, expected in (("front", 2.0), ("rear", -3.0)):
            self.assertEqual(state[f"tire_roll_{axle}_deg"], 0.0)
            self.assertEqual(state[f"total_roll_{axle}_deg"], expected)
        for wheel, expected in zip(WHEELS, (-2.0, 2.0, 3.0, -3.0)):
            self.assertAlmostEqual(state[f"camber_gain_{wheel}_deg"], 0.0, places=10)
            self.assertAlmostEqual(state[f"camber_{wheel}_deg"], expected, places=10)
            self.assertEqual(state[f"roll_camber_{wheel}_deg"], expected)

    def test_tire_roll_uses_each_track_and_adds_to_suspension_roll_once(self):
        for direction in (1.0, -1.0):
            with self.subTest(direction=direction):
                # Equal zs and zu isolate tire roll from suspension travel.
                offsets = (12.5 * direction, -12.5 * direction,
                           -12.4 * direction, 12.4 * direction)
                row = signal_row(roll_front_deg=2.0, roll_rear_deg=-3.0)
                for wheel, offset in zip(WHEELS, offsets):
                    row[f"zs_{wheel}_mm"] = row[f"zu_{wheel}_mm"] = offset
                state = CamberSignalAnalyzer().analyze_row(row)
                # 25 / 1250 and 24.8 / 1240 are both 0.02 rad, with opposite signs.
                front_tire = direction * 1.1459155902616465
                rear_tire = -front_tire
                self.assertTrue(state["front_solver_ok"])
                self.assertTrue(state["rear_solver_ok"])
                self.assertAlmostEqual(state["tire_roll_front_deg"], front_tire, places=10)
                self.assertAlmostEqual(state["tire_roll_rear_deg"], rear_tire, places=10)
                self.assertAlmostEqual(state["total_roll_front_deg"], 2.0 + front_tire, places=10)
                self.assertAlmostEqual(state["total_roll_rear_deg"], -3.0 + rear_tire, places=10)
                for wheel, expected in zip(WHEELS, (-(2.0 + front_tire), 2.0 + front_tire,
                                                    3.0 - rear_tire, -3.0 + rear_tire)):
                    self.assertAlmostEqual(state[f"camber_gain_{wheel}_deg"], 0.0, places=10)
                    self.assertAlmostEqual(state[f"roll_camber_{wheel}_deg"], expected, places=10)
                    self.assertAlmostEqual(state[f"camber_{wheel}_deg"], expected, places=10)

    def test_equal_tire_displacements_produce_no_roll(self):
        row = signal_row(roll_front_deg=1.0, roll_rear_deg=-2.0)
        for wheel, offset in zip(WHEELS, (15.0, 15.0, -8.0, -8.0)):
            row[f"zs_{wheel}_mm"] = row[f"zu_{wheel}_mm"] = offset
        state = CamberSignalAnalyzer().analyze_row(row)
        self.assertEqual(state["tire_roll_front_deg"], 0.0)
        self.assertEqual(state["tire_roll_rear_deg"], 0.0)
        self.assertEqual(state["total_roll_front_deg"], 1.0)
        self.assertEqual(state["total_roll_rear_deg"], -2.0)

    def test_cached_suspension_gain_does_not_cache_tire_roll(self):
        analyzer = CamberSignalAnalyzer()
        first = analyzer.analyze_row(signal_row(zs_FL_mm=-10.0))
        # Keep wheel travel unchanged but move both sprung and unsprung left corners.
        second = analyzer.analyze_row(signal_row(zs_FL_mm=2.5, zu_FL_mm=12.5,
                                                zs_RL_mm=-12.4, zu_RL_mm=-12.4))
        self.assertEqual(len(analyzer._cache["front"]), 1)
        self.assertEqual(len(analyzer._cache["rear"]), 1)
        for wheel, delta in zip(WHEELS, (-0.5729577951308232, 0.5729577951308232,
                                         0.5729577951308232, -0.5729577951308232)):
            self.assertAlmostEqual(first[f"camber_gain_{wheel}_deg"], second[f"camber_gain_{wheel}_deg"])
            self.assertAlmostEqual(second[f"camber_{wheel}_deg"] - first[f"camber_{wheel}_deg"], delta)

    def test_invalid_total_roll_preserves_body_gain_and_other_axle(self):
        state = CamberSignalAnalyzer().analyze_row(
            signal_row(zs_FL_mm=20.0, zu_FL_mm=20.0, roll_front_deg=89.5),
        )
        self.assertFalse(state["front_solver_ok"])
        self.assertTrue(state["rear_solver_ok"])
        self.assertTrue(np.isfinite(state["camber_gain_FL_deg"]))
        self.assertTrue(np.isnan(state["camber_FL_deg"]))
        self.assertIn("total_roll_front_deg", state["front_error"])

    def test_tire_roll_rejects_invalid_track_widths(self):
        for attribute, value in (("tf", 0.0), ("tr", -1.24), ("tf", np.nan), ("tr", np.inf)):
            with self.subTest(attribute=attribute, value=value):
                car = Car()
                setattr(car, attribute, value)
                with self.assertRaisesRegex(ValueError, "track widths"):
                    CamberSignalAnalyzer(car=car)

    def test_front_steering_matches_standalone_and_static_is_counted_once(self):
        static = (-2.0, -1.0, -3.0, -4.0)
        state = CamberSignalAnalyzer(static).analyze_row(
            signal_row(steer_FL_deg=25.0, steer_FR_deg=18.0, roll_front_deg=1.5),
        )
        steering = calculate_steering_camber(
            25.0, 18.0, static_camber_left_deg=static[0], static_camber_right_deg=static[1],
        )
        for wheel, side, sign in (("FL", "left", -1.0), ("FR", "right", 1.0)):
            self.assertAlmostEqual(state[f"steering_camber_gain_{wheel}_deg"], steering[f"camber_change_{side}_deg"])
            self.assertAlmostEqual(state[f"camber_{wheel}_deg"], steering[f"camber_{side}_deg"] + sign * 1.5)
        for wheel, camber in zip(("RL", "RR"), static[2:]):
            self.assertAlmostEqual(state[f"steering_camber_gain_{wheel}_deg"], 0.0, places=10)
            self.assertAlmostEqual(state[f"camber_{wheel}_deg"], camber, places=10)

    def test_rear_uses_its_own_geometry(self):
        row = signal_row(zs_FL_mm=-10.0, zs_FR_mm=-10.0, zs_RL_mm=-10.0, zs_RR_mm=-10.0)
        default = CamberSignalAnalyzer().analyze_row(row)
        rear = SuspensionR()
        rear.fvsa = 0.8
        changed = CamberSignalAnalyzer(rear_suspension=rear).analyze_row(row)
        self.assertAlmostEqual(default["camber_gain_FL_deg"], changed["camber_gain_FL_deg"])
        self.assertNotAlmostEqual(default["camber_gain_RL_deg"], changed["camber_gain_RL_deg"])
        self.assertNotAlmostEqual(default["camber_gain_FL_deg"], default["camber_gain_RL_deg"])

    def test_invalid_front_geometry_does_not_erase_valid_rear_results(self):
        for field, invalid in (("zs_FR_mm", "missing"), ("zu_FL_mm", np.nan),
                               ("steer_FL_deg", 90.0), ("zs_FL_mm", -1e6)):
            with self.subTest(field=field):
                state = CamberSignalAnalyzer().analyze_row(signal_row(**{field: invalid}))
                self.assertFalse(state["front_solver_ok"])
                self.assertTrue(state["rear_solver_ok"])
                self.assertTrue(np.isnan(state["camber_FL_deg"]))
                self.assertTrue(state["front_error"])
                self.assertAlmostEqual(state["camber_RL_deg"], 0.0, places=10)

    def test_missing_roll_keeps_body_gain_and_does_not_fabricate_ground_camber(self):
        state = CamberSignalAnalyzer().analyze_row(signal_row(zs_FL_mm=-10.0, roll_front_deg=""))
        self.assertFalse(state["front_solver_ok"])
        self.assertTrue(np.isfinite(state["camber_gain_FL_deg"]))
        self.assertEqual(state["tire_roll_front_deg"], 0.0)
        self.assertTrue(np.isnan(state["total_roll_front_deg"]))
        self.assertTrue(np.isnan(state["camber_FL_deg"]))
        self.assertIn("roll_front_deg", state["front_error"])

    def test_rear_steering_is_not_silently_discarded(self):
        state = CamberSignalAnalyzer().analyze_row(signal_row(steer_RL_deg=2.0))
        self.assertFalse(state["rear_solver_ok"])
        self.assertTrue(state["front_solver_ok"])
        self.assertIn("rear steering", state["rear_error"])

    def test_csv_preserves_source_rows_and_does_not_zero_first_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output, figure = [Path(directory) / name for name in ("source.csv", "result.csv", "plot.png")]
            comparison = Path(directory) / "front.png"
            rows = [signal_row(zs_FL_mm=-5.0, zu_FL_mm=5.0, ay_m_s2=5.0, extra="original text"),
                    signal_row(time_s=0.5, roll_front_deg="", ay_m_s2=-5.0, extra="second row")]
            with source.open("w", newline="", encoding="utf-8-sig") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
            before = source.read_bytes()
            summary = analyze_csv(source, output, figure_path=figure, front_comparison_figure_path=comparison)
            self.assertEqual(source.read_bytes(), before)
            with output.open(encoding="utf-8-sig", newline="") as handle:
                result = list(csv.DictReader(handle))
            self.assertEqual(len(result), 2)
            self.assertEqual(result[0]["extra"], "original text")
            self.assertNotAlmostEqual(float(result[0]["camber_gain_FL_deg"]), 0.0)
            self.assertEqual(result[0]["roll_front_deg"], "0.0")
            self.assertAlmostEqual(float(result[0]["tire_roll_front_deg"]), 0.2291831180523293)
            self.assertAlmostEqual(float(result[0]["total_roll_front_deg"]), 0.2291831180523293)
            self.assertAlmostEqual(float(result[0]["camber_FL_deg"]),
                                   float(result[0]["camber_gain_FL_deg"]) - 0.2291831180523293)
            self.assertEqual(result[1]["front_solver_ok"], "False")
            self.assertEqual(summary["front_failed_rows"], 1)
            self.assertEqual(summary["rear_failed_rows"], 0)
            self.assertEqual(figure.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            self.assertEqual(comparison.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            self.assertEqual(result[0]["front_outside_wheel"], "FR")
            self.assertEqual(result[0]["front_inside_wheel"], "FL")
            self.assertEqual(result[0]["camber_front_outside_deg"], result[0]["camber_FR_deg"])
            self.assertEqual(result[0]["camber_front_inside_deg"], result[0]["camber_FL_deg"])
            self.assertEqual(result[1]["front_outside_wheel"], "FL")
            self.assertTrue(np.isnan(float(result[1]["camber_front_outside_deg"])))
            self.assertEqual(summary["front_comparison"]["paired_rows"], 1)

    def test_csv_rejects_missing_columns_empty_data_and_same_output_path(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output = Path(directory) / "source.csv", Path(directory) / "output.csv"
            source.write_text("time_s\n0\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Missing required"):
                analyze_csv(source, output)
            with self.assertRaisesRegex(ValueError, "different paths"):
                analyze_csv(source, source)
            with source.open("w", newline="", encoding="utf-8") as handle:
                csv.DictWriter(handle, fieldnames=list(signal_row())).writeheader()
            with self.assertRaisesRegex(ValueError, "no data rows"):
                analyze_csv(source, output)
            self.assertFalse(output.exists())

    def test_comparison_figure_requires_acceleration_and_distinct_path(self):
        with tempfile.TemporaryDirectory() as directory:
            source, output, figure = [Path(directory) / name for name in ("source.csv", "result.csv", "plot.png")]
            with source.open("w", newline="", encoding="utf-8") as handle:
                row = signal_row()
                writer = csv.DictWriter(handle, fieldnames=list(row))
                writer.writeheader()
                writer.writerow(row)
            with self.assertRaisesRegex(ValueError, "ay_m_s2"):
                analyze_csv(source, output, front_comparison_figure_path=figure)
            self.assertFalse(output.exists())
            with self.assertRaisesRegex(ValueError, "different paths"):
                analyze_csv(source, output, figure_path=figure, front_comparison_figure_path=figure)
            # Existing callers can still analyze CSVs without acceleration.
            summary = analyze_csv(source, output)
            self.assertEqual(summary["front_comparison"]["paired_rows"], 0)
            self.assertIsNone(summary["front_comparison"]["outside"])


if __name__ == "__main__":
    unittest.main()
