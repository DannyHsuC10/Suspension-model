"""Run front-view suspension geometry on the selected Run 15 lap data.

The input CSV stores vehicle pose as theta=[front_roll, rear_roll, pitch, yaw]
in radians and sprung corner heights zs=[FL, FR, RL, RR] in metres.  This
analysis uses the front roll angle and the mean front zs displacement as the
heave/roll target for the front-view direct solver.
"""

from __future__ import annotations

import argparse
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import Geometric_Analysis as ga
from car import Car, SuspensionF


HERE = Path(__file__).resolve().parent
DEFAULT_INPUT = HERE / "data_run15_selected.csv"
DEFAULT_OUTPUT = HERE / "data_run15_geometry.csv"
DEFAULT_FIGURE = HERE / "data_run15_geometry.png"


def make_solver() -> ga.SuspensionDirectSolver:
    car = Car()
    suspension = SuspensionF()
    helper = ga.SusGeometryHelper(car, suspension)
    static_points = helper.calc_static_points()

    right = ga.SuspensionSide("Right", is_left=False)
    right.setup_right_side(static_points)
    left = ga.SuspensionSide("Left", is_left=True)
    left.setup_from_right_template(right)

    return ga.SuspensionDirectSolver(helper, left, right)


def analyze(input_path: Path) -> pd.DataFrame:
    data = pd.read_csv(input_path)
    required = [
        "data.t",
        "data.theta[0]",
        "data.zs[0]",
        "data.zs[1]",
    ]
    missing = [column for column in required if column not in data.columns]
    if missing:
        raise ValueError(f"Input CSV is missing required columns: {', '.join(missing)}")

    time_s = data["data.t"].to_numpy(dtype=float)
    front_roll_rad = data["data.theta[0]"].to_numpy(dtype=float)
    front_zs_m = data[["data.zs[0]", "data.zs[1]"]].to_numpy(dtype=float)
    front_heave_m = front_zs_m.mean(axis=1) - front_zs_m[0].mean()

    results: list[dict[str, float]] = []
    for index, (t, roll_rad, heave_m) in enumerate(
        zip(time_s, front_roll_rad, front_heave_m, strict=True)
    ):
        try:
            # Vehicle theta[0] is positive when the left side rises. The
            # solver's left-to-right ground-line angle has the opposite sign.
            # Start from static geometry for every sample. Reusing the previous
            # solution can make fsolve jump between valid kinematic branches.
            solver = make_solver()
            # The existing geometry helper prints every RC point; silence that
            # per-sample debug output for this long lap-data batch.
            with redirect_stdout(StringIO()):
                solved = solver.solve_by_pose(
                    target_heave_m=float(heave_m),
                    target_roll_deg=float(-np.degrees(roll_rad)),
                )
            rc = solved["rc_pos"]
            rc_x, rc_y = (float(rc[0]), float(rc[1])) if rc is not None else (np.nan, np.nan)
            # At the exact static pose the left/right force lines are
            # numerically parallel, so round-off can move their intersection
            # far away. Use the known symmetric static RC for that limit.
            if abs(heave_m) < 1e-5 and abs(roll_rad) < np.deg2rad(1e-4):
                rc_x, rc_y = 0.0, solver.sus_helper.h_rc * 1000.0
            results.append(
                {
                    "time_s": float(t),
                    "vehicle_front_roll_deg": float(np.degrees(roll_rad)),
                    "front_heave_mm": float(heave_m * 1000.0),
                    "travel_left_mm": float(solved["travel_left_mm"]),
                    "travel_right_mm": float(solved["travel_right_mm"]),
                    "camber_left_deg": float(solved["camber_left_deg"]),
                    "camber_right_deg": float(solved["camber_right_deg"]),
                    "roll_center_x_mm": rc_x * 1000.0,
                    "roll_center_y_mm": rc_y * 1000.0,
                    "solver_ok": True,
                }
            )
        except (RuntimeError, ValueError, FloatingPointError) as exc:
            results.append(
                {
                    "time_s": float(t),
                    "vehicle_front_roll_deg": float(np.degrees(roll_rad)),
                    "front_heave_mm": float(heave_m * 1000.0),
                    "travel_left_mm": np.nan,
                    "travel_right_mm": np.nan,
                    "camber_left_deg": np.nan,
                    "camber_right_deg": np.nan,
                    "roll_center_x_mm": np.nan,
                    "roll_center_y_mm": np.nan,
                    "solver_ok": False,
                }
            )
            print(f"Solve failed at row {index} (t={t:.3f} s): {exc}")

    output = pd.DataFrame(results)
    mean_travel = (output["travel_left_mm"] + output["travel_right_mm"]) / 2.0
    # Camber gain is the local camber change per mean front-wheel travel.
    # Set it undefined where travel barely changes between adjacent samples.
    d_travel = np.gradient(mean_travel.to_numpy(dtype=float))
    for side in ("left", "right"):
        d_camber = np.gradient(output[f"camber_{side}_deg"].to_numpy(dtype=float))
        gain = np.divide(
            d_camber,
            d_travel,
            out=np.full_like(d_camber, np.nan),
            where=np.abs(d_travel) > 1e-2,
        )
        output[f"camber_gain_{side}_deg_per_mm"] = gain
    return output


def plot_results(output: pd.DataFrame, figure_path: Path) -> None:
    valid = output["solver_ok"]
    t = output["time_s"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(t[valid], output.loc[valid, "camber_left_deg"], label="Left")
    axes[0].plot(t[valid], output.loc[valid, "camber_right_deg"], label="Right")
    axes[0].set_ylabel("Camber (deg)")
    axes[0].set_title("Run 15 Front Suspension Geometry")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t[valid], output.loc[valid, "roll_center_x_mm"], color="tab:purple")
    axes[1].set_ylabel("RC lateral X (mm)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t[valid], output.loc[valid, "roll_center_y_mm"], color="tab:green")
    axes[2].set_ylabel("RC height Y (mm)")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(
        t[valid], output.loc[valid, "camber_gain_left_deg_per_mm"], label="Left"
    )
    axes[3].plot(
        t[valid], output.loc[valid, "camber_gain_right_deg_per_mm"], label="Right"
    )
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylabel("Camber gain (deg/mm)")
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(figure_path, dpi=160)
    if plt.get_backend().lower() != "agg":
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Run 15 CSV")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Analysis CSV output")
    parser.add_argument("--figure", type=Path, default=DEFAULT_FIGURE, help="Figure output")
    args = parser.parse_args()

    result = analyze(args.input)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.figure.parent.mkdir(parents=True, exist_ok=True)
    result.to_csv(args.output, index=False)
    plot_results(result, args.figure)
    failed = int((~result["solver_ok"]).sum())
    print(f"Analysis complete: {len(result)} rows, {failed} solve failures")
    print(f"CSV: {args.output}")
    print(f"Figure: {args.figure}")


if __name__ == "__main__":
    main()
