"""Add front-view suspension and steering camber changes for both front wheels.

Steering: wheel heading [deg], positive left turn on both sides.
Travel: wheel-side lift relative to chassis [mm], positive compression.
Camber: tire top outward is positive on both sides.
"""

import argparse
import contextlib
import io
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from suspension_geo import Geometric_Analysis as ga
from suspension_geo.car import Car, SuspensionF
from suspension_geo.we3D.steering_camber import calculate_steering_camber


# 可直接修改這四個值後執行；命令列參數會優先使用。
STEER_LEFT_DEG = 25.0      # 左輪輪端轉角 [deg]，左轉正、右轉負
STEER_RIGHT_DEG = 18.0     # 右輪輪端轉角 [deg]，左轉正、右轉負
TRAVEL_LEFT_MM = 10.0      # 左輪相對車身上抬量 [mm]，上壓正、下伸負
TRAVEL_RIGHT_MM = -5.0     # 右輪相對車身上抬量 [mm]，上壓正、下伸負


def _build_fv_solver(car, suspension):
    """Build a fresh instance of the same solver used by fv/fv_camber_rc.py."""
    helper = ga.SusGeometryHelper(car, suspension)
    right = ga.SuspensionSide("Right")
    right.setup_right_side(helper.calc_static_points())
    left = ga.SuspensionSide("Left", is_left=True)
    left.setup_from_right_template(right)
    return ga.SuspensionDirectSolver(helper, left, right)


def calculate_combined_camber(
    steer_left_deg,
    steer_right_deg,
    travel_left_mm,
    travel_right_mm,
    *,
    car=None,
    suspension=None,
    static_camber_left_deg=0.0,
    static_camber_right_deg=0.0,
):
    """Return additive camber changes, final cambers, separate local gains and RC.

    total_change = (FV(travel_left, travel_right) - FV(0, 0))
                   + (steering(heading, static_camber) - static_camber)
    final_camber = static_camber + total_change

    Camber changes are angles [deg]. FV gain [deg/mm] and steering gain
    [deg/deg] are returned separately; adding these derivatives is invalid.
    Each FV derivative fixes the other wheel's travel; each steering
    derivative fixes all travels and the other wheel's heading.

    This deliberately superposes the two existing models. Steering uses the
    design KPI/caster at zero travel, not the traveled upright orientation.
    FV camber keeps its original reference to the two contact points' line.
    """
    travel_left_mm, travel_right_mm = float(travel_left_mm), float(travel_right_mm)
    if not np.isfinite([travel_left_mm, travel_right_mm]).all():
        raise ValueError("Left and right suspension travels must be finite [mm].")
    car = Car() if car is None else car
    suspension = SuspensionF() if suspension is None else suspension
    steering = calculate_steering_camber(
        steer_left_deg, steer_right_deg, suspension=suspension,
        static_camber_left_deg=static_camber_left_deg,
        static_camber_right_deg=static_camber_right_deg,
    )
    solver = _build_fv_solver(car, suspension)
    # The shared RC helper prints debug coordinates; keep the callable quiet.
    with contextlib.redirect_stdout(io.StringIO()):
        baseline = solver.solve_by_travels(0.0, 0.0)
        fv = solver.solve_by_travels(travel_left_mm / 1000.0, travel_right_mm / 1000.0)
    actual_travels = [fv["travel_left_mm"], fv["travel_right_mm"]]
    if not np.allclose(actual_travels, [travel_left_mm, travel_right_mm], rtol=0.0, atol=1e-5):
        raise RuntimeError("FV solver did not reach the requested suspension travels [mm].")
    for side, geometry in (("left", solver.left_sus), ("right", solver.right_sus)):
        if (not np.isfinite([fv[f"theta_{side}"], fv[f"camber_{side}_deg"]]).all()
                or ga.simulate_side_theta(geometry, fv[f"theta_{side}"]) is None):
            raise RuntimeError(f"The requested {side} suspension geometry is unreachable.")

    result = {
        "kpi_deg": steering["kpi_deg"],
        "caster_deg": steering["caster_deg"],
        "heave_mm": float(fv["heave_mm"]),
        "roll_deg": float(fv["roll_deg"]),
        "rc_pos": fv["rc_pos"],  # FV coordinates [m]; None for no finite RC.
    }
    for side in ("left", "right"):
        fv_change = float(fv[f"camber_{side}_deg"] - baseline[f"camber_{side}_deg"])
        steering_change = steering[f"camber_change_{side}_deg"]
        static_camber = steering[f"static_camber_{side}_deg"]
        result.update({
            f"steer_{side}_deg": steering[f"steer_{side}_deg"],
            f"travel_{side}_mm": float(fv[f"travel_{side}_mm"]),
            f"static_camber_{side}_deg": static_camber,
            f"fv_camber_change_{side}_deg": fv_change,
            f"steering_camber_change_{side}_deg": steering_change,
            f"total_camber_change_{side}_deg": fv_change + steering_change,
            f"camber_{side}_deg": static_camber + fv_change + steering_change,
            f"fv_camber_gain_{side}_deg_per_mm": float(fv[f"camber_gain_{side}_deg_per_mm"]),
            f"steering_camber_gain_{side}_deg_per_deg": steering[f"camber_gain_{side}_deg_per_deg"],
        })
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steer-left", type=float, default=STEER_LEFT_DEG,
                        help=f"Left wheel heading [deg], default {STEER_LEFT_DEG}.")
    parser.add_argument("--steer-right", type=float, default=STEER_RIGHT_DEG,
                        help=f"Right wheel heading [deg], default {STEER_RIGHT_DEG}.")
    parser.add_argument("--travel-left-mm", type=float, default=TRAVEL_LEFT_MM,
                        help=f"Left wheel lift/compression [mm], default {TRAVEL_LEFT_MM}.")
    parser.add_argument("--travel-right-mm", type=float, default=TRAVEL_RIGHT_MM,
                        help=f"Right wheel lift/compression [mm], default {TRAVEL_RIGHT_MM}.")
    parser.add_argument("--static-camber-left", type=float, default=0.0,
                        help="Left initial camber [deg], default 0.")
    parser.add_argument("--static-camber-right", type=float, default=0.0,
                        help="Right initial camber [deg], default 0.")
    args = parser.parse_args(argv)
    try:
        result = calculate_combined_camber(
            args.steer_left, args.steer_right, args.travel_left_mm, args.travel_right_mm,
            static_camber_left_deg=args.static_camber_left,
            static_camber_right_deg=args.static_camber_right,
        )
    except (ValueError, RuntimeError) as error:
        parser.error(str(error))

    print(f"KPI = {result['kpi_deg']:.3f} deg; caster = {result['caster_deg']:.3f} deg")
    print("Positive heading: left turn; positive travel: wheel lift/compression.")
    print("Positive camber: tire top outward on each side.")
    print("Angle superposition: total change = FV change + steering change.\n")
    print(f"{'Quantity':<39}{'Left':>13}{'Right':>13}")
    for label, quantity, unit in (
        ("Wheel heading [deg]", "steer", "deg"),
        ("Wheel lift / compression [mm]", "travel", "mm"),
        ("Initial camber [deg]", "static_camber", "deg"),
        ("FV suspension camber change [deg]", "fv_camber_change", "deg"),
        ("Steering camber change [deg]", "steering_camber_change", "deg"),
        ("TOTAL camber change [deg]", "total_camber_change", "deg"),
        ("FINAL camber [deg]", "camber", "deg"),
        ("FV local camber gain [deg/mm]", "fv_camber_gain", "deg_per_mm"),
        ("Steering local camber gain [deg/deg]", "steering_camber_gain", "deg_per_deg"),
    ):
        print(f"{label:<39}{result[f'{quantity}_left_{unit}']:>13.6f}"
              f"{result[f'{quantity}_right_{unit}']:>13.6f}")
    print(f"\nFV heave = {result['heave_mm']:.3f} mm; roll = {result['roll_deg']:.3f} deg")
    rc = result["rc_pos"]
    if rc is None or not np.isfinite(rc).all():
        print("FV roll center: N/A (no finite intersection)")
    else:
        print(f"FV roll center: X = {rc[0] * 1000:.3f} mm; Y = {rc[1] * 1000:.3f} mm")
    return result


if __name__ == "__main__":
    main()
