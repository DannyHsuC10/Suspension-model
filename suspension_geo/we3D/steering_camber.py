"""Signed front-wheel camber and local camber gain caused by steering.

Steering inputs are wheel headings in the ground plane [deg], positive to
the left. Camber is positive when the tire top leans outward. The chassis
is fixed over level ground; suspension travel and body roll are excluded.
"""

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from suspension_geo.car import SuspensionF


def _finite_angle(value, name):
    value = float(value)
    if not np.isfinite(value) or abs(value) >= 90.0:
        raise ValueError(f"{name} must be finite and strictly between -90 and 90 deg.")
    return value


def _wheel_camber(steer_deg, static_camber_deg, side, kpi_rad, caster_rad):
    """Rotate an outward axle normal; side is +1 for left, -1 for right."""
    steer = np.deg2rad(steer_deg)
    static_camber = np.deg2rad(static_camber_deg)

    # X forward, Y left, Z up. Positive caster tilts the kingpin top rearward;
    # positive KPI tilts it inward. Tangents preserve both projected angles.
    axis = np.array([-np.tan(caster_rad), -side * np.tan(kpi_rad), 1.0])
    axis /= np.linalg.norm(axis)
    normal_initial = np.array([
        0.0, side * np.cos(static_camber), -np.sin(static_camber),
    ])

    # Rodrigues: n(theta) = a*cos(theta) + b*sin(theta) + c.
    c = axis * np.dot(axis, normal_initial)
    a = normal_initial - c
    b = np.cross(axis, normal_initial)
    forward = np.array([np.cos(steer), np.sin(steer), 0.0])
    a_heading, b_heading, c_heading = (
        np.dot(forward, vector) for vector in (a, b, c)
    )
    amplitude = np.hypot(a_heading, b_heading)
    if amplitude < 1e-12:
        raise ValueError("The steering axis cannot produce a unique wheel heading.")
    target = -c_heading / amplitude
    if abs(target) > 1.0 + 1e-12:
        raise ValueError("The requested wheel heading is unreachable for this geometry.")

    # Solve n(theta).forward = 0. Reject the solution facing 180 deg away;
    # select the kingpin rotation nearest the straight-ahead configuration.
    phase = np.arctan2(b_heading, a_heading)
    offset = np.arccos(np.clip(target, -1.0, 1.0))
    outward = side * np.array([-np.sin(steer), np.cos(steer), 0.0])
    candidates = []
    for angle in (phase + offset, phase - offset):
        angle = (angle + np.pi) % (2.0 * np.pi) - np.pi
        normal = a * np.cos(angle) + b * np.sin(angle) + c
        if np.dot(normal, outward) > 1e-12:
            candidates.append((angle, normal))
    if not candidates:
        raise ValueError("The requested wheel heading has no upright tire solution.")
    rotation, normal = min(candidates, key=lambda candidate: abs(candidate[0]))

    horizontal_length = np.linalg.norm(normal[:2])
    camber = np.arctan2(-normal[2], horizontal_length)

    # Analytic derivatives with respect to kingpin rotation [rad/rad].
    # Dividing them gives d(camber)/d(wheel heading), also in deg/deg.
    derivative = np.cross(axis, normal)
    heading_derivative = (
        normal[0] * derivative[1] - normal[1] * derivative[0]
    ) / horizontal_length**2
    if abs(heading_derivative) < 1e-12:
        raise ValueError("Camber gain is undefined at this steering singularity.")
    camber_derivative = -derivative[2] / horizontal_length
    camber_deg = float(np.rad2deg(camber))
    return {
        "steer_deg": float(steer_deg),
        "kingpin_rotation_deg": float(np.rad2deg(rotation)),
        "static_camber_deg": float(static_camber_deg),
        "camber_deg": camber_deg,
        "camber_change_deg": camber_deg - static_camber_deg,
        "camber_gain_deg_per_deg": float(camber_derivative / heading_derivative),
    }


def calculate_steering_camber(
    steer_left_deg,
    steer_right_deg=None,
    *,
    suspension=None,
    static_camber_left_deg=0.0,
    static_camber_right_deg=0.0,
):
    """Return both front wheels' camber, steering-induced change and local gain.

    All angles use degrees. Positive steering turns left on both sides;
    positive camber means the tire top leans outward. If only one steering
    angle is supplied, both wheels use it (parallel steering). Supply both
    actual wheel headings to account for Ackermann or measured steering.

    Defaults come from SuspensionF.kpi [rad] and caster_sv_deg [deg]; static
    camber defaults to zero, matching the existing wheel-end model.
    camber_change_*_deg is camber(steer) - camber(0).
    camber_gain_*_deg_per_deg is the local d(camber)/d(wheel heading),
    rather than the change divided by the input angle or travel gain [deg/mm].
    """
    suspension = SuspensionF() if suspension is None else suspension
    if steer_right_deg is None:
        steer_right_deg = steer_left_deg

    kpi_deg = _finite_angle(np.rad2deg(suspension.kpi), "KPI")
    caster_deg = _finite_angle(suspension.caster_sv_deg, "Caster")
    kpi_rad, caster_rad = np.deg2rad([kpi_deg, caster_deg])
    result = {"kpi_deg": kpi_deg, "caster_deg": caster_deg}
    for name, side, steer_deg, static_deg in (
        ("left", 1.0, steer_left_deg, static_camber_left_deg),
        ("right", -1.0, steer_right_deg, static_camber_right_deg),
    ):
        steer_deg = _finite_angle(steer_deg, f"{name} steering angle")
        static_deg = _finite_angle(static_deg, f"{name} static camber")
        wheel = _wheel_camber(steer_deg, static_deg, side, kpi_rad, caster_rad)
        for quantity in ("steer", "kingpin_rotation", "static_camber", "camber", "camber_change"):
            result[f"{quantity}_{name}_deg"] = wheel[f"{quantity}_deg"]
        result[f"camber_gain_{name}_deg_per_deg"] = wheel["camber_gain_deg_per_deg"]
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("steer_deg", nargs="?", type=float,
                        help="Heading of both wheels [deg]; positive turns left.")
    parser.add_argument("--left", type=float, help="Left wheel heading [deg].")
    parser.add_argument("--right", type=float, help="Right wheel heading [deg].")
    parser.add_argument("--kpi", type=float, help="Override SuspensionF KPI [deg].")
    parser.add_argument("--caster", type=float, help="Override SuspensionF caster [deg].")
    parser.add_argument("--static-camber-left", type=float, default=0.0,
                        help="Left straight-ahead camber [deg], default 0.")
    parser.add_argument("--static-camber-right", type=float, default=0.0,
                        help="Right straight-ahead camber [deg], default 0.")
    args = parser.parse_args(argv)
    separate_angles = args.left is not None or args.right is not None
    if separate_angles:
        if args.steer_deg is not None or args.left is None or args.right is None:
            parser.error("Use one positional angle OR both --left and --right.")
        left_deg, right_deg = args.left, args.right
    else:
        left_deg = args.steer_deg
        if left_deg is None:
            try:
                left_deg = float(input("Wheel heading [deg] (positive = left turn): "))
            except (ValueError, EOFError):
                parser.error("Enter a numeric steering angle in degrees.")
        right_deg = left_deg

    suspension = SuspensionF()
    if args.kpi is not None:
        suspension.kpi = np.deg2rad(args.kpi)
    if args.caster is not None:
        suspension.caster_sv_deg = args.caster
    try:
        result = calculate_steering_camber(
            left_deg, right_deg, suspension=suspension,
            static_camber_left_deg=args.static_camber_left,
            static_camber_right_deg=args.static_camber_right,
        )
    except ValueError as error:
        parser.error(str(error))

    print(f"KPI = {result['kpi_deg']:.3f} deg, caster = {result['caster_deg']:.3f} deg")
    print("Positive steering: left turn; positive camber: tire top outward.")
    print("Fixed chassis, level ground; no suspension travel or body roll.\n")
    print(f"{'Quantity':<34}{'Left':>13}{'Right':>13}")
    for label, quantity, suffix in (
        ("Wheel heading [deg]", "steer", "deg"),
        ("Kingpin rotation [deg]", "kingpin_rotation", "deg"),
        ("Static camber [deg]", "static_camber", "deg"),
        ("Camber [deg]", "camber", "deg"),
        ("Camber change from zero steer [deg]", "camber_change", "deg"),
        ("Local camber gain [deg/deg]", "camber_gain", "deg_per_deg"),
    ):
        print(f"{label:<34}{result[f'{quantity}_left_{suffix}']:>13.6f}"
              f"{result[f'{quantity}_right_{suffix}']:>13.6f}")
    return result


if __name__ == "__main__":
    main()
