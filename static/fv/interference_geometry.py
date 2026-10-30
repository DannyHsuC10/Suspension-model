"""Static inboard rim-clearance circles at neutral and front steering limits.

Lengths use metres and angles use degrees.  The steering-limit circles are
rigid rotations about each wheel's actual 3D kingpin line, not about the
vehicle origin or the circle centre.
"""

import numpy as np


def rotation_matrix(axis, angle_deg):
    """Return a right-hand Rodrigues rotation about a finite 3D direction."""
    axis = np.asarray(axis, dtype=float)
    angle_deg = float(angle_deg)
    if axis.shape != (3,) or not np.isfinite(axis).all():
        raise ValueError("Rotation axis must have three finite coordinates.")
    length = np.linalg.norm(axis)
    if not np.isfinite(length) or length < 1e-12 or not np.isfinite(angle_deg):
        raise ValueError("Rotation axis must be nonzero and its angle finite.")
    axis = axis / length
    x, y, z = axis
    cross = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    angle = np.deg2rad(angle_deg)
    cosine, sine = np.cos(angle), np.sin(angle)
    return cosine * np.eye(3) + (1.0 - cosine) * np.outer(axis, axis) + sine * cross


def build_interference_geometry(car, front_plane, rear_plane, kingpins):
    """Return four neutral circles plus the two steering limits on each front.

    Each ``FL``/``FR``/``RL``/``RR`` entry contains the neutral ``center``,
    ``radius``, ``diameter``, upward ``axis_direction`` and ``axis_origin``.
    ``circles`` contains ``neutral`` and, for the front wheels only,
    ``positive``/``negative`` records.  A circle stores ``angle_deg``, closed
    ``points`` (129 by 3), its transformed ``center`` and inward ``normal``,
    and the rotation matrix used to produce it.

    Source FV, kingpin and vehicle objects are not modified.
    """
    rim_diameter = float(car.rim_inner_diameter)
    gap = float(car.gap)
    steer = float(car.steer)
    if not np.isfinite([rim_diameter, gap, steer]).all():
        raise ValueError("Rim diameter, gap and steering angle must be finite.")
    diameter = rim_diameter - 2.0 * gap
    if rim_diameter <= 0.0 or gap < 0.0 or diameter <= 0.0:
        raise ValueError("Rim diameter minus twice the nonnegative gap must be positive.")
    if steer < 0.0:
        raise ValueError("Steering limit must be nonnegative degrees.")
    radius = diameter / 2.0
    angles = np.linspace(0.0, 2.0 * np.pi, 129)
    result = {}

    for prefix, axle_name, plane in (
        ("F", "Front", front_plane), ("R", "Rear", rear_plane),
    ):
        for suffix, side_name, side in (
            ("L", "Left", plane.left_sus), ("R", "Right", plane.right_sus),
        ):
            corner = prefix + suffix
            wheel_center = np.asarray(side.wheel_center.pos, dtype=float)
            width = float(side.tire_width)
            if (wheel_center.shape != (2,) or not np.isfinite(wheel_center).all()
                    or not np.isfinite(width) or width <= 0.0):
                raise ValueError(f"{corner}: tire centre and width must be finite and valid.")
            tire_center = np.array([
                plane.longitudinal_x, -wheel_center[0], wheel_center[1],
            ], dtype=float)
            if not np.isfinite(tire_center).all() or abs(tire_center[1]) < 1e-12:
                raise ValueError(f"{corner}: tire centre must lie off the vehicle centre plane.")
            side_sign = float(np.sign(tire_center[1]))
            center = tire_center + np.array([0.0, -side_sign * width / 2.0, 0.0])
            normal = np.array([0.0, -side_sign, 0.0])
            basis_x = np.array([1.0, 0.0, 0.0])
            basis_z = np.cross(normal, basis_x)
            neutral_points = center + radius * (
                np.cos(angles)[:, None] * basis_x
                + np.sin(angles)[:, None] * basis_z
            )
            neutral_points[-1] = neutral_points[0]

            origin = np.array(kingpins[corner]["ground"], dtype=float, copy=True)
            top = np.asarray(kingpins[corner]["axis_top"], dtype=float)
            if (origin.shape != (3,) or top.shape != (3,)
                    or not np.isfinite([*origin, *top]).all()):
                raise ValueError(f"{corner}: kingpin endpoints must be finite 3D coordinates.")
            axis = top - origin
            axis_length = np.linalg.norm(axis)
            if axis_length < 1e-12 or not np.isfinite(axis_length):
                raise ValueError(f"{corner}: kingpin must have a finite nonzero length.")
            axis /= axis_length
            if axis[2] < 0.0:
                axis = -axis
            if axis[2] <= 1e-12:
                raise ValueError(f"{corner}: kingpin must have an upward direction.")

            poses = [("neutral", 0.0)]
            if prefix == "F":
                poses.extend((("positive", steer), ("negative", -steer)))
            circles = {}
            for pose, angle_deg in poses:
                matrix = rotation_matrix(axis, angle_deg)
                if angle_deg == 0.0:
                    points = neutral_points.copy()
                    rotated_center = center.copy()
                else:
                    points = origin + (neutral_points - origin) @ matrix.T
                    rotated_center = origin + matrix @ (center - origin)
                circles[pose] = {
                    "angle_deg": float(angle_deg),
                    "points": points,
                    "center": rotated_center,
                    "normal": matrix @ normal,
                    "rotation_matrix": matrix,
                }
            result[corner] = {
                "corner": corner,
                "axle": axle_name,
                "side": side_name,
                "tire_center": tire_center,
                "center": center,
                "radius": radius,
                "diameter": diameter,
                "axis_origin": origin,
                "axis_direction": axis,
                "circles": circles,
            }
    return result
