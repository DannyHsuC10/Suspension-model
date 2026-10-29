"""Combine the existing FV and SV kingpin projections at common heights.

Vehicle coordinates use X forward, Y left and Z up, in metres.  FV fixes
the lateral coordinate and height; the aligned SV projection fixes the
longitudinal coordinate at that same height.  This is a static geometric
construction and does not change either source view.
"""

import numpy as np


def _point_at_height(start, end, height, name):
    """Interpolate or extend a fixed 2D line to a requested height."""
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    if start.shape != (2,) or end.shape != (2,):
        raise ValueError(f"{name}: kingpin projections must contain two coordinates.")
    if not np.isfinite([*start, *end, height]).all():
        raise ValueError(f"{name}: kingpin coordinates and height must be finite.")
    height_span = end[1] - start[1]
    if abs(height_span) < 1e-12:
        raise ValueError(f"{name}: kingpin projection must have a nonzero height span.")
    point = start + (height - start[1]) / height_span * (end - start)
    point[1] = height
    return point


def _FV_projection(point, longitudinal_x):
    """Keep a point on its original FV axle plane."""
    return np.array([longitudinal_x, -point[0], point[1]], dtype=float)


def build_kingpin_geometry(front_plane, rear_plane, side_view_plane):
    """Return the combined kingpin and upper/lower ball joints for four corners.

    The returned mapping has ``FL``, ``FR``, ``RL`` and ``RR`` keys.  Each
    corner includes ``ground``, ``UBJ``, ``LBJ`` and ``axis_top`` in vehicle
    coordinates, plus ``FV_UBJ``, ``FV_LBJ``, ``SV_UBJ`` and ``SV_LBJ`` for
    checking the corresponding source projections.  The FV joint height
    is authoritative; an SV joint at another height is evaluated along
    its existing kingpin line at the FV height.

    The plane objects are used by interface only, avoiding a dependency
    on the Matplotlib figure module.
    """
    result = {}
    for prefix, axle_name, plane, sv_side in (
        ("F", "Front", front_plane, side_view_plane.front),
        ("R", "Rear", rear_plane, side_view_plane.rear),
    ):
        for suffix, side_name, fv_side in (
            ("L", "Left", plane.left_sus),
            ("R", "Right", plane.right_sus),
        ):
            corner = prefix + suffix
            fv_ground = np.asarray(fv_side.kg.pos, dtype=float)
            sv_ground = np.asarray(sv_side.kingpin_ground, dtype=float)
            sv_top = np.asarray(sv_side.kingpin_top, dtype=float)
            if not np.isclose(fv_ground[1], sv_ground[1], rtol=0.0, atol=1e-10):
                raise ValueError(f"{corner}: FV and SV kingpins must share a ground height.")

            projected_ground = np.asarray(side_view_plane.project(sv_ground), dtype=float)
            geometry = {
                "corner": corner,
                "axle": axle_name,
                "side": side_name,
                "ground": np.array([projected_ground[0], -fv_ground[0], fv_ground[1]]),
            }

            for joint, fv_name, sv_name in (
                ("UBJ", "kua", "kingpin_Au"),
                ("LBJ", "lak", "kingpin_Al"),
            ):
                fv_point = np.asarray(getattr(fv_side, fv_name).pos, dtype=float)
                sv_point = np.asarray(getattr(sv_side, sv_name), dtype=float)
                if np.isclose(fv_point[1], sv_point[1], rtol=0.0, atol=1e-10):
                    sv_point = sv_point.copy()
                    sv_point[1] = fv_point[1]
                else:
                    sv_point = _point_at_height(sv_ground, sv_top, fv_point[1], corner)
                FV = _FV_projection(fv_point, plane.longitudinal_x)
                SV = np.asarray(side_view_plane.project(sv_point), dtype=float)
                geometry[joint] = np.array([SV[0], FV[1], FV[2]])
                geometry[f"FV_{joint}"] = FV
                geometry[f"SV_{joint}"] = SV

            fv_top = _point_at_height(fv_ground, fv_side.kua.pos, sv_top[1], corner)
            projected_top = np.asarray(side_view_plane.project(sv_top), dtype=float)
            geometry["axis_top"] = np.array([projected_top[0], -fv_top[0], fv_top[1]])

            axis = geometry["axis_top"] - geometry["ground"]
            axis_length = np.linalg.norm(axis)
            if not np.isfinite(axis_length) or axis_length < 1e-12:
                raise ValueError(f"{corner}: the 3D kingpin axis must have a finite length.")
            for joint in ("UBJ", "LBJ"):
                distance = np.linalg.norm(np.cross(geometry[joint] - geometry["ground"], axis)) / axis_length
                if not np.isfinite(distance) or distance > 1e-9:
                    raise ValueError(f"{corner}: {joint} is not on the combined 3D kingpin axis.")
            result[corner] = geometry
    return result
