"""Static triangles through an arm's FV IC, SV IC and outer ball joint."""

import numpy as np


def build_arm_plane_geometry(front_plane, rear_plane, side_view_plane, kingpins):
    """Return the eight upper/lower arm triangles in vehicle coordinates.

    Keys are ``FL_upper``, ``FL_lower`` through ``RR_lower``.  Each entry
    contains the three exact named vertices and a ``vertices`` array ordered
    as ``[FV_IC, SV_IC, kingpin_joint]``.  Upper arms use ``UBJ`` and lower
    arms use ``LBJ``.  The SV instant center remains on the original central
    ``Y=0`` plane; no vertex is moved or clipped to the displayed vehicle.

    ``normal`` is the unit triangle normal, and ``area`` is in square metres.
    This construction does not change the source FV, SV or kingpin data.
    """
    result = {}
    for prefix, axle_name, plane, sv_side in (
        ("F", "Front", front_plane, side_view_plane.front),
        ("R", "Rear", rear_plane, side_view_plane.rear),
    ):
        SV_IC = np.asarray(side_view_plane.project(sv_side.initial_ic), dtype=float)
        if SV_IC.shape != (3,) or not np.isfinite(SV_IC).all():
            raise ValueError(f"{axle_name}: the SV instant center must have three finite coordinates.")

        for suffix, side_name, fv_side in (
            ("L", "Left", plane.left_sus),
            ("R", "Right", plane.right_sus),
        ):
            corner = prefix + suffix
            fv_point = np.asarray(fv_side.IC.pos, dtype=float)
            if fv_point.shape != (2,) or not np.isfinite(fv_point).all():
                raise ValueError(f"{corner}: the FV instant center must have two finite coordinates.")
            FV_IC = np.array([plane.longitudinal_x, -fv_point[0], fv_point[1]], dtype=float)

            for arm, joint in (("upper", "UBJ"), ("lower", "LBJ")):
                key = f"{corner}_{arm}"
                kingpin_joint = np.asarray(kingpins[corner][joint], dtype=float)
                if kingpin_joint.shape != (3,):
                    raise ValueError(f"{key}: the kingpin joint must have three coordinates.")
                vertices = np.array([FV_IC, SV_IC, kingpin_joint], dtype=float)
                if not np.isfinite(vertices).all():
                    raise ValueError(f"{key}: all triangle vertices must be finite.")

                cross = np.cross(vertices[1] - vertices[0], vertices[2] - vertices[0])
                twice_area = np.linalg.norm(cross)
                if not np.isfinite(twice_area) or twice_area < 1e-12:
                    raise ValueError(f"{key}: FV IC, SV IC and {joint} do not define a triangle plane.")

                result[key] = {
                    "corner": corner,
                    "axle": axle_name,
                    "side": side_name,
                    "arm": arm,
                    "joint": joint,
                    "FV_IC": FV_IC.copy(),
                    "SV_IC": SV_IC.copy(),
                    "kingpin_joint": kingpin_joint.copy(),
                    "vertices": vertices,
                    "normal": cross / twice_area,
                    "area": float(twice_area / 2.0),
                }
    return result
