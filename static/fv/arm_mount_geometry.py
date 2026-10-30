"""Construct A-arm chassis pickups at a specified vehicle lateral Y."""

import numpy as np


def _vector3(value, name):
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all():
        raise ValueError(f"{name} must contain three finite coordinates.")
    return point.copy()


def extend_arm_leg(joint_point, selected_point, target_y, *, tolerance=1e-12):
    """Intersect a joint-to-clearance ray with a constant vehicle Y plane.

    The true parameter is retained. ``extended`` means the endpoint lies
    beyond the selected clearance point. Intersections before that point
    or behind the joint retain their status; parallel, coplanar and
    degenerate lines have no unique endpoint.
    """
    joint_point = _vector3(joint_point, "Arm ball joint")
    selected_point = _vector3(selected_point, "Selected arm point")
    target_y = float(target_y)
    tolerance = float(tolerance)
    if not np.isfinite(target_y):
        raise ValueError("Target vehicle Y must be finite.")
    if not np.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("Extension tolerance must be finite and positive.")

    direction = selected_point - joint_point
    if np.linalg.norm(direction) <= tolerance:
        return {"status": "degenerate", "t": None, "endpoint": None,
                "direction": direction, "is_extension": False}
    if abs(direction[1]) <= tolerance:
        status = "coplanar" if abs(joint_point[1] - target_y) <= tolerance else "parallel"
        return {"status": status, "t": None, "endpoint": None,
                "direction": direction, "is_extension": False}

    t = float((target_y - joint_point[1]) / direction[1])
    endpoint = joint_point + t * direction
    if not np.isfinite(t) or not np.isfinite(endpoint).all():
        raise ValueError("The arm extension must have a finite endpoint.")
    endpoint[1] = target_y
    if t < 0.0:
        status = "opposite_direction"
    elif t < 1.0 - tolerance:
        status = "before_clearance_point"
    elif abs(t - 1.0) <= tolerance:
        status = "at_clearance_point"
    else:
        status = "extended"
    return {"status": status, "t": t, "endpoint": endpoint,
            "direction": direction, "is_extension": t >= 1.0 - tolerance}


def build_arm_mount_geometry(front_plane, kingpins, circle_plane_intersections, *, arm,
                             axle="front"):
    """Build one axle's arm pair without changing any input points.

    ``arm`` selects ``upper`` (UBJ, cpb_ru, UFI/URI) or ``lower``
    (LBJ, cpb_rl, LFI/LRI). Each side uses its own original FV pickup's
    lateral position. The finite endpoints are named by descending X.
    ``axle`` selects front FL/FR or rear RL/RR. The ``front_plane`` argument
    name is retained for compatibility and receives that axle's FV plane.
    """
    if arm == "upper":
        joint, pickup, prefix = "UBJ", "cpb_ru", "U"
    elif arm == "lower":
        joint, pickup, prefix = "LBJ", "cpb_rl", "L"
    else:
        raise ValueError("Arm must be 'upper' or 'lower'.")
    if axle == "front":
        corners = ("FL", "FR")
    elif axle == "rear":
        corners = ("RL", "RR")
    else:
        raise ValueError("Axle must be 'front' or 'rear'.")

    result = {}
    for corner, fv_side in zip(corners, (front_plane.left_sus, front_plane.right_sus)):
        fv_point = np.asarray(getattr(fv_side, pickup).pos, dtype=float)
        if fv_point.shape != (2,) or not np.isfinite(fv_point).all():
            raise ValueError(f"{corner}: the FV {arm} chassis pickup must contain two finite coordinates.")
        target_y = float(-fv_point[0])
        FV_mount = np.array([front_plane.longitudinal_x, target_y, fv_point[1]], dtype=float)
        if not np.isfinite(FV_mount).all():
            raise ValueError(f"{corner}: the projected FV {arm} chassis pickup must be finite.")
        joint_point = _vector3(kingpins[corner][joint], f"{corner} {joint}")
        selected = circle_plane_intersections[f"{corner}_{arm}"]["selected"]
        if len(selected) > 2:
            raise ValueError(f"Each {axle} {arm} arm must have at most two selected points.")
        legs = []
        for rank, candidate in enumerate(selected, start=1):
            point = _vector3(candidate["point"], f"{corner} {prefix}{rank}")
            extension = extend_arm_leg(joint_point, point, target_y)
            legs.append({
                "selected_rank": rank,
                "point": point,
                "pose": candidate.get("pose"),
                "angle_deg": candidate.get("angle_deg"),
                "poses": list(candidate.get("poses", [])),
                "angles_deg": list(candidate.get("angles_deg", [])),
                "name": None,
                **extension,
            })

        finite_legs = sorted(
            (leg for leg in legs if leg["endpoint"] is not None),
            key=lambda leg: (-float(leg["endpoint"][0]), leg["selected_rank"]),
        )
        mounts = {}
        for name, leg in zip((f"{prefix}FI", f"{prefix}RI"), finite_legs):
            leg["name"] = name
            mounts[name] = leg["endpoint"].copy()
        result[corner] = {
            "corner": corner,
            joint: joint_point,
            "target_y": target_y,
            "FV_mount": FV_mount,
            "legs": legs,
            "mounts": mounts,
        }
    return result
