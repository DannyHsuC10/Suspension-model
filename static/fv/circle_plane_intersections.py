"""Exact front/rear clearance-circle intersections with extended arm planes."""

import numpy as np


def _vector3(value, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must contain three finite coordinates.")
    return vector


def _unit_normal(value, name):
    vector = _vector3(value, name)
    length = np.linalg.norm(vector)
    if not np.isfinite(length) or length < 1e-12:
        raise ValueError(f"{name} must have a finite nonzero length.")
    return vector / length


def circle_plane_intersections(center, radius, circle_normal, plane_point,
                               plane_normal, *, tolerance=1e-10):
    """Intersect a 3D circle with an infinite plane analytically.

    ``status`` is ``two_points``, ``tangent``, ``none`` or ``coplanar``.
    The latter denotes infinitely many intersections and deliberately returns
    an empty point array.  Discrete results use an ``(N, 3)`` array, independent
    of the circle's display sampling.  Lengths and tolerance use metres.
    """
    center = _vector3(center, "Circle centre")
    plane_point = _vector3(plane_point, "Plane point")
    circle_normal = _unit_normal(circle_normal, "Circle normal")
    plane_normal = _unit_normal(plane_normal, "Plane normal")
    radius = float(radius)
    tolerance = float(tolerance)
    if not np.isfinite(radius) or radius <= 0.0:
        raise ValueError("Circle radius must be finite and positive.")
    if not np.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("Intersection tolerance must be finite and positive.")

    signed_distance = float(np.dot(plane_normal, center - plane_point))
    in_circle_normal = plane_normal - np.dot(plane_normal, circle_normal) * circle_normal
    projection_length = np.linalg.norm(in_circle_normal)
    empty = np.empty((0, 3), dtype=float)
    if projection_length < 1e-12:
        status = "coplanar" if abs(signed_distance) <= tolerance else "none"
        return {"status": status, "points": empty}

    along = in_circle_normal / projection_length
    offset = -signed_distance / projection_length
    if abs(offset) > radius + tolerance:
        return {"status": "none", "points": empty}
    if abs(abs(offset) - radius) <= tolerance:
        point = center + offset * along
        return {"status": "tangent", "points": point.reshape(1, 3)}

    chord_center = center + offset * along
    chord_direction = np.cross(circle_normal, along)
    half_chord = np.sqrt(max(0.0, radius * radius - offset * offset))
    points = np.array([
        chord_center + half_chord * chord_direction,
        chord_center - half_chord * chord_direction,
    ])
    return {"status": "two_points", "points": points}


def build_circle_plane_intersections(arm_planes, interference_geometry):
    """Select each steering circle's nearest-XZ intersection, per front arm.

    Four entries (``FL_upper``, ``FL_lower``, ``FR_upper``, ``FR_lower``)
    contain ``by_pose`` analytic results, all unique discrete ``candidates``
    and at most two ``selected`` points. Each steering pose independently
    contributes its intersection with the smallest ``abs(Y)``. Selected
    points are ordered positive then negative, retaining each pose even
    when their coordinates coincide. A pose without discrete intersections
    contributes no selected point; the other pose never supplies a replacement.

    Each candidate includes ``point``, ``pose``, ``angle_deg`` and
    ``distance_to_xz``.  ``poses``/``angles_deg`` retain both sources when
    steering poses produce the same point.  Arm triangles define extended
    planes: candidates are not limited to the filled triangle's boundary.
    Source arrays are never modified.
    """
    result = {}
    for corner in ("FL", "FR"):
        wheel = interference_geometry[corner]
        for arm in ("upper", "lower"):
            key = f"{corner}_{arm}"
            plane = arm_planes[key]
            plane_point = np.asarray(plane["vertices"], dtype=float)[0]
            plane_normal = plane["normal"]
            candidates = []
            selected = []
            by_pose = {}
            for pose in ("positive", "negative"):
                circle = wheel["circles"][pose]
                intersections = circle_plane_intersections(
                    circle["center"], wheel["radius"], circle["normal"],
                    plane_point, plane_normal,
                )
                angle = float(circle["angle_deg"])
                by_pose[pose] = {
                    "status": intersections["status"],
                    "angle_deg": angle,
                    "points": intersections["points"].copy(),
                }
                if len(intersections["points"]):
                    nearest = min(intersections["points"], key=lambda point: (
                        float(abs(point[1])), float(point[0]), float(point[2]),
                    ))
                    selected.append({
                        "point": nearest.copy(),
                        "pose": pose,
                        "angle_deg": angle,
                        "poses": [pose],
                        "angles_deg": [angle],
                        "distance_to_xz": float(abs(nearest[1])),
                    })
                for point in intersections["points"]:
                    duplicate = next((candidate for candidate in candidates
                                      if np.linalg.norm(candidate["point"] - point) <= 1e-9), None)
                    if duplicate is not None:
                        duplicate["poses"].append(pose)
                        duplicate["angles_deg"].append(angle)
                        continue
                    candidates.append({
                        "point": point.copy(),
                        "pose": pose,
                        "angle_deg": angle,
                        "poses": [pose],
                        "angles_deg": [angle],
                        "distance_to_xz": float(abs(point[1])),
                    })

            candidates.sort(key=lambda candidate: (
                candidate["distance_to_xz"], float(candidate["point"][0]),
                float(candidate["point"][2]), candidate["pose"],
            ))
            result[key] = {
                "corner": corner,
                "arm": arm,
                "joint": plane["joint"],
                "by_pose": by_pose,
                "candidates": candidates,
                "selected": selected,
            }
    return result


def build_front_lower_intersection_candidates(front_intersections):
    """Label every actual front-lower intersection without selecting any.

    Each FL/FR record keeps the original per-pose analytic status and all
    discrete points. Positive steering uses a/b, negative steering c/d;
    within each pose, larger vehicle X comes first, then larger Z. Labels
    belonging to a missing pose remain absent, rather than being reassigned
    to the other circle. A tangent uses its pose's first letter only.
    Coincident coordinates from different steering poses retain separate
    labels. Returned point arrays are copies and do not alter selection.
    """
    result = {}
    for corner in ("FL", "FR"):
        source = front_intersections[f"{corner}_lower"]
        points, by_pose, missing_labels = [], {}, []
        for pose, letters in (("positive", ("a", "b")),
                              ("negative", ("c", "d"))):
            pose_source = source["by_pose"][pose]
            raw_points = np.asarray(pose_source["points"], dtype=float)
            if (raw_points.ndim != 2 or raw_points.shape[1] != 3
                    or len(raw_points) > 2 or not np.isfinite(raw_points).all()):
                raise ValueError(f"{corner} {pose}: expected at most two finite 3D intersections.")
            order = sorted(range(len(raw_points)), key=lambda index: (
                -float(raw_points[index, 0]), -float(raw_points[index, 2]),
            ))
            ordered_points = raw_points[order].copy()
            labels = list(letters[:len(ordered_points)])
            angle = float(pose_source["angle_deg"])
            by_pose[pose] = {
                "status": pose_source["status"],
                "angle_deg": angle,
                "points": ordered_points.copy(),
                "labels": labels.copy(),
            }
            missing_labels.extend(letters[len(ordered_points):])
            for name, point in zip(labels, ordered_points):
                points.append({
                    "point": point.copy(),
                    "name": name,
                    "pose": pose,
                    "angle_deg": angle,
                    "distance_to_xz": float(abs(point[1])),
                })
        result[corner] = {
            "corner": corner,
            "arm": "lower",
            "joint": "LBJ",
            "by_pose": by_pose,
            "points": points,
            "point_count": len(points),
            "missing_labels": missing_labels,
        }
    return result


def build_rear_circle_plane_intersections(arm_planes, interference_geometry):
    """Return both neutral-circle intersections for each rear A-arm plane.

    Entries ``RL_upper``/``RL_lower``/``RR_upper``/``RR_lower`` retain the
    analytic ``status``, an ``(N, 3)`` point array and labeled ``selected``
    points. Two intersections are sorted by descending vehicle X and named
    ``UF``/``UR`` for the upper arm, ``LF``/``LR`` for the lower arm; F/R
    denote the forward/rearward point on that wheel's interference circle.
    A single tangent is named ``UT`` or ``LT``. None/coplanar results retain
    their status and empty points, without inventing discrete intersections.
    These are intersections with the infinite arm planes, independent of
    display clipping. All returned coordinates are copies of source data.
    """
    result = {}
    for corner in ("RL", "RR"):
        wheel = interference_geometry[corner]
        circle = wheel["circles"]["neutral"]
        for arm, prefix in (("upper", "U"), ("lower", "L")):
            key = f"{corner}_{arm}"
            plane = arm_planes[key]
            intersections = circle_plane_intersections(
                circle["center"], wheel["radius"], circle["normal"],
                np.asarray(plane["vertices"], dtype=float)[0], plane["normal"],
            )
            raw_points = intersections["points"]
            order = sorted(range(len(raw_points)), key=lambda index: (
                -float(raw_points[index, 0]), -float(raw_points[index, 2]),
            ))
            points = raw_points[order].copy()
            selected = []
            for index, point in enumerate(points):
                if intersections["status"] == "tangent":
                    name, position = f"{prefix}T", "tangent"
                else:
                    name = f"{prefix}{'F' if index == 0 else 'R'}"
                    position = "front" if index == 0 else "rear"
                selected.append({
                    "point": point.copy(),
                    "name": name,
                    "position": position,
                    "pose": "neutral",
                    "angle_deg": float(circle["angle_deg"]),
                })
            result[key] = {
                "corner": corner,
                "arm": arm,
                "joint": plane["joint"],
                "status": intersections["status"],
                "points": points,
                "selected": selected,
            }
    return result
