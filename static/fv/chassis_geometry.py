"""Construct a static chassis envelope from eight FV inboard arm points."""

import numpy as np


def build_chassis_geometry(front_plane, rear_plane):
    """Return eight exact mounting-point vertices and six connected faces.

    Each FV ``cpb_ru``/``cpb_rl`` is the upper/lower arm's equivalent
    chassis-side point.  Its 3D coordinate is ``(axle X, -FV X, FV Y)``.
    The upper and lower points remain separate; no averaging, height
    extension, or extra mounting-point assumptions are introduced.

    ``vertices`` maps ``FL_upper`` through ``RR_lower`` to 3D arrays.
    ``face_vertices`` gives the outward winding of six conceptual quad
    faces, ``faces`` contains their coordinates, and ``edges`` lists the
    twelve unique vertex-name pairs.  A renderer may triangulate a quad
    if differing front/rear geometry makes that face nonplanar.
    """
    vertices = {}
    for axle_prefix, plane in (("F", front_plane), ("R", rear_plane)):
        longitudinal_x = float(plane.longitudinal_x)
        if not np.isfinite(longitudinal_x):
            raise ValueError("Chassis axle positions must be finite.")
        for side_suffix, side in (("L", plane.left_sus), ("R", plane.right_sus)):
            for level, point_name in (("upper", "cpb_ru"), ("lower", "cpb_rl")):
                point = np.asarray(getattr(side, point_name).pos, dtype=float)
                if point.shape != (2,) or not np.isfinite(point).all():
                    raise ValueError("Chassis FV mounting points must have two finite coordinates.")
                vertices[f"{axle_prefix}{side_suffix}_{level}"] = np.array(
                    [longitudinal_x, -point[0], point[1]], dtype=float
                )

    face_vertices = {
        "front": ("FR_lower", "FL_lower", "FL_upper", "FR_upper"),
        "rear": ("RR_lower", "RR_upper", "RL_upper", "RL_lower"),
        "left": ("FL_lower", "RL_lower", "RL_upper", "FL_upper"),
        "right": ("FR_lower", "FR_upper", "RR_upper", "RR_lower"),
        "top": ("FL_upper", "RL_upper", "RR_upper", "FR_upper"),
        "bottom": ("FL_lower", "FR_lower", "RR_lower", "RL_lower"),
    }
    faces = {
        name: np.array([vertices[key] for key in keys])
        for name, keys in face_vertices.items()
    }

    edges = []
    seen_edges = set()
    for keys in face_vertices.values():
        for start, end in zip(keys, (*keys[1:], keys[0])):
            identity = frozenset((start, end))
            if identity not in seen_edges:
                seen_edges.add(identity)
                edges.append((start, end))

    return {
        "vertices": vertices,
        "faces": faces,
        "face_vertices": face_vertices,
        "edges": edges,
    }
