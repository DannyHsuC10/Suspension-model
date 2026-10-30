"""Render the eight static arm reference triangles without moving their vertices."""

import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


UPPER_ARM_COLOR = "#7c3aed"
LOWER_ARM_COLOR = "#0891b2"


def _clip_polygon_to_box(vertices, bounds):
    """Clip a display-only copy of a convex polygon against six half-spaces."""
    polygon = np.array(vertices, dtype=float, copy=True)
    for dimension, limits in enumerate(bounds):
        for limit, keep_above in ((limits[0], True), (limits[1], False)):
            if len(polygon) == 0:
                return np.empty((0, 3), dtype=float)
            result = []
            previous = polygon[-1]
            previous_inside = (
                previous[dimension] >= limit if keep_above
                else previous[dimension] <= limit
            )
            for current in polygon:
                current_inside = (
                    current[dimension] >= limit if keep_above
                    else current[dimension] <= limit
                )
                if current_inside != previous_inside:
                    fraction = (limit - previous[dimension]) / (
                        current[dimension] - previous[dimension]
                    )
                    intersection = previous + fraction * (current - previous)
                    intersection[dimension] = limit
                    result.append(intersection)
                if current_inside:
                    result.append(current.copy())
                previous, previous_inside = current, current_inside
            polygon = np.asarray(result, dtype=float).reshape(-1, 3)

    # A vertex exactly on a clipping boundary can be emitted twice.
    unique = []
    for point in polygon:
        if not unique or not np.allclose(point, unique[-1], rtol=0.0, atol=1e-12):
            unique.append(point.copy())
    if len(unique) > 1 and np.allclose(unique[0], unique[-1], rtol=0.0, atol=1e-12):
        unique.pop()
    return np.asarray(unique, dtype=float).reshape(-1, 3)


def draw_arm_planes(ax, arm_planes, alpha, bounds=None, clip_segment=None, *,
                    corner_colors=None, sv_color=None, show_faces=True,
                    visible_fv_corners=None, show_sv_anchors=True):
    """Draw each exact triangle, or only its portion within a detail-view box.

    The returned mapping keeps the same eight keys as ``arm_planes``.  Each
    entry contains ``surface``, visible original ``edges``, display-only
    ``display_vertices`` and reused FV/SV ``anchors``.  Clipping never edits
    the source vertices and never draws artificial edges along the cut.
    With bounds, the caller supplies its existing segment-clipping function.
    Optional corner colors group each triangle and its FV anchor by wheel;
    ``sv_color`` independently colors the shared central SV anchors.
    ``show_faces=False`` omits filled surfaces while retaining edges,
    anchors and the same display vertices.
    ``visible_fv_corners`` optionally limits FV anchor markers/labels to
    the supplied wheel corners; ``show_sv_anchors`` controls SV markers/labels.
    Hidden anchors still retain their exact positions in the returned mapping.
    """
    alpha = float(alpha)
    if not np.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
        raise ValueError("Arm-plane opacity must be between zero and one.")
    if bounds is not None:
        bounds = np.array(bounds, dtype=float, copy=True)
        if (bounds.shape != (3, 2) or not np.isfinite(bounds).all()
                or np.any(bounds[:, 0] > bounds[:, 1])):
            raise ValueError("Arm-plane display bounds must contain three finite min/max pairs.")
        if clip_segment is None:
            raise ValueError("A segment-clipping function is required for a bounded arm-plane view.")

    visible_fv_corners = (
        None if visible_fv_corners is None else frozenset(visible_fv_corners)
    )
    anchor_cache = {}

    def anchor(kind, identity, position, color=None):
        key = (kind, identity)
        if key in anchor_cache:
            return anchor_cache[key]
        position = np.array(position, dtype=float, copy=True)
        visible = bounds is None or bool(
            np.all(position >= bounds[:, 0]) and np.all(position <= bounds[:, 1])
        )
        if kind == "FV":
            visible = visible and (
                visible_fv_corners is None or identity in visible_fv_corners
            )
        else:
            visible = visible and show_sv_anchors
        marker, label = None, None
        if visible:
            marker = ax.scatter(
                *position, marker="o" if kind == "FV" else "X",
                color=color if color is not None else (
                    "#3f365f" if kind == "FV" else "#334155"
                ),
                s=22 if kind == "FV" else 35, depthshade=False,
                edgecolor="white", linewidth=0.4,
            )
            label = ax.text(
                *(position + [0.025, 0.03, 0.025]),
                f"{kind} IC {identity}", fontsize=7,
                color=color if color is not None else "0.2",
            )
        result = {"marker": marker, "label": label, "position": position}
        anchor_cache[key] = result
        return result

    artists = {}
    for name, geometry in arm_planes.items():
        vertices = np.array(geometry["vertices"], dtype=float, copy=True)
        if vertices.shape != (3, 3) or not np.isfinite(vertices).all():
            raise ValueError(f"{name}: an arm plane must have three finite 3D vertices.")
        color = UPPER_ARM_COLOR if geometry["arm"] == "upper" else LOWER_ARM_COLOR
        fv_color = None if corner_colors is None else corner_colors.get(geometry["corner"])
        if fv_color is not None:
            color = fv_color
        display_vertices = vertices.copy() if bounds is None else _clip_polygon_to_box(vertices, bounds)
        surface = None
        if show_faces and len(display_vertices) >= 3:
            surface = Poly3DCollection(
                [display_vertices], facecolor=color, edgecolor="none",
                alpha=alpha, linewidth=0,
            )
            ax.add_collection3d(surface)
        edges = []
        for index in range(3):
            segment = vertices[[index, (index + 1) % 3]].copy()
            if bounds is not None:
                segment = clip_segment(segment, bounds)
                if segment is None:
                    continue
            line, = ax.plot(*np.asarray(segment).T, color=color, linewidth=0.8, alpha=0.5)
            edges.append(line)
        artists[name] = {
            "surface": surface,
            "edges": edges,
            "display_vertices": display_vertices.copy(),
            "anchors": {
                "FV_IC": anchor("FV", geometry["corner"], geometry["FV_IC"], fv_color),
                "SV_IC": anchor("SV", geometry["axle"], geometry["SV_IC"], sv_color),
            },
        }
    return artists
