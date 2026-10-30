"""Rectangular tire and rim-inner-envelope projections on the two FV planes.

All coordinates and dimensions use metres.  A rim's available inner diameter
defines the height of its projected inner envelope, not its outer diameter.
"""

import numpy as np


def _positive_length(value, name):
    length = float(value)
    if not np.isfinite(length) or length <= 0.0:
        raise ValueError(f"{name} must be finite and positive.")
    return length


def _rectangle(center, width, height):
    """Return an upright rectangle whose longitudinal coordinate is fixed."""
    offsets = np.array([
        [0.0, -width / 2.0, -height / 2.0],
        [0.0, width / 2.0, -height / 2.0],
        [0.0, width / 2.0, height / 2.0],
        [0.0, -width / 2.0, height / 2.0],
    ])
    vertices = center + offsets
    return {
        "center": center.copy(),
        "width": width,
        "height": height,
        "vertices": vertices,
        "outline": np.vstack((vertices, vertices[0])),
    }


def build_wheel_projection_geometry(car, front_plane, rear_plane, *, rim_width=None):
    """Return the tire and rim-inner rectangles for FL/FR/RL/RR.

    The center maps each original FV wheel center to vehicle coordinates
    ``[axle_X, -FV_X, FV_Y]``. Tire height and width use that side's tire
    radius and section width. Rim height uses ``car.rim_inner_diameter``.

    An explicit ``rim_width`` overrides ``car.rim_width``. If neither is
    supplied, the tire width is used only as a provisional display width,
    recorded as ``rim['width_source'] == 'tire_width_placeholder'``. A real
    rim width can be provided later without moving existing wheel centers.
    Each entry contains independent tire/rim vertices and closed outlines;
    the source FV objects and vehicle parameters are never modified.
    """
    result = build_axle_wheel_projection_geometry(
        car, front_plane, prefix="F", rim_width=rim_width,
    )
    result.update(build_axle_wheel_projection_geometry(
        car, rear_plane, prefix="R", rim_width=rim_width,
    ))
    return result


def build_axle_wheel_projection_geometry(car, plane, *, prefix, rim_width=None):
    """Return one FV plane's two rectangular wheel projections.

    ``prefix`` is ``F`` for FL/FR or ``R`` for RL/RR. Dimensions, width
    precedence and copy semantics match ``build_wheel_projection_geometry``.
    """
    if prefix not in ("F", "R"):
        raise ValueError("Wheel projection prefix must be 'F' or 'R'.")
    axle = "Front" if prefix == "F" else "Rear"
    rim_height = _positive_length(car.rim_inner_diameter, "Rim inner diameter")
    if rim_width is not None:
        chosen_width = _positive_length(rim_width, "Rim width")
        width_source = "argument"
    elif getattr(car, "rim_width", None) is not None:
        chosen_width = _positive_length(car.rim_width, "Rim width")
        width_source = "car.rim_width"
    else:
        chosen_width = None
        width_source = "tire_width_placeholder"

    axle_x = float(plane.longitudinal_x)
    if not np.isfinite(axle_x):
        raise ValueError(f"{axle}: FV plane X must be finite.")
    result = {}
    for suffix, side_name, side in (
        ("L", "Left", plane.left_sus), ("R", "Right", plane.right_sus),
    ):
        corner = prefix + suffix
        fv_center = np.asarray(side.wheel_center.pos, dtype=float)
        if fv_center.shape != (2,) or not np.isfinite(fv_center).all():
            raise ValueError(f"{corner}: FV wheel center must contain two finite coordinates.")
        center = np.array([axle_x, -fv_center[0], fv_center[1]])
        tire_width = _positive_length(side.tire_width, f"{corner} tire width")
        tire_radius = _positive_length(side.tire_radius, f"{corner} tire radius")
        tire = _rectangle(center, tire_width, 2.0 * tire_radius)
        tire["diameter"] = tire["height"]
        rim = _rectangle(
            center, tire_width if chosen_width is None else chosen_width, rim_height,
        )
        rim.update({
            "inner_diameter": rim_height,
            "width_source": width_source,
            "diameter_source": "car.rim_inner_diameter",
        })
        result[corner] = {
            "corner": corner,
            "axle": axle,
            "side": side_name,
            "center": center.copy(),
            "longitudinal_x": axle_x,
            "tire": tire,
            "rim": rim,
        }
    return result
