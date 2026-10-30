"""Extend front upper-arm rays to each FV pickup's vehicle lateral Y."""

from static.fv.arm_mount_geometry import build_arm_mount_geometry, extend_arm_leg


def extend_upper_arm_leg(UBJ, selected_point, target_y, *, tolerance=1e-12):
    """Intersect ``UBJ + t * (selected_point - UBJ)`` with a fixed Y plane."""
    return extend_arm_leg(UBJ, selected_point, target_y, tolerance=tolerance)


def build_upper_arm_mount_geometry(front_plane, kingpins, circle_plane_intersections):
    """Return FL/FR upper arms, stopping at ``target_y = -cpb_ru.x``.

    Each entry contains ``UBJ``, ``target_y``, ``FV_mount``, ``legs`` and
    ``mounts``. X and Z follow the ray through the UBJ and its selected
    U1/U2 clearance point. Endpoints are named UFI/URI by descending X.
    Every leg retains its true extension parameter, status and source pose;
    the original FV pickups and all source arrays remain unchanged.
    """
    return build_arm_mount_geometry(
        front_plane, kingpins, circle_plane_intersections, arm="upper",
    )
