"""Extend front lower-arm rays to each FV pickup's vehicle lateral Y."""

from static.fv.arm_mount_geometry import build_arm_mount_geometry, extend_arm_leg


def extend_lower_arm_leg(LBJ, selected_point, target_y, *, tolerance=1e-12):
    """Intersect ``LBJ + t * (selected_point - LBJ)`` with a fixed Y plane."""
    return extend_arm_leg(LBJ, selected_point, target_y, tolerance=tolerance)


def build_lower_arm_mount_geometry(front_plane, kingpins, circle_plane_intersections):
    """Return FL/FR lower arms, stopping at ``target_y = -cpb_rl.x``.

    Each entry contains ``LBJ``, ``target_y``, ``FV_mount``, ``legs`` and
    ``mounts``. X and Z follow the ray through the LBJ and its selected
    L1/L2 clearance point. Endpoints are named LFI/LRI by descending X.
    Every leg retains its true extension parameter, status and source pose;
    the original FV pickups and all source arrays remain unchanged.
    """
    return build_arm_mount_geometry(
        front_plane, kingpins, circle_plane_intersections, arm="lower",
    )
