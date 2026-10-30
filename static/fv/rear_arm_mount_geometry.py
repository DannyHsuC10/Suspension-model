"""Extend rear A-arm rays through their clearance points to chassis lateral Y."""

from static.fv.arm_mount_geometry import build_arm_mount_geometry


def build_rear_upper_arm_mount_geometry(rear_plane, kingpins, intersections):
    """Return RL/RR upper arms at each FV pickup's ``target_y = -cpb_ru.x``.

    Rays run from UBJ through UF/UR. X and Z follow each ray naturally;
    endpoints are UFI/URI in descending vehicle X order. The returned
    legs retain their extension parameters, statuses and selected points.
    """
    return build_arm_mount_geometry(
        rear_plane, kingpins, intersections, arm="upper", axle="rear",
    )


def build_rear_lower_arm_mount_geometry(rear_plane, kingpins, intersections):
    """Return RL/RR lower arms at each FV pickup's ``target_y = -cpb_rl.x``.

    Rays run from LBJ through LF/LR. X and Z follow each ray naturally;
    endpoints are LFI/LRI in descending vehicle X order. The returned
    legs retain their extension parameters, statuses and selected points.
    """
    return build_arm_mount_geometry(
        rear_plane, kingpins, intersections, arm="lower", axle="rear",
    )
