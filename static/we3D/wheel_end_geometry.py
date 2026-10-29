"""Static single-wheel geometry, in metres and radians.

The fixed origin lies on the kingpin at initial wheel-center height.
Keep the original sus_3Dwe_v3 axis signs. At z=-tire_radius, positive
mechanical trail places the tire-bottom reference along +X from the
kingpin intersection; positive scrub places it along -Y (wheel outward).
These follow the wheel-end/SV/FV axis signs.
"""

import numpy as np


def kingpin_axis(suspension):
    """Return the upward unit axis using the original v3 direction formula."""
    kpi = float(suspension.kpi)
    caster = np.deg2rad(float(suspension.caster_sv_deg))
    if (not np.all(np.isfinite([kpi, caster]))
            or abs(kpi) >= np.pi / 2.0 or abs(caster) >= np.pi / 2.0):
        raise ValueError("KPI and caster must be finite and strictly between -90 and 90 deg.")
    axis = np.array([
        np.sin(caster), np.sin(kpi), np.cos(kpi) * np.cos(caster),
    ])
    return axis / np.linalg.norm(axis)


def initial_wheel_center(suspension, tire_radius):
    """Offset the wheel so ground-plane trail/scrub equal the input lengths.

    Kingpin/ground intersection G = -radius * axis / axis_z.
    Initial tire-bottom P0 = G + [trail, -scrub, 0].
    Initial wheel center C0 = P0 + [0, 0, radius].
    Thus C0.z=0 but C0 generally does not lie on the kingpin.
    """
    radius = float(tire_radius)
    trail = float(suspension.mechanical_trail_sv)
    scrub = float(suspension.scrub_radius)
    if not np.all(np.isfinite([radius, trail, scrub])) or radius <= 0.0:
        raise ValueError("Tire radius must be positive; radius, trail and scrub must be finite.")
    axis = kingpin_axis(suspension)
    if axis[2] < 1e-10:
        raise ValueError("Kingpin is too close to horizontal to define ground offsets.")
    center = -radius * axis / axis[2] + np.array([trail, -scrub, radius])
    center[2] = 0.0
    return center
