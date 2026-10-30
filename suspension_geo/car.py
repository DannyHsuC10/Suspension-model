"""Vehicle, tire and suspension design parameters.

Analysis ranges and plotting/UI options live in analysis_settings.py.
"""

import numpy as np


class Car:
    """Vehicle and tire parameters shared by the front-view analyses."""

    def __init__(self):
        # ------------------------------------------------------------------
        # Vehicle parameters
        # ------------------------------------------------------------------
        self.gravity = 9.81  # Gravitational acceleration [m/s^2]
        self.l = 1.53  # Wheelbase [m]
        self.lf = 0.7498400000000001  # CG to front axle [m]
        self.lr = self.l - self.lf  # CG to rear axle [m]
        self.tf = 1.25  # Front track width [m]
        self.tr = 1.24  # Rear track width [m]
        self.h_cog = 0.3016451511  # CG height above ground [m]
        self.m = 321.0  # Vehicle mass [kg]
        self.w = self.m * self.gravity  # Vehicle weight [N]
        self.cg_rate = np.array([self.lr / self.l, self.lf / self.l])
        self.bottom = 0.050  # Chassis bottom clearance [m]

        # ------------------------------------------------------------------
        # Tire parameters
        # ------------------------------------------------------------------
        self.tir = "tire.tir"  # Tire parameter filename, if used by a model
        self.free_radius = 205.73999999999998 / 1000  # Unloaded radius [m]
        self.K_tire = 80.185 * 1000  # Vertical tire stiffness [N/m] (W8)
        self.tire_width = 190.5 / 1000  # Tire section width [m]

        # Static loaded tire radii for front and rear axles [m].
        self.load_Radius = (
            self.free_radius
            - self.m * self.cg_rate * self.gravity / self.K_tire / 2
        )


class SuspensionF:
    """Front suspension hard points and side/front-view design parameters."""

    def __init__(self):
        # ------------------------------------------------------------------
        # Front-view suspension geometry
        # ------------------------------------------------------------------
        self.scrub_radius = 25 / 1000  # Scrub radius [m]
        self.kpi = np.deg2rad(12)  # Kingpin inclination [rad]
        self.h_rc = 35 / 1000  # Target roll-center height [m]
        self.fvsa = 1000 / 1000  # Front-view swing-arm length [m]

        self.Au = 85 / 1000  # Upper upright point offset from wheel center [m]
        self.Al = 90 / 1000  # Lower upright point offset from wheel center [m]

        # Chassis-side X [m]: right positive, left mirrored negative.
        # Y is solved from each upright joint and the design instant center.
        self.lower_mount_x = 210 / 1000
        self.upper_mount_x = 255 / 1000

        # ------------------------------------------------------------------
        # Side-view suspension geometry and anti settings
        # ------------------------------------------------------------------
        self.mechanical_trail_sv = 15 / 1000  # Mechanical trail [m]
        self.caster_sv_deg = 5  # Caster angle [deg]
        self.front_brake_bias_sv = 0.7  # Fraction of braking force at front axle
        self.target_anti_dive_sv = 0.15  # Anti-dive fraction (0.5 means 50%)
        self.target_anti_lift_sv = 0.0  # Anti-lift fraction (0.0 means 0%)

        # Compatibility aliases retained for older side-view scripts.
        self.mechanical_tail = self.mechanical_trail_sv
        self.caster = self.caster_sv_deg


class SuspensionR:
    """Rear suspension hard points and side/front-view design parameters."""

    def __init__(self):
        # ------------------------------------------------------------------
        # Front-view suspension geometry
        # ------------------------------------------------------------------
        self.scrub_radius = 18 / 1000  # Scrub radius [m]
        self.kpi = np.deg2rad(0)  # Kingpin inclination [rad]
        self.h_rc = 25 / 1000  # Target roll-center height [m]
        self.fvsa = 2000 / 1000  # Front-view swing-arm length [m]

        self.Au = 85 / 1000  # Upper upright point offset from wheel center [m]
        self.Al = 90 / 1000  # Lower upright point offset from wheel center [m]

        # Chassis-side X [m]: right positive, left mirrored negative.
        # Y is solved from each upright joint and the design instant center.
        self.lower_mount_x = 205 / 1000
        self.upper_mount_x = 250 / 1000

        # ------------------------------------------------------------------
        # Side-view suspension geometry and anti settings
        # ------------------------------------------------------------------
        self.mechanical_trail_sv = 0 / 1000  # Mechanical trail [m]
        self.caster_sv_deg = 0.0  # Caster angle [deg]
        self.front_brake_bias_sv = 0.7  # Fraction of braking force at front axle
        self.target_anti_dive_sv = 0.15  # Anti-dive fraction (0.5 means 50%)
        self.target_anti_lift_sv = 0.0  # Anti-lift fraction (0.0 means 0%)

        # Compatibility aliases retained for older side-view scripts.
        self.mechanical_tail = self.mechanical_trail_sv
        self.caster = self.caster_sv_deg
