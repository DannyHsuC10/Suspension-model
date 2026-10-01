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
        self.tf = 1.28  # Front track width [m]
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

        # Vehicle body outline used as a background in the front-view plot.
        # Coordinates are measured from the vehicle centerline [m].
        self.body_face_f = [
            (0.2, self.bottom),
            (-0.2, self.bottom),
            (-0.2, 0.5),
            (0.2, 0.5),
        ]


class SuspensionF:
    """Front suspension hard points and side/front-view setup parameters."""

    def __init__(self):
        # ------------------------------------------------------------------
        # Front-view suspension geometry
        # ------------------------------------------------------------------
        self.scrub_radius = 18 / 1000  # Scrub radius [m]
        self.kpi = np.deg2rad(10)  # Kingpin inclination [rad]
        self.h_rc = 25 / 1000  # Target roll-center height [m]
        self.fvsa = 1000 / 1000  # Front-view swing-arm length [m]

        self.Au = 85 / 1000  # Upper upright point offset from wheel center [m]
        self.Al = 90 / 1000  # Lower upright point offset from wheel center [m]

        # Chassis-side hard points: RL, LL, LU, RU, in that order [m].
        self.sus_contact = [
            (0.205, 0.08),
            (-0.205, 0.08),
            (-0.250, 0.300),
            (0.250, 0.205),
        ]
        self.front_view_target_travel = 0.025  # Kinematics sweep half-range [m]
        self.front_view_mapping_steps = 30  # Sample count per side in mapping
        self.front_view_figure_size = (8, 6)  # Figure size [inches]
        self.front_view_slider_axes = (
            (0.2, 0.12, 0.6, 0.03),
            (0.2, 0.05, 0.6, 0.03),
        )  # Right and left slider axes [figure fraction]
        self.front_view_plot_bottom = 0.25  # Space reserved for sliders
        self.front_view_x_limits = (-1.5, 1.5)  # Plot bounds [m]
        self.front_view_y_limits = (-0.5, 1.0)  # Plot bounds [m]

        # ------------------------------------------------------------------
        # Side-view suspension geometry and anti settings
        # ------------------------------------------------------------------
        self.mechanical_trail_sv = 15 / 1000  # Mechanical trail [m]
        self.caster_sv_deg = 4.0  # Caster angle [deg]
        self.kingpin_length_sv = 0.6  # Drawn kingpin-axis length [m]
        self.front_brake_bias_sv = 0.7  # Fraction of braking force at front axle
        self.target_anti_dive_sv = 0.5  # Anti-dive fraction (0.5 means 50%)
        self.target_anti_lift_sv = 0.0  # Anti-lift fraction (0.0 means 0%)

        # Side-view reference coordinates [m]. The contact point is at origin.
        self.contact_point_sv = np.array([0.0, 0.0])
        self.wheel_center_x_sv = 0.0

        # ------------------------------------------------------------------
        # Side-view visualization and slider settings
        # ------------------------------------------------------------------
        self.side_view_travel_limit = 0.025  # Slider travel either side [m]
        self.side_view_path_half_length = 0.5  # Displayed path half-length [m]
        self.side_view_wheel_outline_points = 48  # Tire polygon resolution
        self.side_view_figure_size = (10, 8)  # Figure size [inches]
        self.side_view_slider_axes = (
            (0.16, 0.10, 0.72, 0.03),  # Front slider [figure fraction]
            (0.16, 0.05, 0.72, 0.03),  # Rear slider [figure fraction]
        )
        self.side_view_plot_bottom = 0.2  # Space reserved for slider

        # Compatibility aliases retained for older side-view scripts.
        self.mechanical_tail = self.mechanical_trail_sv
        self.caster = self.caster_sv_deg

class SuspensionR:
    """Rear suspension settings; defaults currently mirror SuspensionF."""

    def __init__(self):
        # ------------------------------------------------------------------
        # Front-view suspension geometry
        # ------------------------------------------------------------------
        self.scrub_radius = 18 / 1000  # Scrub radius [m]
        self.kpi = np.deg2rad(10)  # Kingpin inclination [rad]
        self.h_rc = 25 / 1000  # Target roll-center height [m]
        self.fvsa = 1000 / 1000  # Front-view swing-arm length [m]

        self.Au = 85 / 1000  # Upper upright point offset from wheel center [m]
        self.Al = 90 / 1000  # Lower upright point offset from wheel center [m]

        # Chassis-side hard points: RL, LL, LU, RU, in that order [m].
        self.sus_contact = [
            (0.205, 0.08),
            (-0.205, 0.08),
            (-0.250, 0.300),
            (0.250, 0.205),
        ]
        self.front_view_target_travel = 0.025  # Kinematics sweep half-range [m]
        self.front_view_mapping_steps = 30  # Sample count per side in mapping
        self.front_view_figure_size = (8, 6)  # Figure size [inches]
        self.front_view_slider_axes = (
            (0.2, 0.12, 0.6, 0.03),
            (0.2, 0.05, 0.6, 0.03),
        )  # Right and left slider axes [figure fraction]
        self.front_view_plot_bottom = 0.25  # Space reserved for sliders
        self.front_view_x_limits = (-1.5, 1.5)  # Plot bounds [m]
        self.front_view_y_limits = (-0.5, 1.0)  # Plot bounds [m]

        # ------------------------------------------------------------------
        # Side-view suspension geometry and anti settings
        # ------------------------------------------------------------------
        self.mechanical_trail_sv = 0 / 1000  # Mechanical trail [m]
        self.caster_sv_deg = 0.0  # Caster angle [deg]
        self.kingpin_length_sv = 0.6  # Drawn kingpin-axis length [m]
        self.front_brake_bias_sv = 0.7  # Fraction of braking force at front axle
        self.target_anti_dive_sv = 0.5  # Anti-dive fraction (0.5 means 50%)
        self.target_anti_lift_sv = 0.0  # Anti-lift fraction (0.0 means 0%)

        # Side-view reference coordinates [m]. The contact point is at origin.
        self.contact_point_sv = np.array([0.0, 0.0])
        self.wheel_center_x_sv = 0.0

        # ------------------------------------------------------------------
        # Side-view visualization and slider settings
        # ------------------------------------------------------------------
        self.side_view_travel_limit = 0.025  # Slider travel either side [m]
        self.side_view_path_half_length = 0.5  # Displayed path half-length [m]
        self.side_view_wheel_outline_points = 48  # Tire polygon resolution
        self.side_view_figure_size = (10, 8)  # Figure size [inches]
        self.side_view_slider_axes = (
            (0.16, 0.10, 0.72, 0.03),
            (0.16, 0.05, 0.72, 0.03),
        )
        self.side_view_plot_bottom = 0.2  # Space reserved for slider

        # Compatibility aliases retained for older side-view scripts.
        self.mechanical_tail = self.mechanical_trail_sv
        self.caster = self.caster_sv_deg
