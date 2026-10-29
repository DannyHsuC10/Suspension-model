"""Static plotting settings, separate from vehicle design parameters."""


class AnalysisSettings:
    """Shared static view settings for the suspension geometry tools."""

    def __init__(self):
        # Static front/rear FV planes shown in a single 3D view.
        self.front_view_figure_size = (12, 8)  # Figure size [inches]
        self.front_view_elevation_deg = 22.0  # Initial camera elevation [deg]
        self.front_view_azimuth_deg = 35.0  # Initial camera azimuth [deg]
        self.front_view_x_limits = (-1.5, 1.5)  # FV transverse bounds [m]
        self.front_view_y_limits = (-0.1, 0.65)  # FV height bounds [m]
        self.chassis_face_alpha = 0.16  # Chassis hexahedron face opacity [0..1]

        # Side-view reference coordinates and drawn construction lines.
        self.contact_point_sv = (0.0, 0.0)  # Local contact reference [m]
        self.wheel_center_x_sv = 0.0  # Rear axle plot origin [m]
        self.kingpin_length_sv = 0.6  # Drawn kingpin-axis length [m]
        self.side_view_tangent_half_length = 0.5  # Static tangent reference [m]
        self.side_view_wheel_outline_points = 48  # Tire polygon resolution

        # Static side-view plot.
        self.side_view_figure_size = (10, 8)  # Figure size [inches]

