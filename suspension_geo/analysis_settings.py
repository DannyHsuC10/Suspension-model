"""Analysis, plotting and UI settings, separate from vehicle design parameters."""


class AnalysisSettings:
    """Shared sweep and view settings for the suspension geometry tools."""

    def __init__(self):
        # Front-view analysis and mapping (not physical suspension limits).
        self.front_view_target_travel = 0.025  # Sweep half-range [m]
        self.front_view_mapping_steps = 30  # Sample count per wheel

        # Front-view plot and sliders.
        self.front_view_figure_size = (8, 6)  # Figure size [inches]
        self.front_view_slider_axes = (
            (0.2, 0.12, 0.6, 0.03),  # Right slider [figure fraction]
            (0.2, 0.05, 0.6, 0.03),  # Left slider [figure fraction]
        )
        self.front_view_plot_bottom = 0.25  # Space reserved for sliders
        self.front_view_x_limits = (-1.5, 1.5)  # Plot bounds [m]
        self.front_view_y_limits = (-0.5, 1.0)  # Plot bounds [m]
        self.front_view_body_half_width = 0.21  # Background outline [m]
        self.front_view_body_top = 0.5  # Background outline height [m]

        # Side-view reference coordinates and drawn construction lines.
        self.contact_point_sv = (0.0, 0.0)  # Local contact reference [m]
        self.wheel_center_x_sv = 0.0  # Rear axle plot origin [m]
        self.kingpin_length_sv = 0.6  # Drawn kingpin-axis length [m]
        self.side_view_path_half_length = 0.5  # Drawn travel path [m]
        self.side_view_wheel_outline_points = 48  # Tire polygon resolution

        # Side-view plot and sliders; retain separate front/rear travel ranges.
        self.side_view_front_travel_limit = 0.03  # Slider half-range [m]
        self.side_view_rear_travel_limit = 0.025  # Slider half-range [m]
        self.side_view_figure_size = (10, 8)  # Figure size [inches]
        self.side_view_slider_axes = (
            (0.16, 0.10, 0.72, 0.03),  # Front slider [figure fraction]
            (0.16, 0.05, 0.72, 0.03),  # Rear slider [figure fraction]
        )
        self.side_view_plot_bottom = 0.2  # Space reserved for sliders

    def front_view_body_outline(self, bottom):
        """Build the plot background using the vehicle's ground clearance [m]."""
        half_width = self.front_view_body_half_width
        top = self.front_view_body_top
        return [
            (half_width, bottom),
            (-half_width, bottom),
            (-half_width, top),
            (half_width, top),
        ]
