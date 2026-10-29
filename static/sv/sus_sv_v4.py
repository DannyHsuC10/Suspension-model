import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from static import geometry as ge
from static import Geometric_Analysis as ga
from static.car import Car, SuspensionF, SuspensionR
from static.analysis_settings import AnalysisSettings
from static.visualization import LinkPlot, PointPlot, PolygonPlot


# Colors identify geometric feature types, not front/rear axles.
STYLE = {
    "wheel": "0.35",
    "kingpin": "tab:blue",
    "svsa": "black",
    "ic_links": "tab:orange",
    "tangent_lines": "tab:purple",
    "anti_lift": "tab:green",
    "hard_points": "tab:blue",
    "contact": "tab:red",
    "ic": "tab:green",
    "pitch_center": "tab:pink",
}


car = Car()
suspension_f = SuspensionF()
suspension_r = SuspensionR()
analysis_settings = AnalysisSettings()


def calc_anti_angle(anti_percent, axle_brake_bias, cg_height, wheelbase):
    """Convert an anti percentage and axle brake share to a side-view angle."""
    tangent = anti_percent * cg_height / (axle_brake_bias * wheelbase)
    return np.arctan(tangent)


class Scene:
    """Static Matplotlib scene using the shared front-view plot helpers."""

    def __init__(self, ax):
        self.ax = ax
        self.objects = []

    def add(self, obj, **kwargs):
        if isinstance(obj, ge.Point):
            self.objects.append(PointPlot(self.ax, obj, **kwargs))
        elif isinstance(obj, ge.Link):
            self.objects.append(LinkPlot(self.ax, obj, **kwargs))
        elif isinstance(obj, ge.Polygon):
            self.objects.append(PolygonPlot(self.ax, obj, **kwargs))


class SideSuspension:
    """One wheel/upright assembly at its initial side-view geometry."""

    def __init__(self, name, suspension, axle_index, direction_sign, center_x, color,
                 *, settings=None, vehicle=None):
        self.name = name
        self.suspension = suspension
        self.car = car if vehicle is None else vehicle
        self.settings = analysis_settings if settings is None else settings
        self.axle_index = axle_index
        self.direction_sign = direction_sign
        self.color = color

        self.contact_point = np.asarray(self.settings.contact_point_sv, dtype=float).copy()
        self.contact_point += np.array([center_x, 0.0])
        self.wheel_center = np.array(
            [center_x, self.car.load_Radius[axle_index]], dtype=float
        )

        anti_drive_angle = calc_anti_angle(
            suspension.target_anti_dive_sv,
            suspension.front_brake_bias_sv,
            self.car.h_cog,
            self.car.l,
        )
        anti_lift_angle = calc_anti_angle(
            suspension.target_anti_lift_sv,
            1.0 - suspension.front_brake_bias_sv,
            self.car.h_cog,
            self.car.l,
        )

        # Rear suspension is mirrored along X: its IC lies behind the rear tire.
        drive_direction = np.array(
            [direction_sign * np.cos(anti_drive_angle), np.sin(anti_drive_angle)]
        )
        lift_direction = np.array(
            [direction_sign * np.cos(anti_lift_angle), np.sin(anti_lift_angle)]
        )
        self.initial_ic = ga.SusGeometryHelper.line_intersection(
            self.wheel_center,
            self.wheel_center + lift_direction,
            self.contact_point,
            self.contact_point + drive_direction,
        )
        if self.initial_ic is None:
            raise ValueError(f"{name}: anti lines are parallel; no finite IC.")

        self.kingpin_ground = np.array(
            [center_x - direction_sign * suspension.mechanical_trail_sv, 0.0]
        )
        caster = np.deg2rad(suspension.caster_sv_deg)
        self.kingpin_direction = np.array(
            [direction_sign * np.sin(caster), np.cos(caster)]
        )
        self.kingpin_top = (
            self.kingpin_ground
            + self.settings.kingpin_length_sv * self.kingpin_direction
        )

        self.kingpin_wc = self._point_on_kingpin(self.wheel_center[1])
        self.kingpin_Au = self._point_on_kingpin(
            self.wheel_center[1] + suspension.Au
        )
        self.kingpin_Al = self._point_on_kingpin(
            self.wheel_center[1] - suspension.Al
        )

        self.wc_pt = ge.Point(self.wheel_center, f"{name} Wheel Center")
        self.contact_pt = ge.Point(self.contact_point, f"{name} Contact")
        self.au_pt = ge.Point(self.kingpin_Au, f"{name} Au")
        self.al_pt = ge.Point(self.kingpin_Al, f"{name} Al")
        self.ic_pt = ge.Point(self.initial_ic, f"{name} IC")
        self.kg_pt = ge.Point(self.kingpin_ground, f"{name} Kingpin Ground")
        self.kt_pt = ge.Point(self.kingpin_top, f"{name} Kingpin Top")
        self.kwc_pt = ge.Point(self.kingpin_wc, f"{name} Kingpin @ WC")

        wheel_angles = np.linspace(
            0.0,
            2.0 * np.pi,
            self.settings.side_view_wheel_outline_points,
            endpoint=False,
        )
        self.wheel_outline_pts = [
            ge.Point(
                self.wheel_center + self.car.free_radius * np.array([np.cos(a), np.sin(a)]),
                f"{name} Wheel {i}",
            )
            for i, a in enumerate(wheel_angles)
        ]
        self.wheel_polygon = ge.Polygon(self.wheel_outline_pts, name=f"{name} Wheel")
        self.au_tangent = self._make_tangent_reference(self.au_pt, f"{name} Au tangent")
        self.al_tangent = self._make_tangent_reference(self.al_pt, f"{name} Al tangent")

        self.kingpin_link = ge.Link(self.kg_pt, self.kt_pt, f"{name} Kingpin")
        self.svsa_link = ge.Link(self.contact_pt, self.ic_pt, f"{name} SVSA")
        self.au_ic_link = ge.Link(self.au_pt, self.ic_pt, f"{name} Au to IC")
        self.al_ic_link = ge.Link(self.al_pt, self.ic_pt, f"{name} Al to IC")
        self.anti_lift_link = ge.Link(self.wc_pt, self.ic_pt, f"{name} anti-lift")

    def _point_on_kingpin(self, target_y):
        t = (target_y - self.kingpin_ground[1]) / self.kingpin_direction[1]
        return self.kingpin_ground + t * self.kingpin_direction

    def _make_tangent_reference(self, point, label):
        """Draw a fixed tangent perpendicular to the point-to-IC reference."""
        radial = self.initial_ic - point.pos
        radial /= np.linalg.norm(radial)
        tangent = np.array([-radial[1], radial[0]])
        half_length = self.settings.side_view_tangent_half_length
        start = ge.Point(point.pos - half_length * tangent, f"{label} start")
        end = ge.Point(point.pos + half_length * tangent, f"{label} end")
        return ge.Link(start, end, label)

    def add_to_scene(self, scene):
        scene.add(
            self.wheel_polygon,
            facecolor="none",
            edgecolor=STYLE["wheel"],
            linewidth=2,
        )
        for link, style in (
            (self.kingpin_link, {"color": STYLE["kingpin"], "linewidth": 2.2}),
            (self.svsa_link, {"color": STYLE["svsa"], "linewidth": 2.6}),
            (self.au_ic_link, {"color": STYLE["ic_links"], "linestyle": "--"}),
            (self.al_ic_link, {"color": STYLE["ic_links"], "linestyle": "--"}),
            (self.au_tangent, {"color": STYLE["tangent_lines"], "linestyle": "-."}),
            (self.al_tangent, {"color": STYLE["tangent_lines"], "linestyle": "-."}),
            (self.anti_lift_link, {"color": STYLE["anti_lift"], "linestyle": ":"}),
        ):
            scene.add(link, **style)

        for point, style in (
            (self.wc_pt, {"color": STYLE["hard_points"]}),
            (self.contact_pt, {"color": STYLE["contact"]}),
            (self.au_pt, {"color": STYLE["hard_points"]}),
            (self.al_pt, {"color": STYLE["hard_points"]}),
            (self.ic_pt, {"color": STYLE["ic"], "markersize": 10}),
            (self.kg_pt, {"color": STYLE["hard_points"]}),
            (self.kt_pt, {"color": STYLE["hard_points"]}),
        ):
            # PointPlot uses circular markers for geometric points.
            style.setdefault("markersize", 6)
            scene.add(point, **style)


def build_side_view_geometry(vehicle=None, suspension_f=None, suspension_r=None,
                             settings=None):
    """Build the original static SV geometry without creating a plot.

    Local SV X increases from the front axle toward the rear axle. The front
    axle is one wheelbase left of the rear axle, retaining the original signs.
    """
    vehicle = car if vehicle is None else vehicle
    suspension_f = SuspensionF() if suspension_f is None else suspension_f
    suspension_r = SuspensionR() if suspension_r is None else suspension_r
    settings = analysis_settings if settings is None else settings
    front_center_x = settings.wheel_center_x_sv - vehicle.l
    rear_center_x = settings.wheel_center_x_sv
    front = SideSuspension(
        "Front", suspension_f, axle_index=0, direction_sign=1.0,
        center_x=front_center_x, color=STYLE["wheel"], settings=settings, vehicle=vehicle,
    )
    rear = SideSuspension(
        "Rear", suspension_r, axle_index=1, direction_sign=-1.0,
        center_x=rear_center_x, color=STYLE["wheel"], settings=settings, vehicle=vehicle,
    )
    pitch_center = ga.SusGeometryHelper.line_intersection(
        front.contact_point,
        front.initial_ic,
        rear.contact_point,
        rear.initial_ic,
    )
    if pitch_center is None:
        raise ValueError("Front and rear contact-to-IC lines do not define an initial PC.")
    return front, rear, pitch_center


if __name__ == '__main__':
    fig, ax = plt.subplots(figsize=analysis_settings.side_view_figure_size)
    scene = Scene(ax)
    front, rear, pitch_center = build_side_view_geometry(
        vehicle=car, suspension_f=suspension_f, suspension_r=suspension_r,
        settings=analysis_settings,
    )
    front_center_x = front.wheel_center[0]
    rear_center_x = rear.wheel_center[0]
    front.add_to_scene(scene)
    rear.add_to_scene(scene)

    # Connect front/rear tire contact points and extend the same line at both ends.
    contact_span = rear.contact_pt.pos - front.contact_pt.pos
    contact_axis_start = ge.Point(
        front.contact_pt.pos - contact_span, "Contact axis start"
    )
    contact_axis_end = ge.Point(
        rear.contact_pt.pos + contact_span, "Contact axis end"
    )
    contact_axis = ge.Link(
        contact_axis_start, contact_axis_end, "Extended tire-contact axis"
    )
    scene.add(contact_axis, color="0.25", linestyle="--", linewidth=1.5)

    pc_pt = ge.Point(pitch_center, "PC")
    scene.add(pc_pt, color=STYLE["pitch_center"], markersize=10)

    ax.axhline(0.0, color="black", linewidth=1)
    ax.axvline(front_center_x, color="0.5", linestyle=":", linewidth=1)
    ax.axvline(rear_center_x, color="0.5", linestyle=":", linewidth=1)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)
    ax.set_xlabel("Longitudinal X [m]")
    ax.set_ylabel("Vertical Y [m]")
    front_svsa = front.ic_pt.x - front.contact_pt.x
    rear_svsa = rear.contact_pt.x - rear.ic_pt.x
    ax.set_title(
        "Static Front and Rear Side-View Suspension Geometry\n"
        f"Front SVSA={front_svsa:.3f} m, Rear SVSA={rear_svsa:.3f} m | "
        f"PC=({pc_pt.x:.3f}, {pc_pt.y:.3f}) m"
    )

    legend_handles = [
        Line2D([0], [0], color=STYLE["wheel"], lw=2, label="Wheel outline"),
        Line2D([0], [0], color=STYLE["kingpin"], lw=2, label="Kingpin"),
        Line2D([0], [0], color=STYLE["svsa"], lw=2, label="Contact point to IC"),
        Line2D([0], [0], color=STYLE["ic_links"], linestyle="--", label="Au/Al to IC"),
        Line2D([0], [0], color=STYLE["tangent_lines"], linestyle="-.", label="Au/Al tangent references"),
        Line2D([0], [0], color=STYLE["anti_lift"], linestyle=":", label="Wheel center to IC"),
        Line2D([0], [0], color=STYLE["ic"], marker="o", linestyle="none", label="IC"),
        Line2D([0], [0], color=STYLE["contact"], marker="o", linestyle="none", label="Contact point"),
        Line2D([0], [0], color=STYLE["pitch_center"], marker="o", linestyle="none", label="PC"),
    ]
    ax.legend(handles=legend_handles, loc="best", fontsize=8)
    ax.annotate(
        "PC", xy=pc_pt.pos, xytext=(6, -10), textcoords="offset points",
        color=STYLE["pitch_center"], fontsize=9,
    )

    plt.show()
