import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.widgets import Slider

import geometry as ge
import Geometric_Analysis as ga
from car import Car, SuspensionF, SuspensionR
from visualization import LinkPlot, PointPlot, PolygonPlot


# Colors identify geometric feature types, not front/rear axles.
STYLE = {
    "wheel": "0.35",
    "kingpin": "tab:blue",
    "svsa": "black",
    "ic_links": "tab:orange",
    "travel_paths": "tab:purple",
    "anti_lift": "tab:green",
    "hard_points": "tab:blue",
    "contact": "tab:red",
    "initial_ic": "0.5",
    "current_ic": "tab:green",
    "pitch_center_initial": "tab:brown",
    "pitch_center": "tab:pink",
}


car = Car()
suspension_f = SuspensionF()
suspension_r = SuspensionR()


def calc_anti_angle(anti_percent, axle_brake_bias, cg_height, wheelbase):
    """Convert an anti percentage and axle brake share to a side-view angle."""
    tangent = anti_percent * cg_height / (axle_brake_bias * wheelbase)
    return np.arctan(tangent)


class Scene:
    """Updateable Matplotlib scene using the shared front-view plot helpers."""

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

    def update(self):
        for obj in self.objects:
            obj.update()


class SideSuspension:
    """One rigid wheel/upright assembly moving on fixed Au/Al travel paths."""

    def __init__(self, name, suspension, axle_index, direction_sign, center_x, color):
        self.name = name
        self.suspension = suspension
        self.axle_index = axle_index
        self.direction_sign = direction_sign
        self.color = color

        self.contact_point = np.asarray(suspension.contact_point_sv, dtype=float).copy()
        self.contact_point += np.array([center_x, 0.0])
        self.wheel_center = np.array(
            [center_x, car.load_Radius[axle_index]], dtype=float
        )

        anti_drive_angle = calc_anti_angle(
            suspension.target_anti_dive_sv,
            suspension.front_brake_bias_sv,
            car.h_cog,
            car.l,
        )
        anti_lift_angle = calc_anti_angle(
            suspension.target_anti_lift_sv,
            1.0 - suspension.front_brake_bias_sv,
            car.h_cog,
            car.l,
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
            + suspension.kingpin_length_sv * self.kingpin_direction
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
        self.ic_initial_pt = ge.Point(self.initial_ic, f"{name} Initial IC")
        self.ic_pt = ge.Point(self.initial_ic, f"{name} Current IC")
        self.kg_pt = ge.Point(self.kingpin_ground, f"{name} Kingpin Ground")
        self.kt_pt = ge.Point(self.kingpin_top, f"{name} Kingpin Top")
        self.kwc_pt = ge.Point(self.kingpin_wc, f"{name} Kingpin @ WC")

        wheel_angles = np.linspace(
            0.0,
            2.0 * np.pi,
            suspension.side_view_wheel_outline_points,
            endpoint=False,
        )
        self.wheel_outline_pts = [
            ge.Point(
                self.wheel_center + car.free_radius * np.array([np.cos(a), np.sin(a)]),
                f"{name} Wheel {i}",
            )
            for i, a in enumerate(wheel_angles)
        ]
        self.wheel_polygon = ge.Polygon(self.wheel_outline_pts, name=f"{name} Wheel")
        self.rigid_body = ge.RigidBody2D(
            [
                self.wc_pt,
                self.au_pt,
                self.al_pt,
                self.contact_pt,
                *self.wheel_outline_pts,
            ],
            reference_pair=(self.al_pt, self.au_pt),
            name=f"{name} upright and tire",
        )

        self.au_path, self.au_path_direction = self._make_path(
            self.au_pt, f"{name} Au path"
        )
        self.al_path, self.al_path_direction = self._make_path(
            self.al_pt, f"{name} Al path"
        )
        if np.isclose(self.al_path_direction[1], 0.0):
            raise ValueError(f"{name}: Al path is horizontal; y cannot control travel.")
        self.au_path_origin = self.au_pt.pos.copy()
        self.al_path_origin = self.al_pt.pos.copy()
        self.upright_length = np.linalg.norm(self.au_pt.pos - self.al_pt.pos)

        self.kingpin_link = ge.Link(self.kg_pt, self.kt_pt, f"{name} Kingpin")
        self.svsa_link = ge.Link(self.contact_pt, self.ic_pt, f"{name} SVSA")
        self.au_ic_link = ge.Link(self.au_pt, self.ic_pt, f"{name} Au to IC")
        self.al_ic_link = ge.Link(self.al_pt, self.ic_pt, f"{name} Al to IC")
        self.anti_lift_link = ge.Link(self.wc_pt, self.ic_pt, f"{name} anti-lift")

    def _point_on_kingpin(self, target_y):
        t = (target_y - self.kingpin_ground[1]) / self.kingpin_direction[1]
        return self.kingpin_ground + t * self.kingpin_direction

    def _make_path(self, point, label):
        radial = self.initial_ic - point.pos
        radial /= np.linalg.norm(radial)
        tangent = np.array([-radial[1], radial[0]])
        half_length = self.suspension.side_view_path_half_length
        start = ge.Point(point.pos - half_length * tangent, f"{label} start")
        end = ge.Point(point.pos + half_length * tangent, f"{label} end")
        return ge.Link(start, end, label), tangent

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
            (self.au_path, {"color": STYLE["travel_paths"], "linestyle": "-."}),
            (self.al_path, {"color": STYLE["travel_paths"], "linestyle": "-."}),
            (self.anti_lift_link, {"color": STYLE["anti_lift"], "linestyle": ":"}),
        ):
            scene.add(link, **style)

        for point, style in (
            (self.wc_pt, {"color": STYLE["hard_points"]}),
            (self.contact_pt, {"color": STYLE["contact"]}),
            (self.au_pt, {"color": STYLE["hard_points"]}),
            (self.al_pt, {"color": STYLE["hard_points"]}),
            (self.ic_initial_pt, {"color": STYLE["initial_ic"]}),
            (self.ic_pt, {"color": STYLE["current_ic"], "markersize": 10}),
            (self.kg_pt, {"color": STYLE["hard_points"]}),
            (self.kt_pt, {"color": STYLE["hard_points"]}),
        ):
            # PointPlot uses circular markers for every updateable point.
            style.setdefault("markersize", 6)
            scene.add(point, **style)

    def update_geometry(self, target_al_y):
        # Put Al on its fixed travel path at the requested y-coordinate.
        al_t = (target_al_y - self.al_path_origin[1]) / self.al_path_direction[1]
        al_new = self.al_path_origin + al_t * self.al_path_direction

        # Preserve the upright length while constraining Au to its own path.
        direction = self.au_path_direction
        delta = self.au_path_origin - al_new
        b = 2.0 * np.dot(direction, delta)
        c = np.dot(delta, delta) - self.upright_length**2
        discriminant = b * b - 4.0 * c
        if discriminant < -1e-10:
            return False
        root = np.sqrt(max(0.0, discriminant))
        au_candidates = [
            self.au_path_origin + ((-b + root) / 2.0) * direction,
            self.au_path_origin + ((-b - root) / 2.0) * direction,
        ]
        au_new = min(
            au_candidates,
            key=lambda candidate: np.linalg.norm(candidate - self.au_pt.pos),
        )

        # Current IC: intersection of normals to the Au/Al travel paths.
        au_normal = np.array([-self.au_path_direction[1], self.au_path_direction[0]])
        al_normal = np.array([-self.al_path_direction[1], self.al_path_direction[0]])
        current_ic = ga.SusGeometryHelper.line_intersection(
            au_new,
            au_new + au_normal,
            al_new,
            al_new + al_normal,
        )
        if current_ic is None:
            return False

        self.rigid_body.update_from_two_points(al_new, au_new)
        self.ic_pt.move(current_ic)
        return True


fig, ax = plt.subplots(figsize=suspension_f.side_view_figure_size)
fig.subplots_adjust(bottom=suspension_f.side_view_plot_bottom)
scene = Scene(ax)

# Swap the axle positions while retaining their direction signs: the front axle
# is one wheelbase left of the rear axle in this plot.
front_center_x = suspension_f.wheel_center_x_sv - car.l
rear_center_x = suspension_f.wheel_center_x_sv
front = SideSuspension(
    "Front", suspension_f, axle_index=0, direction_sign=1.0,
    center_x=front_center_x, color=STYLE["wheel"],
)
rear = SideSuspension(
    "Rear", suspension_r, axle_index=1, direction_sign=-1.0,
    center_x=rear_center_x, color=STYLE["wheel"],
)
front.add_to_scene(scene)
rear.add_to_scene(scene)

# Connect front/rear tire contact points and extend the same line at both ends.
contact_axis_start = ge.Point(front.contact_pt.pos, "Contact axis start")
contact_axis_end = ge.Point(rear.contact_pt.pos, "Contact axis end")
contact_axis = ge.Link(
    contact_axis_start, contact_axis_end, "Extended tire-contact axis"
)
scene.add(contact_axis, color="0.25", linestyle="--", linewidth=1.5)


def update_contact_axis():
    front_contact = front.contact_pt.pos
    rear_contact = rear.contact_pt.pos
    span = rear_contact - front_contact
    contact_axis_start.move(front_contact - span)
    contact_axis_end.move(rear_contact + span)


update_contact_axis()

# The pitch center is the intersection of the front and rear contact-to-IC
# lines. Keep the initial location as a fixed reference and update PC live.
pitch_center_initial = ga.SusGeometryHelper.line_intersection(
    front.contact_point,
    front.initial_ic,
    rear.contact_point,
    rear.initial_ic,
)
if pitch_center_initial is None:
    raise ValueError("Front and rear contact-to-IC lines do not define an initial PC.")
pc_initial_pt = ge.Point(pitch_center_initial, "Initial PC")
pc_pt = ge.Point(pitch_center_initial, "PC")
scene.add(pc_initial_pt, color=STYLE["pitch_center_initial"], markersize=8)
scene.add(pc_pt, color=STYLE["pitch_center"], markersize=10)

ax.axhline(0.0, color="black", linewidth=1)
ax.axvline(front_center_x, color="0.5", linestyle=":", linewidth=1)
ax.axvline(rear_center_x, color="0.5", linestyle=":", linewidth=1)
ax.set_aspect("equal", adjustable="datalim")
ax.grid(True)
ax.set_xlabel("Longitudinal X [m]")
ax.set_ylabel("Vertical Y [m]")
ax.set_title("Front and Rear Side-View Suspension Geometry")

legend_handles = [
    Line2D([0], [0], color=STYLE["wheel"], lw=2, label="Wheel outline"),
    Line2D([0], [0], color=STYLE["kingpin"], lw=2, label="Kingpin"),
    Line2D([0], [0], color=STYLE["svsa"], lw=2, label="Contact point to IC"),
    Line2D([0], [0], color=STYLE["ic_links"], linestyle="--", label="Au/Al to IC"),
    Line2D([0], [0], color=STYLE["travel_paths"], linestyle="-.", label="Au/Al travel paths"),
    Line2D([0], [0], color=STYLE["anti_lift"], linestyle=":", label="Wheel center to IC"),
    Line2D([0], [0], color=STYLE["initial_ic"], marker="o", linestyle="none", label="Initial IC"),
    Line2D([0], [0], color=STYLE["current_ic"], marker="o", linestyle="none", label="Current IC"),
    Line2D([0], [0], color=STYLE["contact"], marker="o", linestyle="none", label="Contact point"),
    Line2D([0], [0], color=STYLE["pitch_center_initial"], marker="o", linestyle="none", label="Initial PC"),
    Line2D([0], [0], color=STYLE["pitch_center"], marker="o", linestyle="none", label="Current PC"),
]
ax.legend(handles=legend_handles, loc="best", fontsize=8)
pc_initial_label = ax.annotate(
    "PC₀", xy=pc_initial_pt.pos, xytext=(6, 6), textcoords="offset points",
    color=STYLE["pitch_center_initial"], fontsize=9,
)
pc_label = ax.annotate(
    "PC", xy=pc_pt.pos, xytext=(6, -10), textcoords="offset points",
    color=STYLE["pitch_center"], fontsize=9,
)

front_slider_ax = fig.add_axes(suspension_f.side_view_slider_axes[0])
rear_slider_ax = fig.add_axes(suspension_f.side_view_slider_axes[1])
front_slider = Slider(
    front_slider_ax,
    "Front Al height [m]",
    front.al_pt.y - suspension_f.side_view_travel_limit,
    front.al_pt.y + suspension_f.side_view_travel_limit,
    valinit=front.al_pt.y,
)
rear_slider = Slider(
    rear_slider_ax,
    "Rear Al height [m]",
    rear.al_pt.y - suspension_r.side_view_travel_limit,
    rear.al_pt.y + suspension_r.side_view_travel_limit,
    valinit=rear.al_pt.y,
)


def update(_value):
    front_ok = front.update_geometry(front_slider.val)
    rear_ok = rear.update_geometry(rear_slider.val)
    pc_position = None
    if front_ok and rear_ok:
        pc_position = ga.SusGeometryHelper.line_intersection(
            front.contact_pt,
            front.ic_pt,
            rear.contact_pt,
            rear.ic_pt,
        )
    if pc_position is not None:
        pc_pt.move(pc_position)
        pc_label.xy = pc_pt.pos
    update_contact_axis()
    scene.update()

    if front_ok and rear_ok and pc_position is not None:
        front_svsa = front.ic_pt.x - front.contact_pt.x
        rear_svsa = rear.contact_pt.x - rear.ic_pt.x
        ax.set_title(
            "Front and Rear Side-View Suspension Geometry | "
            f"Front SVSA={front_svsa:.3f} m, Rear SVSA={rear_svsa:.3f} m | "
            f"PC=({pc_pt.x:.3f}, {pc_pt.y:.3f}) m"
        )
    else:
        ax.set_title("Requested position has no valid rigid pose, IC, or PC")
    fig.canvas.draw_idle()


front_slider.on_changed(update)
rear_slider.on_changed(update)
plt.show()
