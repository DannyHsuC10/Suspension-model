"""Static front/rear FV planes and an orthogonal SV plane in vehicle space."""

import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from static import Geometric_Analysis as ga
from static import geometry as ge
from static.analysis_settings import AnalysisSettings
from static.car import Car, SuspensionF, SuspensionR
from static.fv.chassis_geometry import build_chassis_geometry
from static.fv.kingpin_geometry import build_kingpin_geometry
from static.sv.sus_sv_v4 import STYLE as SV_STYLE, build_side_view_geometry


KINGPIN_COLOR = "#d81b60"
UBJ_COLOR = "#00b8d9"
LBJ_COLOR = "#f6bd16"
CHASSIS_COLOR = "#f59e0b"


def project_to_3d(points, longitudinal_x):
    """Map FV (right, up) to vehicle (forward, left, up), in metres."""
    points = np.asarray(points, dtype=float)
    return np.stack(
        (np.full(points.shape[:-1], longitudinal_x), -points[..., 0], points[..., 1]),
        axis=-1,
    )


def draw_tire_cylinder(ax, center, radius, width):
    """Draw a closed static tire cylinder with its axle along vehicle Y."""
    center = np.asarray(center, dtype=float)
    angles = np.linspace(0.0, 2.0 * np.pi, 65)
    theta, lateral_offset = np.meshgrid(angles, [-width / 2.0, width / 2.0])
    x = center[0] + radius * np.sin(theta)
    y = center[1] + lateral_offset
    z = center[2] + radius * np.cos(theta)
    tread = ax.plot_surface(
        x, y, z, color="0.25", alpha=0.5, linewidth=0,
        antialiased=True, rcount=2, ccount=len(angles),
    )

    caps = []
    for offset in (-width / 2.0, width / 2.0):
        circle = np.column_stack((
            center[0] + radius * np.sin(angles),
            np.full(angles.shape, center[1] + offset),
            center[2] + radius * np.cos(angles),
        ))
        caps.append(circle)
        ax.plot(*circle.T, color="0.15", linewidth=1.0, alpha=0.8)
    sidewalls = Poly3DCollection(
        caps, facecolor="0.4", edgecolor="none", alpha=0.35,
    )
    ax.add_collection3d(sidewalls)
    return tread, sidewalls


def draw_kingpins(ax, kingpins):
    """Overlay physical 3D steering axes and the upper/lower outer ball joints."""
    artists = {}
    for corner, points in kingpins.items():
        axis, = ax.plot(
            *np.vstack((points["ground"], points["axis_top"])).T,
            color=KINGPIN_COLOR, linestyle="--", linewidth=1.8, alpha=0.9,
        )
        upright, = ax.plot(
            *np.vstack((points["LBJ"], points["UBJ"])).T,
            color=KINGPIN_COLOR, linewidth=3.4,
        )
        corner_artists = {"axis": axis, "upright": upright}
        outward = 1.0 if points["side"] == "Left" else -1.0
        for joint, color, marker, height_offset in (
            ("UBJ", UBJ_COLOR, "D", 0.06),
            ("LBJ", LBJ_COLOR, "s", -0.035),
        ):
            position = points[joint]
            corner_artists[joint] = ax.scatter(
                *position, color=color, marker=marker, s=52,
                edgecolor="black", linewidth=0.7, depthshade=False,
            )
            label_position = position + [0.02, outward * 0.13, height_offset]
            ax.plot(*np.vstack((position, label_position)).T,
                    color="0.35", linestyle=":", linewidth=0.7)
            ax.text(
                *label_position, f"{corner} {joint}", fontsize=7.5,
                ha="left" if outward > 0 else "right", color="0.1",
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=0.3),
            )
        artists[corner] = corner_artists
    return artists


def draw_chassis(ax, geometry, alpha):
    """Fill the six faces between the eight equivalent chassis pickup points."""
    # Each side can be slightly twisted; triangulate the fill but draw only
    # the twelve outer edges, preserving the exact eight pickup locations.
    triangles = [triangle for face in geometry["faces"].values()
                 for triangle in (face[[0, 1, 2]], face[[0, 2, 3]])]
    surface = Poly3DCollection(
        triangles, facecolor=CHASSIS_COLOR, edgecolor="none", alpha=alpha,
    )
    ax.add_collection3d(surface)
    edges = []
    for first, last in geometry["edges"]:
        line, = ax.plot(
            *np.vstack((geometry["vertices"][first], geometry["vertices"][last])).T,
            color="#b86b09", linewidth=1.0, alpha=0.8,
        )
        edges.append(line)
    return {"surface": surface, "edges": edges}


def clip_segment_to_box(points, bounds):
    """Clip a display segment to the view without changing its geometry points."""
    start, end = np.asarray(points, dtype=float)
    direction = end - start
    lower_t, upper_t = 0.0, 1.0
    for index, (lower, upper) in enumerate(bounds):
        if abs(direction[index]) < 1e-12:
            if start[index] < lower or start[index] > upper:
                return None
            continue
        first, last = sorted(((lower - start[index]) / direction[index],
                              (upper - start[index]) / direction[index]))
        lower_t = max(lower_t, first)
        upper_t = min(upper_t, last)
        if lower_t > upper_t:
            return None
    return np.array([start + lower_t * direction, start + upper_t * direction])


class Scene:
    """Draw 2D geometry objects on a fixed FV or SV plane in 3D."""

    def __init__(self, ax, longitudinal_x=0.0, *, projector=None, bounds=None):
        self.ax = ax
        self.longitudinal_x = longitudinal_x
        self.project = projector if projector is not None else (
            lambda points: project_to_3d(points, longitudinal_x)
        )
        self.bounds = None if bounds is None else np.asarray(bounds, dtype=float)
        self.objects = []

    def add(self, obj, **kwargs):
        if isinstance(obj, ge.Point):
            xyz = self.project(obj.pos)
            if self.bounds is not None and (
                np.any(xyz < self.bounds[:, 0]) or np.any(xyz > self.bounds[:, 1])
            ):
                return
            kwargs.setdefault("markersize", 3.5)
            artist, = self.ax.plot(
                *xyz.reshape(1, 3).T, marker="o", linestyle="none", **kwargs
            )
        elif isinstance(obj, ge.Link):
            xyz = self.project([obj.start.pos, obj.end.pos])
            if self.bounds is not None:
                xyz = clip_segment_to_box(xyz, self.bounds)
                if xyz is None:
                    return
            kwargs.setdefault("linewidth", 1.3)
            artist, = self.ax.plot(*xyz.T, **kwargs)
        elif isinstance(obj, ge.Polygon):
            xyz = self.project([point.pos for point in obj.points])
            kwargs.setdefault("alpha", 0.45)
            if kwargs.get("facecolor") == "none":
                kwargs.pop("facecolor")
                kwargs["color"] = kwargs.pop("edgecolor", "0.4")
                outline = np.vstack((xyz, xyz[0]))
                artist, = self.ax.plot(*outline.T, **kwargs)
            else:
                artist = Poly3DCollection([xyz], **kwargs)
                self.ax.add_collection3d(artist)
        else:
            raise TypeError(f"Unsupported geometry: {type(obj).__name__}")
        self.objects.append(artist)


class FrontViewPlane:
    """One axle's independent static FV geometry, including both wheel sides."""

    def __init__(self, car, suspension, name, longitudinal_x, color):
        self.name = name
        self.longitudinal_x = longitudinal_x
        self.color = color
        self.helper = ga.SusGeometryHelper(car, suspension)
        self.static_pts = self.helper.calc_static_points()
        self.right_sus = ga.SuspensionSide(f"{name} Right")
        self.right_sus.setup_right_side(self.static_pts)
        self.left_sus = ga.SuspensionSide(f"{name} Left", is_left=True)
        self.left_sus.setup_from_right_template(self.right_sus)

    def draw(self, ax, car, settings):
        scene = Scene(ax, self.longitudinal_x)
        self.right_sus.add_to_scene(scene)
        self.left_sus.add_to_scene(scene)
        self.tire_artists = [
            draw_tire_cylinder(
                ax, project_to_3d(side.wheel_center.pos, self.longitudinal_x),
                side.tire_radius, side.tire_width,
            )
            for side in (self.right_sus, self.left_sus)
        ]

        transverse_min, transverse_max = settings.front_view_x_limits
        height_min, height_max = settings.front_view_y_limits
        plane_corners = np.array([
            [transverse_min, height_min], [transverse_max, height_min],
            [transverse_max, height_max], [transverse_min, height_max],
        ])
        xyz = project_to_3d(plane_corners, self.longitudinal_x)
        ax.add_collection3d(Poly3DCollection(
            [xyz], facecolor=self.color, edgecolor="none", alpha=0.035,
        ))
        border = np.vstack((xyz, xyz[0]))
        ax.plot(*border.T, color=self.color, linestyle="--", linewidth=1.0, alpha=0.6)

        # The transverse ground, loaded-radius and centerline references are fixed.
        for height, alpha in ((0.0, 0.6), (self.helper.load_Radius, 0.25)):
            xyz = project_to_3d(
                [[transverse_min, height], [transverse_max, height]], self.longitudinal_x
            )
            ax.plot(*xyz.T, color="0.3", linestyle="--", linewidth=0.9, alpha=alpha)
        xyz = project_to_3d([[0.0, height_min], [0.0, height_max]], self.longitudinal_x)
        ax.plot(*xyz.T, color="0.5", linestyle=":", linewidth=0.8)

        self.RC_position = project_to_3d(self.static_pts["RC"], self.longitudinal_x)
        ax.scatter(*self.RC_position, color="purple", s=36, depthshade=False)
        ax.text(
            *project_to_3d([0.06, self.static_pts["RC"][1] + 0.035], self.longitudinal_x),
            f"RC {self.name}", color="purple", fontsize=8,
        )
        ax.text(
            *project_to_3d([transverse_max - 0.05, height_max + 0.04], self.longitudinal_x),
            f"{self.name} FV (X = {self.longitudinal_x:.2f} m)",
            color=self.color, fontsize=10, fontweight="bold",
        )
        self.scene = scene


class SideViewPlane:
    """The existing SV geometry mirrored into the vehicle's center plane Y=0."""

    def __init__(self, car, suspension_f, suspension_r, settings):
        self.longitudinal_offset = settings.wheel_center_x_sv - car.l
        self.front, self.rear, self.local_pitch_center = build_side_view_geometry(
            vehicle=car, suspension_f=suspension_f, suspension_r=suspension_r,
            settings=settings,
        )
        self.pitch_center = self.project(self.local_pitch_center)
        self.ic_positions = {
            side.name: self.project(side.initial_ic) for side in (self.front, self.rear)
        }

    def project(self, points):
        """Align SV's original front-at-left convention with vehicle +X forward."""
        points = np.asarray(points, dtype=float)
        return np.stack((self.longitudinal_offset - points[..., 0],
                         np.zeros(points.shape[:-1]), points[..., 1]), axis=-1)

    def draw(self, ax, longitudinal_limits, height_limits):
        height_min, height_max = height_limits
        bounds = [longitudinal_limits, (-0.01, 0.01), height_limits]
        self.scene = Scene(ax, projector=self.project, bounds=bounds)
        self.front.add_to_scene(self.scene)
        self.rear.add_to_scene(self.scene)
        self.scene.add(ge.Point(self.local_pitch_center, "PC"),
                       color=SV_STYLE["pitch_center"], markersize=7)
        ax.text(*(self.pitch_center + [0.03, 0.0, 0.035]), "PC",
                color=SV_STYLE["pitch_center"], fontsize=9)

        longitudinal_min, longitudinal_max = longitudinal_limits
        corners = np.array([
            [longitudinal_min, 0.0, height_min],
            [longitudinal_max, 0.0, height_min],
            [longitudinal_max, 0.0, height_max],
            [longitudinal_min, 0.0, height_max],
        ])
        ax.add_collection3d(Poly3DCollection(
            [corners], facecolor="tab:green", edgecolor="none", alpha=0.035,
        ))
        border = np.vstack((corners, corners[0]))
        ax.plot(*border.T, color="tab:green", linestyle="--", linewidth=1, alpha=0.65)
        ax.plot([longitudinal_min, longitudinal_max], [0.0, 0.0], [0.0, 0.0],
                color="0.3", linestyle="--", linewidth=0.8, alpha=0.5)
        ax.text(
            (self.project(self.front.wheel_center)[0] + self.project(self.rear.wheel_center)[0]) / 2,
            0.0, height_max + 0.085, "SV center plane (Y = 0)",
            color="tab:green", fontsize=10, fontweight="bold",
        )


def create_figure(car=None, suspension_f=None, suspension_r=None, settings=None):
    """Build the two FV planes and the perpendicular central SV plane."""
    car = Car() if car is None else car
    suspension_f = SuspensionF() if suspension_f is None else suspension_f
    suspension_r = SuspensionR() if suspension_r is None else suspension_r
    settings = AnalysisSettings() if settings is None else settings

    front = FrontViewPlane(car, suspension_f, "Front", 0.0, "tab:blue")
    rear = FrontViewPlane(car, suspension_r, "Rear", -car.l, "tab:orange")
    fig = plt.figure(figsize=settings.front_view_figure_size)
    ax = fig.add_subplot(111, projection="3d")
    front.draw(ax, car, settings)
    rear.draw(ax, car, settings)
    longitudinal_margin = car.free_radius + 0.05
    longitudinal_limits = (-car.l - longitudinal_margin, longitudinal_margin)
    side_view_plane = SideViewPlane(car, suspension_f, suspension_r, settings)
    side_view_plane.draw(ax, longitudinal_limits, settings.front_view_y_limits)
    fig.side_view_plane = side_view_plane

    # A single physical CG lies between the axle planes.
    cg_position = np.array([-car.lf, 0.0, car.h_cog])
    ax.scatter(*cg_position, marker="*", color="crimson", s=130, depthshade=False)
    ax.text(*(cg_position + [0.0, 0.04, 0.04]), "CG", color="crimson", fontsize=9)
    ax.plot(
        *np.vstack((front.RC_position, rear.RC_position)).T,
        color="purple", linestyle="--", linewidth=1.2,
    )
    fig.kingpins = build_kingpin_geometry(front, rear, side_view_plane)
    fig.kingpin_artists = draw_kingpins(ax, fig.kingpins)
    fig.chassis_geometry = build_chassis_geometry(front, rear)
    fig.chassis_artists = draw_chassis(ax, fig.chassis_geometry, settings.chassis_face_alpha)

    transverse_min, transverse_max = settings.front_view_x_limits
    height_min, height_max = settings.front_view_y_limits
    lateral_min, lateral_max = -transverse_max, -transverse_min
    ax.set_xlim(*longitudinal_limits)
    ax.set_ylim(lateral_min - 0.08, lateral_max + 0.08)
    ax.set_zlim(height_min, height_max + 0.12)
    ax.set_box_aspect((car.l + 2.0 * longitudinal_margin, lateral_max - lateral_min + 0.16,
                       height_max - height_min + 0.12))
    ax.view_init(elev=settings.front_view_elevation_deg,
                 azim=settings.front_view_azimuth_deg)
    ax.set_xlabel("Longitudinal X [m] (+front)", labelpad=12)
    ax.set_ylabel("Lateral Y [m] (+left)", labelpad=12)
    ax.set_zlabel("Height Z [m]", labelpad=8)
    ax.set_title(
        f"Front / Rear FV and Center SV | Wheelbase = {car.l:.3f} m", pad=22
    )
    ax.grid(True, alpha=0.25)
    fig.legend(handles=[
        Line2D([], [], color=front.color, linestyle="--", label="Front FV plane"),
        Line2D([], [], color=rear.color, linestyle="--", label="Rear FV plane"),
        Line2D([], [], color="tab:green", linestyle="--", label="Center SV plane"),
        Line2D([], [], color="green", label="Suspension links"),
        Line2D([], [], color="blue", label="IC construction lines"),
        Patch(facecolor="0.25", alpha=0.5, label="Cylindrical tires"),
        Patch(facecolor=CHASSIS_COLOR, edgecolor="#b86b09",
              alpha=settings.chassis_face_alpha, label="Chassis (8 pickups)"),
        Line2D([], [], color="purple", marker="o", linestyle="--", label="Roll centers"),
        Line2D([], [], color="crimson", marker="*", linestyle="none", label="Vehicle CG"),
        Line2D([], [], color=SV_STYLE["pitch_center"], marker="o", linestyle="none", label="Pitch center"),
        Line2D([], [], color=KINGPIN_COLOR, linewidth=2.5, label="3D kingpin"),
        Line2D([], [], color=UBJ_COLOR, marker="D", markeredgecolor="black",
               linestyle="none", label="Upper ball joint (UBJ)"),
        Line2D([], [], color=LBJ_COLOR, marker="s", markeredgecolor="black",
               linestyle="none", label="Lower ball joint (LBJ)"),
    ], loc="lower center", bbox_to_anchor=(0.5, 0.02), ncol=4, fontsize=9)
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.15, top=0.90)
    return fig, ax, front, rear


if __name__ == "__main__":
    fig, ax, front_plane, rear_plane = create_figure()
    side_view_plane = fig.side_view_plane
    plt.show()
