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
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from static import Geometric_Analysis as ga
from static import geometry as ge
from static.analysis_settings import AnalysisSettings
from static.car import Car, SuspensionF, SuspensionR
from static.fv.arm_plane_geometry import build_arm_plane_geometry
from static.fv.arm_plane_plot import draw_arm_planes
from static.fv.chassis_geometry import build_chassis_geometry
from static.fv.circle_plane_intersections import (
    build_circle_plane_intersections, build_front_lower_intersection_candidates,
    build_rear_circle_plane_intersections,
)
from static.fv.front_lower_inspection_plot import create_front_lower_inspection_figure
from static.fv.interference_geometry import build_interference_geometry
from static.fv.kingpin_geometry import build_kingpin_geometry
from static.fv.lower_arm_mount_geometry import build_lower_arm_mount_geometry
from static.fv.rear_arm_mount_geometry import (
    build_rear_lower_arm_mount_geometry, build_rear_upper_arm_mount_geometry,
)
from static.fv.upper_arm_mount_geometry import build_upper_arm_mount_geometry
from static.fv.wheel_projection_geometry import build_axle_wheel_projection_geometry
from static.sv.sus_sv_v4 import build_side_view_geometry


# True：顯示 A 臂、前後 FV、SV 透明面；False：隱藏填面，保留線條與接點。
SHOW_REFERENCE_PLANES = False

# 各投影視圖的顯示開關；False 隱藏該視圖，保留實體 3D 幾何與計算。
SHOW_SV = False
SHOW_FRONT_FV = True
SHOW_REAR_FV = False

# 預設繪製完整 A 臂；True 改為檢視前輪下臂全部原始交點 a/b/c/d。
INSPECT_FRONT_LOWER_INTERSECTIONS = False


VIEW_COLORS = {
    "front": "#0072b2",
    "rear": "#d55e00",
    "sv": "#009e73",
}
CHASSIS_COLOR = "0.65"
REFERENCE_COLOR = "0.4"
INTERFERENCE_LINESTYLES = {
    "neutral": "-",
    "positive": "--",
    "negative": ":",
}
INTERSECTION_MARKERS = {"upper": "^", "lower": "v"}


def corner_color(corner, corner_colors=None):
    """Use the axle's FV color for its physical geometry and annotations."""
    if corner_colors is not None:
        return corner_colors[corner]
    return VIEW_COLORS["front" if corner.startswith("F") else "rear"]


def project_to_3d(points, longitudinal_x):
    """Map FV (right, up) to vehicle (forward, left, up), in metres."""
    points = np.asarray(points, dtype=float)
    return np.stack(
        (np.full(points.shape[:-1], longitudinal_x), -points[..., 0], points[..., 1]),
        axis=-1,
    )


def draw_tire_cylinder(ax, center, radius, width):
    """Draw a translucent closed tire while keeping its FV projection visible."""
    center = np.asarray(center, dtype=float)
    angles = np.linspace(0.0, 2.0 * np.pi, 65)
    theta, lateral_offset = np.meshgrid(angles, [-width / 2.0, width / 2.0])
    tread = ax.plot_surface(
        center[0] + radius * np.sin(theta),
        center[1] + lateral_offset,
        center[2] + radius * np.cos(theta),
        color="0.3", alpha=0.22, linewidth=0, antialiased=True,
        rcount=2, ccount=len(angles),
    )
    caps, outlines = [], []
    for offset in (-width / 2.0, width / 2.0):
        circle = np.column_stack((
            center[0] + radius * np.sin(angles),
            np.full(angles.shape, center[1] + offset),
            center[2] + radius * np.cos(angles),
        ))
        caps.append(circle)
        outline, = ax.plot(*circle.T, color="0.35", linewidth=0.9, alpha=0.65)
        outlines.append(outline)
    sidewalls = Poly3DCollection(
        caps, facecolor="0.4", edgecolor="none", alpha=0.12,
    )
    ax.add_collection3d(sidewalls)
    return {"tread": tread, "sidewalls": sidewalls, "outlines": outlines}


def draw_wheel_projections(ax, geometry, color):
    """Draw tire and rim-inner envelopes as rectangles on the axle's FV plane."""
    artists = {}
    for corner, wheel in geometry.items():
        tire, = ax.plot(
            *wheel["tire"]["outline"].T, color=color, linewidth=1.8,
            linestyle="-",
        )
        rim, = ax.plot(
            *wheel["rim"]["outline"].T, color=color, linewidth=1.5,
            linestyle="--",
        )
        artists[corner] = {"tire": tire, "rim": rim}
    return artists


def draw_kingpins(ax, kingpins, *, corner_colors=None):
    """Overlay physical 3D steering axes and the upper/lower outer ball joints."""
    artists = {}
    for corner, points in kingpins.items():
        color = corner_color(corner, corner_colors)
        axis, = ax.plot(
            *np.vstack((points["ground"], points["axis_top"])).T,
            color=color, linestyle="--", linewidth=1.8, alpha=0.9,
        )
        upright, = ax.plot(
            *np.vstack((points["LBJ"], points["UBJ"])).T,
            color=color, linewidth=3.4,
        )
        corner_artists = {"axis": axis, "upright": upright}
        outward = 1.0 if points["side"] == "Left" else -1.0
        for joint, marker, height_offset in (
            ("UBJ", "D", 0.06),
            ("LBJ", "s", -0.035),
        ):
            position = points[joint]
            corner_artists[joint] = ax.scatter(
                *position, color=color, marker=marker, s=52,
                edgecolor="black", linewidth=0.7, depthshade=False,
            )
            label_position = position + [0.02, outward * 0.13, height_offset]
            ax.plot(*np.vstack((position, label_position)).T,
                    color=color, linestyle=":", linewidth=0.7)
            ax.text(
                *label_position, f"{corner} {joint}", fontsize=7.5,
                ha="left" if outward > 0 else "right", color=color,
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
            color=REFERENCE_COLOR, linewidth=1.0, alpha=0.8,
        )
        edges.append(line)
    return {"surface": surface, "edges": edges}


def draw_interference_circles(ax, geometry, *, corner_colors=None):
    """Overlay neutral inner-rim circles and the two fixed front-steer extremes."""
    artists = {}
    for corner, wheel in geometry.items():
        artists[corner] = {}
        color = corner_color(corner, corner_colors)
        for name, circle in wheel["circles"].items():
            line, = ax.plot(
                *circle["points"].T, color=color, linestyle=INTERFERENCE_LINESTYLES[name],
                linewidth=2.0, alpha=1.0,
            )
            artists[corner][name] = line
    return artists


def draw_selected_intersections(ax, geometry, *, corner_colors=None):
    """Mark the selected front/rear clearance intersections on their arm planes."""
    artists = {}
    for key, arm in geometry.items():
        color = corner_color(arm["corner"], corner_colors)
        marker = INTERSECTION_MARKERS[arm["arm"]]
        prefix = "U" if arm["arm"] == "upper" else "L"
        outward = 1.0 if arm["corner"].endswith("L") else -1.0
        artists[key] = []
        for rank, intersection in enumerate(arm["selected"], start=1):
            point = intersection["point"]
            name = intersection.get("name", f"{prefix}{rank}")
            scatter = ax.scatter(
                *point, color=color, marker=marker, s=82, edgecolor="black",
                linewidth=0.8, depthshade=False,
            )
            # Separate labels for nearby intersections while retaining exact points.
            label_position = point + [0.035 if rank == 1 else -0.035,
                                      outward * 0.09, 0.045 if prefix == "U" else -0.035]
            ax.plot(*np.vstack((point, label_position)).T,
                    color=color, linestyle=":", linewidth=0.8)
            label = ax.text(
                *label_position, f"{arm['corner']} {name}", fontsize=7.5,
                color=color, ha="left" if outward > 0 else "right",
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.9, pad=0.4),
            )
            artists[key].append({"marker": scatter, "label": label})
    return artists


def draw_front_lower_candidates(ax, geometry, *, corner_colors=None):
    """Label every raw front-lower intersection without choosing arm endpoints."""
    artists = {}
    for corner, arm in geometry.items():
        color = corner_color(corner, corner_colors)
        outward = 1.0 if corner.endswith("L") else -1.0
        artists[corner] = {}
        for candidate in arm["points"]:
            point, name = candidate["point"], candidate["name"]
            marker = ax.scatter(
                *point, color=color, marker="o" if candidate["pose"] == "positive" else "s",
                s=95, edgecolor="black", linewidth=0.9, depthshade=False,
            )
            forward = name in ("a", "c")
            label_position = point + [0.07 if forward else -0.07,
                                      outward * 0.11, -0.055 if forward else -0.085]
            leader, = ax.plot(*np.vstack((point, label_position)).T,
                              color=color, linestyle=":", linewidth=0.8)
            label = ax.text(
                *label_position, f"{corner} {name}", color=color, fontsize=10,
                fontweight="bold", ha="left" if outward > 0 else "right",
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.95, pad=0.5),
            )
            artists[corner][name] = {"marker": marker, "label": label, "leader": leader}
    return artists


def draw_arm_mounts(ax, geometry, *, joint, label_height, corner_colors=None,
                    leg_color=None, mount_color=None):
    """Draw ball-joint-to-chassis legs through the selected arm intersections."""
    artists = {}
    for corner, arm in geometry.items():
        artists[corner] = {}
        color = corner_color(corner, corner_colors)
        line_color = color if leg_color is None else leg_color
        point_color = color if mount_color is None else mount_color
        outward = 1.0 if corner.endswith("L") else -1.0
        for leg in arm["legs"]:
            endpoint = leg["endpoint"]
            if endpoint is None:
                continue
            name = leg["name"]
            line, = ax.plot(
                *np.vstack((arm[joint], endpoint)).T, color=line_color,
                linewidth=2.6, linestyle="-" if leg["is_extension"] else "--",
            )
            marker = ax.scatter(
                *endpoint, color=point_color, marker="s", s=66,
                edgecolor="black", linewidth=0.8, depthshade=False,
            )
            label_position = endpoint + [0.04 if name.endswith("FI") else -0.04,
                                         outward * 0.06, label_height]
            ax.plot(*np.vstack((endpoint, label_position)).T,
                    color=point_color, linestyle=":", linewidth=0.8)
            label = ax.text(
                *label_position, f"{corner} {name}", fontsize=8,
                color=point_color, ha="left" if outward > 0 else "right",
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.9, pad=0.4),
            )
            artists[corner][name] = {"leg": line, "marker": marker, "label": label}
    return artists


def draw_upper_arm_mounts(ax, geometry, *, corner_colors=None):
    """Draw UBJ-to-chassis legs through the selected upper intersections."""
    return draw_arm_mounts(
        ax, geometry, joint="UBJ", label_height=0.04, corner_colors=corner_colors,
    )


def draw_lower_arm_mounts(ax, geometry, *, corner_colors=None):
    """Draw LBJ-to-chassis legs through the selected lower intersections."""
    return draw_arm_mounts(
        ax, geometry, joint="LBJ", label_height=-0.04, corner_colors=corner_colors,
    )


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

    def __init__(self, ax, longitudinal_x=0.0, *, projector=None, bounds=None, color=None):
        self.ax = ax
        self.color = color
        self.longitudinal_x = longitudinal_x
        self.project = projector if projector is not None else (
            lambda points: project_to_3d(points, longitudinal_x)
        )
        self.bounds = None if bounds is None else np.asarray(bounds, dtype=float)
        self.objects = []

    def add(self, obj, **kwargs):
        if self.color is not None:
            if isinstance(obj, ge.Polygon):
                kwargs["edgecolor"] = self.color
                if kwargs.get("facecolor") != "none":
                    kwargs["facecolor"] = self.color
            else:
                kwargs["color"] = self.color
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
        self.axle_prefix = "R" if isinstance(suspension, SuspensionR) else "F"
        self.longitudinal_x = longitudinal_x
        self.color = color
        self.helper = ga.SusGeometryHelper(car, suspension)
        self.static_pts = self.helper.calc_static_points()
        self.right_sus = ga.SuspensionSide(f"{name} Right")
        self.right_sus.setup_right_side(self.static_pts)
        self.left_sus = ga.SuspensionSide(f"{name} Left", is_left=True)
        self.left_sus.setup_from_right_template(self.right_sus)

    def draw(self, ax, car, settings, *, visible=True):
        previous_artists = set(ax.get_children())
        scene = Scene(ax, self.longitudinal_x, color=self.color)
        self.right_sus.add_to_scene(scene)
        self.left_sus.add_to_scene(scene)
        self.wheel_projection_geometry = build_axle_wheel_projection_geometry(
            car, self, prefix=self.axle_prefix,
        )
        self.tire_artists = {
            corner: draw_tire_cylinder(
                ax, wheel["center"], wheel["tire"]["height"] / 2.0,
                wheel["tire"]["width"],
            )
            for corner, wheel in self.wheel_projection_geometry.items()
        }
        self.wheel_projection_artists = draw_wheel_projections(
            ax, self.wheel_projection_geometry, self.color,
        )

        transverse_min, transverse_max = settings.front_view_x_limits
        height_min, height_max = settings.front_view_y_limits
        plane_corners = np.array([
            [transverse_min, height_min], [transverse_max, height_min],
            [transverse_max, height_max], [transverse_min, height_max],
        ])
        xyz = project_to_3d(plane_corners, self.longitudinal_x)
        self.plane_surface = None
        if SHOW_REFERENCE_PLANES and visible:
            self.plane_surface = Poly3DCollection(
                [xyz], facecolor=self.color, edgecolor="none", alpha=0.035,
            )
            ax.add_collection3d(self.plane_surface)
        border = np.vstack((xyz, xyz[0]))
        ax.plot(*border.T, color=self.color, linestyle="--", linewidth=1.0, alpha=0.6)

        # The transverse ground, loaded-radius and centerline references are fixed.
        for height, alpha in ((0.0, 0.6), (self.helper.load_Radius, 0.25)):
            xyz = project_to_3d(
                [[transverse_min, height], [transverse_max, height]], self.longitudinal_x
            )
            ax.plot(*xyz.T, color=self.color, linestyle="--", linewidth=0.9, alpha=alpha)
        xyz = project_to_3d([[0.0, height_min], [0.0, height_max]], self.longitudinal_x)
        ax.plot(*xyz.T, color=self.color, linestyle=":", linewidth=0.8)

        self.RC_position = project_to_3d(self.static_pts["RC"], self.longitudinal_x)
        ax.scatter(*self.RC_position, color=self.color, s=36, depthshade=False)
        ax.text(
            *project_to_3d([0.06, self.static_pts["RC"][1] + 0.035], self.longitudinal_x),
            f"RC {self.name}", color=self.color, fontsize=8,
        )
        ax.text(
            *project_to_3d([transverse_max - 0.05, height_max + 0.04], self.longitudinal_x),
            f"{self.name} FV (X = {self.longitudinal_x:.2f} m)",
            color=self.color, fontsize=10, fontweight="bold",
        )
        self.scene = scene
        tire_artists = {
            artist for tire in self.tire_artists.values()
            for artist in (tire["tread"], tire["sidewalls"], *tire["outlines"])
        }
        self.visible = visible
        self.view_artists = [
            artist for artist in ax.get_children()
            if artist not in previous_artists and artist not in tire_artists
        ]
        for artist in self.view_artists:
            artist.set_visible(visible)


class SideViewPlane:
    """The existing SV geometry mirrored into the vehicle's center plane Y=0."""

    def __init__(self, car, suspension_f, suspension_r, settings, *, color=None):
        self.color = VIEW_COLORS["sv"] if color is None else color
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

    def draw(self, ax, longitudinal_limits, height_limits, *, visible=True):
        previous_artists = set(ax.get_children())
        height_min, height_max = height_limits
        bounds = [longitudinal_limits, (-0.01, 0.01), height_limits]
        self.scene = Scene(ax, projector=self.project, bounds=bounds, color=self.color)
        self.front.add_to_scene(self.scene)
        self.rear.add_to_scene(self.scene)
        self.scene.add(ge.Point(self.local_pitch_center, "PC"),
                       color=self.color, markersize=7)
        ax.text(*(self.pitch_center + [0.03, 0.0, 0.035]), "PC",
                color=self.color, fontsize=9)

        longitudinal_min, longitudinal_max = longitudinal_limits
        corners = np.array([
            [longitudinal_min, 0.0, height_min],
            [longitudinal_max, 0.0, height_min],
            [longitudinal_max, 0.0, height_max],
            [longitudinal_min, 0.0, height_max],
        ])
        self.plane_surface = None
        if SHOW_REFERENCE_PLANES and visible:
            self.plane_surface = Poly3DCollection(
                [corners], facecolor=self.color, edgecolor="none", alpha=0.035,
            )
            ax.add_collection3d(self.plane_surface)
        border = np.vstack((corners, corners[0]))
        ax.plot(*border.T, color=self.color, linestyle="--", linewidth=1, alpha=0.65)
        ax.plot([longitudinal_min, longitudinal_max], [0.0, 0.0], [0.0, 0.0],
                color=self.color, linestyle="--", linewidth=0.8, alpha=0.5)
        ax.text(
            (self.project(self.front.wheel_center)[0] + self.project(self.rear.wheel_center)[0]) / 2,
            0.0, height_max + 0.085, "SV center plane (Y = 0)",
            color=self.color, fontsize=10, fontweight="bold",
        )
        self.visible = visible
        self.view_artists = [
            artist for artist in ax.get_children() if artist not in previous_artists
        ]
        for artist in self.view_artists:
            artist.set_visible(visible)


def create_figure(car=None, suspension_f=None, suspension_r=None, settings=None,
                  fit_arm_planes=True):
    """Build the static FV/SV geometry and all eight IC-to-ball-joint planes.

    ``fit_arm_planes=False`` keeps the vehicle close-up and clips only the
    displayed arm faces; the stored triangle vertices always remain exact.
    """
    car = Car() if car is None else car
    suspension_f = SuspensionF() if suspension_f is None else suspension_f
    suspension_r = SuspensionR() if suspension_r is None else suspension_r
    settings = AnalysisSettings() if settings is None else settings

    front = FrontViewPlane(car, suspension_f, "Front", 0.0, VIEW_COLORS["front"])
    rear = FrontViewPlane(car, suspension_r, "Rear", -car.l, VIEW_COLORS["rear"])
    fig = plt.figure(figsize=settings.front_view_figure_size)
    ax = fig.add_subplot(111, projection="3d")
    front.draw(ax, car, settings, visible=SHOW_FRONT_FV)
    rear.draw(ax, car, settings, visible=SHOW_REAR_FV)
    fig.tire_artists = {**front.tire_artists, **rear.tire_artists}
    fig.wheel_projection_geometry = {
        **front.wheel_projection_geometry, **rear.wheel_projection_geometry,
    }
    fig.wheel_projection_artists = {
        **front.wheel_projection_artists, **rear.wheel_projection_artists,
    }
    longitudinal_margin = car.free_radius + 0.05
    longitudinal_limits = (-car.l - longitudinal_margin, longitudinal_margin)
    side_view_plane = SideViewPlane(car, suspension_f, suspension_r, settings)
    side_view_plane.draw(
        ax, longitudinal_limits, settings.front_view_y_limits, visible=SHOW_SV,
    )
    fig.side_view_plane = side_view_plane
    fig.show_reference_planes = SHOW_REFERENCE_PLANES
    fig.view_visibility = {"front": SHOW_FRONT_FV, "rear": SHOW_REAR_FV, "sv": SHOW_SV}
    fig.view_colors = {"front": front.color, "rear": rear.color, "sv": side_view_plane.color}
    corner_colors = {"FL": front.color, "FR": front.color, "RL": rear.color, "RR": rear.color}

    # A single physical CG lies between the axle planes.
    cg_position = np.array([-car.lf, 0.0, car.h_cog])
    ax.scatter(*cg_position, marker="*", color=REFERENCE_COLOR, s=130, depthshade=False)
    ax.text(*(cg_position + [0.0, 0.04, 0.04]), "CG", color=REFERENCE_COLOR, fontsize=9)
    ax.plot(
        *np.vstack((front.RC_position, rear.RC_position)).T,
        color=REFERENCE_COLOR, linestyle="--", linewidth=1.2,
    )
    fig.kingpins = build_kingpin_geometry(front, rear, side_view_plane)
    fig.kingpin_artists = draw_kingpins(ax, fig.kingpins, corner_colors=corner_colors)
    fig.chassis_geometry = build_chassis_geometry(front, rear)
    fig.chassis_artists = draw_chassis(ax, fig.chassis_geometry, settings.chassis_face_alpha)
    fig.arm_planes = build_arm_plane_geometry(front, rear, side_view_plane, fig.kingpins)
    fig.interference_geometry = build_interference_geometry(car, front, rear, fig.kingpins)
    fig.circle_plane_intersections = build_circle_plane_intersections(
        fig.arm_planes, fig.interference_geometry,
    )
    fig.front_lower_intersection_candidates = build_front_lower_intersection_candidates(
        fig.circle_plane_intersections,
    )
    fig.inspect_front_lower_intersections = INSPECT_FRONT_LOWER_INTERSECTIONS
    fig.upper_arm_mount_geometry = build_upper_arm_mount_geometry(
        front, fig.kingpins, fig.circle_plane_intersections,
    )
    fig.lower_arm_mount_geometry = build_lower_arm_mount_geometry(
        front, fig.kingpins, fig.circle_plane_intersections,
    )
    fig.rear_circle_plane_intersections = build_rear_circle_plane_intersections(
        fig.arm_planes, fig.interference_geometry,
    )
    fig.rear_upper_arm_mount_geometry = build_rear_upper_arm_mount_geometry(
        rear, fig.kingpins, fig.rear_circle_plane_intersections,
    )
    fig.rear_lower_arm_mount_geometry = build_rear_lower_arm_mount_geometry(
        rear, fig.kingpins, fig.rear_circle_plane_intersections,
    )

    transverse_min, transverse_max = settings.front_view_x_limits
    height_min, height_max = settings.front_view_y_limits
    lateral_min, lateral_max = -transverse_max, -transverse_min
    view_bounds = np.array([
        longitudinal_limits, (lateral_min - 0.08, lateral_max + 0.08),
        (height_min, height_max + 0.12),
    ])
    circle_points = np.concatenate([
        circle["points"] for wheel in fig.interference_geometry.values()
        for circle in wheel["circles"].values()
    ])
    view_bounds[:, 0] = np.minimum(view_bounds[:, 0], circle_points.min(axis=0) - 0.02)
    view_bounds[:, 1] = np.maximum(view_bounds[:, 1], circle_points.max(axis=0) + 0.02)
    mount_points = [
        endpoint for geometry in (
            fig.upper_arm_mount_geometry, fig.lower_arm_mount_geometry,
            fig.rear_upper_arm_mount_geometry, fig.rear_lower_arm_mount_geometry,
        )
        for arm in geometry.values()
        for endpoint in arm["mounts"].values()
    ]
    if mount_points:
        mount_points = np.asarray(mount_points)
        view_bounds[:, 0] = np.minimum(view_bounds[:, 0], mount_points.min(axis=0) - 0.06)
        view_bounds[:, 1] = np.maximum(view_bounds[:, 1], mount_points.max(axis=0) + 0.06)
    if fit_arm_planes:
        vertices = np.concatenate([plane["vertices"] for plane in fig.arm_planes.values()])
        padding = np.array([longitudinal_margin, 0.08, 0.06])
        view_bounds[:, 0] = np.minimum(view_bounds[:, 0], vertices.min(axis=0) - padding)
        view_bounds[:, 1] = np.maximum(view_bounds[:, 1], vertices.max(axis=0) + padding)
    fig.arm_plane_view_bounds = view_bounds.copy()
    fig.arm_plane_artists = draw_arm_planes(
        ax, fig.arm_planes, settings.arm_plane_alpha,
        bounds=None if fit_arm_planes else view_bounds,
        clip_segment=clip_segment_to_box,
        corner_colors=corner_colors, sv_color=side_view_plane.color,
        show_faces=SHOW_REFERENCE_PLANES,
        visible_fv_corners={
            corner for corner in corner_colors
            if (SHOW_FRONT_FV if corner.startswith("F") else SHOW_REAR_FV)
        },
        show_sv_anchors=SHOW_SV,
    )
    fig.interference_artists = draw_interference_circles(
        ax, fig.interference_geometry, corner_colors=corner_colors,
    )
    fig.circle_plane_intersection_artists = draw_selected_intersections(
        ax, {
            key: arm for key, arm in fig.circle_plane_intersections.items()
            if not INSPECT_FRONT_LOWER_INTERSECTIONS or arm["arm"] != "lower"
        }, corner_colors=corner_colors,
    )
    fig.front_lower_intersection_candidate_artists = (
        draw_front_lower_candidates(
            ax, fig.front_lower_intersection_candidates, corner_colors=corner_colors,
        ) if INSPECT_FRONT_LOWER_INTERSECTIONS else {}
    )
    fig.upper_arm_mount_artists = draw_upper_arm_mounts(
        ax, fig.upper_arm_mount_geometry, corner_colors=corner_colors,
    )
    fig.lower_arm_mount_artists = (
        {} if INSPECT_FRONT_LOWER_INTERSECTIONS else draw_lower_arm_mounts(
            ax, fig.lower_arm_mount_geometry, corner_colors=corner_colors,
        )
    )
    fig.rear_circle_plane_intersection_artists = draw_selected_intersections(
        ax, fig.rear_circle_plane_intersections, corner_colors=corner_colors,
    )
    fig.rear_upper_arm_mount_artists = draw_upper_arm_mounts(
        ax, fig.rear_upper_arm_mount_geometry, corner_colors=corner_colors,
    )
    fig.rear_lower_arm_mount_artists = draw_lower_arm_mounts(
        ax, fig.rear_lower_arm_mount_geometry, corner_colors=corner_colors,
    )
    ax.set_xlim(*view_bounds[0])
    ax.set_ylim(*view_bounds[1])
    ax.set_zlim(*view_bounds[2])
    ax.zaxis.set_major_locator(MaxNLocator(nbins=4))
    ax.set_box_aspect(view_bounds[:, 1] - view_bounds[:, 0])
    ax.view_init(elev=settings.front_view_elevation_deg,
                 azim=settings.front_view_azimuth_deg)
    ax.set_xlabel("Longitudinal X [m] (+front)", labelpad=12)
    ax.set_ylabel("Lateral Y [m] (+left)", labelpad=12)
    ax.set_zlabel("Height Z [m]", labelpad=8)
    ax.set_title(
        f"FV / SV and 8 A-arm Planes | Wheelbase = {car.l:.3f} m", pad=22
    )
    ax.grid(True, alpha=0.25)
    legend_handles = [
        Line2D([], [], color=front.color, linewidth=2,
               label="Front FV / front geometry" if SHOW_FRONT_FV else "Front 3D geometry"),
        Line2D([], [], color=rear.color, linewidth=2,
               label="Rear FV / rear geometry" if SHOW_REAR_FV else "Rear 3D geometry"),
        Line2D([], [], color=side_view_plane.color, linewidth=2, label="SV geometry"),
        Line2D([], [], color=REFERENCE_COLOR, linestyle="--", linewidth=2,
               label="3D kingpin axis"),
        Line2D([], [], color=REFERENCE_COLOR, marker="D", markeredgecolor="black",
               linestyle="none", label="Upper ball joint (UBJ)"),
        Line2D([], [], color=REFERENCE_COLOR, marker="s", markeredgecolor="black",
               linestyle="none", label="Lower ball joint (LBJ)"),
        Line2D([], [], color=REFERENCE_COLOR, marker=INTERSECTION_MARKERS["upper"],
               markeredgecolor="black", linestyle="none", label="Upper clearance intersections"),
        Line2D([], [], color=REFERENCE_COLOR, marker=INTERSECTION_MARKERS["lower"],
               markeredgecolor="black", linestyle="none", label="Lower clearance intersections"),
        Line2D([], [], color=REFERENCE_COLOR, marker="s", markeredgecolor="black",
               linewidth=2.6, label="A-arm legs / chassis joints"),
        Line2D([], [], color=REFERENCE_COLOR, linewidth=2,
               linestyle=INTERFERENCE_LINESTYLES["neutral"], label="Neutral clearance circle"),
        Line2D([], [], color=REFERENCE_COLOR, linewidth=2,
               linestyle=INTERFERENCE_LINESTYLES["positive"], label=f"Clearance +{car.steer:g} deg"),
        Line2D([], [], color=REFERENCE_COLOR, linewidth=2,
               linestyle=INTERFERENCE_LINESTYLES["negative"], label=f"Clearance -{car.steer:g} deg"),
        Line2D([], [], color=REFERENCE_COLOR, linewidth=1.8,
               linestyle="-", label="Tire FV rectangle"),
        Line2D([], [], color=REFERENCE_COLOR, linewidth=1.5,
               linestyle="--", label="Rim inner FV rectangle"),
        Patch(facecolor="0.3", alpha=0.22, label="3D cylindrical tires"),
        Patch(facecolor=CHASSIS_COLOR, edgecolor=REFERENCE_COLOR,
              alpha=settings.chassis_face_alpha, label="Chassis (8 pickups)"),
        Line2D([], [], color=REFERENCE_COLOR, marker="*", linestyle="none", label="Vehicle CG"),
    ]
    if SHOW_REFERENCE_PLANES:
        legend_handles.insert(-1, Patch(
            facecolor=REFERENCE_COLOR, alpha=settings.arm_plane_alpha,
            label="A-arm reference planes",
        ))
    if INSPECT_FRONT_LOWER_INTERSECTIONS:
        legend_handles.insert(-1, Line2D(
            [], [], color=front.color, marker="o", markeredgecolor="black",
            linestyle="none", label="Front lower candidates (a/b: +, c/d: -)",
        ))
    if not SHOW_SV:
        legend_handles = [handle for handle in legend_handles if handle.get_label() != "SV geometry"]
    fig.legend(handles=legend_handles, loc="lower center", bbox_to_anchor=(0.5, 0.02),
               ncol=4, fontsize=9)
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.15, top=0.90)
    return fig, ax, front, rear


if __name__ == "__main__":
    fig, ax, front_plane, rear_plane = create_figure()
    side_view_plane = fig.side_view_plane
    if INSPECT_FRONT_LOWER_INTERSECTIONS:
        inspection_fig, inspection_axes = create_front_lower_inspection_figure(
            fig.arm_planes, fig.interference_geometry, fig.front_lower_intersection_candidates,
        )
    plt.show()
