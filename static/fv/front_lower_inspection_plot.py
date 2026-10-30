"""Close-up views of every front lower-arm / steering-circle intersection."""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.text import Annotation
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


FRONT_COLOR = "#0072b2"
POSE_STYLES = {"positive": "--", "negative": ":"}
POSE_NAMES = {"positive": "a / b", "negative": "c / d"}


class _PointLabel(Annotation):
    """Keep letter labels separated in screen space as the 3D view rotates."""

    def __init__(self, text, point, offset):
        self.point = np.asarray(point, dtype=float).copy()
        super().__init__(
            text, xy=(0, 0), xytext=offset, textcoords="offset points",
            fontsize=15, fontweight="bold", ha="center", va="center",
            arrowprops=dict(arrowstyle="-", color="0.25", linewidth=.8),
            bbox=dict(facecolor="white", alpha=.95, edgecolor="none", pad=1.2),
            annotation_clip=False,
        )

    def draw(self, renderer):
        self.xy = proj3d.proj_transform(*self.point, self.axes.get_proj())[:2]
        super().draw(renderer)


def _plane_patch(point, normal, bounds):
    """Make a local piece of the infinite plane, independent of its IC triangle."""
    solved = int(np.argmax(np.abs(normal)))
    free = [dimension for dimension in range(3) if dimension != solved]
    patch = []
    for first, second in ((0, 0), (1, 0), (1, 1), (0, 1)):
        vertex = point.copy()
        vertex[free[0]] = bounds[free[0], first]
        vertex[free[1]] = bounds[free[1], second]
        vertex[solved] = point[solved] - sum(
            normal[dimension] * (vertex[dimension] - point[dimension])
            for dimension in free
        ) / normal[solved]
        patch.append(vertex)
    return np.asarray(patch)


def _circle_plane_line(circle, plane_point, plane_normal, radius):
    """Return the two planes' local intersection line even when it misses the circle."""
    center = np.asarray(circle["center"], dtype=float)
    circle_normal = np.asarray(circle["normal"], dtype=float)
    circle_normal = circle_normal / np.linalg.norm(circle_normal)
    in_circle = plane_normal - np.dot(plane_normal, circle_normal) * circle_normal
    length = np.linalg.norm(in_circle)
    if length < 1e-12:
        return None
    along = in_circle / length
    offset = -np.dot(plane_normal, center - plane_point) / length
    line_center = center + offset * along
    direction = np.cross(circle_normal, along)
    return line_center + np.array([-1.2, 1.2])[:, None] * radius * direction


def create_front_lower_inspection_figure(arm_planes, interference_geometry, candidates):
    """Return ``(figure, axes)`` showing both front lower arms without selecting points.

    Input coordinates use metres; the close-ups display vehicle X/Y/Z in mm.
    Letters a/b always belong to +steer and c/d to -steer. Missing poses keep
    their missing letters and are explicitly described in the panel. All
    source coordinates remain unchanged, including the original distant ICs.
    """
    figure = plt.figure(figsize=(16, 8))
    axes = []
    for index, corner in enumerate(("FL", "FR"), start=1):
        axis = figure.add_subplot(1, 2, index, projection="3d")
        axes.append(axis)
        plane = arm_planes[f"{corner}_lower"]
        wheel = interference_geometry[corner]
        record = candidates[corner]
        joint = np.asarray(plane["kingpin_joint"], dtype=float)
        normal = np.asarray(plane["normal"], dtype=float)
        normal = normal / np.linalg.norm(normal)
        circle_cloud = np.concatenate([
            wheel["circles"][pose]["points"] for pose in POSE_STYLES
        ])
        cloud = np.vstack((circle_cloud, joint))
        padding = np.maximum(np.ptp(cloud, axis=0) * .12, .015)
        bounds = np.column_stack((cloud.min(axis=0) - padding, cloud.max(axis=0) + padding))
        patch = _plane_patch(joint, normal, bounds)
        axis.add_collection3d(Poly3DCollection(
            [patch * 1000], facecolor=FRONT_COLOR, alpha=.12,
            edgecolor="0.5", linewidth=.6,
        ))
        status_lines = []
        for pose, linestyle in POSE_STYLES.items():
            circle = wheel["circles"][pose]
            axis.plot(*np.asarray(circle["points"]).T * 1000,
                      color=FRONT_COLOR, linestyle=linestyle, linewidth=2.1)
            line = _circle_plane_line(circle, joint, normal, wheel["radius"])
            if line is not None:
                axis.plot(*line.T * 1000, color="0.4", linestyle=linestyle, linewidth=1.0)
                cloud = np.vstack((cloud, line))
            source = record["by_pose"][pose]
            count = len(source["points"])
            status = {
                "none": "no intersection", "two_points": "2 intersections",
                "tangent": "1 tangent intersection", "coplanar": "coplanar",
            }.get(source["status"], f"{count} intersections")
            status_lines.append(
                f"{circle['angle_deg']:+g} deg ({POSE_NAMES[pose]}): {status}"
            )

        axis.scatter(*(joint * 1000), color=FRONT_COLOR, marker="s", s=55,
                     edgecolor="black", depthshade=False)
        axis.text(*(joint * 1000 + [12, 0, 10]), f"{corner} LBJ", fontsize=9)
        for candidate in record["points"]:
            point = np.asarray(candidate["point"], dtype=float)
            name = candidate["name"]
            forward = name in ("a", "c")
            axis.scatter(*(point * 1000), color=FRONT_COLOR,
                         marker="o" if candidate["pose"] == "positive" else "D",
                         s=100, edgecolor="black", linewidth=.9, depthshade=False)
            axis.add_artist(_PointLabel(
                name, point * 1000, (32, 30) if forward else (-32, -30),
            ))

        cloud = np.vstack((cloud, patch))
        limits = np.column_stack((cloud.min(axis=0) - .008, cloud.max(axis=0) + .008))
        axis.set_xlim(*(limits[0] * 1000))
        axis.set_ylim(*(limits[1] * 1000))
        axis.set_zlim(*(limits[2] * 1000))
        axis.set_box_aspect(limits[:, 1] - limits[:, 0])
        axis.view_init(elev=24, azim=-58 if corner == "FL" else 58)
        axis.set_xlabel("Vehicle X [mm] (+front)", labelpad=10)
        axis.set_ylabel("Vehicle Y [mm] (+left)", labelpad=10)
        axis.set_zlabel("Vehicle Z [mm]", labelpad=7)
        axis.set_title(f"{corner}: lower A-arm plane and steering circles", pad=22)
        axis.text2D(.02, .96, "\n".join(status_lines), transform=axis.transAxes,
                    fontsize=9, va="top",
                    bbox=dict(facecolor="white", edgecolor="0.8", alpha=.95, pad=4))
        axis.grid(True, alpha=.25)

    figure.suptitle("All front lower-arm intersections — no automatic point selection", fontsize=15)
    figure.legend(handles=[
        Line2D([], [], color=FRONT_COLOR, linestyle="--", linewidth=2, label="+steer circle: a / b"),
        Line2D([], [], color=FRONT_COLOR, linestyle=":", linewidth=2, label="-steer circle: c / d"),
        Line2D([], [], color="0.4", linestyle="--", label="Arm plane / circle-plane intersection line"),
        Patch(facecolor=FRONT_COLOR, alpha=.12, label="Local infinite lower-arm plane"),
    ], loc="lower center", ncol=2, bbox_to_anchor=(.5, .025), fontsize=10)
    figure.subplots_adjust(left=.015, right=.985, top=.88, bottom=.17, wspace=.08)
    return figure, tuple(axes)
