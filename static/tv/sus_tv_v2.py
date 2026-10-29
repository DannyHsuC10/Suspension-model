"""Static top-view steering geometry."""

import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

from static import geometry as ge
from static.car import Car
from static.visualization import LinkPlot, PointPlot, PolygonPlot


# -----------------------------------------------------------------------------
# Editable geometry and plot parameters
# Coordinates use X = vehicle longitudinal direction, Y = vehicle track direction.
# -----------------------------------------------------------------------------
car = Car()

WHEEL_CENTER_X = 0.0
AA_POINT_LEFT = np.array([0.0, 0.5])
P_TIE_LEFT = np.array([-0.30, 0.45])

RACK_LENGTH = 0.5
RACK_X = -0.4

FIGURE_SIZE = (8, 8)
PLOT_X_LIMITS = (-0.9, 0.45)
PLOT_Y_LIMITS = (-0.95, 0.95)


class Scene:
    """Draw the initial geometry using the shared Matplotlib helpers."""

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

class SteeringAssembly:
    """Tire, A-arm reference, and tie pickup at their initial positions."""

    def __init__(self, name, side, car, rack_point, tire_color):
        self.name = name
        self.side = side  # left is +y, right is -y
        self.car = car
        self.wheel_center_initial = np.array(
            [WHEEL_CENTER_X, side * car.tf / 2.0]
        )

        tie_position = np.array([P_TIE_LEFT[0], side * P_TIE_LEFT[1]])
        aa_position = np.array([AA_POINT_LEFT[0], side * AA_POINT_LEFT[1]])
        self.tie_local = tie_position - self.wheel_center_initial
        self.aa_local = aa_position - self.wheel_center_initial
        half_length_x = car.free_radius
        half_width_y = car.tire_width / 2.0
        wheel_local = [
            np.array([-half_length_x, -half_width_y]),
            np.array([half_length_x, -half_width_y]),
            np.array([half_length_x, half_width_y]),
            np.array([-half_length_x, half_width_y]),
        ]

        self.wheel_center = ge.Point(self.wheel_center_initial, f"{name} wheel center")
        self.tie_point = ge.Point(
            self.wheel_center_initial + self.tie_local, f"{name} P_tie"
        )
        self.aa_point = ge.Point(
            self.wheel_center_initial + self.aa_local, f"{name} AA point"
        )
        self.wheel_points = [
            ge.Point(self.wheel_center_initial + p, f"{name} tire corner {i}")
            for i, p in enumerate(wheel_local)
        ]
        self.tire = ge.Polygon(self.wheel_points, name=f"{name} tire")
        self.upright_arm = ge.Link(
            self.aa_point, self.tie_point, f"{name} AA to tie-rod pickup"
        )
        self.tie_rod = ge.Link(self.tie_point, rack_point, f"{name} tie rod")
        self.color = tire_color

    def add_to_scene(self, scene):
        scene.add(self.tire, facecolor="none", edgecolor=self.color, linewidth=2.2)
        scene.add(self.upright_arm, color="tab:green", linewidth=2)
        scene.add(self.tie_rod, color="tab:orange", linewidth=2)
        scene.add(self.wheel_center, color=self.color, markersize=5)
        scene.add(self.tie_point, color="tab:orange", markersize=5)
        scene.add(self.aa_point, color="tab:green", markersize=5)


fig, ax = plt.subplots(figsize=FIGURE_SIZE)
scene = Scene(ax)

# Draw the fixed steering rack at its initial location.
rack_left = ge.Point([RACK_X, RACK_LENGTH / 2.0], "Rack left end")
rack_right = ge.Point([RACK_X, -RACK_LENGTH / 2.0], "Rack right end")
rack = ge.Link(rack_right, rack_left, "Steering rack")
left = SteeringAssembly("Left", +1.0, car, rack_left, "tab:blue")
right = SteeringAssembly("Right", -1.0, car, rack_right, "tab:red")
left.add_to_scene(scene)
right.add_to_scene(scene)
scene.add(rack, color="black", linewidth=3)
scene.add(rack_left, color="black", markersize=6)
scene.add(rack_right, color="black", markersize=6)


ax.axvline(0.0, color="0.55", linestyle="--", linewidth=1, label="Vehicle y-axis")
ax.axhline(0.0, color="0.75", linewidth=0.8)
ax.set_aspect("equal", adjustable="box")
ax.set_xlim(*PLOT_X_LIMITS)
ax.set_ylim(*PLOT_Y_LIMITS)
ax.grid(True, linestyle=":")
ax.set_xlabel("Longitudinal X [m]")
ax.set_ylabel("Lateral Y [m]")
ax.set_title("Static Top-View Steering Geometry")
ax.legend(handles=[
    Line2D([0], [0], color="black", lw=3, label="Steering rack"),
    Line2D([0], [0], color="tab:orange", lw=2, label="Fixed-length tie rod"),
    Line2D([0], [0], color="tab:green", lw=2, label="Upright: AA to tie-rod pickup"),
    Line2D([0], [0], color="tab:blue", lw=2, label="Left tire"),
    Line2D([0], [0], color="tab:red", lw=2, label="Right tire"),
], loc="upper right", fontsize=8)

plt.show()
