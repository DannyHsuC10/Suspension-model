"""Static wheel-end geometry at the design pose."""

import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import matplotlib.pyplot as plt
from static.car import Car, SuspensionF
from static.we3D.wheel_end_geometry import (
    initial_wheel_center, kingpin_axis,
)


car = Car()
suspension_f = SuspensionF()

# Wheel-end parameters
tire_radius = car.free_radius  # [m]
tire_width = car.tire_width  # [m]
initial_center = initial_wheel_center(suspension_f, tire_radius)
# A material point initially at the tire bottom; no ground-contact correction.
initial_contact_point = initial_center + np.array([0.0, 0.0, -tire_radius])


# Tire surface, modeled as a cylinder with its axis along Y.
theta = np.linspace(0.0, 2.0 * np.pi, 50)
width_axis = np.linspace(-tire_width / 2.0, tire_width / 2.0, 20)
theta_grid, width_grid = np.meshgrid(theta, width_axis)
x = tire_radius * np.sin(theta_grid)
y = width_grid
z = tire_radius * np.cos(theta_grid)

# Fixed kingpin through the origin; the tire is initially offset from it.
axis_dir = kingpin_axis(suspension_f)

# The tire is modeled with its width along Y, so its centerline is the Y axis.
wheel_axis = np.array([
    [0.0, -tire_width / 2.0, 0.0],
    [0.0, tire_width / 2.0, 0.0],
]) + initial_center
axis_line = np.outer(np.linspace(-1.5 * tire_radius, 1.5 * tire_radius, 10), axis_dir)

fig = plt.figure(figsize=(9, 8))
ax = fig.add_subplot(111, projection="3d")
fig.subplots_adjust(top=0.80, bottom=0.08, left=0.08, right=0.95)

# Reference (unsteered) tire and kingpin axis.
ax.plot_surface(x + initial_center[0], y + initial_center[1], z + initial_center[2],
                alpha=0.8, color="orange", linewidth=0)
ax.plot(axis_line[:, 0], axis_line[:, 1], axis_line[:, 2], color="black", label="Kingpin Axis")
ax.set_box_aspect((1, 1, 1))
view_limit = tire_radius + tire_width / 2.0 + np.max(np.abs(initial_center))
ax.set_xlim(-view_limit, view_limit)
ax.set_ylim(-view_limit, view_limit)
ax.set_zlim(-view_limit, view_limit)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("Static Wheel End Geometry")

# Draw the unsteered wheel-end references once.
contact = initial_contact_point
center = initial_center
projection = np.dot(contact, axis_dir) * axis_dir
arm_length = np.linalg.norm(contact - projection)
ax.scatter(*contact, color="red", s=55, label="Tire-bottom Reference")
ax.scatter(*center, color="tab:purple", s=35, label="Wheel Center")
ax.scatter(*projection, color="green", s=40, label="Axis Projection")
ax.plot([projection[0], contact[0]], [projection[1], contact[1]],
        [projection[2], contact[2]], color="black", linewidth=2, label="Moment Arm")
ax.plot([0.0, contact[0]], [0.0, contact[1]], [0.0, contact[2]],
        linestyle="--", color="red", alpha=0.7)
ax.plot(*wheel_axis.T, color="purple", linewidth=2.5, label="Wheel Centerline")
fig.text(
    0.06, 0.975,
    f"Steer = 0.0°    Bottom ref. to axis = {arm_length * 1000:.1f} mm\n"
    f"Center [mm] = ({center[0]*1000:+.2f}, {center[1]*1000:+.2f}, {center[2]*1000:+.2f})\n"
    f"Trail = {suspension_f.mechanical_trail_sv*1000:.1f} mm, "
    f"scrub = {suspension_f.scrub_radius*1000:.1f} mm; tilt = +0.00°",
    va="top", fontsize=9,
)
fig.legend(*ax.get_legend_handles_labels(), loc="upper left",
           bbox_to_anchor=(0.05, 0.915), ncol=3, fontsize=8)
plt.show()
