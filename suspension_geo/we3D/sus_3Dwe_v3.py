import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from suspension_geo.car import Car, SuspensionF
from suspension_geo.we3D.wheel_end_geometry import (
    initial_wheel_center, kingpin_axis, rotation_matrix,
)


car = Car()
suspension_f = SuspensionF()

# Wheel-end parameters
kpi_rad = suspension_f.kpi
caster_rad = np.deg2rad(suspension_f.caster_sv_deg)
tire_radius = car.free_radius  # [m]
tire_width = car.tire_width  # [m]
steer_limit_deg = 30.0
steer_samples = 100
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
points = np.column_stack((x.ravel(), y.ravel(), z.ravel())) + initial_center

# Fixed origin = static kingpin/ground intersection; initial ground is z=0.
axis_dir = kingpin_axis(suspension_f)

# The tire is modeled with its width along Y, so its centerline is the Y axis.
wheel_axis = np.array([
    [0.0, -tire_width / 2.0, 0.0],
    [0.0, tire_width / 2.0, 0.0],
]) + initial_center
axis_line = np.outer(np.linspace(0.0, 2.5 * tire_radius, 10), axis_dir)

# Precompute the contact-point path and signed moment arms for the lower plots.
steer_range = np.linspace(-steer_limit_deg, steer_limit_deg, steer_samples)
contact_path = np.empty((steer_samples, 3))
center_path = np.empty((steer_samples, 3))
moment_arms = np.empty((steer_samples, 3))
tilt_angles_deg = np.empty(steer_samples)
unit_forces = np.eye(3)
for i, steer in enumerate(steer_range):
    matrix = rotation_matrix(axis_dir, np.radians(steer))
    center_path[i] = matrix @ initial_center
    contact_path[i] = matrix @ initial_contact_point
    centerline_direction = matrix @ np.array([0.0, 1.0, 0.0])
    # Signed elevation of the rotated +Y wheel axis: positive up, negative down.
    tilt_angles_deg[i] = np.degrees(np.arctan2(
        centerline_direction[2], np.linalg.norm(centerline_direction[:2])
    ))
    for j, force in enumerate(unit_forces):
        moment_arms[i, j] = np.dot(np.cross(contact_path[i], force), axis_dir)


fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(221, projection="3d")
ax_contact = fig.add_subplot(222)
ax_arms = fig.add_subplot(223)
ax_tilt = fig.add_subplot(224)
fig.subplots_adjust(top=0.78, bottom=0.18, hspace=0.45, wspace=0.35)

# Reference (unsteered) tire and kingpin axis.
ax.plot_surface(x + initial_center[0], y + initial_center[1], z + initial_center[2],
                alpha=0.16, color="steelblue", linewidth=0)
ax.plot(axis_line[:, 0], axis_line[:, 1], axis_line[:, 2], color="black", label="Kingpin Axis")
ax.scatter(0.0, 0.0, 0.0, color="black", s=25, label="Kingpin-ground Origin")
ax.plot(*center_path.T, color="tab:purple", linestyle=":", label="Wheel Center Path")
view_limit = tire_radius + tire_width / 2.0 + np.max(np.abs(center_path[:, :2]))
z_min = min(-0.1 * tire_radius, np.min(center_path[:, 2]) - tire_radius - tire_width / 2.0)
z_max = max(np.max(axis_line[:, 2]), np.max(center_path[:, 2]) + tire_radius + tire_width / 2.0)
ax.plot([-view_limit, view_limit, view_limit, -view_limit, -view_limit],
        [-view_limit, -view_limit, view_limit, view_limit, -view_limit], np.zeros(5),
        color="gray", alpha=0.45, linewidth=0.8)
ax.set_box_aspect((2.0 * view_limit, 2.0 * view_limit, z_max - z_min))
ax.set_xlim(-view_limit, view_limit)
ax.set_ylim(-view_limit, view_limit)
ax.set_zlim(z_min, z_max)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("Wheel End Steering: ground origin")

# Contact point path and moment-arm curves.
contact_lines = []
for component, label in enumerate(("X", "Y", "Z")):
    line, = ax_contact.plot(steer_range, contact_path[:, component] * 1000.0, label=label)
    contact_lines.append(line)
ax_contact.set_xlabel("Steer Angle [deg]")
ax_contact.set_ylabel("Tire-bottom Reference Point [mm]")
ax_contact.set_title("Tire-bottom Reference vs Steering Angle")
ax_contact.grid(True)
ax_contact.legend(ncol=3)

arm_labels = ("Fx moment arm", "Fy moment arm", "Fz moment arm")
arm_lines = []
for j, label in enumerate(arm_labels):
    line, = ax_arms.plot(steer_range, moment_arms[:, j] * 1000.0, label=label)
    arm_lines.append(line)
ax_arms.set_xlabel("Steer Angle [deg]")
ax_arms.set_ylabel("Moment Arm [mm]")
ax_arms.set_title("Steering Moment Arm")
ax_arms.grid(True)
ax_arms.legend(loc="upper right", fontsize=8)

tilt_line, = ax_tilt.plot(steer_range, tilt_angles_deg, color="tab:purple",
                          label="+Y wheel axis: +up / -down")
ax_tilt.axhline(0.0, color="gray", linewidth=0.8)
ax_tilt.set_xlabel("Steer Angle [deg]")
ax_tilt.set_ylabel("Signed Angle to Ground Plane [deg]")
ax_tilt.set_title("Signed Tire Centerline Tilt")
ax_tilt.grid(True)
ax_tilt.legend(fontsize=8)

contact_markers = [
    ax_contact.axvline(steer_range[0], color="black", linestyle="--", alpha=0.6),
    ax_arms.axvline(steer_range[0], color="black", linestyle="--", alpha=0.6),
    ax_tilt.axvline(steer_range[0], color="black", linestyle="--", alpha=0.6),
]

# Dynamic artists are replaced/updated as the slider moves.
steer_slider_ax = fig.add_axes((0.20, 0.06, 0.62, 0.035))
steer_slider = Slider(
    steer_slider_ax,
    "Steer [deg]",
    -steer_limit_deg,
    steer_limit_deg,
    valinit=0.0,
    valstep=0.5,
)


def update(steer_deg):
    matrix = rotation_matrix(axis_dir, np.radians(steer_deg))
    rotated = (matrix @ points.T).T
    xr = rotated[:, 0].reshape(x.shape)
    yr = rotated[:, 1].reshape(y.shape)
    zr = rotated[:, 2].reshape(z.shape)
    contact = matrix @ initial_contact_point
    center = matrix @ initial_center
    projection = np.dot(contact, axis_dir) * axis_dir
    arm_length = np.linalg.norm(contact - projection)
    centerline_direction = matrix @ np.array([0.0, 1.0, 0.0])
    tilt_angle_deg = np.degrees(np.arctan2(
        centerline_direction[2], np.linalg.norm(centerline_direction[:2])
    ))

    # Matplotlib's 3D surface has no in-place coordinate setter, so replace it.
    if hasattr(update, "tire_surface"):
        update.tire_surface.remove()
    update.tire_surface = ax.plot_surface(xr, yr, zr, alpha=0.8, color="orange", linewidth=0)

    if not hasattr(update, "artists"):
        contact_artist = ax.scatter(*contact, color="red", s=55, label="Tire-bottom Reference")
        center_artist = ax.scatter(*center, color="tab:purple", s=35, label="Wheel Center")
        projection_artist = ax.scatter(*projection, color="green", s=40, label="Axis Projection")
        radius_line, = ax.plot([projection[0], contact[0]], [projection[1], contact[1]],
                               [projection[2], contact[2]], color="black", linewidth=2,
                               label="Moment Arm")
        radius_vector, = ax.plot([0.0, contact[0]], [0.0, contact[1]], [0.0, contact[2]],
                                 linestyle="--", color="red", alpha=0.7)
        rotated_axis, = ax.plot([], [], [], color="purple", linewidth=2.5,
                                label="Wheel Centerline")
        arm_text = fig.text(0.06, 0.975, "", va="top", fontsize=9)
        fig.legend(*ax.get_legend_handles_labels(), loc="upper left",
                   bbox_to_anchor=(0.05, 0.915), ncol=4, fontsize=8)
        update.artists = (contact_artist, center_artist, projection_artist, radius_line, radius_vector,
                          rotated_axis, arm_text)

    (contact_artist, center_artist, projection_artist, radius_line, radius_vector,
     rotated_axis, arm_text) = update.artists
    contact_artist._offsets3d = ([contact[0]], [contact[1]], [contact[2]])
    center_artist._offsets3d = ([center[0]], [center[1]], [center[2]])
    projection_artist._offsets3d = ([projection[0]], [projection[1]], [projection[2]])
    radius_line.set_data_3d(
        [projection[0], contact[0]], [projection[1], contact[1]], [projection[2], contact[2]]
    )
    radius_vector.set_data_3d([0.0, contact[0]], [0.0, contact[1]], [0.0, contact[2]])
    # Rotate both endpoints of the tire centerline with the same steering matrix.
    rotated_wheel_axis = (matrix @ wheel_axis.T).T
    rotated_axis.set_data_3d(
        rotated_wheel_axis[:, 0], rotated_wheel_axis[:, 1], rotated_wheel_axis[:, 2]
    )
    arm_text.set_text(
        f"Steer = {steer_deg:.1f}°    Bottom ref. to axis = {arm_length * 1000:.1f} mm\n"
        f"Center [mm] = ({center[0]*1000:+.2f}, {center[1]*1000:+.2f}, {center[2]*1000:+.2f})\n"
        f"Trail = {suspension_f.mechanical_trail_sv*1000:.1f} mm, "
        f"scrub = {suspension_f.scrub_radius*1000:.1f} mm; tilt = {tilt_angle_deg:+.2f}°"
    )
    for marker in contact_markers:
        marker.set_xdata([steer_deg, steer_deg])
    fig.canvas.draw_idle()


steer_slider.on_changed(update)
update(steer_slider.val)
plt.show()
