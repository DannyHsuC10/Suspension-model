import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from suspension_geo.car import Car, SuspensionF


car = Car()
suspension_f = SuspensionF()

# Wheel-end parameters
kpi_rad = suspension_f.kpi
caster_rad = np.deg2rad(suspension_f.caster_sv_deg)
tire_radius = car.free_radius  # [m]
tire_width = car.tire_width  # [m]
steer_limit_deg = 30.0
steer_samples = 100
initial_contact_point = np.array([0.0, 0.0, -tire_radius])


def rotation_matrix(axis, theta):
    """Return the Rodrigues rotation matrix for rotation about ``axis``."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    kx, ky, kz = axis
    cross_matrix = np.array([
        [0.0, -kz, ky],
        [kz, 0.0, -kx],
        [-ky, kx, 0.0],
    ])
    identity = np.eye(3)
    return identity + np.sin(theta) * cross_matrix + (1.0 - np.cos(theta)) * (cross_matrix @ cross_matrix)


# Tire surface, modeled as a cylinder with its axis along Y.
theta = np.linspace(0.0, 2.0 * np.pi, 50)
width_axis = np.linspace(-tire_width / 2.0, tire_width / 2.0, 20)
theta_grid, width_grid = np.meshgrid(theta, width_axis)
x = tire_radius * np.sin(theta_grid)
y = width_grid
z = tire_radius * np.cos(theta_grid)
points = np.column_stack((x.ravel(), y.ravel(), z.ravel()))

# Kingpin axis: caster about X and KPI about Y, expressed in the wheel-end frame.
axis_dir = np.array([
    np.sin(caster_rad),
    np.sin(kpi_rad),
    np.cos(kpi_rad) * np.cos(caster_rad),
])
axis_dir /= np.linalg.norm(axis_dir)

# The tire is modeled with its width along Y, so its centerline is the Y axis.
wheel_axis = np.array([
    [0.0, -tire_width / 2.0, 0.0],
    [0.0, tire_width / 2.0, 0.0],
])
axis_line = np.outer(np.linspace(-1.5 * tire_radius, 1.5 * tire_radius, 10), axis_dir)

# Precompute the contact-point path and signed moment arms for the lower plots.
steer_range = np.linspace(-steer_limit_deg, steer_limit_deg, steer_samples)
contact_path = np.empty((steer_samples, 3))
moment_arms = np.empty((steer_samples, 3))
tilt_angles_deg = np.empty(steer_samples)
unit_forces = np.eye(3)
for i, steer in enumerate(steer_range):
    matrix = rotation_matrix(axis_dir, np.radians(steer))
    contact_path[i] = matrix @ initial_contact_point
    centerline_direction = matrix @ np.array([0.0, 1.0, 0.0])
    # Smallest angle between the tire centerline and the horizontal ground plane.
    tilt_angles_deg[i] = np.degrees(np.arctan2(
        abs(centerline_direction[2]), np.linalg.norm(centerline_direction[:2])
    ))
    for j, force in enumerate(unit_forces):
        moment_arms[i, j] = np.dot(np.cross(contact_path[i], force), axis_dir)


fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(221, projection="3d")
ax_contact = fig.add_subplot(222)
ax_arms = fig.add_subplot(223)
ax_tilt = fig.add_subplot(224)
fig.subplots_adjust(bottom=0.16, hspace=0.35, wspace=0.25)

# Reference (unsteered) tire and kingpin axis.
ax.plot_surface(x, y, z, alpha=0.16, color="steelblue", linewidth=0)
ax.plot(axis_line[:, 0], axis_line[:, 1], axis_line[:, 2], color="black", label="Kingpin Axis")
ax.set_box_aspect((1, 1, 1))
view_limit = tire_radius + tire_width / 2.0
ax.set_xlim(-view_limit, view_limit)
ax.set_ylim(-view_limit, view_limit)
ax.set_zlim(-view_limit, view_limit)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title("Wheel End Steering")

# Contact point path and moment-arm curves.
contact_lines = []
for component, label in enumerate(("X", "Y", "Z")):
    line, = ax_contact.plot(steer_range, contact_path[:, component] * 1000.0, label=label)
    contact_lines.append(line)
ax_contact.set_xlabel("Steer Angle [deg]")
ax_contact.set_ylabel("Contact Point [mm]")
ax_contact.set_title("Contact Point vs Steering Angle")
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
ax_arms.legend(ncol=3)

tilt_line, = ax_tilt.plot(steer_range, tilt_angles_deg, color="tab:purple",
                          label="Tire tilt angle")
ax_tilt.set_xlabel("Steer Angle [deg]")
ax_tilt.set_ylabel("Angle to Ground Plane [deg]")
ax_tilt.set_title("Tire Centerline Tilt")
ax_tilt.grid(True)
ax_tilt.legend()

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
    projection = np.dot(contact, axis_dir) * axis_dir
    arm_length = np.linalg.norm(contact - projection)
    centerline_direction = matrix @ np.array([0.0, 1.0, 0.0])
    tilt_angle_deg = np.degrees(np.arctan2(
        abs(centerline_direction[2]), np.linalg.norm(centerline_direction[:2])
    ))

    # Matplotlib's 3D surface has no in-place coordinate setter, so replace it.
    if hasattr(update, "tire_surface"):
        update.tire_surface.remove()
    update.tire_surface = ax.plot_surface(xr, yr, zr, alpha=0.8, color="orange", linewidth=0)

    if not hasattr(update, "artists"):
        contact_artist = ax.scatter(*contact, color="red", s=55, label="Contact Point")
        projection_artist = ax.scatter(*projection, color="green", s=40, label="Axis Projection")
        radius_line, = ax.plot([projection[0], contact[0]], [projection[1], contact[1]],
                               [projection[2], contact[2]], color="black", linewidth=2,
                               label="Moment Arm")
        radius_vector, = ax.plot([0.0, contact[0]], [0.0, contact[1]], [0.0, contact[2]],
                                 linestyle="--", color="red", alpha=0.7)
        rotated_axis, = ax.plot([], [], [], color="purple", linewidth=2.5,
                                label="Wheel Centerline")
        arm_text = ax.text2D(0.03, 0.95, "", transform=ax.transAxes)
        ax.legend(loc="upper left", fontsize="small")
        update.artists = (contact_artist, projection_artist, radius_line, radius_vector,
                          rotated_axis, arm_text)

    contact_artist, projection_artist, radius_line, radius_vector, rotated_axis, arm_text = update.artists
    contact_artist._offsets3d = ([contact[0]], [contact[1]], [contact[2]])
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
        f"Steer = {steer_deg:.1f}°    Contact to axis = {arm_length * 1000:.1f} mm\n"
        f"Tire centerline angle to ground = {tilt_angle_deg:.2f}°"
    )
    for marker in contact_markers:
        marker.set_xdata([steer_deg, steer_deg])
    fig.canvas.draw_idle()


steer_slider.on_changed(update)
update(steer_slider.val)
plt.show()
