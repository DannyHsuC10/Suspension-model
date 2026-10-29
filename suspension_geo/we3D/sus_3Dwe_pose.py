"""Wheel-center frame and XYZ Euler angles for sus_3Dwe_v3 steering.

The input is rotation ABOUT THE KINGPIN, matching the original slider.
The fixed origin is the static kingpin/ground intersection; ground is z=0.
Trail/scrub offset the initial wheel center before rotation about this axis.
R maps wheel-local coordinates to reference coordinates. Its columns are
the wheel's X/Y/Z unit axes, and R = Rz(rz) @ Ry(ry) @ Rx(rx) (fixed XYZ).
Tire width is along local Y; position is expressed in the fixed kingpin frame.
"""

import argparse
import csv
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np

from suspension_geo.car import Car, SuspensionF
from suspension_geo.we3D.wheel_end_geometry import (
    initial_wheel_center, kingpin_axis, rotation_matrix,
)


STEER_LIMIT_DEG = 30.0
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parent / "wheel_pose_results"
POSE_CSV_FIELDS = [
    "kingpin_rotation_deg", "center_x_mm", "center_y_mm", "center_z_mm",
    "rx_deg", "ry_deg", "rz_deg",
    *[f"wheel_{local}_in_ref_{ref}" for local in "xyz" for ref in "xyz"],
]
KINGPIN_CSV_FIELDS = [f"kingpin_axis_{axis}" for axis in "xyz"]


def calculate_wheel_pose(steer_deg, *, suspension=None, car=None):
    """Return center [m], rotation matrix and fixed XYZ Euler angles [deg].

    Positive input follows the right-hand rule about the original kingpin.
    This is the single wheel-end frame of sus_3Dwe_v3, without left/right
    mirroring, wheel spin or suspension travel. Ground-plane mechanical
    trail and scrub set the initial center offset; center = R @ center_0.
    Euler angles describe orientation relative to the zero-steer frame;
    they are not three independent rotations about the kingpin.
    """
    steer_deg = float(steer_deg)
    if not np.isfinite(steer_deg) or abs(steer_deg) > STEER_LIMIT_DEG:
        raise ValueError("Kingpin rotation must be finite and within [-30, 30] deg.")
    suspension = SuspensionF() if suspension is None else suspension
    car = Car() if car is None else car
    axis = kingpin_axis(suspension)
    center_0 = initial_wheel_center(suspension, car.free_radius)
    matrix = rotation_matrix(axis, np.deg2rad(steer_deg))

    # A total rotation <= 30 deg cannot reach the XYZ pitch singularity.
    rx = np.arctan2(matrix[2, 1], matrix[2, 2])
    ry = np.arctan2(-matrix[2, 0], np.hypot(matrix[0, 0], matrix[1, 0]))
    rz = np.arctan2(matrix[1, 0], matrix[0, 0])
    return {
        "kingpin_rotation_deg": steer_deg,
        "center_m": matrix @ center_0,
        "initial_center_m": center_0,
        "kingpin_axis": axis,
        "rotation_matrix": matrix,
        "euler_xyz_deg": np.rad2deg([rx, ry, rz]),
    }


def sweep_wheel_poses(step_deg=0.5, *, suspension=None, car=None):
    """Sample both signs, zero and both limits; spacing is at most step_deg."""
    step_deg = float(step_deg)
    if not np.isfinite(step_deg) or not 0.0 < step_deg <= STEER_LIMIT_DEG:
        raise ValueError("Step must be finite and within (0, 30] deg.")
    suspension = SuspensionF() if suspension is None else suspension
    car = Car() if car is None else car
    positive = np.append(np.arange(0.0, STEER_LIMIT_DEG, step_deg), STEER_LIMIT_DEG)
    angles = np.concatenate((-positive[:0:-1], positive))
    return [calculate_wheel_pose(angle, suspension=suspension, car=car) for angle in angles]


def export_pose_csv(path, poses):
    """Export position [mm], Euler angles [deg] and local axes in reference XYZ."""
    with Path(path).open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        has_axis = all("kingpin_axis" in pose for pose in poses)
        writer.writerow(POSE_CSV_FIELDS + (KINGPIN_CSV_FIELDS if has_axis else []))
        for pose in poses:
            writer.writerow([
                pose["kingpin_rotation_deg"], *(pose["center_m"] * 1000.0),
                *pose["euler_xyz_deg"], *pose["rotation_matrix"].T.ravel(),
                *(pose["kingpin_axis"] if has_axis else []),
            ])


def load_pose_csv(path):
    """Read positions and complete wheel frames from CSV, ordered -30 to +30.

    The exported unit-axis columns supply orientation directly; no steering
    geometry or Euler-angle calculation is used to reconstruct the motion.
    """
    poses = []
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(POSE_CSV_FIELDS) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV is missing columns: {', '.join(sorted(missing))}")
        axis_columns = set(KINGPIN_CSV_FIELDS) & set(reader.fieldnames or [])
        if axis_columns and axis_columns != set(KINGPIN_CSV_FIELDS):
            raise ValueError("CSV must include all three kingpin-axis columns or none.")
        for line_number, row in enumerate(reader, start=2):
            try:
                values = np.array([float(row[field]) for field in POSE_CSV_FIELDS])
            except (TypeError, ValueError) as error:
                raise ValueError(f"CSV row {line_number} must contain numeric values.") from error
            if not np.all(np.isfinite(values)):
                raise ValueError(f"CSV row {line_number} contains a nonfinite value.")
            if abs(values[0]) > STEER_LIMIT_DEG:
                raise ValueError(f"CSV row {line_number}: kingpin angle must be within [-30, 30] deg.")
            matrix = values[7:].reshape(3, 3).T
            if (not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6, rtol=0.0)
                    or not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-6, rtol=0.0)):
                raise ValueError(f"CSV row {line_number}: wheel axes must form a right-handed unit frame.")
            poses.append({
                "kingpin_rotation_deg": float(values[0]),
                "center_m": values[1:4] / 1000.0,
                "euler_xyz_deg": values[4:7],
                "rotation_matrix": matrix,
            })
            if axis_columns:
                try:
                    axis = np.array([float(row[field]) for field in KINGPIN_CSV_FIELDS])
                except (TypeError, ValueError) as error:
                    raise ValueError(f"CSV row {line_number}: invalid kingpin axis.") from error
                if (not np.all(np.isfinite(axis))
                        or not np.isclose(np.linalg.norm(axis), 1.0, atol=1e-6, rtol=0.0)):
                    raise ValueError(f"CSV row {line_number}: kingpin axis must be a unit vector.")
                if poses[0].get("kingpin_axis") is not None and not np.allclose(
                        poses[0]["kingpin_axis"], axis, atol=1e-6, rtol=0.0):
                    raise ValueError("CSV kingpin axis must be fixed throughout the sweep.")
                poses[-1]["kingpin_axis"] = axis
    if not poses:
        raise ValueError("CSV contains no wheel poses.")
    return sorted(poses, key=lambda pose: pose["kingpin_rotation_deg"])


def create_pose_animation(csv_path, *, car=None, interval_ms=50.0):
    """Reload the saved CSV and create a repeating -30 to +30 animation."""
    poses = load_pose_csv(csv_path)
    figure = create_pose_figure(poses, car=car, animate=True, interval_ms=interval_ms)
    figure.suptitle(f"Wheel steering animation from CSV\n{Path(csv_path).name}")
    return figure


def create_pose_figure(poses, *, suspension=None, car=None, initial_angle=0.0,
                       animate=False, interval_ms=50.0):
    """Create a steering slider view, or animate the supplied CSV pose records."""
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.widgets import Button, Slider

    car = Car() if car is None else car
    if animate:
        initial_pose = poses[0]
    else:
        suspension = SuspensionF() if suspension is None else suspension
        initial_pose = calculate_wheel_pose(initial_angle, suspension=suspension, car=car)
    initial_angle = initial_pose["kingpin_rotation_deg"]
    angles = np.array([pose["kingpin_rotation_deg"] for pose in poses])
    euler = np.array([pose["euler_xyz_deg"] for pose in poses])
    centers = np.array([pose["center_m"] for pose in poses]) * 1000.0
    radius, width = car.free_radius, car.tire_width
    length = 1.35 * max(radius, width / 2.0)
    colors = ("tab:red", "tab:green", "tab:blue")

    fig = plt.figure(figsize=(13, 8))
    grid = fig.add_gridspec(2, 2, width_ratios=(1.25, 1.0))
    ax_frame = fig.add_subplot(grid[:, 0], projection="3d")
    ax_angles = fig.add_subplot(grid[0, 1])
    ax_position = fig.add_subplot(grid[1, 1])
    fig.subplots_adjust(left=0.04, right=0.97, top=0.87, bottom=0.19, hspace=0.4, wspace=0.25)
    if not animate:
        fig.suptitle(
            "sus_3Dwe_v3: Wheel-center coordinate frame\n"
            f"KPI = {np.rad2deg(suspension.kpi):.2f} deg, "
            f"caster = {suspension.caster_sv_deg:.2f} deg; "
            f"trail = {suspension.mechanical_trail_sv * 1000:.1f} mm, "
            f"scrub = {suspension.scrub_radius * 1000:.1f} mm",
        )

    for index, name in enumerate("XYZ"):
        reference = np.eye(3)[index] * length
        ax_frame.plot(*np.column_stack((-reference, reference)), "--", color="gray", alpha=0.6)
        ax_frame.text(*(-1.08 * reference), f"{name}0", color="gray")
        ax_angles.plot(angles, euler[:, index], color=colors[index], label=f"R{name.lower()}")
        ax_position.plot(angles, centers[:, index], color=colors[index],
                         linestyle=("-", "--", ":")[index], label=name)
    axis = initial_pose.get("kingpin_axis")
    if axis is not None:
        axis_line = np.outer([0.0, 2.0 * length], axis)
        ax_frame.plot(*axis_line.T, color="black", linewidth=1.5)
    ax_frame.plot(*(centers / 1000.0).T, color="tab:purple", linestyle=":", linewidth=1.5)
    ax_frame.scatter(0.0, 0.0, 0.0, color="black", s=30)
    ax_frame.text(0.0, 0.0, 0.0, " O", color="black")
    limit = length * 1.4 + np.max(np.abs(centers[:, :2])) / 1000.0
    tire_extent = radius + width / 2.0
    z_min = min(-0.1 * radius, np.min(centers[:, 2]) / 1000.0 - tire_extent)
    z_max = max(2.0 * length, np.max(centers[:, 2]) / 1000.0 + tire_extent)
    ax_frame.plot([-limit, limit, limit, -limit, -limit],
                  [-limit, -limit, limit, limit, -limit], np.zeros(5),
                  color="gray", alpha=0.45, linewidth=0.8)
    ax_frame.set(xlim=(-limit, limit), ylim=(-limit, limit), zlim=(z_min, z_max),
                 xlabel="X [m]", ylabel="Y [m]", zlabel="Z [m]",
                 title="Kingpin-ground origin O / axes at wheel center")
    ax_frame.set_box_aspect((2.0 * limit, 2.0 * limit, z_max - z_min))
    ax_frame.view_init(elev=23, azim=-55)
    fig.legend(handles=[
        Line2D([], [], color="gray", linestyle="--", label="Reference frame"),
        *([] if axis is None else [Line2D([], [], color="black", label="Kingpin through O")]),
        Line2D([], [], color="tab:purple", linestyle=":", label="Wheel-center path"),
        *[Line2D([], [], color=color, label=f"Wheel {name}")
          for name, color in zip("xyz", colors)],
    ], loc="lower left", bbox_to_anchor=(0.055, 0.14), ncol=3, fontsize="small")
    status = ax_frame.text2D(0.02, 0.98, "", transform=ax_frame.transAxes, va="top", fontsize=10)

    ax_angles.set(title="Fixed XYZ Euler angles: R = Rz @ Ry @ Rx", ylabel="Rotation [deg]")
    position_limit = max(1.0, np.max(np.abs(centers)) * 1.1)
    ax_position.set(title="Wheel center relative to kingpin-ground origin",
                    ylabel="Position [mm]", ylim=(-position_limit, position_limit))
    markers = []
    for axes in (ax_angles, ax_position):
        axes.set_xlabel("Kingpin rotation [deg]")
        axes.set_xlim(-STEER_LIMIT_DEG, STEER_LIMIT_DEG)
        axes.grid(True, alpha=0.4)
        axes.legend(ncol=3)
        markers.append(axes.axvline(initial_angle, color="black", linestyle="--", alpha=0.6))

    theta, lateral = np.meshgrid(np.linspace(0.0, 2.0 * np.pi, 50),
                                 np.linspace(-width / 2.0, width / 2.0, 12))
    tire = np.stack((radius * np.sin(theta), lateral, radius * np.cos(theta)), axis=-1)
    moving_artists = []

    def draw_pose(pose):
        steer_deg = pose["kingpin_rotation_deg"]
        matrix = pose["rotation_matrix"]
        center = pose["center_m"]
        for artist in moving_artists:
            artist.remove()
        moving_artists.clear()
        rotated = tire @ matrix.T + center
        moving_artists.append(ax_frame.plot_surface(
            rotated[..., 0], rotated[..., 1], rotated[..., 2],
            color="orange", alpha=0.25, linewidth=0,
        ))
        for index, (name, color) in enumerate(zip("xyz", colors)):
            direction = matrix[:, index] * length
            moving_artists.append(ax_frame.quiver(
                *center, *direction, color=color, linewidth=2.5, arrow_length_ratio=0.12,
            ))
            moving_artists.append(ax_frame.text(*(center + 1.08 * direction), f"{name}w", color=color))
        moving_artists.append(ax_frame.scatter(*center, color="black", s=30))
        rx, ry, rz = pose["euler_xyz_deg"]
        cx, cy, cz = center * 1000.0
        status.set_text(
            f"Kingpin = {steer_deg:+.2f} deg\n"
            f"Center [mm] = ({cx:.3f}, {cy:.3f}, {cz:.3f})\n"
            f"Rx = {rx:+.3f}, Ry = {ry:+.3f}, Rz = {rz:+.3f} deg"
        )
        for marker in markers:
            marker.set_xdata([steer_deg, steer_deg])
        return [*moving_artists, status, *markers]

    if animate:
        from matplotlib.animation import FuncAnimation

        animation = FuncAnimation(
            fig, draw_pose, frames=poses, init_func=lambda: draw_pose(poses[0]),
            interval=interval_ms, repeat=True, blit=False, cache_frame_data=False,
        )
        button = Button(fig.add_axes((0.44, 0.08, 0.12, 0.045)), "Pause")
        paused = False

        def toggle_pause(event):
            nonlocal paused
            paused = not paused
            if paused:
                animation.pause()
            else:
                animation.resume()
            button.label.set_text("Resume" if paused else "Pause")
            fig.canvas.draw_idle()

        button.on_clicked(toggle_pause)
        # Keep animation and controls alive while the GUI timer is running.
        fig.wheel_animation = animation
        fig.animation_button = button
    else:
        def update(steer_deg):
            draw_pose(calculate_wheel_pose(steer_deg, suspension=suspension, car=car))
            fig.canvas.draw_idle()

        slider = Slider(fig.add_axes((0.22, 0.09, 0.57, 0.035)), "Kingpin [deg]",
                        -STEER_LIMIT_DEG, STEER_LIMIT_DEG, valinit=initial_angle)
        slider.on_changed(update)
        fig.steering_slider = slider
    fig.text(0.5, 0.025,
             "Origin: static kingpin/ground intersection (Z = 0). Tire axle: local +Y. "
             + ("Animation: saved CSV poses, -30 to +30 deg, repeating." if animate else
                "Input: rotation about kingpin. All angles use the right-hand rule."),
             ha="center", fontsize=9)
    draw_pose(initial_pose)
    return fig


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--angle", type=float, default=0.0,
                        help="Initial displayed kingpin rotation [-30, 30] deg, default 0.")
    parser.add_argument("--step", type=float, default=0.5,
                        help="Maximum sweep spacing [deg], default 0.5.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--interval", type=float, default=50.0,
                        help="Animation frame interval [ms], default 50.")
    parser.add_argument("--no-show", action="store_true",
                        help="Save CSV/PNG without prompting or opening a window.")
    args = parser.parse_args(argv)
    suspension = SuspensionF()
    car = Car()
    if not np.isfinite(args.interval) or args.interval <= 0.0:
        parser.error("Animation interval must be finite and positive.")
    try:
        selected = calculate_wheel_pose(args.angle, suspension=suspension, car=car)
        poses = sweep_wheel_poses(args.step, suspension=suspension, car=car)
    except ValueError as error:
        parser.error(str(error))

    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "wheel_center_pose.csv"
    png_path = args.output_dir / "wheel_center_pose.png"
    export_pose_csv(csv_path, poses)
    figure = create_pose_figure(poses, suspension=suspension, car=car, initial_angle=args.angle)
    figure.savefig(png_path, dpi=160)
    plt.close(figure)

    print("Input = kingpin rotation, matching sus_3Dwe_v3; not ground-plane wheel heading.")
    print("Fixed origin = static kingpin/ground intersection (z=0); center = R @ initial_center.")
    print(f"Trail = {suspension.mechanical_trail_sv * 1000:.3f} mm (+X); "
          f"scrub = {suspension.scrub_radius * 1000:.3f} mm (-Y), measured at z=0.")
    print("Initial center [mm]:", np.round(selected["initial_center_m"] * 1000.0, 6))
    print("Fixed XYZ Euler: R = Rz @ Ry @ Rx.\n")
    print(f"{'Kingpin [deg]':>14} {'X [mm]':>10} {'Y [mm]':>10} {'Z [mm]':>10}"
          f" {'Rx [deg]':>11} {'Ry [deg]':>11} {'Rz [deg]':>11}")
    for angle in sorted({-30.0, -15.0, 0.0, 15.0, 30.0, args.angle}):
        pose = calculate_wheel_pose(angle, suspension=suspension, car=car)
        values = [angle, *(pose["center_m"] * 1000.0), *pose["euler_xyz_deg"]]
        print(f"{values[0]:14.3f}" + "".join(f"{value:11.4f}" for value in values[1:]))
    print(f"\nWheel axes at {args.angle:+.3f} deg (columns = wheel x/y/z in reference XYZ):")
    print(np.array2string(selected["rotation_matrix"], precision=6, suppress_small=True))
    print(f"\nCSV ({len(poses)} samples): {csv_path.resolve()}")
    print(f"PNG (initial pose): {png_path.resolve()}")
    if not args.no_show:
        try:
            answer = input("\nEnter Y to read the CSV and play the -30 to +30 deg wheel animation; "
                           "any other input exits: ")
        except EOFError:
            answer = ""
        if answer.strip().lower() == "y":
            try:
                animation_figure = create_pose_animation(csv_path, interval_ms=args.interval)
            except (OSError, ValueError) as error:
                parser.error(f"Cannot animate CSV: {error}")
            print(f"Playing CSV: {csv_path.resolve()} (Pause/Resume button; close window to exit)")
            plt.show()
            plt.close(animation_figure)
    return poses


if __name__ == "__main__":
    main()
