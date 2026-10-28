"""Calculate four-wheel camber changes and ground camber from data_signals.csv.

zs - zu is extension [mm]; wheel compression passed to FV is zu - zs.
Camber gain here means finite camber change [deg] relative to the chassis.
CSV roll is suspension-only; add tire roll = (zu_left - zu_right) / track [rad].
Positive roll raises the left side: subtract roll on the left, add on the right.
Lateral acceleration ay is positive left: FR is outside for ay > 0, FL for ay < 0.
"""

import argparse
import csv
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from suspension_geo.car import Car, SuspensionF, SuspensionR
from suspension_geo.combined_camber import calculate_combined_camber


BASE_DIR = Path(__file__).resolve().parent
WHEELS = ("FL", "FR", "RL", "RR")
STATIC_CAMBER_DEG = (0.0, 0.0, 0.0, 0.0)  # 可修改，順序 FL、FR、RL、RR。
WHEEL_QUANTITIES = (
    "relative_displacement_{wheel}_mm",
    "wheel_travel_{wheel}_mm",
    "static_camber_{wheel}_deg",
    "fv_camber_gain_{wheel}_deg",
    "steering_camber_gain_{wheel}_deg",
    "camber_gain_{wheel}_deg",
    "roll_camber_{wheel}_deg",
    "camber_{wheel}_deg",
)
DERIVED_COLUMNS = [template.format(wheel=wheel) for wheel in WHEELS for template in WHEEL_QUANTITIES]
DERIVED_COLUMNS += [
    "fv_contact_line_front_deg", "fv_contact_line_rear_deg",
    "tire_roll_front_deg", "tire_roll_rear_deg",
    "total_roll_front_deg", "total_roll_rear_deg",
    "front_solver_ok", "rear_solver_ok", "front_error", "rear_error",
]
FRONT_COMPARISON_COLUMNS = (
    "front_outside_wheel", "front_inside_wheel",
    "camber_front_outside_deg", "camber_front_inside_deg",
    "camber_front_outside_minus_inside_deg",
)
DERIVED_COLUMNS += list(FRONT_COMPARISON_COLUMNS)
REQUIRED_COLUMNS = (
    "time_s", *(f"zs_{wheel}_mm" for wheel in WHEELS),
    *(f"zu_{wheel}_mm" for wheel in WHEELS),
    "steer_FL_deg", "steer_FR_deg", "roll_front_deg", "roll_rear_deg",
)


def _number(row, column):
    try:
        value = float(row[column])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"Missing or nonnumeric {column}.") from error
    if not np.isfinite(value):
        raise ValueError(f"Nonfinite {column}.")
    return value


def front_camber_comparison(row):
    """Select ground cambers by ay alone; zero/missing ay has no inside/outside.

    Body axes are X forward, Y left, Z up. Acceleration points into the turn,
    so positive ay makes FL inside and FR outside. Preserve each tire's camber
    sign (top outward positive), including when its role changes between turns.
    """
    result = {column: float("nan") for column in FRONT_COMPARISON_COLUMNS}
    result.update(front_outside_wheel="", front_inside_wheel="")
    try:
        ay = _number(row, "ay_m_s2")
    except ValueError:
        return result
    if ay == 0.0:
        return result
    outside, inside = ("FR", "FL") if ay > 0.0 else ("FL", "FR")
    result.update(front_outside_wheel=outside, front_inside_wheel=inside)
    for role, wheel in (("outside", outside), ("inside", inside)):
        try:
            result[f"camber_front_{role}_deg"] = _number(row, f"camber_{wheel}_deg")
        except ValueError:
            pass
    result["camber_front_outside_minus_inside_deg"] = (
        result["camber_front_outside_deg"] - result["camber_front_inside_deg"]
    )
    return result


class CamberSignalAnalyzer:
    """Reuse the additive axle model while keeping roll out of body-relative gain.

    Each row is independent of its predecessor. Repeated inputs reuse cached
    results; fresh axle geometry is built for every new combination of inputs.
    Front and rear use their own suspension parameters, loaded tire radii and
    track widths (Car.tf = 1.250 m and Car.tr = 1.240 m by default).
    """

    def __init__(self, static_camber_deg=STATIC_CAMBER_DEG, *, car=None,
                 front_suspension=None, rear_suspension=None):
        self.static_camber = np.asarray(static_camber_deg, dtype=float)
        if (self.static_camber.shape != (4,) or not np.isfinite(self.static_camber).all()
                or np.any(np.abs(self.static_camber) >= 90.0)):
            raise ValueError("Provide four finite static camber angles between -90 and 90 deg.")
        self.car = Car() if car is None else car
        self.track_widths_mm = {"front": float(self.car.tf) * 1000.0,
                                "rear": float(self.car.tr) * 1000.0}
        if any(not np.isfinite(track) or track <= 0.0 for track in self.track_widths_mm.values()):
            raise ValueError("Front and rear track widths must be finite and positive [m].")
        self.suspensions = {
            "front": SuspensionF() if front_suspension is None else front_suspension,
            "rear": SuspensionR() if rear_suspension is None else rear_suspension,
        }
        self._cache = {"front": {}, "rear": {}}

    def analyze_row(self, row):
        """Return derived fields; an invalid axle gets NaNs and an explicit error."""
        result = {column: float("nan") for column in DERIVED_COLUMNS}
        for axle, wheel_indices in (("front", (0, 1)), ("rear", (2, 3))):
            result[f"{axle}_solver_ok"] = False
            result[f"{axle}_error"] = ""
            try:
                relative = []
                for index in wheel_indices:
                    wheel = WHEELS[index]
                    displacement = _number(row, f"zs_{wheel}_mm") - _number(row, f"zu_{wheel}_mm")
                    relative.append(displacement)
                    result[f"relative_displacement_{wheel}_mm"] = displacement
                    result[f"wheel_travel_{wheel}_mm"] = -displacement
                    result[f"static_camber_{wheel}_deg"] = float(self.static_camber[index])
                if axle == "front":
                    steer = [_number(row, "steer_FL_deg"), _number(row, "steer_FR_deg")]
                else:
                    # This analysis uses front steering only. Reject unsupported
                    # rear steering instead of silently discarding a real signal.
                    steer = [0.0, 0.0]
                    for wheel in ("RL", "RR"):
                        if f"steer_{wheel}_deg" in row and _number(row, f"steer_{wheel}_deg") != 0.0:
                            raise ValueError("Nonzero rear steering is not supported by this front-steering analysis.")
                static = tuple(float(self.static_camber[index]) for index in wheel_indices)
                cache_key = (*steer, *relative, *static)
                if cache_key not in self._cache[axle]:
                    self._cache[axle][cache_key] = calculate_combined_camber(
                        *steer, -relative[0], -relative[1], car=self.car,
                        suspension=self.suspensions[axle],
                        static_camber_left_deg=static[0], static_camber_right_deg=static[1],
                    )
                combined = self._cache[axle][cache_key]
                contact_line = combined["roll_deg"]
                result[f"fv_contact_line_{axle}_deg"] = contact_line
                for index, side, sign in zip(wheel_indices, ("left", "right"), (1.0, -1.0)):
                    wheel = WHEELS[index]
                    # FV camber is measured against the moving contact line.
                    # Undo that reference: left +line angle; right -line angle.
                    # Static symmetric geometry has a zero contact-line angle.
                    fv_body = combined[f"fv_camber_change_{side}_deg"] + sign * contact_line
                    steering_change = combined[f"steering_camber_change_{side}_deg"]
                    result[f"fv_camber_gain_{wheel}_deg"] = fv_body
                    result[f"steering_camber_gain_{wheel}_deg"] = steering_change
                    result[f"camber_gain_{wheel}_deg"] = fv_body + steering_change

                # Evaluate roll after gain: missing roll still leaves the
                # independently calculable body-relative changes available.
                left_wheel, right_wheel = (WHEELS[index] for index in wheel_indices)
                delta_zu_mm = _number(row, f"zu_{left_wheel}_mm") - _number(row, f"zu_{right_wheel}_mm")
                # Use the requested small-angle ratio [rad], then convert to deg.
                # Calculate per row: equal suspension travel can have different zu.
                tire_roll = float(np.rad2deg(delta_zu_mm / self.track_widths_mm[axle]))
                result[f"tire_roll_{axle}_deg"] = tire_roll
                suspension_roll = _number(row, f"roll_{axle}_deg")
                if abs(suspension_roll) >= 90.0:
                    raise ValueError(f"roll_{axle}_deg must be between -90 and 90 deg.")
                roll = suspension_roll + tire_roll
                if not np.isfinite(roll) or abs(roll) >= 90.0:
                    raise ValueError(f"total_roll_{axle}_deg must be finite and between -90 and 90 deg.")
                result[f"total_roll_{axle}_deg"] = roll
                for index, sign in zip(wheel_indices, (1.0, -1.0)):
                    wheel = WHEELS[index]
                    result[f"roll_camber_{wheel}_deg"] = -sign * roll
                    result[f"camber_{wheel}_deg"] = (
                        self.static_camber[index] + result[f"camber_gain_{wheel}_deg"] - sign * roll
                    )
                result[f"{axle}_solver_ok"] = True
            except (ValueError, RuntimeError) as error:
                result[f"{axle}_error"] = str(error)
        result.update(front_camber_comparison({**row, **result}))
        return result


def save_plot(rows, output_path):
    """Render displacement, suspension/tire/total roll and calculated camber."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    fig = Figure(figsize=(13, 8.5), constrained_layout=True)
    FigureCanvasAgg(fig)
    axes = fig.subplots(2, 2, sharex=True)
    time = np.array([float(row["time_s"]) for row in rows])
    def roll_values(column):
        values = []
        for row in rows:
            try:
                values.append(_number(row, column))
            except ValueError:
                values.append(float("nan"))
        return values
    colors = {"FL": "tab:blue", "FR": "tab:orange", "RL": "tab:green", "RR": "tab:red"}
    for axis, template, title, unit in (
        (axes[0, 0], "relative_displacement_{wheel}_mm", "Sprung minus unsprung displacement", "mm"),
        (axes[0, 1], "camber_gain_{wheel}_deg", "Camber change relative to chassis", "deg"),
        (axes[1, 1], "camber_{wheel}_deg", "Camber relative to level ground", "deg"),
    ):
        for wheel in WHEELS:
            axis.plot(time, [row[template.format(wheel=wheel)] for row in rows],
                      label=wheel, color=colors[wheel], linewidth=1.1)
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.legend(ncol=4, fontsize=8)
    for axle, color in (("front", "tab:purple"), ("rear", "black")):
        for prefix, label, style in (("roll", "suspension", "--"),
                                     ("tire_roll", "tire", ":"),
                                     ("total_roll", "total", "-")):
            axes[1, 0].plot(time, roll_values(f"{prefix}_{axle}_deg"),
                            label=f"{axle.title()} {label}", color=color, linestyle=style,
                            linewidth=1.4 if label == "total" else 0.9)
    axes[1, 0].set_title("Roll = suspension + tire (positive: left side up)")
    axes[1, 0].set_ylabel("deg")
    axes[1, 0].legend(fontsize=8, ncol=2)
    for axis in axes.ravel():
        axis.grid(True, alpha=0.3)
        axis.axhline(0.0, color="0.5", linewidth=0.6)
    for axis in axes[1, :]:
        axis.set_xlabel("Time [s]")
    fig.suptitle("Four-wheel camber from suspension displacement, front steering and body roll")
    fig.savefig(output_path, dpi=160)


def front_comparison_summary(rows):
    """Compare the same paired samples, excluding straight/invalid observations."""
    outside = np.array([float(row["camber_front_outside_deg"]) for row in rows])
    inside = np.array([float(row["camber_front_inside_deg"]) for row in rows])
    paired = np.isfinite(outside) & np.isfinite(inside)
    summary = {
        "turning_rows": sum(bool(row["front_outside_wheel"]) for row in rows),
        "paired_rows": int(paired.sum()),
        "unclassified_rows": sum(not row["front_outside_wheel"] for row in rows),
    }
    for role, values in (("outside", outside), ("inside", inside),
                         ("outside_minus_inside", outside - inside)):
        valid = values[paired]
        summary[role] = None if not len(valid) else {
            "mean_deg": float(valid.mean()),
            "min_deg": float(valid.min()), "max_deg": float(valid.max()),
        }
    return summary


def save_front_comparison_plot(rows, output_path):
    """Plot ground camber, outside-minus-inside difference and signed ay."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    fig = Figure(figsize=(14, 9.5), constrained_layout=True)
    FigureCanvasAgg(fig)
    axes = fig.subplots(3, 1, sharex=True, height_ratios=(2.2, 1.2, 1.0))
    time = np.array([float(row["time_s"]) for row in rows])
    outside_wheel = np.array([row["front_outside_wheel"] for row in rows])
    # Break lines when the selected physical tire changes, even if no sampled
    # ay is exactly zero between opposite turns.
    changes = np.flatnonzero(outside_wheel[1:] != outside_wheel[:-1]) + 1
    plot_time = np.insert(time, changes, (time[changes - 1] + time[changes]) / 2.0)
    for role, label, color in (("outside", "Outside front", "#c44e3b"),
                               ("inside", "Inside front", "#2878b5")):
        values = np.array([float(row[f"camber_front_{role}_deg"]) for row in rows])
        axes[0].plot(plot_time, np.insert(values, changes, np.nan),
                     label=label, color=color, linewidth=1.25)
    axes[0].set_title("Camber relative to level ground (positive: tire top outward)")
    axes[0].set_ylabel("Camber [deg]")
    axes[0].legend(loc="upper left", ncol=2)
    summary = front_comparison_summary(rows)
    if summary["paired_rows"]:
        stats = [f"Paired turning samples: {summary['paired_rows']} / {len(rows)}"]
        for role, label in (("outside", "Outside"), ("inside", "Inside")):
            values = summary[role]
            stats.append(f"{label}: mean {values['mean_deg']:+.3f} deg; "
                         f"range [{values['min_deg']:+.3f}, {values['max_deg']:+.3f}]")
        axes[0].set_title(
            "Camber relative to level ground (positive: tire top outward)\n"
            + " | ".join(stats[1:]), fontsize=11,
        )
    else:
        axes[0].text(0.5, 0.5, "No paired turning samples", transform=axes[0].transAxes,
                     ha="center", va="center")
    difference = np.array([float(row["camber_front_outside_minus_inside_deg"]) for row in rows])
    axes[1].plot(plot_time, np.insert(difference, changes, np.nan), color="#7851a9", linewidth=1.1)
    axes[1].set_title("Outside minus inside camber (negative: outside is more negative)")
    axes[1].set_ylabel("Difference [deg]")
    ay = []
    for row in rows:
        try:
            ay.append(_number(row, "ay_m_s2"))
        except ValueError:
            ay.append(float("nan"))
    ay = np.asarray(ay)
    axes[2].plot(time, ay, color="0.3", linewidth=0.8)
    axes[2].fill_between(time, 0.0, ay, where=ay > 0.0, color="#2b9d75", alpha=0.35,
                         label="ay > 0: left turn; outside FR / inside FL")
    axes[2].fill_between(time, 0.0, ay, where=ay < 0.0, color="#df9c36", alpha=0.4,
                         label="ay < 0: right turn; outside FL / inside FR")
    axes[2].set_title("Lateral acceleration: positive left; zero / invalid ay excluded above")
    axes[2].set_ylabel("ay [m/s²]")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend(loc="lower right", fontsize=8, ncol=2)
    for axis in axes:
        axis.axhline(0.0, color="0.4", linewidth=0.7)
        axis.grid(True, alpha=0.25)
        axis.margins(x=0.01)
    fig.suptitle("Front tire camber comparison by lateral acceleration direction\n"
                 f"Paired turning samples: {summary['paired_rows']} / {len(rows)}; "
                 "statistics are sample means", fontsize=14)
    fig.savefig(output_path, dpi=160)


def save_vehicle_comparison_plot(rows, output_path):
    """Compare speed, three-axis acceleration, front steering and ground camber."""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    def values(column):
        result = []
        for row in rows:
            try:
                result.append(_number(row, column))
            except ValueError:
                result.append(float("nan"))
        return np.asarray(result)

    time = values("time_s")
    fig = Figure(figsize=(15, 12), constrained_layout=True)
    FigureCanvasAgg(fig)
    axes = fig.subplots(4, 1, sharex=True, height_ratios=(1.0, 1.25, 1.0, 1.5))

    axes[0].plot(time, values("speed_km_h"), color="#254f78", linewidth=1.3)
    axes[0].set_title("Vehicle speed", loc="left", fontsize=12)
    axes[0].set_ylabel("Speed [km/h]")

    for column, label, color, style in (
        ("ax_m_s2", "ax: longitudinal", "#8c6bb1", "-"),
        ("ay_m_s2", "ay: lateral (+ left)", "#16866d", "-"),
        ("az_m_s2", "az: vertical", "#777777", "--"),
    ):
        axes[1].plot(time, values(column), label=label, color=color, linestyle=style,
                     linewidth=1.15 if column == "ay_m_s2" else 0.9)
    axes[1].set_title("Vehicle acceleration", loc="left", fontsize=12)
    axes[1].set_ylabel("Acceleration [m/s²]")
    axes[1].legend(loc="upper right", ncol=3, fontsize=9)

    for wheel, color, style in (("FL", "#2878b5", "-"), ("FR", "#d68929", "--")):
        axes[2].plot(time, values(f"steer_{wheel}_deg"), label=wheel,
                     color=color, linestyle=style, linewidth=1.2)
    axes[2].set_title("Front wheel steering angles (+ left turn)", loc="left", fontsize=12)
    axes[2].set_ylabel("Steering [deg]")
    axes[2].legend(loc="upper right", ncol=2, fontsize=9)

    outside_wheel = np.array([row["front_outside_wheel"] for row in rows])
    changes = np.flatnonzero(outside_wheel[1:] != outside_wheel[:-1]) + 1
    plot_time = np.insert(time, changes, (time[changes - 1] + time[changes]) / 2.0)
    for role, label, color in (("outside", "Outside front", "#c44e3b"),
                               ("inside", "Inside front", "#2878b5")):
        camber = np.insert(values(f"camber_front_{role}_deg"), changes, np.nan)
        axes[3].plot(plot_time, camber, label=label, color=color, linewidth=1.3)
    axes[3].set_title("Front camber relative to level ground (+ tire top outward)",
                      loc="left", fontsize=12)
    axes[3].set_ylabel("Camber [deg]")
    axes[3].set_xlabel("Time [s]")
    axes[3].legend(loc="upper right", ncol=2, fontsize=9)

    for axis in axes:
        axis.axhline(0.0, color="0.5", linewidth=0.65)
        axis.grid(True, alpha=0.23)
        axis.margins(x=0.01)
        axis.tick_params(labelbottom=True)
    fig.suptitle("Speed, acceleration, steering and front tire camber\n"
                 "ay > 0: outside FR / inside FL   |   ay < 0: outside FL / inside FR\n"
                 "Zero / invalid ay excluded from camber; lines break when tire roles change",
                 fontsize=13)
    fig.savefig(output_path, dpi=180)


def analyze_csv(input_path, output_path, *, figure_path=None, front_comparison_figure_path=None,
                vehicle_comparison_figure_path=None, static_camber_deg=STATIC_CAMBER_DEG):
    """Keep all source rows/columns and append four-wheel angle changes and camber."""
    input_path, output_path = Path(input_path).resolve(), Path(output_path).resolve()
    figure_path = None if figure_path is None else Path(figure_path).resolve()
    comparison_path = (None if front_comparison_figure_path is None
                       else Path(front_comparison_figure_path).resolve())
    vehicle_path = (None if vehicle_comparison_figure_path is None
                    else Path(vehicle_comparison_figure_path).resolve())
    paths = [path for path in (input_path, output_path, figure_path, comparison_path, vehicle_path)
             if path is not None]
    if len(set(paths)) != len(paths):
        raise ValueError("Input CSV, output CSV and figures must use different paths.")
    analyzer = CamberSignalAnalyzer(static_camber_deg)
    with input_path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.DictReader(source)
        columns = reader.fieldnames or []
        required = set(REQUIRED_COLUMNS)
        if comparison_path is not None:
            required.add("ay_m_s2")
        if vehicle_path is not None:
            required.update(("speed_km_h", "ax_m_s2", "ay_m_s2", "az_m_s2"))
        missing = sorted(required - set(columns))
        if missing:
            raise ValueError(f"Missing required CSV columns: {', '.join(missing)}")
        if len(set(columns)) != len(columns):
            raise ValueError("Duplicate CSV column names are not supported.")
        conflicts = sorted(set(DERIVED_COLUMNS) & set(columns))
        if conflicts:
            raise ValueError(f"Input already contains derived columns: {', '.join(conflicts)}")
        rows = list(reader)
    if not rows:
        raise ValueError("The input CSV contains no data rows.")
    previous_time = -np.inf
    for index, row in enumerate(rows, start=2):
        try:
            time = _number(row, "time_s")
        except ValueError as error:
            raise ValueError(f"CSV row {index}: {error}") from error
        if time < previous_time:
            raise ValueError(f"CSV row {index}: time_s must be nondecreasing.")
        previous_time = time
        row.update(analyzer.analyze_row(row))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8-sig", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=[*columns, *DERIVED_COLUMNS])
        writer.writeheader()
        writer.writerows(rows)
    if figure_path is not None:
        figure_path.parent.mkdir(parents=True, exist_ok=True)
        save_plot(rows, figure_path)
    if comparison_path is not None:
        comparison_path.parent.mkdir(parents=True, exist_ok=True)
        save_front_comparison_plot(rows, comparison_path)
    if vehicle_path is not None:
        vehicle_path.parent.mkdir(parents=True, exist_ok=True)
        save_vehicle_comparison_plot(rows, vehicle_path)

    summary = {
        "rows": len(rows),
        "front_failed_rows": sum(not row["front_solver_ok"] for row in rows),
        "rear_failed_rows": sum(not row["rear_solver_ok"] for row in rows),
        "output_path": str(output_path), "figure_path": None if figure_path is None else str(figure_path),
        "front_comparison_figure_path": None if comparison_path is None else str(comparison_path),
        "vehicle_comparison_figure_path": None if vehicle_path is None else str(vehicle_path),
        "front_comparison": front_comparison_summary(rows),
        "ranges": {},
    }
    for wheel in WHEELS:
        for quantity in ("camber_gain", "camber"):
            column = f"{quantity}_{wheel}_deg"
            values = np.array([row[column] for row in rows])
            finite = values[np.isfinite(values)]
            summary["ranges"][column] = None if len(finite) == 0 else [float(finite.min()), float(finite.max())]
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=BASE_DIR / "data_signals.csv")
    parser.add_argument("--output", type=Path, default=BASE_DIR / "data_signals_camber.csv")
    parser.add_argument("--figure", type=Path, default=BASE_DIR / "data_signals_camber.png")
    parser.add_argument("--front-comparison-figure", type=Path,
                        default=BASE_DIR / "data_signals_camber_front_inside_outside.png",
                        help="Front inside/outside ground camber figure; ay is positive left.")
    parser.add_argument("--vehicle-comparison-figure", type=Path,
                        default=BASE_DIR / "data_signals_camber_vehicle_comparison.png",
                        help="Shared-time speed, acceleration, front steering and inside/outside camber figure.")
    parser.add_argument("--no-plot", action="store_true", help="Only write the calculated CSV.")
    parser.add_argument("--static-camber", nargs=4, type=float, default=STATIC_CAMBER_DEG,
                        metavar=("FL", "FR", "RL", "RR"), help="Initial cambers [deg], default all zero.")
    args = parser.parse_args(argv)
    print(f"Reading {args.input}", flush=True)
    try:
        summary = analyze_csv(args.input, args.output, figure_path=None if args.no_plot else args.figure,
                              front_comparison_figure_path=None if args.no_plot else args.front_comparison_figure,
                              vehicle_comparison_figure_path=None if args.no_plot else args.vehicle_comparison_figure,
                              static_camber_deg=args.static_camber)
    except (ValueError, OSError) as error:
        parser.error(str(error))
    print(f"Rows: {summary['rows']}; failed front: {summary['front_failed_rows']}; failed rear: {summary['rear_failed_rows']}")
    print(f"Initial camber FL/FR/RL/RR [deg]: {tuple(args.static_camber)}")
    print("Gain columns are body-relative angle changes [deg], not derivatives.")
    print("Tire roll [deg] = rad2deg((zu_left_mm - zu_right_mm) / track_mm).")
    print("Total roll = CSV suspension roll + tire roll (Car.tf/tr: front/rear tracks).")
    print("Camber = initial + gain - total roll (left); initial + gain + total roll (right).")
    for wheel in WHEELS:
        print(f"{wheel}: gain range {summary['ranges'][f'camber_gain_{wheel}_deg']}; "
              f"ground camber range {summary['ranges'][f'camber_{wheel}_deg']} deg")
    print(f"CSV: {summary['output_path']}")
    if summary["figure_path"]:
        print(f"Figure: {summary['figure_path']}")
    if summary["front_comparison_figure_path"]:
        print(f"Front comparison figure: {summary['front_comparison_figure_path']}")
    if summary["vehicle_comparison_figure_path"]:
        print(f"Vehicle comparison figure: {summary['vehicle_comparison_figure_path']}")
    comparison = summary["front_comparison"]
    print(f"Front inside/outside: {comparison['paired_rows']} paired samples; "
          f"{comparison['unclassified_rows']} zero/invalid ay samples excluded.")
    print("ay > 0: outside FR / inside FL; ay < 0: outside FL / inside FR.")
    for role in ("outside", "inside", "outside_minus_inside"):
        print(f"Front {role} camber [deg]: {comparison[role]}")
    return 1 if summary["front_failed_rows"] or summary["rear_failed_rows"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
