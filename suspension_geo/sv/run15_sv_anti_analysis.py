"""Evaluate side-view anti geometry through the selected Run 15 lap.

The selected CSV contains vehicle pitch and sprung corner heights, but no
unsprung wheel positions. This analysis therefore uses the front/rear mean
``zs`` change from the first sample as the side-view Al-height input, matching
the lap-data pose workflow used by the front-view analysis. Results should be
read as a geometry response to those sprung-height inputs.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from suspension_geo import Geometric_Analysis as ga
from suspension_geo.sv.sus_sv_v4 import SideSuspension, car, suspension_f, suspension_r


HERE = Path(__file__).resolve().parent
DEFAULT_INPUT = HERE.parent / "data" / "data_run15_selected.csv"
DEFAULT_OUTPUT = HERE / "data_run15_sv_anti.csv"
DEFAULT_FIGURE = HERE / "data_run15_sv_anti.png"


def build_axles() -> tuple[SideSuspension, SideSuspension]:
    front_x = suspension_f.wheel_center_x_sv - car.l
    rear_x = suspension_f.wheel_center_x_sv
    front = SideSuspension(
        "Front", suspension_f, axle_index=0, direction_sign=1.0,
        center_x=front_x, color="tab:blue",
    )
    rear = SideSuspension(
        "Rear", suspension_r, axle_index=1, direction_sign=-1.0,
        center_x=rear_x, color="tab:orange",
    )
    return front, rear


def anti_percent(
    origin: np.ndarray,
    ic: np.ndarray,
    direction_sign: float,
    load_share: float,
) -> float:
    """Convert the current force-line angle into an anti percentage."""
    dx = direction_sign * (ic[0] - origin[0])
    dy = ic[1] - origin[1]
    if abs(dx) < 1e-9:
        return float(np.copysign(np.inf, dy))
    tangent = dy / dx
    return float(100.0 * tangent * load_share * car.l / car.h_cog)


def analyze(input_path: Path, rear_drive_share: float = 1.0) -> pd.DataFrame:
    data = pd.read_csv(input_path)
    required = [
        "data.t", "data.theta[2]",
        "data.zs[0]", "data.zs[1]", "data.zs[2]", "data.zs[3]",
    ]
    missing = [column for column in required if column not in data.columns]
    if missing:
        raise ValueError(f"Input CSV is missing required columns: {', '.join(missing)}")

    time_s = data["data.t"].to_numpy(dtype=float)
    pitch_deg = np.degrees(data["data.theta[2]"].to_numpy(dtype=float))
    zs = data[["data.zs[0]", "data.zs[1]", "data.zs[2]", "data.zs[3]"]].to_numpy(
        dtype=float
    )
    front_zs = zs[:, :2].mean(axis=1)
    rear_zs = zs[:, 2:].mean(axis=1)
    front_delta = front_zs - front_zs[0]
    rear_delta = rear_zs - rear_zs[0]

    front, rear = build_axles()
    front_al_y0 = float(front.al_pt.y)
    rear_al_y0 = float(rear.al_pt.y)
    front_brake_share = suspension_f.front_brake_bias_sv
    rear_brake_share = 1.0 - suspension_f.front_brake_bias_sv

    rows: list[dict[str, float | bool]] = []
    for i, t in enumerate(time_s):
        # ``zs`` is the only vertical pose signal in this extracted lap file;
        # apply the front/rear sprung-height change to the corresponding SV Al.
        front_ok = front.update_geometry(front_al_y0 + front_delta[i])
        rear_ok = rear.update_geometry(rear_al_y0 + rear_delta[i])
        valid = front_ok and rear_ok

        row: dict[str, float | bool] = {
            "time_s": float(t),
            "pitch_deg": float(pitch_deg[i]),
            "front_zs_change_mm": float(front_delta[i] * 1000.0),
            "rear_zs_change_mm": float(rear_delta[i] * 1000.0),
            "geometry_ok": bool(valid),
            "front_ic_x_mm": np.nan,
            "front_ic_y_mm": np.nan,
            "rear_ic_x_mm": np.nan,
            "rear_ic_y_mm": np.nan,
            "pitch_center_x_m": np.nan,
            "pitch_center_y_mm": np.nan,
            "front_anti_dive_pct": np.nan,
            "rear_anti_lift_pct": np.nan,
            "rear_anti_squat_pct": np.nan,
        }
        if valid:
            front_ic = front.ic_pt.pos.copy()
            rear_ic = rear.ic_pt.pos.copy()
            pc = ga.SusGeometryHelper.line_intersection(
                front.contact_pt, front_ic, rear.contact_pt, rear_ic
            )

            row.update(
                {
                    "front_ic_x_mm": float(front_ic[0] * 1000.0),
                    "front_ic_y_mm": float(front_ic[1] * 1000.0),
                    "rear_ic_x_mm": float(rear_ic[0] * 1000.0),
                    "rear_ic_y_mm": float(rear_ic[1] * 1000.0),
                    "front_anti_dive_pct": anti_percent(
                        front.contact_pt.pos, front_ic, front.direction_sign,
                        front_brake_share,
                    ),
                    "rear_anti_lift_pct": anti_percent(
                        rear.wc_pt.pos, rear_ic, rear.direction_sign,
                        rear_brake_share,
                    ),
                    "rear_anti_squat_pct": anti_percent(
                        rear.contact_pt.pos, rear_ic, rear.direction_sign,
                        rear_drive_share,
                    ),
                }
            )
            if pc is not None:
                row["pitch_center_x_m"] = float(pc[0])
                row["pitch_center_y_mm"] = float(pc[1] * 1000.0)
        rows.append(row)

    return pd.DataFrame(rows)


def plot_results(output: pd.DataFrame, figure_path: Path) -> None:
    valid = output["geometry_ok"]
    t = output["time_s"]
    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    for column, label in (
        ("front_anti_dive_pct", "Front anti-dive"),
        ("rear_anti_lift_pct", "Rear anti-lift"),
        ("rear_anti_squat_pct", "Rear anti-squat"),
    ):
        axes[0].plot(t[valid], output.loc[valid, column], label=label)
    axes[0].set_ylabel("Anti (%)")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t[valid], output.loc[valid, "front_ic_y_mm"], label="Front IC")
    axes[1].plot(t[valid], output.loc[valid, "rear_ic_y_mm"], label="Rear IC")
    axes[1].set_ylabel("IC height (mm)")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t[valid], output.loc[valid, "pitch_center_x_m"], color="tab:purple")
    axes[2].set_ylabel("Pitch center X (m)")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t[valid], output.loc[valid, "pitch_center_y_mm"], color="tab:green")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylabel("Pitch center Y (mm)")
    axes[3].grid(True, alpha=0.3)

    fig.suptitle("Run 15 Side-View Anti Geometry")
    fig.tight_layout()
    fig.savefig(figure_path, dpi=160)
    if plt.get_backend().lower() != "agg":
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT, help="Run 15 CSV")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Analysis CSV")
    parser.add_argument("--figure", type=Path, default=DEFAULT_FIGURE, help="Figure output")
    parser.add_argument(
        "--rear-drive-share", type=float, default=1.0,
        help="Rear axle share of drive force used for anti-squat (default: 1.0)",
    )
    args = parser.parse_args()

    result = analyze(args.input, rear_drive_share=args.rear_drive_share)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.figure.parent.mkdir(parents=True, exist_ok=True)
    result.to_csv(args.output, index=False)
    plot_results(result, args.figure)
    failed = int((~result["geometry_ok"]).sum())
    print(f"Analysis complete: {len(result)} rows, {failed} invalid geometry rows")
    print(f"CSV: {args.output}")
    print(f"Figure: {args.figure}")


if __name__ == "__main__":
    main()
