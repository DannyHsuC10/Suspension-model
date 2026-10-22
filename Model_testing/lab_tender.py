import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from data_visualization.Dynamic_Visualizer_quarter import Visualizer
from data_visualization.Simulation_Logger import SimulationLogger
from Parameter_set.Suspension import (
    SuspensionQuarteravg,
    SuspensionQuarterheave,
    SuspensionQuartertender,
)
from Parameter_set.suspension_state import quarter_suspension_state
from road import quarter_road as qr
from model import quarter_model as sm


current_dir = Path(__file__).resolve().parent


def run_bump_simulation(suspension_class, h, sim_time=2.0):
    sus = suspension_class()
    logger = SimulationLogger()
    state = quarter_suspension_state()

    t = np.arange(0, sim_time, state.dt)

    for _ in t:
        zr = qr.road_bump(state, height=h)
        sm.Quarter_suspension_model(sus, state)
        state.Integration(sus)
        logger.add_state(state)

    return logger.result()


def get_force_extrema(result):
    if "Ft" not in result or "Fs" not in result:
        raise KeyError("result must contain both 'Ft' and 'Fs'")

    ft = np.asarray(result["Ft"], dtype=float)
    fs = np.asarray(result["Fs"], dtype=float)

    if not np.any(np.isfinite(ft)):
        raise ValueError("Ft has no finite values")

    if not np.any(np.isfinite(fs)):
        raise ValueError("Fs has no finite values")

    return {
        "Ft_max": np.nanmax(ft),
        "Ft_min": np.nanmin(ft),
        "Fs_max": np.nanmax(fs),
        "Fs_min": np.nanmin(fs),
    }


def print_extrema(title, h, extrema):
    print(f"\n=== {title} | bump height = {h:.3f} m ===")
    print(f"Ft max = {extrema['Ft_max']:.6f} N")
    print(f"Ft min = {extrema['Ft_min']:.6f} N")
    print(f"Fs max = {extrema['Fs_max']:.6f} N")
    print(f"Fs min = {extrema['Fs_min']:.6f} N")


def sweep_bump_heights(suspension_name, suspension_class, heights):
    sweep_result = {
        "height": [],
        "Ft_max": [],
        "Ft_min": [],
        "Fs_max": [],
        "Fs_min": [],
    }

    print(f"\n\n===== Sweep: {suspension_name} =====")

    for h in heights:
        result = run_bump_simulation(suspension_class, h)
        extrema = get_force_extrema(result)
        print_extrema(suspension_name, h, extrema)

        sweep_result["height"].append(h)
        for key in ("Ft_max", "Ft_min", "Fs_max", "Fs_min"):
            sweep_result[key].append(extrema[key])

    return {
        key: np.asarray(value, dtype=float)
        for key, value in sweep_result.items()
    }


def plot_single_suspension_sweep(suspension_name, sweep_result):
    plt.figure(figsize=(10, 6))

    h = sweep_result["height"]

    plt.plot(h, sweep_result["Ft_max"], marker="o", label="Ft max")
    plt.plot(h, sweep_result["Ft_min"], marker="o", label="Ft min")
    plt.plot(h, sweep_result["Fs_max"], marker="o", label="Fs max")
    plt.plot(h, sweep_result["Fs_min"], marker="o", label="Fs min")

    plt.title(f"{suspension_name}: Bump Height vs Force Extrema")
    plt.xlabel("Bump height [m]")
    plt.ylabel("Force [N]")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()


def plot_suspension_comparison(all_sweep_results):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)

    metrics = [
        ("Ft_max", "Ft max"),
        ("Ft_min", "Ft min"),
        ("Fs_max", "Fs max"),
        ("Fs_min", "Fs min"),
    ]

    for ax, (key, title) in zip(axes.ravel(), metrics):
        for suspension_name, sweep_result in all_sweep_results.items():
            ax.plot(
                sweep_result["height"],
                sweep_result[key],
                marker="o",
                label=suspension_name,
            )

        ax.set_title(title)
        ax.set_ylabel("Force [N]")
        ax.grid(True)
        ax.legend()

    for ax in axes[-1, :]:
        ax.set_xlabel("Bump height [m]")

    fig.suptitle("Bump Height vs Ft/Fs Extrema")
    fig.tight_layout()


if __name__ == "__main__":
    heights = np.arange(0.005, 0.050 + 0.0001, 0.005)

    # Step 1: run one simulation and print Ft/Fs max/min.
    h = 0.005
    result = run_bump_simulation(SuspensionQuartertender, h)
    extrema = get_force_extrema(result)
    print_extrema("Single run: SuspensionQuartertender", h, extrema)

    # Step 2 and 3: sweep bump height for tender setting and plot four force lines.
    tender_sweep = sweep_bump_heights(
        "SuspensionQuartertender",
        SuspensionQuartertender,
        heights,
    )
    plot_single_suspension_sweep("SuspensionQuartertender", tender_sweep)

    # Step 4: sweep three suspension settings and compare each force extrema.
    suspension_cases = {
        "tender": SuspensionQuartertender,
        "avg": SuspensionQuarteravg,
        "heave": SuspensionQuarterheave,
    }

    all_sweep_results = {}

    for suspension_name, suspension_class in suspension_cases.items():
        all_sweep_results[suspension_name] = sweep_bump_heights(
            suspension_name,
            suspension_class,
            heights,
        )

    plot_suspension_comparison(all_sweep_results)

    # Optional visual check for the first single run.
    # viewer = Visualizer(result)
    # viewer.show()

    plt.show()
