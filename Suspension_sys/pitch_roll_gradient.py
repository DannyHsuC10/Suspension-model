"""Quasi-static pitch and roll gradients for the Suspension_sys vehicle model.

Run from any directory with ``python Suspension_sys/pitch_roll_gradient.py``.
The two plots show the local attitude gradient (degrees per g) over positive
longitudinal and lateral acceleration sweeps from 1 to 2.5 g.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares


# Keep the existing project's imports working whether this file is launched
# from the repository root or from Suspension_sys itself.
SUSPENSION_SYS_DIR = Path(__file__).resolve().parent
if str(SUSPENSION_SYS_DIR) not in sys.path:
    sys.path.insert(0, str(SUSPENSION_SYS_DIR))

with contextlib.redirect_stdout(io.StringIO()):
    from Suspension_LP03 import Suspension_A
    from Vehicle_status import full_suspension_state
    from model import Suspension_model_v13 as suspension_model


G = 9.81


def _equilibrium_residual(
    values: np.ndarray,
    state: full_suspension_state,
    sus: Suspension_A,
) -> np.ndarray:
    """Return normalized force and moment balance errors for static solving."""
    # Unknowns are body heave, front/rear roll, pitch, and four unsprung heights.
    state.s[2] = values[0]
    state.theta[:3] = values[1:4]
    state.zu[:] = values[4:8]
    state.zs[:] = state.body_to_corner(state.s, state.theta, sus)

    with contextlib.redirect_stdout(io.StringIO()):
        fz, moments, tire_forces, _normal_loads, _suspension_forces = (
            suspension_model.Suspension_output(state, sus)
        )

    # M[0:2] are the front and rear roll moments, M[2] is pitch; yaw is zero.
    force_scale = sus.m * G
    moment_scale = force_scale * max(sus.tf, sus.tr, sus.l)
    return np.concatenate(
        (
            np.array([fz / force_scale]),
            moments[:3] / moment_scale,
            tire_forces / force_scale,
        )
    )


def solve_attitude(
    ax_g: float,
    ay_g: float,
    sus: Suspension_A,
    initial_guess: np.ndarray | None = None,
) -> tuple[float, float, np.ndarray]:
    """Solve static body attitude for given acceleration magnitudes in g."""
    state = full_suspension_state(sus)
    ax = ax_g * G
    ay = ay_g * G

    # Represent steady acceleration with four equal tire forces. Their sum is
    # also applied at the CG so the model's external-moment path sees the load.
    state.Fx[:] = sus.m * ax / 4.0
    state.Fy[:] = sus.m * ay / 4.0
    state.F_cg[:] = (sus.m * ax, sus.m * ay, -sus.m * G)

    guess = np.zeros(8) if initial_guess is None else initial_guess.copy()
    solution = least_squares(
        _equilibrium_residual,
        guess,
        args=(state, sus),
        xtol=1e-11,
        ftol=1e-11,
        gtol=1e-11,
        max_nfev=500,
    )
    residual_norm = np.linalg.norm(solution.fun)
    if not solution.success or residual_norm > 1e-6:
        raise RuntimeError(
            f"Static equilibrium did not converge at ax={ax_g:.3f} g, "
            f"ay={ay_g:.3f} g (residual={residual_norm:.3g})."
        )

    pitch_deg = float(np.rad2deg(solution.x[3]))
    # Front and rear roll are separate state variables; report their mean as
    # the vehicle roll attitude while retaining chassis twist in the model.
    roll_deg = float(np.rad2deg(np.mean(solution.x[1:3])))
    return pitch_deg, roll_deg, solution.x


def calculate_gradients(points: int = 31) -> tuple[np.ndarray, ...]:
    """Return acceleration, attitude, and local-gradient arrays."""
    if points < 3:
        raise ValueError("points must be at least 3 to calculate a gradient.")

    acceleration_g = np.linspace(1.0, 2.5, points)
    sus = Suspension_A()
    pitch_deg = np.empty(points)
    roll_deg = np.empty(points)

    guess = None
    for index, acceleration in enumerate(acceleration_g):
        pitch_deg[index], _roll_unused, guess = solve_attitude(
            acceleration, 0.0, sus, guess
        )

    guess = None
    for index, acceleration in enumerate(acceleration_g):
        _pitch_unused, roll_deg[index], guess = solve_attitude(
            0.0, acceleration, sus, guess
        )

    pitch_gradient = np.gradient(pitch_deg, acceleration_g)
    roll_gradient = np.gradient(roll_deg, acceleration_g)
    return acceleration_g, pitch_deg, pitch_gradient, roll_deg, roll_gradient


def plot_gradients(points: int = 31, save_path: Path | None = None) -> None:
    acceleration_g, pitch_deg, pitch_gradient, roll_deg, roll_gradient = (
        calculate_gradients(points)
    )

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), sharex="col")
    axes[0, 0].plot(acceleration_g, pitch_deg, color="tab:red", linewidth=2)
    axes[0, 0].set_title("Pitch response to longitudinal acceleration")
    axes[0, 0].set_ylabel("Pitch (deg)")
    axes[1, 0].plot(
        acceleration_g, pitch_gradient, color="tab:orange", linewidth=2
    )
    axes[1, 0].set_title("Pitch gradient")
    axes[1, 0].set_ylabel("d pitch / d ax (deg/g)")
    axes[1, 0].set_xlabel("Longitudinal acceleration ax (g)")

    axes[0, 1].plot(acceleration_g, roll_deg, color="tab:blue", linewidth=2)
    axes[0, 1].set_title("Roll response to lateral acceleration")
    axes[0, 1].set_ylabel("Roll (deg)")
    axes[1, 1].plot(
        acceleration_g, roll_gradient, color="tab:cyan", linewidth=2
    )
    axes[1, 1].set_title("Roll gradient")
    axes[1, 1].set_ylabel("d roll / d ay (deg/g)")
    axes[1, 1].set_xlabel("Lateral acceleration ay (g)")

    for axis in axes.flat:
        axis.grid(True, alpha=0.35)
    figure.suptitle("Suspension_sys quasi-static attitude gradients")
    figure.tight_layout()

    if save_path is not None:
        figure.savefig(save_path, dpi=160, bbox_inches="tight")
        print(f"Saved plot: {save_path}")
    plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--points", type=int, default=31, help="number of sweep points (default: 31)"
    )
    parser.add_argument(
        "--save",
        type=Path,
        help="optional path for saving the figure, for example pitch_roll_gradient.png",
    )
    arguments = parser.parse_args()
    plot_gradients(arguments.points, arguments.save)


if __name__ == "__main__":
    main()
