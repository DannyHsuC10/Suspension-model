"""Step-road experiment for SuspensionQuartertender.

Run from the project root: python dynamics_analyze/lab_quarter.py
"""

from pathlib import Path

import numpy as np

from data_visualization.Dynamic_Visualizer_quarter import Visualizer
from data_visualization.Simulation_Logger import SimulationLogger
from model.model import Quarter_suspension_model
from road.quarter_road import road_bump
from suspension_set.Suspension import SuspensionQuartertender,SuspensionQuarter
from suspension_set.suspension_state import quarter_suspension_state


# Experiment settings (SI units).
STEP_HEIGHT = 0.01
STEP_START = 0.5
SIM_TIME = 3.0
DT = 0.001
FPS = 30
SAVE_GIF = False


def run_step_simulation(height=STEP_HEIGHT, start=STEP_START,
                        sim_time=SIM_TIME, dt=DT):
    """Return aligned state, force and road samples, including both endpoints."""
    if not np.isfinite([height, start, sim_time, dt]).all():
        raise ValueError("Step settings must be finite.")
    if dt <= 0 or sim_time <= 0 or not 0 <= start < sim_time:
        raise ValueError("Require dt > 0 and 0 <= start < sim_time, with sim_time > 0.")
    steps = round(sim_time / dt)
    if not np.isclose(steps * dt, sim_time, rtol=0, atol=1e-10):
        raise ValueError("sim_time must be an integer multiple of dt.")

    suspension = SuspensionQuarter()
    #suspension = SuspensionQuartertender()
    state = quarter_suspension_state(dt=dt)
    logger = SimulationLogger()

    for index in range(steps + 1):
        state.t = index * dt
        # Existing road_bump applies the step strictly after start (t > start).
        road_bump(state, height=height, start=start)
        Quarter_suspension_model(suspension, state)
        logger.add_state(state)
        if index < steps:
            state.Integration(suspension)

    result = logger.result()
    if not all(np.isfinite(values).all() for values in result.values()):
        raise FloatingPointError("Simulation contains non-finite values.")
    return result


def main():
    result = run_step_simulation()
    print(f"SuspensionQuartertender | step={STEP_HEIGHT:g} m after {STEP_START:g} s")
    print(f"Simulation: {SIM_TIME:g} s | dt={DT:g} s")
    for channel in ("zs", "zu", "azs", "Fs", "Ft"):
        values = result[channel]
        print(f"{channel}: min={values.min():.6g}, max={values.max():.6g}")

    viewer = Visualizer(
        result,
        save_filename=str(Path(__file__).resolve().parent / "quarter_step.gif"),
    )
    viewer.fig.suptitle(
        f"SuspensionQuartertender | Step {STEP_HEIGHT * 1000:g} mm after {STEP_START:g} s"
    )
    viewer.show(fps=FPS, save=SAVE_GIF)


if __name__ == "__main__":
    main()
