"""Single-wheel inputs. All excitations start at 0.25 s by default."""
import numpy as np


def road_step(state, h=0.01, start=0.25):
    state.zr = h if state.t >= start else 0.0
    state.vzr = 0.0

def road_bump(state, height=0.01, start=0.25, duration=0.2):
    if start <= state.t < start + duration:
        phase = np.pi * (state.t - start) / duration
        state.zr = height * np.sin(phase)
        state.vzr = height * np.pi / duration * np.cos(phase)
    else:
        state.zr = state.vzr = 0.0


def road_sine(state, amplitude=0.01, frequency=5, start=0.25):
    if state.t >= start:
        w = 2 * np.pi * frequency
        phase = w * (state.t - start)
        state.zr = amplitude * np.sin(phase)
        state.vzr = amplitude * w * np.cos(phase)
    else:
        state.zr = state.vzr = 0.0


class RandomRoad:
    """Seeded piecewise-constant road, independent for each experiment."""

    def __init__(self, amplitude=0.01, dt_update=0.01, seed=0, start=0.25):
        if dt_update <= 0:
            raise ValueError("dt_update must be positive")
        self.amplitude = amplitude
        self.dt_update = dt_update
        self.start = start
        self.rng = np.random.default_rng(seed)
        self.last_t = None
        self.zr = 0.0

    def __call__(self, state):
        if state.t < self.start:
            state.zr = state.vzr = 0.0
            return
        if self.last_t is None:
            self.last_t = self.start - self.dt_update
        elapsed = state.t - self.last_t
        if elapsed + 1e-12 >= self.dt_update:
            old_zr = self.zr
            self.zr = self.rng.uniform(-1, 1) * self.amplitude
            state.vzr = (self.zr - old_zr) / elapsed
            self.last_t = state.t
        else:
            state.vzr = 0.0
        state.zr = self.zr
