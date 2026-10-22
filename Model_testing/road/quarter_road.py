import numpy as np


def road_step(state, h=0.01, start=0.5):# Step input (躍階)
    """Step input"""

    if state.t > start:
        state.zr = h
    else:
        state.zr = 0.0

    state.vzr = 0.0

def road_impulse(state,h = 0.01,start = 0.5,duration = 0.1):# impulse input (衝擊)
    
    if start < state.t < start + duration:
        state.zr = h
    else:
        state.zr = 0.0
    state.vzr = 0

def road_bump(state, height=0.01, start=0.5, duration=0.2):# Bump input (凸起)
    """Half sine bump"""

    if start < state.t < start + duration:

        x = (state.t - start) / duration

        state.zr = height * np.sin(np.pi * x)

        state.vzr = (
            height
            * np.pi
            / duration
            * np.cos(np.pi * x)
        )

    else:
        state.zr = 0.0
        state.vzr = 0.0

def road_sine(state, amplitude=0.01, frequency=5):# Sin road (正弦路面)
    """Sin road"""

    w = 2 * np.pi * frequency

    state.zr = amplitude * np.sin(w * state.t)

    state.vzr = amplitude * w * np.cos(w * state.t)

class RandomRoad:
    """
    Random road
    """

    def __init__(
        self,
        amplitude=0.01,
        dt_update=0.01,
        seed=0
    ):

        self.amplitude = amplitude
        self.dt_update = dt_update

        self.rng = np.random.default_rng(seed)

        self.last_t = None
        self.zr = 0.0

    def __call__(self, state):

        if self.last_t is None:
            self.last_t = state.t

        if state.t - self.last_t >= self.dt_update:

            zr_old = self.zr

            self.zr = (
                self.rng.uniform(-1, 1)
                * self.amplitude
            )

            state.vzr = (
                self.zr - zr_old
            ) / (state.t - self.last_t)

            self.last_t = state.t

        else:

            state.vzr = 0.0

        state.zr = self.zr