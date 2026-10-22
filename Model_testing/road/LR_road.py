"""Left/right inputs. These require an LR state, never a quarter state."""
import numpy as np

g = 9.81


def _set_road(state, zl=0.0, zr=0.0, vl=0.0, vr=0.0):
    state.left.zr, state.right.zr = zl, zr
    state.left.vzr, state.right.vzr = vl, vr
    state.ay = 0.0

def _bump(state, height, start, duration, right_sign):
    # Match quarter_road.road_bump, including its analytic road velocity.
    if start <= state.t < start + duration:
        phase = np.pi * (state.t - start) / duration
        z = height * np.sin(phase)
        v = height * np.pi / duration * np.cos(phase)
        _set_road(state, z, right_sign * z, v, right_sign * v)
    else:
        _set_road(state)


def road_bump_left(state, height=0.01, start=0.25, duration=0.2):
    """Half-sine bump under the left wheel only."""
    _bump(state, height, start, duration, right_sign=0)


def road_bump_both(state, height=0.01, start=0.25, duration=0.2):
    """Simultaneous half-sine bumps under both wheels."""
    _bump(state, height, start, duration, right_sign=1)

def _sine(state, t0, amplitude, freq, right_sign):
    if state.t >= t0:
        w = 2 * np.pi * freq
        phase = w * (state.t - t0)
        z = amplitude * np.sin(phase)
        v = amplitude * w * np.cos(phase)
        _set_road(state, z, right_sign * z, v, right_sign * v)
    else:
        _set_road(state)

def road_sin_left(state, t0=0.25, amplitude=0.05, freq=3):
    _sine(state, t0, amplitude, freq, right_sign=0)

def Cornering_left(state, t0=0.25, ay=-2):
    _set_road(state)
    state.ay = ay * g if state.t >= t0 else 0.0
