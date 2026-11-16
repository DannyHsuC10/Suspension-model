"""Front/rear road and longitudinal inputs; default onset is 0.25 s."""
import numpy as np

g = 9.81

def _set_road(state, zf=0.0, zr=0.0, vf=0.0, vr=0.0):
    state.front.zr, state.rear.zr = zf, zr
    state.front.vzr, state.rear.vzr = vf, vr
    state.ax = state.Fx_front = state.Fx_rear = 0.0


def road_impulse_drive(state, l=1.5, speed=1, t0=0.25, h=0.1, duration=0.05):
    if speed <= 0:
        raise ValueError("speed must be positive")
    delay = l / speed
    zf = h if t0 <= state.t < t0 + duration else 0.0
    zr = h if t0 + delay <= state.t < t0 + delay + duration else 0.0
    _set_road(state, zf, zr)

def accelerate(state, m=160, t0=0.25, ax=2):
    if state.t >= t0:
        state.ax = ax * g
        state.Fx_front = state.Fx_rear = state.ax * m / 2