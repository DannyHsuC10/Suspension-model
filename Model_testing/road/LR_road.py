import numpy as np

g = 9.81
# ==========================================
# Step input (單邊階躍)
# ==========================================
def road_step_left(state, t0=0.5, h=0.1):

    if state.t > t0:

        state.left.zr = h
        state.right.zr = 0

    else:

        state.left.zr = 0
        state.right.zr = 0


    state.left.vzr = 0
    state.right.vzr = 0



# ==========================================
# Step input (雙邊階躍)
# ==========================================
def road_step_both(state, t0=0.5, h=0.1):

    if state.t > t0:

        state.left.zr = h
        state.right.zr = h

    else:

        state.left.zr = 0
        state.right.zr = 0


    state.left.vzr = 0
    state.right.vzr = 0



# ==========================================
# Impulse input (衝擊)
# ==========================================
def road_impulse_both(state, t0=0.5, h=0.1, duration=0.02):

    if t0 <= state.t <= t0 + duration:

        state.left.zr = h
        state.right.zr = h

    else:

        state.left.zr = 0
        state.right.zr = 0


    state.left.vzr = 0
    state.right.vzr = 0



# ==========================================
# Impulse input (衝擊)
# ==========================================
def road_impulse_left(state, t0=0.5, h=0.1, duration=0.02):

    if t0 <= state.t <= t0 + duration:

        state.left.zr = h
        state.right.zr = 0

    else:

        state.left.zr = 0
        state.right.zr = 0


    state.left.vzr = 0
    state.right.vzr = 0


# ==========================================
# 單邊 Sin 路面
# ==========================================
def road_sin_left(
        state,
        t0=0.5,
        amplitude=0.05,
        freq=3):


    if state.t > t0:

        w = 2*np.pi*freq

        z = amplitude*np.sin(
            w*(state.t-t0)
        )

        dz = amplitude*w*np.cos(
            w*(state.t-t0)
        )


        state.left.zr = z
        state.right.zr = 0

        state.left.vzr = dz
        state.right.vzr = 0


    else:

        state.left.zr = 0
        state.right.zr = 0

        state.left.vzr = 0
        state.right.vzr = 0




# ==========================================
# 雙邊 Sin 路面 (同步)
# ==========================================
def road_sin_both(
        state,
        t0=0.5,
        amplitude=0.05,
        freq=3):


    if state.t > t0:

        w = 2*np.pi*freq

        z = amplitude*np.sin(
            w*(state.t-t0)
        )

        dz = amplitude*w*np.cos(
            w*(state.t-t0)
        )


        state.left.zr = z
        state.right.zr = z

        state.left.vzr = dz
        state.right.vzr = dz


    else:

        state.left.zr = 0
        state.right.zr = 0

        state.left.vzr = 0
        state.right.vzr = 0



# ==========================================
# 交錯 Sin (左右反相)
# ==========================================
def road_sin_alternate(
        state,
        t0=0.5,
        amplitude=0.05,
        freq=5):


    if state.t > t0:

        w = 2*np.pi*freq

        phase = w*(state.t-t0)


        zl = amplitude*np.sin(phase)
        zr = amplitude*np.sin(phase+np.pi)


        dzl = amplitude*w*np.cos(phase)
        dzr = amplitude*w*np.cos(phase+np.pi)


        state.left.zr = zl
        state.right.zr = zr

        state.left.vzr = dzl
        state.right.vzr = dzr


    else:

        state.left.zr = 0
        state.right.zr = 0

        state.left.vzr = 0
        state.right.vzr = 0


def Cornering_left(state, t0=0.5, ay=-2):
    
    if state.t > t0:
        state.ay = ay*g
        state.left.zr = 0
        state.right.zr = 0

    else:

        state.left.zr = 0
        state.right.zr = 0


    state.left.vzr = 0
    state.right.vzr = 0


def Cornering_ramp_to_max(state, t0=0.1, rise_time=0.000001, ay=-2):

    if state.t < t0:

        state.ay = 0

    elif state.t < t0 + rise_time:

        ratio = (state.t - t0) / rise_time
        state.ay = ay * g * ratio

    else:

        state.ay = ay * g


    state.left.zr = 0
    state.right.zr = 0

    state.left.vzr = 0
    state.right.vzr = 0