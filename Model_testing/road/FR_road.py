import numpy as np

g = 9.81

def road_impulse_front_rear(state, t0=0.5, h=0.1, duration=0.02):
    
    if t0 <= state.t <= t0 + duration:

        state.front.zr = h
        state.rear.zr = h

    else:

        state.front.zr = 0
        state.rear.zr = 0


    state.front.vzr = 0
    state.rear.vzr = 0

def road_impulse_rear(state, t0=0.5, h=0.1, duration=0.02):
    
    if t0 <= state.t <= t0 + duration:

        state.rear.zr = h

    else:

        state.rear.zr = 0


    state.front.zr = 0


    state.front.vzr = 0
    state.rear.vzr = 0

def road_impulse_drive(state,
                       l = 1.5,
                       speed = 1,
                       t0=0.5,
                       h=0.1,
                       duration=0.02):


    delay = l / speed


    # front wheel hit

    if t0 <= state.t <= t0 + duration:

        state.front.zr = h

    else:

        state.front.zr = 0



    # rear wheel hit later

    if (
        t0+delay 
        <= state.t 
        <= t0+delay+duration
    ):

        state.rear.zr = h

    else:

        state.rear.zr = 0



    state.front.vzr = 0
    state.rear.vzr = 0

def road_sine_front_rear(state,t0 = 0.1,
                          freq=2,
                          amp=0.02):

    if state.t > t0:
        zr = amp*np.sin(
            2*np.pi*freq*state.t
        )

        w = 2*np.pi*freq

        dz = amp*w*np.cos(
            w*(state.t-t0)
        )
        state.front.zr = zr
        state.rear.zr = -zr

        state.front.vzr = dz
        state.rear.vzr = -dz
    else:
        state.front.zr = 0
        state.rear.zr = 0


        state.front.vzr = 0
        state.rear.vzr = 0

def accelerate(state,m = 160, t0=0.5, ax=2):
    
    if state.t > t0:
        state.Fx_rear = ax*g*m/2
        state.Fx_front = ax*g*m/2
        state.ax = ax*g
        state.front.zr = 0
        state.rear.zr = 0

    else:

        state.front.zr = 0
        state.rear.zr = 0


    state.front.vzr = 0
    state.rear.vzr = 0

def breaking(state,m = 160, t0=0.5, ax=-2):
    
    if state.t > t0:
        state.Fx_front = ax*g*m/2
        state.Fx_rear = ax*g*m/2
        state.ax = ax*g
        state.front.zr = 0
        state.rear.zr = 0

    else:

        state.front.zr = 0
        state.rear.zr = 0


    state.front.vzr = 0
    state.rear.vzr = 0