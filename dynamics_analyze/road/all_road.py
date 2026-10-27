import numpy as np



def road_D(state):

    state.zr = np.zeros(4)

    if 0.5< state.t:
        state.Fy = (-1000.0,-1000.0,-1000.0,-1000.0)
        state.Fx = (-1000.0,-1000.0,-1000.0,-1000.0)
        state.F_cg = np.array([sum(state.Fx),sum(state.Fy),-321*9.81])

def road_FL_bump(state):
    
    state.zr = np.zeros(4)


    # FL bump
    if 0.5 < state.t < 0.7:

        state.zr[0] = 0.01   # FL 上升 5 cm

