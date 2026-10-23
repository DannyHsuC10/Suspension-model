import numpy as np

class ConstantCamber:# 單常數

    def __init__(self,camber):

        self.camber = camber # 4 array
    def get_camber(self,state):

        return self.camber

class TableCamber:
    """
    Camber lookup table.

    Parameters
    ----------
    travel : array_like
        Wheel travel (m).

    camber : array_like
        Camber angle (rad).

        shape
        -----
        (N,)      -> same table for all wheels
        (4, N)    -> independent table for each wheel
    """

    def __init__(self, travel, camber):

        self.travel = np.asarray(travel, dtype=float)
        self.camber = np.asarray(camber, dtype=float)

        if self.camber.ndim == 1:
            # 同一張表給四輪
            self.camber = np.tile(self.camber, (4, 1))

    def get_camber(self, state):

        z = np.asarray(state.zu)

        gamma = np.empty(4)

        for i in range(4):
            gamma[i] = np.interp(
                z[i],
                self.travel,
                self.camber[i]
            )

        return gamma

class GeoCamber: # 完全幾何求解(未完成等之後處理)
    
    def __init__():
        pass

    def get_rate(self,state):
        pass

"""

# ============================================
# Example 1 : Constant camber
# ============================================

camber = ConstantCamber(
    np.deg2rad([
        -3.0,   # FL
        -3.0,   # FR
        -2.0,   # RL
        -2.0    # RR
    ])
)

# gamma = camber.get_camber(state)


# ============================================
# Example 2 : Same camber table for all wheels
# ============================================

travel = np.array([
    -0.05,
    -0.025,
     0.00,
     0.025,
     0.05
])

camber_table = np.deg2rad([
    -1.0,
    -2.0,
    -3.0,
    -4.0,
    -5.0
])

camber = TableCamber(travel, camber_table)

# gamma = camber.get_camber(state)


# ============================================
# Example 3 : Independent table for each wheel
# ============================================

travel = np.array([
    -0.05,
    -0.025,
     0.00,
     0.025,
     0.05
])

camber_table = np.deg2rad([

    # FL
    [-1, -2, -3, -4, -5],

    # FR
    [-1, -2, -3, -4, -5],

    # RL
    [-2, -2.5, -3, -3.5, -4],

    # RR
    [-2, -2.5, -3, -3.5, -4],
])

camber = TableCamber(travel, camber_table)

# gamma = camber.get_camber(state)

"""