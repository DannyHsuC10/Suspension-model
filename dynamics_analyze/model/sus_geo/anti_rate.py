import numpy as np


class ConstantAnti:# 單常數

    def __init__(
        self,
        rate = 0.15
    ):
        self.dive_front = -rate
        self.lift_front = -rate
        self.squat_rear = rate
        self.lift_rear = rate

    def get_rate(self,state):

        return self.dive_front,self.lift_front,self.squat_rear,self.lift_rear

class MultiConstantAnti:# 多常數

    def __init__(
        self,
        dive_front = 0.15,
        lift_front = 0.15,
        squat_rear = 0.15,
        lift_rear = 0.15
    ):
        self.dive_front = -dive_front
        self.lift_front = -lift_front
        self.squat_rear = squat_rear
        self.lift_rear = lift_rear

    def get_rate(self,state):

        return self.dive_front,self.lift_front,self.squat_rear,self.lift_rear

class TableAnti:# 查表pitch

    def __init__(
        self,
        ptich,
        dive_front,
        lift_front,
        squat_rear,
        lift_rear
    ):

        self.ptich = np.array(ptich)

        self.dive_front = np.array(dive_front)
        self.lift_front = np.array(lift_front)
        self.squat_rear = np.array(squat_rear)
        self.lift_rear = np.array(lift_rear)



    def get_rate(self,state):

        ptich = state.theta[2]


        dive_front = np.interp(
            ptich,
            self.ptich,
            self.dive_front
        )

        lift_front = np.interp(
            ptich,
            self.ptich,
            self.lift_front
        )


        squat_rear = np.interp(
            ptich,
            self.ptich,
            self.squat_rear
        )


        lift_rear = np.interp(
            ptich,
            self.ptich,
            self.lift_rear
        )


        return (# 方向性定義設定在這邊比較方便
            -dive_front,
            -lift_front,
            squat_rear,
            lift_rear
        )

class GeoAnti: # 完全幾何求解(未完成等之後處理)
    
    def __init__():
        pass

    def get_rate(self,state):
        pass

"""

# =====================================================
# 1. Basic constant geometry
# =====================================================

# Anti geometry
anti = ConstantAnti(
    rate=0.15    # 15%
)



# =====================================================
# 2. Front / Rear different geometry
# =====================================================

anti_multi = MultiConstantAnti(
    dive_front=-0.10,
    lift_front=-0.10,
    squat_rear=0.20,
    lift_rear=0.15
)



# =====================================================
# 3. Lookup table example
# =====================================================

# Anti varies with pitch angle

pitch_table = np.array([
    -5.0,
     0.0,
     5.0
])


anti_table = TableAnti(
    ptich=pitch_table,

    dive_front=np.array([
        -0.10,
        -0.15,
        -0.20
    ]),

    lift_front=np.array([
        -0.05,
        -0.10,
        -0.15
    ]),

    squat_rear=np.array([
        0.10,
        0.15,
        0.20
    ]),

    lift_rear=np.array([
        0.05,
        0.10,
        0.15
    ])
)

"""