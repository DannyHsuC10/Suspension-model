import numpy as np

class ConstantRollCenter:

    def __init__(self,h):
        self.h_front = h
        self.h_rear = h

    def get_height(self,state):

        return self.h_front,self.h_rear

class MultiConstantRollCenter:

    def __init__(self,h_front,h_rear):
        self.h_front = h_front
        self.h_rear = h_rear


    def get_height(self,state):

        return self.h_front,self.h_rear


class TableRollCenter:

    def __init__(
        self,
        roll_front,
        h_front,
        roll_rear,
        h_rear
    ):

        self.roll_front = np.array(roll_front)
        self.h_front_table = np.array(h_front)

        self.roll_rear = np.array(roll_rear)
        self.h_rear_table = np.array(h_rear)


    def get_height(self,state):

        h_front = np.interp(
            state.state.theta[0],
            self.roll_front,
            self.h_front_table
        )

        h_rear = np.interp(
            state.state.theta[1],
            self.roll_rear,
            self.h_rear_table
        )

        return h_front,h_rear


class GeoRollCenter: # 完全幾何求解(未完成等之後處理)
    
    def __init__():
        pass

    def get_rate(self,state):
        pass


"""

# =====================================================
# 1. Basic constant geometry
# =====================================================

# Roll center
roll_center = ConstantRollCenter(
    h=0.05       # 50 mm
)

# =====================================================
# 2. Front / Rear different geometry
# =====================================================

roll_center_multi = MultiConstantRollCenter(
    h_front=0.04,
    h_rear=0.08
)

# =====================================================
# 3. Lookup table example
# =====================================================

# Roll center varies with roll angle
roll_table = np.array([
    -5.0,
     0.0,
     5.0
])


front_rc_height = np.array([
    0.035,
    0.050,
    0.065
])


rear_rc_height = np.array([
    0.050,
    0.070,
    0.090
])


roll_center_table = TableRollCenter(
    roll_front=roll_table,
    h_front=front_rc_height,

    roll_rear=roll_table,
    h_rear=rear_rc_height
)



"""
