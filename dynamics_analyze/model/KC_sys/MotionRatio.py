import numpy as np


class MotionRatio:
    """基本 Motion Ratio"""

    def ratio(self, displacement):
        raise NotImplementedError

class ConstantMR(MotionRatio):
    
    def __init__(self, mr = 1):
        self.mr = mr

    def ratio(self, displacement):
        return self.mr

class LookupMR(MotionRatio):
    """
    使用輪胎位移查表 Motion Ratio

    displacement:
        wheel displacement

    ratio:
        motion ratio
    """

    def __init__(
        self,
        displacement,
        ratio
    ):

        self.displacement = np.array(displacement)
        self.ratio_data = np.array(ratio)


    def ratio(self, displacement):

        return np.interp(
            displacement,
            self.displacement,
            self.ratio_data
        )

