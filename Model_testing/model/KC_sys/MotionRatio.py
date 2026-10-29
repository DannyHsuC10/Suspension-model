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
