import numpy as np
import matplotlib.pyplot as plt

class Damper():
    """基本架構"""    
    def force(self, velocity):
        raise NotImplementedError

    def damping(self, velocity):
        raise NotImplementedError
    
class NullDamper(Damper):# 不安裝
    """沒裝阻尼"""    
    def force(self, velocity):
        return 0.0

    def damping(self, velocity):
        return 0.0

class LinearDamper(Damper):# 線性
    """線性"""    
    def __init__(self, c):
        self.c = c

    def damping(self, velocity):
        return self.c

    def force(self, velocity):
        return -self.c * velocity

class HLSpeedSwitchDamper(Damper):# HL
    """
    高低速阻尼

    閥值transition
    當速度高於閥值 >> 高速阻尼
    速度低於閥值 >> 高速阻尼
    """
    def __init__(
        self,
        low_comp,
        high_comp,
        low_reb,
        high_reb,
        transition
    ):

        self.transition = transition

        self.low_comp = low_comp
        self.high_comp = high_comp

        self.low_reb = low_reb
        self.high_reb = high_reb

    def damping(self, velocity):

        if velocity >= 0:

            if velocity < self.transition:
                return self.low_comp

            return self.high_comp

        else:

            if -velocity < self.transition:
                return self.low_reb

            return self.high_reb

    def force(self, velocity):

        c = self.damping(velocity)
        return -c * velocity
