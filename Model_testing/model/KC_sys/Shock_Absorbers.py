import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":# 模組控制
    import spring as spr
    import damper as dam
    import MotionRatio as mr
    from spring import NullSpring
    from damper import NullDamper
    from MotionRatio import ConstantMR
else:
    from .spring import NullSpring
    from .damper import NullDamper
    from .MotionRatio import ConstantMR

class KCSystem:
    """
    避震器組合

    Force:
        spring + damper

    Motion Ratio:
        wheel force transform
    """

    def __init__(
        self,
        spring = NullSpring(),
        damper = NullDamper(),
        motion_ratio = ConstantMR(mr=1),
        name="Shock"
    ):

        self.spring = spring
        self.damper = damper
        self.motion_ratio = motion_ratio
        self.name = name


    def force(
        self,
        displacement,
        velocity):

        Fs = self.spring.force(displacement)

        Fd = self.damper.force(velocity)


        MR = self.motion_ratio.ratio(displacement)


        F = (
            Fs + Fd
        ) * MR**2


        return F,{
            "force":F,

            "spring_force":Fs,
            "damper_force":Fd,

            "motion_ratio":MR,

            "spring_k":
                self.spring.stiffness(displacement),

            "damper_c":
                self.damper.damping(velocity)
        }
