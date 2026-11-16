import numpy as np
from abc import ABC, abstractmethod

# MR
class MotionRatio:
    """基本 Motion Ratio"""

    def ratio(self, displacement):
        raise NotImplementedError

class ConstantMR(MotionRatio):
    
    def __init__(self, mr = 1):
        self.mr = mr

    def ratio(self, displacement):
        return self.mr

# 彈簧
# ============================================================================

class Spring(ABC):

    def __init__(
        self,
        travel_lim_compress=np.inf,
        travel_lim_stretch=np.inf,
    ):
        self.travel_lim_compress = travel_lim_compress
        self.travel_lim_stretch = travel_lim_stretch

    @abstractmethod
    def stiffness(self, displacement):
        ...

    @abstractmethod
    def force(self, displacement):
        ...

    @abstractmethod
    def energy(self, displacement):
        ...

    @abstractmethod
    def displacement(self, force):
        ...


    def is_bottomed(self, displacement):
        return (
            displacement >= self.travel_lim_compress or
            displacement <= -self.travel_lim_stretch
        )

    def clamp_displacement(self, displacement):
        return np.clip(
            displacement,
            -self.travel_lim_stretch,
            self.travel_lim_compress
        )

class NullSpring(Spring):# 不安裝
    """沒裝彈簧"""    
    def stiffness(self, displacement):
        return 0.0

    def force(self, displacement):
        return 0.0

    def energy(self, displacement):
        return 0.0

    def displacement(self, force):
        return 0.0

    def is_bottomed(self, displacement): 
        return False

class LinearSpring(Spring):# 線性
    """Linear spring"""

    def __init__(
        self,
        k,
        travel_lim_compress=np.inf,
        travel_lim_stretch=np.inf,
    ):
        super().__init__(
            travel_lim_compress,
            travel_lim_stretch,
        )
        self.k = k

    def stiffness(self, displacement):
        return self.k

    def force(self, displacement):
        displacement = self.clamp_displacement(displacement)
        F = -self.k * displacement
        return F

    def energy(self, displacement):
        displacement = self.clamp_displacement(displacement)
        E = 0.5 * self.k * displacement**2
        return E

    def displacement(self, force):
        s = force/(-self.k)
        return s

# 阻尼
# ===========================================================================

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

# 避震器
# ============================================================================

class KCSystem:# 避震器組合
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


def LinearKC(
    k,
    c,
    mr_value=1.0,
    travel_lim_compress=np.inf,
    travel_lim_stretch=np.inf,
    name="Linear KCSystem",
):
    return KCSystem(
        spring=LinearSpring(
            k=k,
            travel_lim_compress=travel_lim_compress,
            travel_lim_stretch=travel_lim_stretch,
        ),
        damper=LinearDamper(c),
        motion_ratio=ConstantMR(mr_value),
        name=name,
    )

def NullKC():
    return KCSystem()

