import numpy as np
from .MotionRatio import ConstantMR, LookupMR
from .Shock_Absorbers import KCSystem
from .damper import  LinearDamper
from .spring import LinearSpring


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

