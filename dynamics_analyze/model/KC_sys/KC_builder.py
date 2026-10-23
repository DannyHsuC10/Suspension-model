import numpy as np
from .MotionRatio import ConstantMR, LookupMR
from .Shock_Absorbers import KCSystem
from .damper import HSWeightedDamper, LinearDamper, LookupDamper
from .spring import LinearSpring, LookupSpring, SeriesSpring


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

def SeriesKC(
    main,
    low,
    travel,

    low_comp,
    high_comp,
    low_reb,
    high_reb,
    transition,

    mr_value=1.0,
    travel_lim_compress=np.inf,
    travel_lim_stretch=np.inf,
    name="Series KCSystem",
):
    return KCSystem(
        spring=SeriesSpring(
            main=main,
            tender=low,
            travel=travel,
            travel_lim_compress=travel_lim_compress,
            travel_lim_stretch=travel_lim_stretch,
        ),
        damper=HSWeightedDamper(
            low_comp,
            high_comp,
            low_reb,
            high_reb,
            transition,
        ),
        motion_ratio=ConstantMR(mr_value),
        name=name,
    )

def Ohlins(
    k,
    low_comp,
    high_comp,
    low_reb,
    high_reb,
    transition,

    mr_value=1.0,
    name="Ohlins",
):
    return KCSystem(

        spring=LinearSpring(k),

        damper=HSWeightedDamper(
            low_comp,
            high_comp,
            low_reb,
            high_reb,
            transition,
        ),

        motion_ratio=ConstantMR(mr_value),

        name=name,
    )

def LookupKC(
    spring_displacement,
    spring_force,

    damper_velocity,
    damper_force,

    mr_displacement,
    mr_ratio,

    travel_lim_compress=np.inf,
    travel_lim_stretch=np.inf,
    name="Lookup KCSystem",
):
    return KCSystem(

        spring=LookupSpring(
            displacement=spring_displacement,
            force=spring_force,
            travel_lim_compress=travel_lim_compress,
            travel_lim_stretch=travel_lim_stretch,
        ),

        damper=LookupDamper(
            velocity=damper_velocity,
            force=damper_force,
        ),

        motion_ratio=LookupMR(
            displacement=mr_displacement,
            ratio=mr_ratio,
        ),

        name=name,
    )

def NullKC():
    return KCSystem()

