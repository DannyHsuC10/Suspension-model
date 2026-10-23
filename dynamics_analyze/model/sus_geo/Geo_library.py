from model.sus_geo import *


"""
Suspension Geometry Library
"""


# ============================================================
# Basic FSAE geometry
# ============================================================

basic_roll_center = MultiConstantRollCenter(

    h_front=0.025,     # 25 mm
    h_rear=0.050       # 50 mm

)


basic_anti = MultiConstantAnti(

    dive_front=-0.15,
    lift_front=-0.15,

    squat_rear=0.15,
    lift_rear=0.15

)



# ============================================================
# Low Roll Center setup
# ============================================================

low_roll_center = MultiConstantRollCenter(

    h_front=0.010,
    h_rear=0.020

)


low_anti = MultiConstantAnti(

    dive_front=-0.10,
    lift_front=-0.10,

    squat_rear=0.10,
    lift_rear=0.10

)



# ============================================================
# High Anti geometry
# ============================================================

high_anti = MultiConstantAnti(

    dive_front=-0.30,
    lift_front=-0.20,

    squat_rear=0.40,
    lift_rear=0.20

)



# ============================================================
# Kinematics Lookup Example
# CAD linkage result
# ============================================================


roll_table_geometry = TableRollCenter(

    roll_front=[
        -0.10,
        -0.05,
         0.00,
         0.05,
         0.10
    ],

    h_front=[
        0.035,
        0.045,
        0.055,
        0.070,
        0.080
    ],


    roll_rear=[
        -0.10,
        -0.05,
         0.00,
         0.05,
         0.10
    ],

    h_rear=[
        0.050,
        0.065,
        0.080,
        0.095,
        0.110
    ]

)



anti_table_geometry = TableAnti(

    ptich=[

        -0.05,
         0.0,
         0.05

    ],


    dive_front=[

        0.10,
        0.15,
        0.20

    ],


    lift_front=[

        0.10,
        0.15,
        0.20

    ],


    squat_rear=[

        0.15,
        0.20,
        0.25

    ],


    lift_rear=[

        0.10,
        0.15,
        0.20

    ]

)



# ============================================================
# Full geometry preset
# ============================================================

FSAE_basic_geometry = {

    "roll_center": basic_roll_center,

    "anti": basic_anti

}



FSAE_low_roll_geometry = {

    "roll_center": low_roll_center,

    "anti": low_anti

}



FSAE_lookup_geometry = {

    "roll_center": roll_table_geometry,

    "anti": anti_table_geometry

}