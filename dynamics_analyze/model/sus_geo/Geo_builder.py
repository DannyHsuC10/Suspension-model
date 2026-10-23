from model.sus_geo import *
import numpy as np

def create_fast_geometry(vehicle,h = 0.3,rate = 1, camber = np.zeros(4)):
    """
    快速建立簡化幾何模型
    """
    roll_center = ConstantRollCenter(h)
    anti = ConstantAnti(rate)
    ConstantCamber(camber)

    return GeometryModel(vehicle,roll_center,anti)

def create_basic_geometry(
        vehicle,
        h_front=0.025,
        h_rear=0.05,
        dive_front=0.15,
        lift_front=0.15,
        squat_rear=0.15,
        lift_rear=0.15,
        camber = np.zeros(4)
        ):
    """
    前後軸分離的固定幾何模型
    """

    roll_center = MultiConstantRollCenter(h_front,h_rear)
    anti = MultiConstantAnti(
        dive_front,
        lift_front,
        squat_rear,
        lift_rear)
    camber = ConstantCamber(camber)

    return GeometryModel(vehicle,roll_center,anti)

def create_table_geometry(
        vehicle,

        roll_front,
        h_front,
        roll_rear,
        h_rear,

        pitch,
        dive_front,
        lift_front,
        squat_rear,
        lift_rear,

        travel, 
        camber_table,
    ):
    """
    查表幾何模型

    roll_center_table:
        TableRollCenter

    anti_table:
        TableAnti
    """

    roll_center = TableRollCenter(
        roll_front,
        h_front,
        roll_rear,
        h_rear
    )


    anti = TableAnti(
        pitch,
        dive_front,
        lift_front,
        squat_rear,
        lift_rear
    )

    camber = TableCamber(
        travel, camber_table
    )


    return GeometryModel(
        vehicle,
        roll_center,
        anti,
        camber,
    )

def create_analytic_geometry():
    pass

