import numpy as np
from pathlib import Path
import os
import pandas as pd
from scipy.interpolate import interp1d

# 取得目前這個 .py 檔案的絕對路徑
base_path = Path(__file__).resolve().parent

g = 9.81

class Car:
    """
    Vehicle parameters
    """

    def __init__(self):
        # Geometry
        self.l = 1.53 # wheelbase
        self.lf = 0.7498400000000001 # CG>>fa
        self.lr = self.l-self.lf # CG>>ra
        self.tf = 1.25 # front_track
        self.tr = 1.24 # rear_track
        self.h_cog = 0.3016451511 # 重心高 
        self.m = 321 # mass
        self.w = self.m*g
        self.cg_rate = np.array([self.lr/self.l,self.lf/self.l])# 重心配比
        self.bottom = 50/1000 # 車底距離地板 m
        # 前懸吊車體節面
        # 3 4
        # 2 1
        self.body_face_f = [(0.2, self.bottom), (-0.2, self.bottom), (-0.2, 0.5), (0.2, 0.5)]

        # =================================(輪胎)
        self.tir = "tire.tir"
        self.free_radius = 205.73999999999998/1000 # m
        self.K_tire = 80.185*1000 # N/mm W8
        # W7 : 99.1699 N/mm
        self.tire_width = 190.5/1000 # m
        self.load_Radius = self.free_radius-self.m*self.cg_rate*g/self.K_tire/2# 前後輪負載半徑[f,r]

        print("load_Radius",self.load_Radius)

        # =================================(懸吊幾何fv)
        self.scrub_radius_f = 18/1000 # m
        self.kpi_f = np.deg2rad(9) # rad
        self.h_rc_f = 25/1000 # m
        self.fvsa_f = 1050/1000 # m

        # 輪端接點
        self.Au_f = 85/1000
        self.Al_f = 90/1000
        # 前懸吊接點定位連線
        # 3 4
        # 2 1
        # rl,ll,lu,ru
        self.sus_contact_f = [(0.205, 0.08), (-0.205, 0.08), (-0.250, 0.205), (0.250, 0.205)]

        # ================================(懸吊幾何sv)
        self.mechanical_tail = 15/1000 # m
        self.caster = 4 # deg

Car()