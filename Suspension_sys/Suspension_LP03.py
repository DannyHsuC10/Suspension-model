import numpy as np
from model.KC_sys import *
from model.sus_geo import *

g = 9.81

class Suspension_A:

    def __init__(self,):

        """
        1. 測試的時候先用LinearKC進行,之後要改其他的還有更多擴充
        2. corner_fl is for 直推,可以用這個來驗證車體的左右耦合是否正常
        3. 車輛單輪禿然受到大量衝擊是有可能扭轉1deg以上的
        """

        heave_f = LinearKC(k=11000,c=1200,name="heave_f")# 30000
        heave_r = LinearKC(k=11000,c=1200,name="heave_r")# 28000
        roll_f = LinearKC(k=11000,c=1200,name="roll_f")#25000
        roll_r = LinearKC(k=11000,c=1200,name="roll_r")#22000


        roll_c = NullKC()
        warp = LinearKC(k=5000,c=1500,name="warp")

        # 扭轉剛性需要換成rad 車架設定一個小小阻尼
        self.body_twist = LinearKC(k=180000,c=50,name="body_twist")
        
        corner_fl = NullKC()
        corner_fr = NullKC()
        corner_rl = NullKC()
        corner_rr = NullKC()

        # 輪胎保持一個小小阻尼
        tire_fl = LinearKC(k=80000,c=50,name="tire_fl")
        tire_fr = LinearKC(k=80000,c=50,name="tire_fr")
        tire_rl = LinearKC(k=80000,c=50,name="tire_rl")
        tire_rr = LinearKC(k=80000,c=50,name="tire_rr")



        self.shock_list = [heave_f,heave_r,roll_f,roll_r,roll_c,warp,
                           corner_fl,corner_fr,corner_rl,corner_rr]
        self.tire_list = [tire_fl,tire_fr,tire_rl,tire_rr]

        mu_f = 15
        mu_r = 15

        self.mu_list = np.array([mu_f,mu_f,mu_r,mu_r])

        self.m = 321
        self.ms = self.m-mu_f*2-mu_r*2

        self.tf = 1.28
        self.tr = 1.24
        self.lf = 0.765
        self.lr = 0.765
        self.l = self.lf+self.lr

        self.h_cg = 0.3# 重心

        self.I_rate = np.array([self.lr/self.l,self.lf/self.l])

        """
        1. 為了不在懸吊模型裡面玩方向猜猜樂必須要強制把方向在這邊訂好
        2. 幾何傳力設定到100%還是會發生姿態改變，這是因為輪胎被壓縮
        3. roll 和 pitch 設定到100% 給相同的正交輸入姿態角度會不同，這是因為軸距輪距差異和motion ratio 一樣成2次正比
        4. pitch 沒有輸入的時候也有初始姿態這是彈簧剛性與重心位置的問題
        """
        # 建立 GeometryModel
        # 把懸吊喀了的驗證版本
        

        self.geometry = create_basic_geometry(

            self,# 注意傳入順序建議整個function往後掉
            # rc
            h_front=0.025,
            h_rear=0.025,
            # anti
            dive_front=0.15,
            lift_front=0.15,

            squat_rear=0.15,
            lift_rear=0.15,

            camber=np.zeros(4)
        )

        #self.geometry = create_fast_geometry(self)


        Ixx = 60
        Iyy = 100
        Izz = 150
        
        self.I = np.array([Ixx*self.I_rate[0],Ixx*self.I_rate[1],Iyy,Izz])


        self.R_tire = 205.73999999999998/1000 # m


        self.W = self.m * g



Suspension_A()

