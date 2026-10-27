"""Vehicle parameters and replaceable KC systems for the dynamics models."""

import numpy as np



if __package__ == "suspension_set":
    from model.KC_sys import LinearKC, NullKC, SeriesKC
    from model.sus_geo.Geo_builder import create_basic_geometry
else:
    from ..model.KC_sys import LinearKC, NullKC, SeriesKC
    from ..model.sus_geo.Geo_builder import create_basic_geometry

g = 9.81

class SuspensionQuarter:
    def __init__(self, k=70000):
        self.ms = 67      # sprung mass
        self.mu = 13      # unsprung mass
        self.m_corner = self.ms + self.mu

        # Keep the original Quarter convention: use unsprung mass as the
        # reference mass for suspension/tire component damping ratios.
        self.zeta = 0.7
        self.corner = LinearKC(
            k=k,
            c=2 * self.zeta * np.sqrt(k * self.mu),
            mr_value=1.0,
            name="corner",
        )
        k_tire = 80000
        self.tire = LinearKC(
            k=k_tire,
            c=2 * self.zeta * np.sqrt(k_tire * self.mu),
            name="tire",
        )

    @property
    def ride_rate(self):
        """Wheel/tire equivalent stiffness at zero displacement (N/m)."""
        kw = self.corner.spring.stiffness(0.0) * self.corner.motion_ratio.ratio(0.0)**2
        kt = self.tire.spring.stiffness(0.0) * self.tire.motion_ratio.ratio(0.0)**2
        return kw * kt / (kw + kt) if kw + kt != 0 else 0.0


class SuspensionQuartertender(SuspensionQuarter):
    def __init__(self):
        super().__init__()
        k_main = 70000
        # Critical damping based on the main spring and Quarter's reference mass.
        c_critical = 2 * np.sqrt(k_main * self.mu)
        self.corner = SeriesKC(
            main=k_main,
            low=k_main * 0.5,
            travel=0.0043,
            low_comp=c_critical,
            high_comp=c_critical,
            low_reb=c_critical,
            high_reb=c_critical,
            transition=0.05,
            mr_value=1.0,
            travel_lim_compress=0.08,
            travel_lim_stretch=0.08,
            name="corner_tender",
        )


class SuspensionQuarteravg(SuspensionQuarter):
    def __init__(self):
        super().__init__(k=(36000 + 50000) / 2)


class SuspensionQuarterheave(SuspensionQuarter):
    def __init__(self):
        super().__init__(k=70000)


class SuspensionLR:
    def __init__(self):
        self.track = 1.25
        self.Ix = 30
        self.ms = 65      # sprung mass per corner
        self.mu = 15      # unsprung mass per corner
        self.m_corner = self.ms + self.mu

        self.h_rc = 0.0
        self.h_cg = 0.3
        self.tau_rc = 0.005

        self.zeta = 0.7
        k_heave = 18000
        k_roll = 50000
        self.heave = LinearKC(
            k=k_heave,
            c=2 * self.zeta * np.sqrt(k_heave * self.mu),
            mr_value=1.0,
            name="heave",
        )
        self.roll = LinearKC(
            k=k_roll,
            c=2 * self.zeta * np.sqrt(k_roll * self.mu),
            mr_value=1.0,
            name="roll",
        )

        # No direct corner shocks in the original setup. These can be replaced
        # independently with LinearKC, SeriesKC, Ohlins or LookupKC objects.
        self.corner_left = NullKC()
        self.corner_right = NullKC()

        k_tire = 800000
        self.tire_left = LinearKC(
            k=k_tire,
            c=50,
            name="tire_left",
        )
        self.tire_right = LinearKC(
            k=k_tire,
            c=50,
            name="tire_right",
        )

        self.m_frame = 65     # virtual frame mass (kg)
        k_frame = k_roll * 10
        self.frame = LinearKC(
            k=k_frame,
            c=2 * self.zeta * np.sqrt(k_frame * self.m_frame),
            name="frame",
        )


class SuspensionFR:
    def __init__(self):
        self.Iy = 50
        self.ms = 65      # sprung mass per corner
        self.mu = 15      # unsprung mass per corner
        self.m_corner = self.ms + self.mu

        self.lf = 0.765
        self.lr = 0.765
        self.h_cg = 0.3
        self.l = self.lf + self.lr

        self.anti_dive_front = 0
        self.anti_squat_rear = 0

        self.zeta = 0.7
        k_heave_f = 30000
        k_heave_r = 28000
        self.heave_f = LinearKC(
            k=k_heave_f,
            c=2 * self.zeta * np.sqrt(k_heave_f * self.mu),
            mr_value=1.0,
            name="heave_f",
        )
        self.heave_r = LinearKC(
            k=k_heave_r,
            c=2 * self.zeta * np.sqrt(k_heave_r * self.mu),
            mr_value=1.0,
            name="heave_r",
        )

        self.corner_front = NullKC()
        self.corner_rear = NullKC()

        k_tire = 80000
        self.tire_front = LinearKC(
            k=k_tire,
            c=2 * self.zeta * np.sqrt(k_tire * self.mu),
            name="tire_front",
        )
        self.tire_rear = LinearKC(
            k=k_tire,
            c=2 * self.zeta * np.sqrt(k_tire * self.mu),
            name="tire_rear",
        )


class Suspension_A:

    def __init__(self,):

        """
        1. 測試的時候先用LinearKC進行,之後要改其他的還有更多擴充
        2. corner_fl is for 直推,可以用這個來驗證車體的左右耦合是否正常
        3. 車輛單輪禿然受到大量衝擊是有可能扭轉1deg以上的

        heave_f = LinearKC(k=35000,c=3350,name="heave_f")# 30000
        heave_r = LinearKC(k=31500,c=3175,name="heave_r")# 28000
        roll_f = LinearKC(k=20000,c=2500,name="roll_f")#25000
        roll_r = LinearKC(k=18000,c=2400,name="roll_r")#22000

        """

        heave_f = LinearKC(k=35000,c=3350,name="heave_f")# 30000
        heave_r = LinearKC(k=31500,c=3175,name="heave_r")# 28000
        roll_f = LinearKC(k=20000,c=2500,name="roll_f")#25000
        roll_r = LinearKC(k=18000,c=2400,name="roll_r")#22000


        roll_c = NullKC()
        warp = LinearKC(k=0.0,c=0.0,name="warp")

        # 扭轉剛性需要換成rad 車架設定一個小小阻尼
        self.body_twist = LinearKC(k=180000,c=300,name="body_twist")
        
        corner_fl = NullKC()
        corner_fr = NullKC()
        corner_rl = NullKC()
        corner_rr = NullKC()

        # 輪胎保持一個小小阻尼
        tire_fl = LinearKC(k=80000,c=300,name="tire_fl")
        tire_fr = LinearKC(k=80000,c=300,name="tire_fr")
        tire_rl = LinearKC(k=80000,c=300,name="tire_rl")
        tire_rr = LinearKC(k=80000,c=300,name="tire_rr")



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

