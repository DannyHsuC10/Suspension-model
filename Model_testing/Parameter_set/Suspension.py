import numpy as np
g = 9.81

class SuspensionQuarter:

    def __init__(self):

        self.ms = 67      # sprung mass
        self.mu = 13      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量

        self.ks = 28000   # 懸吊 stiffness

        zeta = 0.7           # Damping ratio
        self.cs = 2 * zeta * np.sqrt(self.ks * self.mu)   # 單角 damping
        # (輪端1/4)=====================================
        MR  = 1 # Motion ratio(沒有直推直接不管她=1)

        self.kt = 80000   # tire stiffness
        self.ct = 50       # tire damping (盡量保留一個小數字)

        self.kw = self.ks*MR **2    # 到輪胎上 stiffness
        self.cw = self.cs*MR **2

        # (整體1/8)=====================================
        self.ride_rate = self.kw*self.kt/(self.kw+self.kt)# ride rate

class SuspensionLR:
    
    def __init__(self):
        self.track = 1.28
        self.Ix = 30
        self.ms = 65      # sprung mass
        self.mu = 14      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量
        self.zeta = 0.7   # 避震器的名義阻尼比
        
        self.h_rc = 0.025
        self.h_cg = 0.3

        self.tau_rc = 0.005

        k_heave = 100000
        k_roll = 100000

        MR_heave = 1
        MR_roll = 1

        # 模態避震器
        self.k_heave = k_heave*MR_heave**2
        self.k_roll = k_roll*MR_roll**2

        # F_heave 分配到兩角；車身升沉的 K=2*k_heave、M=2*ms。
        self.c_heave = 2*self.zeta*np.sqrt(self.k_heave*self.ms)
        # d_roll 為左右位移差；K_theta=k_roll*track**2/2，C_theta 同理。
        m_roll = 2*self.Ix/self.track**2
        self.c_roll = 2*self.zeta*np.sqrt(self.k_roll*m_roll)

        k_corner = 0
        MR_corner = 1

        # 直推的避震器
        self.k_left = k_corner*MR_corner**2
        self.k_right = k_corner*MR_corner**2# 確保左右相同
        # 單角作用力同時造成車身升沉與側傾。
        m_eff_corner = 1/(1/(2*self.ms) + (self.track/2)**2/self.Ix)
        self.c_left = 2*self.zeta*np.sqrt(self.k_left*m_eff_corner)
        self.c_right = 2*self.zeta*np.sqrt(self.k_right*m_eff_corner)

        self.kt = 800000   # tire stiffness
        self.ct = 50       # tire damping (盡量保留一個小數字)

        self.k_frame = k_roll*10      # N/m

        self.m_frame = 65     # kg (虛擬質量)
        self.c_frame = 150

class SuspensionFR:
    
    def __init__(self):

        self.Iy = 50
        self.ms = 65      # sprung mass
        self.mu = 15      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量
        self.zeta = 0.7   # 前後懸吊通道的名義阻尼比

        self.lf = 0.765
        self.lr = 0.765
        self.h_cg = 0.3
        self.l = self.lf+self.lr

        self.anti_dive_front =0.2
        self.anti_squat_rear =0.2
        #self.anti = 0.15

        k_heave_f = 70000
        k_heave_r = 70000

        MR_heave_f = 1
        MR_heave_r = 1

        # 模態避震器
        self.k_heave_f = k_heave_f*MR_heave_f**2
        self.k_heave_r = k_heave_r*MR_heave_r**2
        # 單軸作用力同時造成車身升沉與俯仰；1/m_eff=1/M+l_arm**2/Iy。
        # 以 4DOF 模型的完整軸力定義各通道的名義阻尼比。
        m_eff_front = 1/(1/(2*self.ms) + self.lf**2/self.Iy)
        m_eff_rear = 1/(1/(2*self.ms) + self.lr**2/self.Iy)
        self.c_heave_f = 2*self.zeta*np.sqrt(self.k_heave_f*m_eff_front)
        self.c_heave_r = 2*self.zeta*np.sqrt(self.k_heave_r*m_eff_rear)

        k_front = 0
        k_rear = 0
        MR_corner = 1

        # 直推的避震器
        self.k_front = k_front*MR_corner**2
        self.k_rear = k_rear*MR_corner**2# 確保左右相同
        self.c_front = 2*self.zeta*np.sqrt(self.k_front*m_eff_front)
        self.c_rear = 2*self.zeta*np.sqrt(self.k_rear*m_eff_rear)

        self.kt = 80000   # tire stiffness
        self.ct = 50       # tire damping (盡量保留一個小數字)

class Suspension_full:

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

