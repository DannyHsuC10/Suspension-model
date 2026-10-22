import numpy as np

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

