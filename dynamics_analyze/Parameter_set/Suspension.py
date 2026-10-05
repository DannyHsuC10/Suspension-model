import numpy as np

class SuspensionQuarter:

    def __init__(self):

        self.ms = 67      # sprung mass
        self.mu = 13      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量

        # roll 40000
        # heave 70000
        # heave tender = 28000
        # ks avg = 34000
        self.ks = 40000   # 懸吊 stiffness

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

class SuspensionQuartertender:

    def __init__(self):

        self.ms = 67      # sprung mass
        self.mu = 13      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量

        # roll 40000
        # heave 70000
        # heave tender = 28000
        # ks avg = 34000
        self.ks = 34000   # 懸吊 stiffness

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

class SuspensionQuarteravg:

    def __init__(self):

        self.ms = 67      # sprung mass
        self.mu = 13      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量

        # roll 40000
        # heave 70000
        # heave tender = 28000
        # ks avg = 34000
        self.ks = 40000   # 懸吊 stiffness

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

class SuspensionQuarterheave:

    def __init__(self):

        self.ms = 67      # sprung mass
        self.mu = 13      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量

        # roll 40000
        # heave 70000
        # heave tender = 28000
        # ks avg = 34000
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
        self.mu = 15      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量
        
        self.h_rc = 0.0
        self.h_cg = 0.3

        self.tau_rc = 0.005

        c_heave = 6000
        c_roll = 1200

        k_heave = 18000
        k_roll = 40000#*0.8

        MR_heave = 1
        MR_roll = 1

        # 模態避震器
        self.k_heave = k_heave*MR_heave**2
        self.k_roll = k_roll*MR_roll**2

        self.c_heave = c_heave*MR_heave**2
        self.c_roll = c_roll*MR_roll**2

        k_corner = 0
        c_corner = 0
        MR_corner = 1

        # 直推的避震器
        self.k_left = k_corner*MR_corner**2
        self.k_right = k_corner*MR_corner**2# 確保左右相同
        self.c_left = c_corner*MR_corner**2
        self.c_right = c_corner*MR_corner**2# 確保左右相同

        #self.kw = (self.k_heave+self.k_roll)/2+self.k_left# 直推以左側為主(基本上左右相同)   # 懸吊 stiffness
        #self.cw = (self.c_heave+self.c_roll)/2+self.c_left    # 單角 damping

        self.kt = 1000000   # tire stiffness
        self.ct = 50       # tire damping (盡量保留一個小數字)

        self.k_frame = k_roll*10      # N/m

        self.m_frame = 65     # kg (虛擬質量)
        zeta = 0
        self.c_frame = 2*zeta*np.sqrt(self.k_frame*self.m_frame)
        print(self.c_frame)

class SuspensionFR:
    
    def __init__(self):

        self.Iy = 50
        self.ms = 65      # sprung mass
        self.mu = 15      # unsprung mass
        self.m_corner = self.ms+self.mu # 整體質量

        self.lf = 0.765
        self.lr = 0.765
        self.h_cg = 0.3
        self.l = self.lf+self.lr

        self.anti_dive_front =0
        self.anti_squat_rear =0
        #self.anti = 0.15

        c_heave_f = 1500
        k_heave_f = 30000
        c_heave_r = 1200
        k_heave_r = 28000

        MR_heave_f = 1
        MR_heave_r = 1

        # 模態避震器
        self.k_heave_f = k_heave_f*MR_heave_f**2
        self.c_heave_f = c_heave_f*MR_heave_f**2
        self.k_heave_r = k_heave_r*MR_heave_r**2
        self.c_heave_r = c_heave_r*MR_heave_r**2

        k_front = 0
        c_front = 0
        k_rear = 0
        c_rear = 0
        MR_corner = 1

        # 直推的避震器
        self.k_front = k_front*MR_corner**2
        self.k_rear = k_rear*MR_corner**2# 確保左右相同
        self.c_front = c_front*MR_corner**2
        self.c_rear = c_rear*MR_corner**2# 確保左右相同

        self.kt = 80000   # tire stiffness
        self.ct = 50       # tire damping (盡量保留一個小數字)

