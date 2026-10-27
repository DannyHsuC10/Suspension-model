import numpy as np
import json

class quarter_suspension_state:
    
    def __init__(self,dt = 0.001):

        self.t = 0

        self.dt = dt
        self.zs = 0       # sprung displacement
        self.zu = 0       # unsprung displacement

        self.vzs = 0      # sprung velocity
        self.vzu = 0      # unsprung velocity

        self.azs = 0
        self.azu = 0

        self.Fs = 0
        self.Ft = 0
        # 路面資訊
        self.vzr = 0
        self.zr = 0

    def Integration(self,sus):
        self.t += self.dt
        
        # semi implicit Euler
        self.vzs += self.azs*self.dt
        self.vzu += self.azu*self.dt

        self.zs += self.vzs*self.dt
        self.zu += self.vzu*self.dt

class LR_suspension_state:
    
    def __init__(self,left,right,dt = 0.001):
        self.dt = dt
        left.dt = self.dt
        right.dt = self.dt
        
        self.left = left
        self.right = right
        
        self.t = 0.0
        self.ay = 0.0

        # Modal information
        self.F_heave = 0.0
        self.F_roll = 0.0

        self.Fz = 0.0
        self.Mx = 0.0

        self.az_body = 0.0
        self.alpha_roll = 0.0

        # derived self
        self.heave = 0.0
        self.heave_rate = 0.0

        self.roll = 0.0
        self.roll_rate = 0.0

        self.Fs_l = 0.0
        self.Fs_r = 0.0

        self.Ft_l = 0.0
        self.Ft_r = 0.0

        self.dF_roll = 0.0

        self.dF_roll_l = 0.0
        self.dF_roll_r = 0.0

        self.d_frame = 0.0      # 車架左右變形位移
        self.v_frame = 0.0      # 車架變形速度
        self.a_frame = 0.0

    def Integration(self,sus):
        
        self.t += self.dt

        self.left.Integration(sus)
        self.right.Integration(sus)

        # derived

        self.heave = (self.left.zs+self.right.zs)/2

        self.heave_rate = (self.left.vzs+self.right.vzs)/2

        self.roll_rate += self.alpha_roll*self.dt

        self.roll += self.roll_rate*self.dt

        self.v_frame += self.a_frame*self.dt
        self.d_frame += self.v_frame*self.dt

class FR_suspension_state:
    
    def __init__(self,front,rear,dt = 0.001):

        self.dt = dt

        front.dt = self.dt
        rear.dt = self.dt

        self.front = front
        self.rear = rear


        # ===============================
        # Time
        # ===============================

        self.t = 0.0


        # ===============================
        # Vehicle longitudinal input
        # ===============================

        self.ax = 0.0

        # ===============================
        # Body dynamics
        # ===============================

        self.Fz = 0.0
        self.My = 0.0

        self.az_body = 0.0
        self.alpha_pitch = 0.0


        # ===============================
        # Tire / longitudinal force
        # ===============================

        self.Fx_front = 0.0
        self.Fx_rear = 0.0


        # ===============================
        # Derived body motion
        # ===============================

        self.heave = 0.0
        self.heave_rate = 0.0

        self.pitch = 0.0
        self.pitch_rate = 0.0



    def Integration(self,sus):

        self.t += self.dt


        # ===============================
        # integrate corner states
        # ===============================

        self.front.Integration(sus)
        self.rear.Integration(sus)

        # ===============================
        # derived
        # ===============================

        self.heave = (
            self.front.zs*sus.lr
            +
            self.rear.zs*sus.lf
        ) / 2 / sus.l


        self.heave_rate = (
            self.front.vzs*sus.lr
            +
            self.rear.vzs*sus.lf
        ) / 2 /sus.l

        # pitch
        #
        # positive:
        # front higher than rear
        #

        self.pitch_rate += self.alpha_pitch*self.dt
        self.pitch += self.pitch_rate*self.dt


class full_suspension_state:
     
    def __init__(self,sus,dt = 0.001):

        self.t = 0

        self.dt = dt

        # wheel
        # =============================================
        # 簧上 & 簧下
        self.zs = np.zeros(4)# 簧上

        self.zu =  np.zeros(4) # 簧下

        self.vzs = np.zeros(4)
        self.vzu = np.zeros(4)

        self.azs = np.zeros(4)
        self.azu = np.zeros(4)# << 輸入
        
        # 對簧上下的力量，並不是正向力量
        self.Fs = np.zeros(4)# << 輸入
        self.Ft = np.zeros(4)# << 輸入
        
        # 路面資訊
        self.vzr = np.zeros(4)# << 輸入
        self.zr = np.zeros(4)# << 輸入

        # 4輪力量
        self.Fx = np.zeros(4)# << 輸入
        self.Fy = np.zeros(4)# << 輸入
        self.Fz = np.zeros(4)# << 輸入
        self.Mx_w = np.zeros(4)# << 輸入
        self.Mz_w = np.zeros(4)# << 輸入
        self.RRT_w = np.zeros(4)# << 輸入


        # 輪胎轉動
        self.omega_w = np.zeros(4)
        self.alpha_w = np.zeros(4)

        self.IA = np.zeros(4)
        self.steer = np.zeros(4)

        self.SL = np.zeros(4)
        self.SA = np.zeros(4)

        # 正向力量
        self.N = np.zeros(4)

        # body
        # ==================================================
        # 車身轉動(Xf,Xr,Y,Z)
        self.M = np.zeros(3)# << 外界輸入
        self.theta = np.zeros(4)  # Roll Pitch Yaw
        self.omega = np.zeros(4)  # Angular velocity
        self.alpha = np.zeros(4)  # Angular acceleration # << 輸入

        # CG 變化
        self.F_cg = np.zeros(3)# << 輸入
        self.s = np.zeros(3)      # [x, y, z]
        self.v = np.zeros(3)      # [vx, vy, vz]
        self.a = np.zeros(3)      # [ax, ay, az]# << 輸入

        # other
        # ===================================================
        # 避震器力量
        self.F_heave_f = 0.0
        self.F_heave_r = 0.0
        self.F_roll_f = 0.0
        self.F_roll_r = 0.0

        self.F_roll_c = 0.0
        self.F_warp = 0.0

        self.F_corner = np.zeros(4)

        # 踏板命令
        self.gas_cmd = 0.0
        self.brake_cmd = 0.0
        

    def Integration(self, sus):

        dt = self.dt
        
        # Time
        self.t += dt

        # Sprung mass(避免兩次積分造成誤差)
        self.azs = self.body_to_corner(self.a,self.alpha,sus)
        self.vzs = self.body_to_corner(self.v,self.omega,sus)
        self.zs  = self.body_to_corner(self.s,self.theta,sus)

        # Unsprung mass
        self.vzu += self.azu * dt
        self.zu  += self.vzu * dt

        # Vehicle attitude
        self.omega += self.alpha * dt
        self.theta += self.omega * dt
        #print(np.rad2deg(self.theta))
        # CG motion
        self.v += self.a * dt
        self.s += self.v * dt

        # wheel
        self.omega_w += self.alpha_w * dt

    @staticmethod
    def body_to_corner(s,theta,sus):# 車身姿態轉到角落
        """用於四分之一模型可以使用檢視"""
        z = s[2]

        fl = z - theta[2]*sus.lf + theta[0]*sus.tf/2
        fr = z - theta[2]*sus.lf - theta[0]*sus.tf/2
        rl = z + theta[2]*sus.lr + theta[1]*sus.tr/2
        rr = z + theta[2]*sus.lr - theta[1]*sus.tr/2

        zs = np.array([fl,fr,rl,rr])

        return zs

    def save_state_json(self, filename):# 儲存狀態
        """
        可以讓運作到穩態後存起來觀察情況
        """
        
        def convert(obj):

            if isinstance(obj, np.ndarray):
                return convert(obj.tolist())

            if isinstance(obj, np.generic):
                return convert(obj.item())

            if isinstance(obj, dict):
                return {
                    k: convert(v)
                    for k, v in obj.items()
                }

            if isinstance(obj, list):
                return [
                    convert(v)
                    for v in obj
                ]

            if isinstance(obj, (float, int)):

                if abs(obj) < 1e-6:
                    return 0.0

                return round(obj, 6)

            if hasattr(obj, "__dict__"):
                return convert(obj.__dict__)

            return obj


        with open(filename, "w") as f:
            json.dump(
                convert(self),
                f,
                indent=4
            )

    def load_state_json(self, filename):
        """
        從 json 載入 state 狀態
        """

        def convert(obj):

            if isinstance(obj, list):
                return np.array(obj)

            if isinstance(obj, dict):
                return {
                    k: convert(v)
                    for k, v in obj.items()
                }

            return obj


        with open(filename, "r") as f:
            data = json.load(f)


        data = convert(data)


        for key, value in data.items():

            if hasattr(self, key):
                setattr(self, key, value)

