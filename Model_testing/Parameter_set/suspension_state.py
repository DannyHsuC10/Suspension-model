
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



