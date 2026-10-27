import numpy as np
print("Suspension model Ready!!!")
print("""

                     Road input
                         |
                         v
              +--------------------+
              |  Tire KC force     |
              | tires_KC_force()   |
              +--------------------+
                         |
                         v
              +--------------------+
              | Unsprung dynamics  |
              |   azu = Ft/mu      |
              +--------------------+
                         |
                         v
              +--------------------+
              | relative_motion()  |
              | dz , dv            |
              +--------------------+
                         |
             -----------------------------
             |                           |
             v                           v
 +----------------------+      +----------------------+
 | Suspension KC model  |      | Geometry force       |
 | shocks_KC_force()    |      | rc_force()           |
 |                      |      | anti_force()         |
 +----------------------+      +----------------------+
             |                           |
             v                           |
 +----------------------+                |
 | Hydraulic_system()   |                |
 | Modal coordinates    |                |
 |                      |                |
 |  Heave               |                |
 |  Roll                |                |
 |  Warp                |                |
 +----------------------+                |
             |                           |
             v                           |
 +----------------------+                |
 | get_KC_force()       |                |
 | Spring + Damper      |                |
 +----------------------+                |
             |                           |
             v                           |
 +----------------------+                |
 | Modal_to_corner()    |<---------------
 | Modal -> FL FR RL RR |
 +----------------------+
             |
             v
 +----------------------+
 | corner_to_body()     |
 | Fz + Mx My Mz        |
 +----------------------+
             |
             v
 +----------------------+
 | external_moment()    |
 | CG force moment      |
 +----------------------+
             |
             v
 +----------------------+
 | Body dynamics        |
 | az_body              |
 | alpha                |
 +----------------------+
             |
             v
 +----------------------+
 | body_to_corner()     |
 | sprung acceleration  |
 +----------------------+
             |
             v
          State update
""")


def relative_motion(z1,z2,v1,v2):# 相對運動(v & s)
    dz = z1 - z2
    dv = v1 - v2

    return dz,dv

# ==========================================================
# tire
def tires_KC_force(tires,zu,zr,vzu,vzr):# 輪胎kc力量
    dz,dv = relative_motion(zu,zr,vzu,vzr)

    F = np.empty(len(tires))
    results = []

    for i, (tire, x, v) in enumerate(zip(tires, dz, dv)):
        F[i], result = tire.force(x, v)
        results.append(result)

    return F, results

# ==========================================================
# chassis_torsion

def chassis_torsion(state, sus):
    phi_front = state.theta[0]
    phi_rear = state.theta[1]

    omega_front = state.omega[0]
    omega_rear = state.omega[1]

    twist = phi_front - phi_rear
    twist_rate = omega_front - omega_rear

    M_twist,_ = sus.body_twist.force(twist, twist_rate)

    return M_twist

# ==========================================================
# shocks
def Hydraulic_system(fl,fr,rl,rr):# 液壓系統
    heave_f = (fl+fr)
    heave_r = (rl+rr)
    
    roll_f = (fl-fr)
    roll_r = (rl-rr)

    roll_c = (roll_f+roll_r)
    warp = (roll_f-roll_r)

    return heave_f,heave_r,roll_f,roll_r,roll_c,warp

def shocks_KC_force(shock_list, zs, zu, vzs, vzu):
    """
    陣列順序
    heave_f, heave_r, roll_f, roll_r,
    roll_c, warp,
    fl, fr, rl, rr
    """

    dz, dv = relative_motion(zs, zu, vzs, vzu)

    heave_f, heave_r, roll_f, roll_r, roll_c, warp = Hydraulic_system(
        dz[0], dz[1], dz[2], dz[3]
    )

    v_heave_f, v_heave_r, v_roll_f, v_roll_r, v_roll_c, v_warp = Hydraulic_system(
        dv[0], dv[1], dv[2], dv[3]
    )


    displacement_list = np.array([
        heave_f, heave_r,
        roll_f, roll_r,
        roll_c, warp,
        dz[0], dz[1], dz[2], dz[3]
    ])

    velocity_list = np.array([
        v_heave_f, v_heave_r,
        v_roll_f, v_roll_r,
        v_roll_c, v_warp,
        dv[0], dv[1], dv[2], dv[3]
    ])

    F = np.empty(len(shock_list))
    results = []

    for i, (tire, x, v) in enumerate(zip(shock_list, displacement_list, velocity_list)):
        F[i], result = tire.force(x, v)
        results.append(result)

    return F, results

# ==========================================================
# external
def external_moment(F_cg,M,sus):

    # CG 相對於參考點的位置
    r = np.array([0,0,sus.h_cg])
    M_by_f = np.cross(r, -F_cg)

    moment = M_by_f + M # 其中M是外界扭矩預設基本為 0
    
    moment = np.array([moment[0]*sus.I_rate[0],moment[0]*sus.I_rate[1],moment[1],moment[2]])

    return moment

# ==========================================================
# Modal transf
def Modal_to_corner(F_kc,state):# 轉換避震器力量>>角落力量
    """
    F_kc : 彈簧力量傳遞到四個角對抗姿態變化的扭矩
    """
    F_heave_f,F_heave_r,F_roll_f,F_roll_r,F_roll_c,F_warp,F_fl,F_fr,F_rl,F_rr = F_kc # 解包處理
    
    Fs_fl = F_heave_f + F_roll_f + F_fl + F_roll_c + F_warp
    Fs_fr = F_heave_f - F_roll_f + F_fr - F_roll_c - F_warp
    Fs_rl = F_heave_r + F_roll_r + F_rl + F_roll_c - F_warp
    Fs_rr = F_heave_r - F_roll_r + F_rr - F_roll_c + F_warp

    # 避震器力量
    state.F_heave_f = F_heave_f
    state.F_heave_r = F_heave_r
    state.F_roll_f = F_roll_f
    state.F_roll_r = F_roll_r

    state.F_roll_c = F_roll_c
    state.F_warp = F_warp

    state.F_corner = np.array([F_fl,F_fr,F_rl,F_rr])
    
    return  Fs_fl, Fs_fr, Fs_rl, Fs_rr

def corner_to_body(Fs_fl, Fs_fr, Fs_rl, Fs_rr,sus):# 角落力量作用在車身影響

    Fs_list = np.array([Fs_fl, Fs_fr, Fs_rl, Fs_rr])# 未經過車體耦合影響
    
    Fs_rear = Fs_rl + Fs_rr
    Fs_front = Fs_fl + Fs_fr
    dF_front = Fs_fl - Fs_fr
    dF_rear = Fs_rl - Fs_rr
    
    Fz_spring = sum(Fs_list)

    My_spring = -(Fs_front * sus.lf-Fs_rear * sus.lr)# pitch

    Mx_f_spring = dF_front * sus.tf
    Mx_r_spring = dF_rear * sus.tr# roll

    M_spring = np.array([Mx_f_spring,Mx_r_spring,My_spring,0])
    
    return Fz_spring,M_spring,Fs_list

# =================================================
# sprung & unsprung

def sprung_moment(M_ext ,M_spring, M_geo, state, sus): # 簧上
    """
    幾何轉移先轉移部份力矩
    """
    M_twist = chassis_torsion(state, sus)

    moment = M_ext + M_spring - M_geo

    # chassis torsion
    moment[0] += M_twist
    moment[1] -= M_twist

    print("M",M_ext,M_spring,M_geo,moment)
    
    return moment


def unsprung_force(F_geo , F_kc , Fs): #簧下
    """
    F_rc : 必須要保持力矩平衡,負回受自動會扣除rc先傳過去的力量,下一個步階會去壓縮輪胎重新達到系統平衡
    """

    Ft = F_kc - Fs + F_geo

    print("F",F_kc,Fs,F_geo)

    return Ft

# ==========================================================
# motion
def Suspension_output(state,sus):# 懸吊力量輸出
    
    Fs_kc,results_sus = shocks_KC_force(sus.shock_list,state.zs,state.zu,state.vzs,state.vzu)# 懸吊彈簧力量
    
    Fs_fl, Fs_fr, Fs_rl, Fs_rr = Modal_to_corner(Fs_kc,state)# 轉換角落

    Fz_spring,M_spring,Fs = corner_to_body(Fs_fl, Fs_fr, Fs_rl, Fs_rr,sus)# 轉換車體

    Fz = Fz_spring + state.F_cg[2]
    
    # -----------------------------------------
    F_geo, M_geo = sus.geometry.force(state)# 幾何力量
    
    M_ext = external_moment(state.F_cg,state.M,sus)# 外界力

    M = sprung_moment(M_ext ,M_spring , M_geo, state, sus)# 合力矩
    # -----------------------------------------

    Ft_kc, results_tir = tires_KC_force(sus.tire_list,state.zu,state.zr,state.vzu,state.vzr)# 輪胎彈簧力量

    Ft = unsprung_force(F_geo , Ft_kc , Fs)# 合力量

    N = tire_normal_force(Ft_kc, sus)
    #results_sus,results_tir# 資料(之後處理)

    return Fz,M,Ft,N,Fs


def tire_normal_force(Ft_kc, sus):
    """
    回傳輪胎真正的法向力
    """

    N = np.maximum(Ft_kc, np.zeros(4))

    N = Ft_kc
    return N

def motion(state,sus):# 加速度運算

    Fz,M,Ft,N,Fs = Suspension_output(state,sus)
    
    az_body = Fz/sus.m

    alpha = M/sus.I
    
    azu = Ft/sus.mu_list

    #print(Ft,sum(Ft))# 其中Ft 是作用於簧下的力量平衡之後正常會接近0

    state.azu = azu
    state.alpha = alpha
    state.a[2] = az_body
    state.N = N
    state.Ft = N
    state.Fs = Fs
    # 統一在state 中處理
    #azs = body_to_corner(az_body,alpha,sus)
    #state.azs = azs


