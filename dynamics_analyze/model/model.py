"""Dynamics equations using KCSystem.force(displacement, velocity).

KC returns the restoring force -(k*x + c*v). Fs/Ft in the quarter and
left/right models retain their original reaction-force convention, so those
values are the negative of the KC force. Motion ratios are handled inside KC.
"""

def Quarter_suspension_model(sus,state):# 1/4懸吊模型
    """
    雙彈簧阻尼模型
    """
    sus_dz = state.zs-state.zu # 懸吊相對位移
    sus_dvz = state.vzs-state.vzu # 懸吊相對速度
    F_suspension, _ = sus.corner.force(sus_dz, sus_dvz)
    Fs = -F_suspension # 保留原本 Fs 的反力正負號

    tire_dz = state.zu-state.zr # 輪胎相對位移
    tire_dvz = state.vzu-state.vzr # 輪胎相對速度
    F_tire, _ = sus.tire.force(tire_dz, tire_dvz)
    Ft = -F_tire

    azs = -Fs/sus.ms # 簧上加速度
    azu = (Fs-Ft)/sus.mu # 簧下加速度

    state.azs = azs
    state.azu = azu
    state.Fs = Fs
    state.Ft = Ft

def Half_rollcenter_LR_model(sus, state):# 1/2懸吊模型(4DOF)
    """
    for roll center test
    加入車輛剛性傳遞延遲系統
    """

    left = state.left
    right = state.right


    # Suspension
    ds_l = left.zs-left.zu
    ds_r = right.zs-right.zu

    dvs_l = left.vzs-left.vzu
    dvs_r = right.vzs-right.vzu


    # Modal suspension
    d_heave = (ds_l+ds_r)
    d_roll = (ds_r-ds_l)

    dv_heave = (dvs_l+dvs_r)
    dv_roll = (dvs_r-dvs_l)

    F_heave_kc, _ = sus.heave.force(d_heave, dv_heave)
    F_roll_kc, _ = sus.roll.force(d_roll, dv_roll)
    F_heave = -F_heave_kc
    F_roll = -F_roll_kc

    # optional corner stiffness
    Fq_l_kc, _ = sus.corner_left.force(ds_l, dvs_l)
    Fq_r_kc, _ = sus.corner_right.force(ds_r, dvs_r)
    Fq_l = -Fq_l_kc
    Fq_r = -Fq_r_kc

    # modal -> corner
    Fs_l = F_heave/2 - F_roll/2 + Fq_l
    Fs_r = F_heave/2 + F_roll/2 + Fq_r


    # Tire
    dt_l = left.zu-left.zr
    dt_r = right.zu-right.zr

    dvt_l = left.vzu-left.vzr
    dvt_r = right.vzu-right.vzr

    Ft_l_kc, _ = sus.tire_left.force(dt_l, dvt_l)
    Ft_r_kc, _ = sus.tire_right.force(dt_r, dvt_r)
    Ft_l = -Ft_l_kc
    Ft_r = -Ft_r_kc



    # Roll center
    # 理論幾何力
    
    Mx_rc = -state.ay*(sus.m_corner*2)*(sus.h_rc)

    F_frame_cmd = Mx_rc / sus.track

    F_frame, _ = sus.frame.force(state.d_frame, state.v_frame)
    state.a_frame = (F_frame_cmd + F_frame) / sus.m_frame

    dF_roll = -F_frame

    state.dF_roll = dF_roll

    # Virtual body dynamics
    Fz = Fs_l+Fs_r
    
    M_spring = (Fs_r-Fs_l)*sus.track/2

    M_ext = state.ay*(sus.m_corner*2)*(sus.h_cg)

    Mx = M_spring+M_ext+Mx_rc# 總側傾力矩

    #print("M_spring:",M_spring,"M_ext:",M_ext,"Mx_rc:",Mx_rc)

    az_body = -Fz/(sus.ms*2)

    alpha_roll = Mx/sus.Ix

    azsl = (az_body+sus.track/2*alpha_roll)
    azsr = (az_body-sus.track/2*alpha_roll)

    # Unsprung
    azul = (Fs_l-Ft_l-dF_roll)/sus.mu
    azur = (Fs_r-Ft_r+dF_roll)/sus.mu


    # State output
    state.left.azs = azsl
    state.right.azs = azsr

    state.left.azu = azul
    state.right.azu = azur

    state.left.Fs = Fs_l
    state.right.Fs = Fs_r

    state.left.Ft = Ft_l
    state.right.Ft = Ft_r

    # 為了方便把資料提到物件上層
    state.Fs_l = Fs_l
    state.Fs_r = Fs_r

    state.Ft_l = Ft_l
    state.Ft_r = Ft_r

    state.dF_roll_l = -dF_roll
    state.dF_roll_r = dF_roll

    # body record
    state.Fz = Fz
    state.Mx = Mx

    state.az_body = az_body
    state.alpha_roll = alpha_roll

    state.F_heave = F_heave
    state.F_roll = F_roll

def Half_suspension_model(sus, state):# 1/2懸吊模型(4DOF)
    """有 anti 的4DOF模型"""
    front = state.front
    rear = state.rear

    # ==================================
    # Suspension displacement
    # ==================================
    ds_f = front.zs - front.zu
    ds_r = rear.zs - rear.zu

    dvs_f = front.vzs - front.vzu
    dvs_r = rear.vzs - rear.vzu


    F_heave_f_kc, _ = sus.heave_f.force(ds_f, dvs_f)
    F_heave_r_kc, _ = sus.heave_r.force(ds_r, dvs_r)
    Fq_f_kc, _ = sus.corner_front.force(ds_f, dvs_f)
    Fq_r_kc, _ = sus.corner_rear.force(ds_r, dvs_r)

    F_heave_f = -F_heave_f_kc
    F_heave_r = -F_heave_r_kc
    Fq_f = -Fq_f_kc
    Fq_r = -Fq_r_kc
    
    # 幾乎等效果
    Fs_f = F_heave_f + Fq_f
    Fs_r = F_heave_r + Fq_r

    # ==================================
    # Tire
    # ==================================

    dt_f = front.zu - front.zr
    dt_r = rear.zu - rear.zr

    # KC tire forces act directly on the unsprung masses. In particular,
    # damping opposes vzu-vzr (the old expression used the opposite sign).
    F_tire_f, _ = sus.tire_front.force(dt_f, front.vzu - front.vzr)
    F_tire_r, _ = sus.tire_rear.force(dt_r, rear.vzu - rear.vzr)
    Ft_f = F_tire_f + Fs_f
    Ft_r = F_tire_r + Fs_r

    # ==================================
    # anti
    # ==================================
    My_inertia = state.ax*(sus.m_corner*2)*(sus.h_cg)

    if state.ax>0 :
        anti = sus.anti_dive_front
    else:
        anti = sus.anti_squat_rear
    M_geo = My_inertia*anti

    Ft_f += state.Fx_front*sus.h_cg/sus.l# 上態車身對輪胎力量是負的
    Ft_r += -state.Fx_rear*sus.h_cg/sus.l
   
    
    # ==================================
    # Virtual body dynamics
    # ==================================
    My_spring = (Fs_r * sus.lr-Fs_f * sus.lf)
    


    My =  My_spring+ My_inertia - M_geo
    Fz = Fs_f + Fs_r  
    
    az_body = -Fz/(sus.ms*2)

    alpha_pitch = My/sus.Iy

    azsf = (
        az_body
        + sus.lf * alpha_pitch
    )

    azsr = (
        az_body
        - sus.lr * alpha_pitch
    )

    azuf = (Ft_f)/sus.mu
    azur = (Ft_r)/sus.mu


    # ==================================
    # State output
    # ==================================

    state.front.azs = azsf
    state.rear.azs  = azsr

    state.front.azu = azuf
    state.rear.azu  = azur

    state.front.Fs = Fs_f
    state.rear.Fs  = Fs_r

    state.front.Ft = Ft_f
    state.rear.Ft  = Ft_r

    state.Fz = Fz
    state.My = My

    state.az_body = az_body
    state.alpha_pitch = alpha_pitch
