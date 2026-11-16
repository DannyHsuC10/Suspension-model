
def Half_suspension_2DOF_model(sus, state):# 2DOF1/2懸吊模型
    """
    不考慮簧下 & anti 的簡化模型
    """
    front = state.front
    rear = state.rear

    # ==================================
    # Suspension displacement
    # ==================================
    ds_f = front.zs - front.zr
    ds_r = rear.zs - rear.zr

    dvs_f = front.vzs - front.vzr
    dvs_r = rear.vzs - rear.vzr


    F_heave_f = (
        sus.k_heave_f * ds_f
        + sus.c_heave_f * dvs_f
    )

    F_heave_r = (
        sus.k_heave_r * ds_r
        + sus.c_heave_r * dvs_r
    )

    Fq_f = (
        sus.k_front * ds_f
        + sus.c_front * dvs_f
    )

    Fq_r = (
        sus.k_rear * ds_r
        + sus.c_rear * dvs_r
    )
    # 幾乎等效果
    Fs_f = F_heave_f/2 + Fq_f
    Fs_r = F_heave_r/2 + Fq_r

    # ==================================
    # Virtual body dynamics
    # ==================================
    My_spring = (Fs_r * sus.lr-Fs_f * sus.lf)

    My_inertia = state.ax*(sus.m_corner*2)*(sus.h_cg)

    My = My_spring + My_inertia
    

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

    # ==================================
    # State output
    # ==================================

    state.front.azs = azsf
    state.rear.azs  = azsr

    state.front.azu = azsf
    state.rear.azu  = azsr

    state.front.Fs = Fs_f
    state.rear.Fs  = Fs_r

    state.front.Ft = Fs_f
    state.rear.Ft  = Fs_r

    state.Fz = Fz
    state.My = My

    state.az_body = az_body
    state.alpha_pitch = alpha_pitch

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


    F_heave_f = (
        sus.k_heave_f * ds_f
        + sus.c_heave_f * dvs_f)

    F_heave_r = (
        sus.k_heave_r * ds_r
        + sus.c_heave_r * dvs_r)

    Fq_f = (
        sus.k_front * ds_f
        + sus.c_front * dvs_f)

    Fq_r = (
        sus.k_rear * ds_r
        + sus.c_rear * dvs_r)
    
    # 幾乎等效果
    Fs_f = F_heave_f + Fq_f
    Fs_r = F_heave_r + Fq_r

    # ==================================
    # Tire
    # ==================================

    dt_f = front.zu - front.zr
    dt_r = rear.zu - rear.zr

    Ft_f = -sus.kt*dt_f + sus.ct*(front.vzu-front.vzr) + Fs_f
    Ft_r = -sus.kt*dt_r + sus.ct*(rear.vzu-rear.vzr) + Fs_r

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
    print(My_spring,My_inertia,M_geo,My)
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


