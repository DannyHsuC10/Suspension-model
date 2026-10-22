
# 雙質點無偶和
def two_mass_suspension_2DOF_model(sus, state):# 2質點2DOF模型(2DFO)
    """
    1/2 Car Suspension (Modal Model)

    Heave mode
    Roll mode
    """
    left = state.left
    right = state.right
    
    # Wheel displacement
    dz_l = left.zs - left.zr
    dz_r = right .zs - right.zr

    dvs_l = left.vzs
    dvs_r = right.vzs

    # Modal displacement
    d_heave = (dz_l + dz_r)
    d_roll  = (dz_r - dz_l)

    dv_heave = (dvs_l + dvs_r) 
    dv_roll  = (dvs_r - dvs_l)

    # force==============================
    # Modal force
    F_heave = (sus.k_heave * d_heave +sus.c_heave * dv_heave)
    F_roll = (sus.k_roll * d_roll +sus.c_roll * dv_roll)
    # quarter force
    Fq_l = (sus.k_left * dz_l +sus.c_left * dvs_l)
    Fq_r = (sus.k_right * dz_r +sus.c_right * dvs_r)
    
    # Transform back
    Fs_l = F_heave - F_roll + Fq_l
    Fs_r = F_heave + F_roll + Fq_r

    # Vehicle dynamics
    azsl = -Fs_l / sus.m_corner
    azsr = -Fs_r / sus.m_corner
    
    # acc
    Ft_l = Fs_l
    Ft_r = Fs_r

    azul = azsl
    azur = azsr

    # Left corner
    state.left.azs = azsl
    state.left.azu = azul

    state.left.Fs = Fs_l
    state.left.Ft = Ft_l

    # Right corner
    state.right.azs = azsr
    state.right.azu = azur

    state.right.Fs = Fs_r
    state.right.Ft = Ft_r

    # Modal force (如果需要記錄)
    state.F_heave = F_heave
    state.F_roll = F_roll

def two_mass_suspension_model(sus, state):# 雙質點模型無側傾(4DOF)
    """
    4 DOF Half-car Suspension

    DOF
    ----
    zsl : sprung left
    zsr : sprung right
    zul : unsprung left
    zur : unsprung right
    """

    # ==========================
    # Suspension deformation
    # ==========================
    left = state.left
    right = state.right

    ds_l = left.zs - left.zu
    ds_r = right.zs - right.zu

    dvs_l = left.vzs - left.vzu
    dvs_r = right.vzs - right.vzu

    # ==========================
    # Modal suspension
    d_heave = (ds_l + ds_r)
    d_roll  = (ds_r - ds_l)

    dv_heave = (dvs_l + dvs_r)
    dv_roll  = (dvs_r - dvs_l)

    # quarter force
    Fq_l = (sus.k_left * ds_l +sus.c_left * dvs_l)
    Fq_r = (sus.k_right * ds_r +sus.c_right * dvs_r)
    # ==========================
    # Suspension force
    F_heave = (sus.k_heave * d_heave +sus.c_heave * dv_heave)
    F_roll = (sus.k_roll * d_roll +sus.c_roll * dv_roll)

    # Transform back

    Fs_l = F_heave/2 - F_roll/2 +Fq_l
    Fs_r = F_heave/2 + F_roll/2 +Fq_r

    # ==========================
    # Tire force
    dt_l = left.zu - left.zr
    dt_r = right.zu - right.zr

    dvt_l = left.vzu-left.vzr
    dvt_r = right.vzu-left.vzr

    Ft_l = (sus.kt * dt_l +sus.ct * dvt_l)
    Ft_r = (sus.kt * dt_r +sus.ct * dvt_r)

    # ==========================
    # Equations of motion
    # ==========================

    # Sprung mass
    azsl = -Fs_l / sus.ms
    azsr = -Fs_r / sus.ms

    # Unsprung mass
    azul = (Fs_l - Ft_l) / sus.mu
    azur = (Fs_r - Ft_r) / sus.mu

    # Left corner
    state.left.azs = azsl
    state.left.azu = azul

    state.left.Fs = Fs_l
    state.left.Ft = Ft_l

    # Right corner
    state.right.azs = azsr
    state.right.azu = azur

    state.right.Fs = Fs_r
    state.right.Ft = Ft_r

    # Modal force (如果需要記錄)
    state.F_heave = F_heave
    state.F_roll = F_roll

# 車體耦合
def Half_suspension_2DOF_model(sus, state):# 2DOF1/2懸吊模型
    """
    1/2 Car Suspension (Modal Model)

    Heave mode
    Roll mode

    Virtual rigid body coupling

    without roll center
    """

    left = state.left
    right = state.right


    # Suspension displacement

    dz_l = left.zs - left.zr
    dz_r = right.zs - right.zr

    dvs_l = left.vzs - left.vzr
    dvs_r = right.vzs - right.vzr

    # Modal coordinate
    d_heave = (dz_l + dz_r) 
    d_roll  = (dz_r - dz_l)

    dv_heave = (dvs_l + dvs_r)
    dv_roll  = (dvs_r - dvs_l)



    # Modal force
    F_heave = (sus.k_heave*d_heave+ sus.c_heave*dv_heave)
    F_roll = (sus.k_roll*d_roll+ sus.c_roll*dv_roll)

    # optional corner stiffness
    Fq_l = (sus.k_left*dz_l+ sus.c_left*dvs_l)
    Fq_r = (sus.k_right*dz_r+ sus.c_right*dvs_r)


    # Transform modal -> corner force
    Fs_l = F_heave/2 - F_roll/2 + Fq_l
    Fs_r = F_heave/2 + F_roll/2 + Fq_r


    # Virtual rigid body coupling
    #========================================
    # total vertical force
    Fz = Fs_l + Fs_r

    # roll moment
    Mx_roll = state.ay*(sus.m_corner*2)*(sus.h_cg)
    Mx_spring = (Fs_r - Fs_l) * sus.track/2
    Mx = Mx_spring+Mx_roll

    # body acceleration
    az_body = -Fz / (sus.ms*2)
    alpha_roll = Mx / sus.Ix

    # map back to virtual corner
    azsl = (az_body+ sus.track/2*alpha_roll)
    azsr = (az_body- sus.track/2*alpha_roll)



    # record body state
    state.Fz = Fz
    state.Mx = Mx

    state.az_body = az_body
    state.alpha_roll = alpha_roll

    # wheel force
    Ft_l = Fs_l
    Ft_r = Fs_r

    # unsprung
    azul = azsl
    azur = azsr

    # output
    state.left.azs = azsl
    state.left.azu = azul

    state.left.Fs = Fs_l
    state.left.Ft = Ft_l

    state.right.azs = azsr
    state.right.azu = azur

    state.right.Fs = Fs_r
    state.right.Ft = Ft_r

    state.F_heave = F_heave
    state.F_roll = F_roll

def Half_suspension_model(sus, state):# 1/2懸吊模型(4DOF)
    """
    4DOF Half Car

    Virtual sprung body:
        zsl
        zsr

    Unsprung:
        zul
        zur
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

    F_heave = (sus.k_heave*d_heave+sus.c_heave*dv_heave)
    F_roll = (sus.k_roll*d_roll+sus.c_roll*dv_roll)

    # optional corner stiffness
    Fq_l = (sus.k_left*ds_l+ sus.c_left*dvs_l)
    Fq_r = (sus.k_right*ds_r+ sus.c_right*dvs_r)

    # modal -> corner
    Fs_l = F_heave/2 - F_roll/2 + Fq_l
    Fs_r = F_heave/2 + F_roll/2 + Fq_r


    # Tire
    dt_l = left.zu-left.zr
    dt_r = right.zu-right.zr

    dvt_l = left.vzu-left.vzr
    dvt_r = right.vzu-right.vzr

    Ft_l = (sus.kt*dt_l+sus.ct*dvt_l)
    Ft_r = (sus.kt*dt_r+sus.ct*dvt_r)



    # Roll center
    Mx_rc = -state.ay*(sus.m_corner*2)*(sus.h_rc)# 慣性力量用負的

    dF_roll = Mx_rc/sus.track # 這個力量直接傳遞到輪胎

    # Virtual body dynamics
    Fz = Fs_l+Fs_r
    
    M_spring = (Fs_r-Fs_l)*sus.track/2

    M_ext = state.ay*(sus.m_corner*2)*(sus.h_cg)

    Mx = M_spring+M_ext+Mx_rc# 總側傾力矩

    print("M_spring:",M_spring,"M_ext:",M_ext,"Mx_rc:",Mx_rc)

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

    # body record
    state.Fz = Fz
    state.Mx = Mx

    state.az_body = az_body
    state.alpha_roll = alpha_roll

    state.F_heave = F_heave
    state.F_roll = F_roll

def Half_rollcenter_all_model(sus, state):# 1/2懸吊模型(4DOF)
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

    F_heave = (sus.k_heave*d_heave+sus.c_heave*dv_heave)
    F_roll = (sus.k_roll*d_roll+sus.c_roll*dv_roll)

    # optional corner stiffness
    Fq_l = (sus.k_left*ds_l+ sus.c_left*dvs_l)
    Fq_r = (sus.k_right*ds_r+ sus.c_right*dvs_r)

    # modal -> corner
    Fs_l = F_heave/2 - F_roll/2 + Fq_l
    Fs_r = F_heave/2 + F_roll/2 + Fq_r


    # Tire
    dt_l = left.zu-left.zr
    dt_r = right.zu-right.zr

    dvt_l = left.vzu-left.vzr
    dvt_r = right.vzu-right.vzr

    Ft_l = (sus.kt*dt_l+sus.ct*dvt_l)
    Ft_r = (sus.kt*dt_r+sus.ct*dvt_r)



    # Roll center
    # 理論幾何力
    
    Mx_rc = -state.ay*(sus.m_corner*2)*(sus.h_rc)

    F_frame_cmd = Mx_rc / sus.track

    state.a_frame = (
        F_frame_cmd
        - sus.k_frame*state.d_frame
        - sus.c_frame*state.v_frame
    ) / sus.m_frame

    dF_roll = (
        sus.k_frame*state.d_frame
        + sus.c_frame*state.v_frame
    )

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
