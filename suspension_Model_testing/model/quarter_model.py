
def single_spring_model(sus,state):# 單彈簧模型
    """
    用ride rate 合併一個彈簧
    """
    
    sus_dz = state.zs-state.zr
    Fs = (sus.ride_rate*sus_dz) # 簧上力量

    Ft = Fs # 輪胎力量

    azs = -Fs/sus.m_corner # 加速度
    azu = azs

    state.azs = azs
    state.azu = azu
    state.Fs = Fs
    state.Ft = Ft

def one_eighth_suspension_model(sus,state):# 1/8懸吊模型
    """
    用ride rate 合併一個彈簧+懸吊阻尼
    直接忽略輪胎阻尼(都簡化到這種程度不需要輪胎阻尼)
    """
    sus_dz = state.zs-state.zr
    sus_dvz = state.vzs-state.vzr
    Fs = (sus.ride_rate*sus_dz+sus.cs*sus_dvz) # 簧上力量

    Ft = Fs # 輪胎力量

    azs = -Fs/sus.m_corner # 簧上加速度
    azu = azs

    state.azs = azs
    state.azu = azu
    state.Fs = Fs
    state.Ft = Ft

def Double_spring_model(sus,state):# 雙彈簧模型
    """
    純雙彈簧模型
    """
    sus_dz = state.zs-state.zu # 懸吊相對位移
    Fs = (sus.kw*sus_dz) # 簧上力量

    tire_dz = state.zu-state.zr # 輪胎相對位移
    Ft = (sus.kt*tire_dz) # 輪胎力量

    azs = -Fs/sus.ms # 簧上加速度
    azu = (Fs-Ft)/sus.mu # 簧下加速度

    state.azs = azs
    state.azu = azu
    state.Fs = Fs
    state.Ft = Ft

def Quarter_suspension_model(sus,state):# 1/4懸吊模型
    """
    雙彈簧阻尼模型
    """
    sus_dz = state.zs-state.zu # 懸吊相對位移
    sus_dvz = state.vzs-state.vzu # 懸吊相對速度
    Fs = (sus.kw*sus_dz+sus.cs*sus_dvz) # 簧上力量

    tire_dz = state.zu-state.zr # 輪胎相對位移
    tire_dvz = state.vzu-state.vzr # 輪胎相對速度
    Ft = (sus.kt*tire_dz+sus.ct*tire_dvz) # 輪胎力量

    azs = -Fs/sus.ms # 簧上加速度
    azu = (Fs-Ft)/sus.mu # 簧下加速度

    state.azs = azs
    state.azu = azu
    state.Fs = Fs
    state.Ft = Ft

