import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":# 模組控制
    import spring as spr
    import damper as dam
    import MotionRatio as mr
    from spring import NullSpring
    from damper import NullDamper
    from MotionRatio import ConstantMR
else:
    from .spring import NullSpring
    from .damper import NullDamper
    from .MotionRatio import ConstantMR


class KCSystem:
    """
    避震器組合

    Force:
        spring + damper

    Motion Ratio:
        wheel force transform
    """

    def __init__(
        self,
        spring = NullSpring(),
        damper = NullDamper(),
        motion_ratio = ConstantMR(mr=1),
        name="Shock"
    ):

        self.spring = spring
        self.damper = damper
        self.motion_ratio = motion_ratio
        self.name = name


    def force(
        self,
        displacement,
        velocity):

        Fs = self.spring.force(displacement)

        Fd = self.damper.force(velocity)


        MR = self.motion_ratio.ratio(displacement)


        F = (
            Fs + Fd
        ) * MR**2


        return F,{
            "force":F,

            "spring_force":Fs,
            "damper_force":Fd,

            "motion_ratio":MR,

            "spring_k":
                self.spring.stiffness(displacement),

            "damper_c":
                self.damper.damping(velocity)
        }


# ======================================
if __name__ == "__main__":
    """避震器模型範例"""

    print("顯使避震器範例")

    # 空
    null = KCSystem()

    # 初步設計用線性範例
    shock_linear = KCSystem(
        spring=spr.LinearSpring(k=30000,),
        damper=dam.LinearDamper(c=1500),
        motion_ratio=mr.ConstantMR(mr=0.95),
        name="Linear")

    # 進階範本
    ohlins = KCSystem(
        spring=spr.LinearSpring(k=30000,),
        damper=dam.HSWeightedDamper(
            low_comp=1800, high_comp=700,
            low_reb=2500, high_reb=1000,
            transition=0.05),
        motion_ratio=mr.ConstantMR(mr=1.0),
        name="ohlins")

    # 進階範本
    LP03_template = KCSystem(
        spring=spr.SeriesSpring(
            main=50000, tender=15000, travel=0.03,
            travel_lim_compress=0.08, travel_lim_stretch=0.08),
        damper=dam.HSWeightedDamper(
            low_comp=1800, high_comp=700,
            low_reb=2500, high_reb=1000,
            transition=0.05),
        motion_ratio=mr.ConstantMR(mr=1.0),
        name="LP03_template")

    # 全查表驗證用範例
    shock_full_lookup = KCSystem(
        spring=spr.LookupSpring(
            displacement=[-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5],
            force=[-150.0, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0],
            travel_lim_compress=0.08, travel_lim_stretch=0.08),
        damper=dam.LookupDamper(
            velocity=[-2.0, -1.0, 0.0, 1.0, 2.0],
            force=[-400.0, -200.0, 0.0, 200.0, 400.0],),
        motion_ratio=mr.LookupMR(
            displacement=[-0.08,-0.04,0.0,0.04,0.08],
            ratio=[0.88,0.91,0.95,0.98,1.02]),
        name="Full Lookup Shock")

    shocks = [null, shock_linear, ohlins, LP03_template, shock_full_lookup]

    # 系統參數
    m = 80.0   # 質量 (kg)
    x0 = 0.05   # 初始位移 (m)
    v0 = 0.0    # 初始速度 (m/s)

    # 時間設定
    dt = 0.001
    time = np.arange(0,1, dt)

    # 模擬函數
    def simulate(shock):
        x = np.zeros_like(time)
        v = np.zeros_like(time)
        a = np.zeros_like(time)
        F_list = np.zeros_like(time)
        x[0] = x0
        v[0] = v0
        for i in range(1, len(time)):
            F, _ = shock.force(x[i-1], v[i-1])
            a[i] = F / m
            v[i] = v[i-1] + a[i] * dt
            x[i] = x[i-1] + v[i] * dt
            F_list[i] = F
        return x, v, a, F_list

    # 畫圖
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

    for shock in shocks:
        x, v, a, F_list = simulate(shock)
        axs[0].plot(time, x, label=shock.name)
        axs[1].plot(time, v, label=shock.name)
        axs[2].plot(time, a, label=shock.name)
        axs[3].plot(time, F_list, label=shock.name)

    axs[0].set_ylabel("Displacement (m)")
    axs[1].set_ylabel("Velocity (m/s)")
    axs[2].set_ylabel("Acceleration (m/s^2)")
    axs[3].set_ylabel("Force (N)")
    axs[3].set_xlabel("Time (s)")

    for ax in axs:
        ax.grid(True)
        ax.legend()

    fig.suptitle("Shock Comparison - Displacement, Velocity, Acceleration, Force")
    plt.show()
