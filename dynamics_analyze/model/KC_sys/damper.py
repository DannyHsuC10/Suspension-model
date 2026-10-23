import numpy as np
import matplotlib.pyplot as plt

class Damper():
    """基本架構"""    
    def force(self, velocity):
        raise NotImplementedError

    def damping(self, velocity):
        raise NotImplementedError
    
class NullDamper(Damper):# 不安裝
    """沒裝阻尼"""    
    def force(self, velocity):
        return 0.0

    def damping(self, velocity):
        return 0.0

class LinearDamper(Damper):# 線性
    """線性"""    
    def __init__(self, c):
        self.c = c

    def damping(self, velocity):
        return self.c

    def force(self, velocity):
        return -self.c * velocity

class HLSpeedSwitchDamper(Damper):# HL
    """
    高低速阻尼

    閥值transition
    當速度高於閥值 >> 高速阻尼
    速度低於閥值 >> 高速阻尼
    """
    def __init__(
        self,
        low_comp,
        high_comp,
        low_reb,
        high_reb,
        transition
    ):

        self.transition = transition

        self.low_comp = low_comp
        self.high_comp = high_comp

        self.low_reb = low_reb
        self.high_reb = high_reb

    def damping(self, velocity):

        if velocity >= 0:

            if velocity < self.transition:
                return self.low_comp

            return self.high_comp

        else:

            if -velocity < self.transition:
                return self.low_reb

            return self.high_reb

    def force(self, velocity):

        c = self.damping(velocity)
        return -c * velocity

class HSWeightedDamper(Damper):# 高速加重
    """
    高速加重加權阻尼
    和高低速切換阻尼類似，但參數定義不同

    compression / rebound 分開設定

    velocity:
        positive  : compression
        negative  : rebound
    """

    def __init__(
        self,
        low_comp,
        high_comp,
        low_reb,
        high_reb,
        transition
    ):

        self.low_comp = low_comp
        self.high_comp = high_comp

        self.low_reb = low_reb
        self.high_reb = high_reb

        self.transition = transition


    def _force_curve(self, velocity, low, high):

        """
        單方向阻尼曲線
        """

        v = abs(velocity)

        if v < self.transition:

            F = low * v

        else:

            F = (
                low * self.transition +
                high * (v - self.transition)
            )

        return F


    def force(self, velocity):

        if velocity >= 0:
            # compression

            F = self._force_curve(
                velocity,
                self.low_comp,
                self.high_comp
            )

        else:
            # rebound

            F = self._force_curve(
                velocity,
                self.low_reb,
                self.high_reb
            )


        return -np.sign(velocity) * F



    def damping(self, velocity):

        if abs(velocity) < 1e-8:
            return 0.0

        return -self.force(velocity) / velocity

class VGainDamper(Damper):# 速度加權
    """
    速度相關阻尼

    c = c_base + c_gain * |v|

    positive velocity:
        compression

    negative velocity:
        rebound
    """

    def __init__(
        self,
        c_base_comp,
        c_gain_comp,

        c_base_reb,
        c_gain_reb,
    ):

        self.c_base_comp = c_base_comp
        self.c_gain_comp = c_gain_comp

        self.c_base_reb = c_base_reb
        self.c_gain_reb = c_gain_reb


    def damping(self, velocity):

        if velocity >= 0:
            # compression

            c = (
                self.c_base_comp +
                self.c_gain_comp * abs(velocity)
            )

        else:
            # rebound

            c = (
                self.c_base_reb +
                self.c_gain_reb * abs(velocity)
            )

        return c


    def force(self, velocity):

        c = self.damping(velocity)

        return -c * velocity

class LookupDamper(Damper):# 差值
    """
    使用速度-力量資料表
    """

    def __init__(
        self,
        velocity,
        force
    ):

        self.velocity = np.array(velocity)
        self.force_data = np.array(force)


    def force(self, velocity):
        F = -np.interp(
            velocity,
            self.velocity,
            self.force_data
        )
        return F


    def damping(self, velocity):

        if abs(velocity) < 1e-8:
            return 0.0
        else:
            return self.force(velocity) / velocity


# ===============================================
if __name__ == "__main__":
    """阻尼模型效果範例"""
    print("顯使範例阻尼曲線")
    # 建立阻尼器物件
    null_damper = NullDamper()
    linear_damper = LinearDamper(c=100.0)
    hl_damper = HLSpeedSwitchDamper(
        low_comp=50.0, high_comp=200.0,
        low_reb=60.0, high_reb=220.0,
        transition=0.5
    )
    hs_weighted_damper = HSWeightedDamper(
        low_comp=30.0, high_comp=150.0,
        low_reb=40.0, high_reb=160.0,
        transition=0.3
    )
    vgain_damper = VGainDamper(
        c_base_comp=20.0, c_gain_comp=10.0,
        c_base_reb=25.0, c_gain_reb=20
    )
    lookup_damper = LookupDamper(
        velocity=[-2.0, -1.0, 0.0, 1.0, 2.0],
        force=[-400.0, -200.0, 0.0, 200.0, 400.0]
    )

    # 測試速度範圍
    velocities = np.linspace(-1.0, 1.0, 200)

    # 計算各阻尼器的力
    forces = {
        "NullDamper": [null_damper.force(v) for v in velocities],
        "LinearDamper": [linear_damper.force(v) for v in velocities],
        "HLSpeedSwitchDamper": [hl_damper.force(v) for v in velocities],
        "HSWeightedDamper": [hs_weighted_damper.force(v) for v in velocities],
        "VGainDamper": [vgain_damper.force(v) for v in velocities],
        "LookupDamper": [lookup_damper.force(v) for v in velocities],
    }

    # 畫圖
    plt.figure(figsize=(10, 6))
    for name, f in forces.items():
        plt.plot(velocities, f, label=name)

    plt.axhline(0, color='black', linewidth=0.8, linestyle='--')
    plt.axvline(0, color='black', linewidth=0.8, linestyle='--')
    plt.xlabel("Velocity")
    plt.ylabel("Force")
    plt.title("Damper Force-Velocity Curves")
    plt.legend()
    plt.grid(True)
    plt.show()
