# spring.py
from abc import ABC, abstractmethod
import numpy as np
import matplotlib.pyplot as plt

class Spring(ABC):

    def __init__(
        self,
        travel_lim_compress=np.inf,
        travel_lim_stretch=np.inf,
    ):
        self.travel_lim_compress = travel_lim_compress
        self.travel_lim_stretch = travel_lim_stretch

    @abstractmethod
    def stiffness(self, displacement):
        ...

    @abstractmethod
    def force(self, displacement):
        ...

    @abstractmethod
    def energy(self, displacement):
        ...

    @abstractmethod
    def displacement(self, force):
        ...


    def is_bottomed(self, displacement):
        return (
            displacement >= self.travel_lim_compress or
            displacement <= -self.travel_lim_stretch
        )

    def clamp_displacement(self, displacement):
        return np.clip(
            displacement,
            -self.travel_lim_stretch,
            self.travel_lim_compress
        )

class NullSpring(Spring):# 不安裝
    """沒裝彈簧"""    
    def stiffness(self, displacement):
        return 0.0

    def force(self, displacement):
        return 0.0

    def energy(self, displacement):
        return 0.0

    def displacement(self, force):
        return 0.0

    def is_bottomed(self, displacement): 
        return False

class LinearSpring(Spring):# 線性
    """Linear spring"""

    def __init__(
        self,
        k,
        travel_lim_compress=np.inf,
        travel_lim_stretch=np.inf,
    ):
        super().__init__(
            travel_lim_compress,
            travel_lim_stretch,
        )
        self.k = k

    def stiffness(self, displacement):
        return self.k

    def force(self, displacement):
        displacement = self.clamp_displacement(displacement)
        F = -self.k * displacement
        return F

    def energy(self, displacement):
        displacement = self.clamp_displacement(displacement)
        E = 0.5 * self.k * displacement**2
        return E

    def displacement(self, force):
        s = force/(-self.k)
        return s
    
class SeriesSpring(Spring):# 串聯
    """
    串聯彈簧

    到壓縮極限travel時候第一彈簧撞底
    剛性變成只剩下main
    """
    def __init__(self, main, tender , travel,
        travel_lim_compress=np.inf,
        travel_lim_stretch=np.inf,
                 ):
        
        super().__init__(
            travel_lim_compress,
            travel_lim_stretch,
        )
        self.main = main
        self.tender  = tender 
        self.travel = travel

    def stiffness(self, displacement):
        """壓縮負 拉升正"""
        if displacement < -self.travel:# 壓縮的時候才會變剛性 拉升不會不用取絕對值
            K = self.main
        else:
            K = self.main * self.tender  / (self.main + self.tender )
        return K

    def force(self, displacement):

        displacement = self.clamp_displacement(displacement)
        k = self.stiffness(displacement)
        F = -k * displacement
        return F

    def energy(self, displacement):

        displacement = self.clamp_displacement(displacement)
        x = displacement

        k1 = self.main * self.tender  / (self.main + self.tender )
        k2 = self.main

        if x <= -self.travel:
            E = 0.5 * k1 * x**2
            
        else:
            E = 0.5 * k1 * self.travel**2 +0.5 * k2 * (x**2 - self.travel**2)

        return E

    def displacement(self, force):
        
        k1 = self.main * self.tender / (self.main + self.tender)
        k2 = self.main

        F_switch = k1 * self.travel

        if force <= F_switch:
            x = -force / k1
        else:
            x = -self.travel - (force - F_switch) / k2

        return self.clamp_displacement(x)
    
class LookupSpring(Spring):# 差值
    """
    使用位移-力量資料表的彈簧
    """

    def __init__(
        self,
        displacement,
        force,
        travel_lim_compress=np.inf,
        travel_lim_stretch=np.inf,
    ):

        super().__init__(
            travel_lim_compress,
            travel_lim_stretch,
        )

        self.displacement = np.array(displacement)
        self.force_data = np.array(force)


    def force(self, displacement):

        displacement = self.clamp_displacement(displacement)

        F = np.interp(
            displacement,
            self.displacement,
            self.force_data
        )

        return -F


    def stiffness(self, displacement):

        dx = 1e-5

        F1 = self.force(displacement + dx)
        F2 = self.force(displacement - dx)

        k = -(F1 - F2) / (2*dx)

        return k


    def energy(self, displacement):

        x = self.displacement
        F = self.force_data

        E = np.trapz(
            F,
            x
        )

        return E

    def displacement(self, force):
        
        # force() 回傳的是 -F，所以要先轉回查表的正向力量
        F = -force

        x = np.interp(
            F,
            self.force_data,
            self.displacement
        )

        return self.clamp_displacement(x)
    
if __name__ == "__main__":
    """彈簧模型效果範例"""
    print("顯示範例剛性曲線")
    # 建立彈簧物件
    null_spring = NullSpring()
    linear_spring = LinearSpring(k=100.0, travel_lim_compress=1.5, travel_lim_stretch=1.5)
    series_spring = SeriesSpring(main=200.0, tender =50.0, travel=0.5,
                                travel_lim_compress=1.5, travel_lim_stretch=1.5)
    lookup_spring = LookupSpring(
        displacement=[-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5],
        force=[-150.0, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0],
        travel_lim_compress=1.5,
        travel_lim_stretch=1.5
    )

    # 測試位移範圍
    displacements = np.linspace(-1.5, 1.5, 300)

    # 計算各彈簧的力
    forces = {
        "NullSpring": [null_spring.force(x) for x in displacements],
        "LinearSpring": [linear_spring.force(x) for x in displacements],
        "SeriesSpring": [series_spring.force(x) for x in displacements],
        "LookupSpring": [lookup_spring.force(x) for x in displacements],
    }

    # 畫圖
    plt.figure(figsize=(10, 6))
    for name, f in forces.items():
        plt.plot(displacements, f, label=name)

    plt.axhline(0, color='black', linewidth=0.8, linestyle='--')
    plt.axvline(0, color='black', linewidth=0.8, linestyle='--')
    plt.xlabel("Displacement")
    plt.ylabel("Force")
    plt.title("Spring Force-Displacement Curves")
    plt.legend()
    plt.grid(True)
    plt.show()
