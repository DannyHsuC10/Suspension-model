import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 啟用 3D 繪圖
from scipy.interpolate import griddata
from car import Car, SuspensionF
import Geometric_Analysis as ga



# 初始化懸吊系統
car = Car()
suspension_f = SuspensionF()
sus_helper = ga.SusGeometryHelper(car, suspension_f)
static_pts = sus_helper.calc_static_points()

right_sus = ga.SuspensionSide("Right", is_left=False)
right_sus.setup_right_side(static_pts)

left_sus = ga.SuspensionSide("Left", is_left=True)
left_sus.setup_from_right_template(right_sus)

# ==============================================================================
# 建立無須 Mapping 的即時求解器
# ==============================================================================
solver = ga.SuspensionDirectSolver(sus_helper, left_sus, right_sus)


# 範例
if __name__ == "__main__":
    # ------------------------------------------------------------------------------
    # 情境 A: 我知道左右關節角度，想知道即時的 Camber & RC
    # ------------------------------------------------------------------------------
    res_a = solver.solve_by_thetas(theta_l=0.15, theta_r=0.18)

    print("=== 情境 A 求解結果 ===")
    print(f"Roll Center 座標 : X={res_a['rc_pos'][0]:.4f} m, Y={res_a['rc_pos'][1]:.4f} m")
    print(f"左輪 Camber      : {res_a['camber_left_deg']:.3f}°")
    print(f"右輪 Camber      : {res_a['camber_right_deg']:.3f}°")

    # ------------------------------------------------------------------------------
    # 情境 B: 車身目前處於下沉 10mm (Heave=0.01m)、側傾 1.5 度 (Roll=1.5 deg)，求 Camber & RC
    # ------------------------------------------------------------------------------
    res_b = solver.solve_by_pose(target_heave_m=0.010, target_roll_deg=1.5)

    print("\n=== 情境 B (姿態反算) 求解結果 ===")
    print(f"反算得到的 Left Theta  : {res_b['theta_left']:.4f} rad")
    print(f"反算得到的 Right Theta : {res_b['theta_right']:.4f} rad")
    print(f"Roll Center 座標       : X={res_b['rc_pos'][0]:.4f} m, Y={res_b['rc_pos'][1]:.4f} m")
    print(f"左輪 Camber            : {res_b['camber_left_deg']:.3f}°")
    print(f"右輪 Camber            : {res_b['camber_right_deg']:.3f}°")
    # ------------------------------------------------------------------------------
    # 情境 C: 左輪上壓 10mm (+0.010m)，右輪下伸 5mm (-0.005m)
    # ------------------------------------------------------------------------------

    # 假設：左輪上壓 10mm (+0.010m)，右輪下伸 5mm (-0.005m)
    l_travel = 0.010   # +10 mm
    r_travel = -0.005  # -5 mm

    res = solver.solve_by_travels(l_travel_m=l_travel, r_travel_m=r_travel)

    print("=== 從左右懸吊行程計算的結果 ===")
    print(f"輸入左輪行程 : {res['travel_left_mm']:.2f} mm")
    print(f"輸入右輪行程 : {res['travel_right_mm']:.2f} mm")
    print(f"反算 Left  Theta : {res['theta_left']:.4f} rad")
    print(f"反算 Right Theta : {res['theta_right']:.4f} rad")
    print(f"Roll Center 座標 : X={res['rc_pos'][0]:.4f} m, Y={res['rc_pos'][1]:.4f} m")
    print(f"左輪 Camber      : {res['camber_left_deg']:.3f}°")
    print(f"右輪 Camber      : {res['camber_right_deg']:.3f}°")
