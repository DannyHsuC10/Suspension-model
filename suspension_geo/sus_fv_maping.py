import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 啟用 3D 繪圖
from scipy.interpolate import griddata
from car import Car, SuspensionF
import Geometric_Analysis as ga
from pathlib import Path
current_dir = Path(__file__).resolve().parent

# ==============================================================================
# 主程式執行段
# ==============================================================================

car = Car()
suspension_f = SuspensionF()
sus_helper = ga.SusGeometryHelper(car, suspension_f)

# 1. 初始化靜態懸吊結構
static_pts = sus_helper.calc_static_points()

right_sus = ga.SuspensionSide("Right", is_left=False)
right_sus.setup_right_side(static_pts)

left_sus = ga.SuspensionSide("Left", is_left=True)
left_sus.setup_from_right_template(right_sus)

# 2. 計算關節極限
target_travel = suspension_f.front_view_target_travel
theta0 = right_sus.lower_arm.angle
theta_down, theta_up = ga.calculate_theta_limits(
    sus=right_sus,
    target_travel=target_travel,)

# 3. 定義掃描網格
steps = suspension_f.front_view_mapping_steps
theta_left_array = np.linspace(theta_down, theta_up, steps)
theta_right_array = np.linspace(theta_down, theta_up, steps)

# 準備矩陣
rc_x_matrix = np.zeros((steps, steps))
rc_y_matrix = np.zeros((steps, steps))
heave_matrix = np.zeros((steps, steps))
roll_matrix = np.zeros((steps, steps))
camber_left_matrix = np.zeros((steps, steps))
camber_right_matrix = np.zeros((steps, steps))

# 靜態基準點
gnd_y0_r = right_sus.gnd.pos[1]
gnd_y0_l = left_sus.gnd.pos[1]

dataset = []

print("開始進行懸吊雙重迴圈幾何掃描與運動學分析...")

# ==============================================================================
# 4. 核心掃描迴圈 (Batch Sweep Calculation)
# ==============================================================================
for i, theta_l in enumerate(theta_left_array):
    for j, theta_r in enumerate(theta_right_array):

        # 1. 更新懸吊運動學
        left_sus.update_kinematics(theta_l, sus_helper)
        right_sus.update_kinematics(theta_r, sus_helper)

        # 2. 計算單輪行程 (Travel)
        l_travel = left_sus.gnd.pos[1] - gnd_y0_l
        r_travel = right_sus.gnd.pos[1] - gnd_y0_r

        # 3. 計算 Heave 與 Roll 角度
        # (1) Heave: 左右輪接地點平均升降量 (車身相對下沉量)
        heave = (l_travel + r_travel) / 2.0

        # (2) Roll Angle: 兩輪 GND 連線與 X 軸夾角
        gnd_vec = right_sus.gnd.pos - left_sus.gnd.pos
        roll_angle_rad = np.arctan2(gnd_vec[1], gnd_vec[0])
        roll_angle_deg = np.degrees(roll_angle_rad)

        # 4. 計算 Camber 角度 (考慮當前 GND 連線向量)
        gnd_norm_vec = gnd_vec / np.linalg.norm(gnd_vec)
        camber_l = left_sus.calc_camber_angle(gnd_norm_vec)
        camber_r = right_sus.calc_camber_angle(gnd_norm_vec)

        # 5. 計算 Roll Center
        rc_pos = sus_helper.calc_roll_center(left_sus, right_sus)

        # 填入繪圖矩陣
        heave_matrix[i, j] = heave
        roll_matrix[i, j] = roll_angle_deg
        camber_left_matrix[i, j] = camber_l
        camber_right_matrix[i, j] = camber_r

        if rc_pos is not None:
            rc_x_matrix[i, j] = rc_pos[0]
            rc_y_matrix[i, j] = rc_pos[1]
        else:
            rc_x_matrix[i, j] = np.nan
            rc_y_matrix[i, j] = np.nan

        # 打包狀態數據 (Data Storage)
        state_data = {
            "travel_left": l_travel,
            "travel_right": r_travel,
            "heave": heave,
            "roll_deg": roll_angle_deg,
            "camber_left_deg": camber_l,
            "camber_right_deg": camber_r,
            "theta_left": theta_l,
            "theta_right": theta_r,
            "rc": rc_pos.copy() if rc_pos is not None else np.array([np.nan, np.nan]),
            "ic_left": left_sus.IC.pos.copy(),
            "ic_right": right_sus.IC.pos.copy(),
            "gnd_left": left_sus.gnd.pos.copy(),
            "gnd_right": right_sus.gnd.pos.copy(),
            "kua_left": left_sus.kua.pos.copy(),
            "kua_right": right_sus.kua.pos.copy(),
            "lak_left": left_sus.lak.pos.copy(),
            "lak_right": right_sus.lak.pos.copy(),
            "wheel_pts_left": [p.pos.copy() for p in left_sus.wheel_pts],
            "wheel_pts_right": [p.pos.copy() for p in right_sus.wheel_pts],
        }
        dataset.append(state_data)

# ==============================================================================
# 5. 儲存 PKL
# ==============================================================================
pkl_filename = current_dir / "suspension_kinematics_data.pkl"
with open(pkl_filename, "wb") as f:
    pickle.dump(dataset, f)
print(f"完整數據已成功存入 {pkl_filename}，總計 {len(dataset)} 筆狀態資料。")

import rc_camber_map # 畫圖執行
