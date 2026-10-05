import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 啟用 3D 繪圖
from pathlib import Path
current_dir = Path(__file__).resolve().parent
# ==============================================================================
# 1. 讀取 PKL 檔案
# ==============================================================================
pkl_filename = current_dir / "suspension_kinematics_data.pkl"

try:
    with open(pkl_filename, "rb") as f:
        dataset = pickle.load(f)
    print(f"成功讀取 {pkl_filename}，共 {len(dataset)} 筆資料。")
except FileNotFoundError:
    print(f"錯誤：找不到檔案 {pkl_filename}，請先執行資料產出主程式。")
    exit()

# ==============================================================================
# 2. 將 1D List 轉換/重組為 2D 網格矩陣 (Mesh Grid)
# ==============================================================================
# 根據資料筆數推算 steps (假設陣列為正方形 N x N)
total_pts = len(dataset)
steps = int(np.sqrt(total_pts))

if steps * steps != total_pts:
    raise ValueError(f"資料筆數 ({total_pts}) 無法構成完整的 N x N 網格矩陣！")

# 建立 2D 陣列來儲存還原後的資料
heave_matrix = np.zeros((steps, steps))
roll_matrix = np.zeros((steps, steps))
camber_left_matrix = np.zeros((steps, steps))
camber_right_matrix = np.zeros((steps, steps))
left_travel_matrix = np.zeros((steps, steps))
right_travel_matrix = np.zeros((steps, steps))
rc_x_matrix = np.zeros((steps, steps))
rc_y_matrix = np.zeros((steps, steps))

# 重構為 2D 矩陣
for idx, data in enumerate(dataset):
    i = idx // steps  # 列 (Left Theta 步驟)
    j = idx % steps   # 行 (Right Theta 步驟)

    heave_matrix[i, j] = data["heave"]
    roll_matrix[i, j] = data["roll_deg"]
    camber_left_matrix[i, j] = data["camber_left_deg"]
    camber_right_matrix[i, j] = data["camber_right_deg"]
    left_travel_matrix[i, j] = data["travel_left"]
    right_travel_matrix[i, j] = data["travel_right"]

    rc = data["rc"]
    rc_x_matrix[i, j] = rc[0]
    rc_y_matrix[i, j] = rc[1]

# ==============================================================================
# 3. 繪製圖表 - 格式 1：Left & Right Camber 3D 曲面圖
# ==============================================================================
fig1 = plt.figure(figsize=(16, 7))

# 單位轉換：米 (m) 轉 毫米 (mm)
Heave_mm = heave_matrix * 1000
Roll_deg = roll_matrix

# 圖一：3D 曲面 (Heave & Roll vs. Left Camber)
ax1 = fig1.add_subplot(1, 2, 1, projection="3d")
surf1 = ax1.plot_surface(
    Heave_mm, Roll_deg, camber_left_matrix, cmap="plasma", edgecolor="none", alpha=0.85
)
ax1.set_title("3D Surface: Left Camber vs. Heave & Roll")
ax1.set_xlabel("Heave [mm]")
ax1.set_ylabel("Roll Angle [deg]")
ax1.set_zlabel("Left Camber [deg]")
fig1.colorbar(surf1, ax=ax1, shrink=0.5, aspect=10, label="Camber [deg]")

# 圖二：3D 曲面 (Heave & Roll vs. Right Camber)
ax2 = fig1.add_subplot(1, 2, 2, projection="3d")
surf2 = ax2.plot_surface(
    Heave_mm, Roll_deg, camber_right_matrix, cmap="viridis", edgecolor="none", alpha=0.85
)
ax2.set_title("3D Surface: Right Camber vs. Heave & Roll")
ax2.set_xlabel("Heave [mm]")
ax2.set_ylabel("Roll Angle [deg]")
ax2.set_zlabel("Right Camber [deg]")
fig1.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10, label="Camber [deg]")

plt.tight_layout()
plt.show()

# ==============================================================================
# 4. 繪製圖表 - 格式 2：Roll Center 軌跡與 3D 高度變化曲面圖
# ==============================================================================
fig2 = plt.figure(figsize=(15, 6))

# 左圖：Roll Center 2D 軌跡散佈圖
ax1_2 = fig2.add_subplot(1, 2, 1)
sc = ax1_2.scatter(
    rc_x_matrix.flatten(),
    rc_y_matrix.flatten(),
    c=left_travel_matrix.flatten(),
    cmap="viridis",
    s=15,
)
ax1_2.set_title("Dynamic Roll Center Migration Cloud")
ax1_2.set_xlabel("RC X Position [m]")
ax1_2.set_ylabel("RC Y Position [m]")
ax1_2.grid(True)
fig2.colorbar(sc, ax=ax1_2, label="Left Travel [m]")

# 右圖：3D 曲面圖 (Roll Center Y Height 隨左右輪行程變化)
ax2_2 = fig2.add_subplot(1, 2, 2, projection="3d")

# 將座標單位轉成 mm 增加可讀性
X_mm = right_travel_matrix * 1000
Y_mm = left_travel_matrix * 1000
Z_mm = rc_y_matrix * 1000

surf_rc = ax2_2.plot_surface(
    X_mm, Y_mm, Z_mm, cmap="coolwarm", edgecolor="none", alpha=0.9
)

ax2_2.set_title("3D Surface: RC Height vs. Wheel Travel")
ax2_2.set_xlabel("Right Wheel Travel [mm]")
ax2_2.set_ylabel("Left Wheel Travel [mm]")
ax2_2.set_zlabel("Roll Center Y [mm]")
fig2.colorbar(surf_rc, ax=ax2_2, shrink=0.5, aspect=10, label="RC Y Height [mm]")

plt.tight_layout()
plt.show()