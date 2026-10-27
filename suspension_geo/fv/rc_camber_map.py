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
# 3. Independent left/right wheel travel maps
# X and Y are independently controlled wheel travels; Z is the measured response.
right_travel_mm = right_travel_matrix * 1000.0
left_travel_mm = left_travel_matrix * 1000.0
rc_x_mm = rc_x_matrix * 1000.0
rc_y_mm = rc_y_matrix * 1000.0

fig = plt.figure(figsize=(16, 12))
surface_specs = [
    (camber_left_matrix, "Left Camber vs. Left/Right Wheel Travel", "Left Camber [deg]", "plasma"),
    (camber_right_matrix, "Right Camber vs. Left/Right Wheel Travel", "Right Camber [deg]", "viridis"),
    (rc_y_mm, "Roll Center Height vs. Left/Right Wheel Travel", "RC Height Y [mm]", "coolwarm"),
    (rc_x_mm, "Roll Center Longitudinal Position vs. Wheel Travel", "RC Position X [mm]", "cividis"),
]

for index, (response, title, z_label, cmap) in enumerate(surface_specs, start=1):
    ax = fig.add_subplot(2, 2, index, projection="3d")
    surface = ax.plot_surface(
        right_travel_mm, left_travel_mm, response,
        cmap=cmap, edgecolor="none", alpha=0.9,
    )
    ax.set_title(title)
    ax.set_xlabel("Right Wheel Travel [mm]")
    ax.set_ylabel("Left Wheel Travel [mm]")
    ax.set_zlabel(z_label)
    fig.colorbar(surface, ax=ax, shrink=0.58, aspect=12, pad=0.08, label=z_label)

fig.suptitle(
    "Front-View Suspension Map: Two Independent Wheel Degrees of Freedom\n"
    "Camber: + top outward / - top inward",
    fontsize=15,
)
fig.tight_layout()
plt.show()

# Complementary roll-center migration cloud.
fig_rc, ax_rc = plt.subplots(figsize=(8, 6))
scatter = ax_rc.scatter(
    rc_x_matrix.flatten(), rc_y_matrix.flatten(),
    c=left_travel_mm.flatten(), cmap="viridis", s=15,
)
ax_rc.set_title("Dynamic Roll Center Migration")
ax_rc.set_xlabel("RC X Position [m]")
ax_rc.set_ylabel("RC Y Position [m]")
ax_rc.grid(True)
fig_rc.colorbar(scatter, ax=ax_rc, label="Left Wheel Travel [mm]")
fig_rc.tight_layout()
plt.show()
