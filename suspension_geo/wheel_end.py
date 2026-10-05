import numpy as np
import matplotlib.pyplot as plt
from car import Car, SuspensionF


car = Car()
suspension_f = SuspensionF()

# =========================
# 車輛與懸吊參數
# =========================
kpi_rad = suspension_f.kpi
caster_rad = np.deg2rad(suspension_f.caster_sv_deg)
tire_radius = car.free_radius  # [m]
tire_width = car.tire_width  # [m]
# Contact point in the unsteered wheel frame: tire bottom at [0, 0, -r].
# The kingpin axis is currently defined through the origin, so steering rotates
# this point directly with the same Rodrigues rotation matrix.
initial_contact_point = np.array([0.0, 0.0, -tire_radius])

# Analysis inputs not currently stored in Car/SuspensionF.
steer_angle_deg = -30.0
steer_limit_deg = 30.0
steer_samples = 100

# =========================
# Rodrigues rotation matrix
# =========================
def rotation_matrix(axis, theta): # 旋轉矩陣
    axis = axis / np.linalg.norm(axis)
    kx, ky, kz = axis

    K = np.array([
        [0, -kz, ky],
        [kz, 0, -kx],
        [-ky, kx, 0]
    ])

    I = np.eye(3)
    R = I + np.sin(theta)*K + (1 - np.cos(theta))*(K @ K)
    return R

# =============================================(轉向情況圖)
# =========================
# 建立圓柱體（輪胎）
# =========================
# 建立範圍
theta = np.linspace(0, 2*np.pi, 50)
z_lin = np.linspace(-tire_width / 2, tire_width / 2, 20)
theta_grid, z_grid = np.meshgrid(theta, z_lin)

# 圓方程式
y = z_grid                      # 厚度方向
x = tire_radius * np.sin(theta_grid)
z = tire_radius * np.cos(theta_grid)

# 攤平成 Nx3 點
points = np.vstack((x.flatten(), y.flatten(), z.flatten())).T

# =========================
# kingpin 軸
# =========================
# 角度轉換
axis_dir = np.array([# 軸線向量
    np.sin(caster_rad),
    np.sin(kpi_rad),
    np.cos(kpi_rad) * np.cos(caster_rad)
])
axis_dir = axis_dir / np.linalg.norm(axis_dir)

# 旋轉
theta_rot = np.radians(steer_angle_deg)
R_mat = rotation_matrix(axis_dir, theta_rot)

rot_points = (R_mat @ points.T).T

# reshape 回去
x_r = rot_points[:,0].reshape(x.shape)
y_r = rot_points[:,1].reshape(y.shape)
z_r = rot_points[:,2].reshape(z.shape)

# =========================
# 軸線
# =========================
t = np.linspace(-1.5 * tire_radius, 1.5 * tire_radius, 10)
axis_line = np.outer(t, axis_dir)

wheel_axis = np.array([
    [0, 0, -1.5 * tire_radius],
    [0, 0, 1.5 * tire_radius]
])

# =========================
# 固定初始接地點，隨輪胎姿態旋轉
# =========================
contact_point = R_mat @ initial_contact_point

# =========================
# 計算 kingpin 到接地點的最短距離（力矩半徑）
# =========================
k = axis_dir  # 已經是單位向量

r = contact_point

# 投影到 kingpin 軸
proj = np.dot(r, k) * k

# 垂直向量（真正力臂）
r_perp = r - proj

moment_arm_length = np.linalg.norm(r_perp)

# =========================
# 畫圖
# =========================
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# =========================
# 畫接地點
# =========================
ax.scatter(contact_point[0], contact_point[1], contact_point[2],
           color='red', s=80, label="Contact Point")

# =========================
# 畫 r 向量（原點 → 接地點）
# =========================
ax.plot([0, contact_point[0]],
        [0, contact_point[1]],
        [0, contact_point[2]],
        linestyle='--', label="r vector")

# =========================
# 畫 kingpin 投影點
# =========================
proj_point = proj
ax.scatter(proj_point[0], proj_point[1], proj_point[2],
           color='green', s=60, label="Projection on Kingpin")

# =========================
# 畫力臂（最重要）
# =========================
ax.plot([proj_point[0], contact_point[0]],
        [proj_point[1], contact_point[1]],
        [proj_point[2], contact_point[2]],
        color='black', linewidth=3, label="Moment Arm")

# 顯示數值
ax.text(contact_point[0], contact_point[1], contact_point[2],
        f"\nArm={moment_arm_length * 1000:.1f} mm",
        color='black')

# 原始輪胎（淡色）
ax.plot_surface(x, y, z, alpha=0.2,color='blue')

# 旋轉後輪胎
ax.plot_surface(x_r, y_r, z_r, alpha=0.8)

# kingpin
ax.plot(axis_line[:,0], axis_line[:,1], axis_line[:,2], label="Kingpin Axis")

# 原輪軸
ax.plot(wheel_axis[:,0], wheel_axis[:,1], wheel_axis[:,2], label="Wheel Axis")

ax.set_aspect('equal', 'box')
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.set_title(
    f"KPI={np.degrees(kpi_rad):.1f}°, "
    f"caster={suspension_f.caster_sv_deg:.1f}°, "
    f"steer={steer_angle_deg:.1f}°"
)
ax.legend()
plt.show()

# ============================================================(接地點分析)
# =========================
# 掃描 steer 並找接地點
# =========================
steer_range = np.linspace(-steer_limit_deg, steer_limit_deg, steer_samples)

contact_x = []
contact_y = []
contact_z = []

for steer in steer_range:

    theta_rot = np.radians(steer)
    R_mat = rotation_matrix(axis_dir, theta_rot)

    rot_points = (R_mat @ points.T).T

    # 只旋轉初始底部點，不從旋轉後表面重新搜尋最低點。
    contact_rotated = R_mat @ initial_contact_point
    x_c, y_c, z_c = contact_rotated

    contact_x.append(x_c)
    contact_y.append(y_c)
    contact_z.append(z_c)

contact_x = np.array(contact_x)
contact_y = np.array(contact_y)
contact_z = np.array(contact_z)

# =========================
# 畫接地點軌跡
# =========================
fig = plt.figure(figsize=(10,6))

plt.subplot(3,1,1)
plt.plot(steer_range, contact_x * 1000.0)
plt.ylabel("X (mm)")
plt.grid()

plt.subplot(3,1,2)
plt.plot(steer_range, contact_y * 1000.0)
plt.ylabel("Y (mm)")
plt.grid()

plt.subplot(3,1,3)
plt.plot(steer_range, contact_z * 1000.0)
plt.ylabel("Z (mm)")
plt.xlabel("Steer (deg)")
plt.grid()

plt.suptitle("Contact Patch vs Steering Angle")
plt.show()

# =======================================================(力臂計算)
# =========================
# 計算三個方向的等效力臂
# =========================
arm_Fx = []
arm_Fy = []
arm_Fz = []

k = axis_dir  # kingpin unit vector

for i in range(len(steer_range)):

    r = np.array([
        contact_x[i],
        contact_y[i],
        contact_z[i]
    ])

    # 三個單位力
    Fx = np.array([1, 0, 0])
    Fy = np.array([0, 1, 0])
    Fz = np.array([0, 0, 1])

    # 力矩 進行外積
    Mx = np.cross(r, Fx)
    My = np.cross(r, Fy)
    Mz = np.cross(r, Fz)

    # 投影到 kingpin 內積旋轉軸
    arm_Fx.append(np.dot(Mx, k))
    arm_Fy.append(np.dot(My, k))
    arm_Fz.append(np.dot(Mz, k))

arm_Fx = np.array(arm_Fx)
arm_Fy = np.array(arm_Fy)
arm_Fz = np.array(arm_Fz)

# 畫圖
fig = plt.figure(figsize=(10,6))

plt.plot(steer_range, np.asarray(arm_Fx) * 1000.0, label="Fx moment arm")
plt.plot(steer_range, np.asarray(arm_Fy) * 1000.0, label="Fy moment arm")
plt.plot(steer_range, np.asarray(arm_Fz) * 1000.0, label="Fz moment arm")

plt.xlabel("Steer (deg)")
plt.ylabel("Moment arm (mm)")
plt.title("Steering Moment Arm vs Steering Angle")
plt.grid()
plt.legend()
plt.show()
