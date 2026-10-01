import geometry as ge
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import visualization as vis
from car import Car, SuspensionF
from matplotlib.widgets import Slider
from visualization import LinkPlot, PointPlot, PolygonPlot
import Geometric_Analysis as ga
car = Car()
suspension_f = SuspensionF()


# ==============================================================================
# 1. 場景繪製管理器 Scene
# ==============================================================================
class Scene:

    def __init__(self, ax):
        self.ax = ax
        self.objects = []

    def add(self, obj, **kwargs):
        if isinstance(obj, ge.Point):
            self.objects.append(PointPlot(self.ax, obj, **kwargs))
        elif isinstance(obj, ge.Link):
            self.objects.append(LinkPlot(self.ax, obj, **kwargs))
        elif isinstance(obj, ge.Polygon):
            self.objects.append(PolygonPlot(self.ax, obj, **kwargs))

    def update(self):
        for obj in self.objects:
            obj.update()

# ==============================================================================
# 4. 主程式流程 (Main Execution)
# ==============================================================================
sus_helper = ga.SusGeometryHelper(car, suspension_f)
fig, ax = plt.subplots(figsize=suspension_f.front_view_figure_size)

# 靜態幾何計算
static_pts = sus_helper.calc_static_points()

# 建立左右兩側懸吊
right_sus = ga.SuspensionSide("Right", is_left=False)
right_sus.setup_right_side(static_pts)

left_sus = ga.SuspensionSide("Left", is_left=True)
left_sus.setup_from_right_template(right_sus)

# 加入車體 Polygon Patch
body_patch = patches.Polygon(car.body_face_f, closed=True, color="orange")
ax.add_patch(body_patch)

# Scene 管理與渲染
scene = Scene(ax)
right_sus.add_to_scene(scene)
left_sus.add_to_scene(scene)

# Connect the two tire contact points and continue their line beyond both ends.
gnd_axis_start = ge.Point(right_sus.gnd.pos, "Ground axis start")
gnd_axis_end = ge.Point(left_sus.gnd.pos, "Ground axis end")
gnd_axis = ge.Link(gnd_axis_start, gnd_axis_end, "Extended ground-contact axis")
scene.add(gnd_axis, color="0.25", linestyle="--", linewidth=1.5)


def update_gnd_axis():
    right_gnd = right_sus.gnd.pos
    left_gnd = left_sus.gnd.pos
    span = left_gnd - right_gnd
    gnd_axis_start.move(right_gnd - span)
    gnd_axis_end.move(left_gnd + span)


update_gnd_axis()

# 畫出重心與滾動中心
cg_pt = ge.Point(static_pts["cg"], "cg")
rc_pt = ge.Point(static_pts["rc"], "rc")
vis.plot_joint(ax, cg_pt, color="r")
vis.plot_joint(ax, rc_pt, color="r")
# 讓roll center 移動
scene.add(rc_pt, color="purple")

# 輔助參考線
plt.axhline(
    y=sus_helper.load_Radius, color="black", linestyle="--", linewidth=1
)
plt.axhline(y=0, color="black", linestyle="--", linewidth=1)
plt.axvline(x=0, color="black", linestyle="--", linewidth=1)

# 計算關節極限角度
theta0 = right_sus.lower_arm.angle
theta_down_r, theta_up_r = ga.calculate_theta_limits(
    sus=right_sus,
    target_travel=suspension_f.front_view_target_travel,
)

# GUI 滑桿設定
fig.subplots_adjust(bottom=suspension_f.front_view_plot_bottom)
ax_slider_r = plt.axes(suspension_f.front_view_slider_axes[0])
ax_slider_l = plt.axes(suspension_f.front_view_slider_axes[1])

slider_r = Slider(
    ax_slider_r, "Right Arm Angle", theta_down_r, theta_up_r, valinit=theta0
)
slider_l = Slider(
    ax_slider_l, "Left Arm Angle", theta_down_r, theta_up_r, valinit=theta0
)


# 超簡潔更新函式 (Update Callback)
def update(val):
    # 1. 更新左右兩側懸吊運動學 (內部會自動更新各自的 gnd, IC 與力線)
    right_sus.update_kinematics(slider_r.val, sus_helper)
    left_sus.update_kinematics(slider_l.val, sus_helper)

    # 2. 動態計算最新的 Roll Center (RC) 座標
    new_rc_pos = sus_helper.calc_roll_center(left_sus, right_sus)

    if new_rc_pos is not None:
        # 1. 更新內部 pos 數值
        rc_pt.pos = new_rc_pos
        
        # 2. 如果 rc_pt 內部有 Matplotlib artist/line 物件 (假設叫 artist 或 line)
        if hasattr(rc_pt, 'artist'):
            rc_pt.artist.set_data([new_rc_pos[0]], [new_rc_pos[1]])

    update_gnd_axis()

    # 4. 刷新畫布場景
    scene.update()

slider_r.on_changed(update)
slider_l.on_changed(update)

# 圖表顯示設定
ax.set_aspect("equal")
ax.set_xlim(*suspension_f.front_view_x_limits)
ax.set_ylim(*suspension_f.front_view_y_limits)
ax.grid(True)
plt.title("Suspension Kinematics Front View")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")

plt.show()
