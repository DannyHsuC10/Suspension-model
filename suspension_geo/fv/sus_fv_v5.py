import csv
import sys
from pathlib import Path

# Support direct script execution as well as python -m from the project root.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from suspension_geo import geometry as ge
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from suspension_geo import visualization as vis
from suspension_geo.car import Car, SuspensionF
from suspension_geo.analysis_settings import AnalysisSettings
from matplotlib.widgets import Slider
from suspension_geo.visualization import LinkPlot, PointPlot, PolygonPlot
from suspension_geo import Geometric_Analysis as ga
car = Car()
suspension_f = SuspensionF()
analysis_settings = AnalysisSettings()
POINTS_CSV_PATH = Path(__file__).resolve().with_name("sus_fv_v5_points.csv")


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
fig, ax = plt.subplots(figsize=analysis_settings.front_view_figure_size)

# 靜態幾何計算
static_pts = sus_helper.calc_static_points()

# 建立左右兩側懸吊
right_sus = ga.SuspensionSide("Right", is_left=False)
right_sus.setup_right_side(static_pts)

left_sus = ga.SuspensionSide("Left", is_left=True)
left_sus.setup_from_right_template(right_sus)

# 與 mapping / lab 共用姿態與有號 camber 計算，並記錄初始接地點高度。
solver = ga.SuspensionDirectSolver(sus_helper, left_sus, right_sus)

# 加入車體 Polygon Patch
body_patch = patches.Polygon(
    analysis_settings.front_view_body_outline(car.bottom), closed=True, color="orange"
)
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
    target_travel=analysis_settings.front_view_target_travel,
)

# GUI 滑桿設定
fig.subplots_adjust(bottom=analysis_settings.front_view_plot_bottom, top=0.78)
ax_slider_r = plt.axes(analysis_settings.front_view_slider_axes[0])
ax_slider_l = plt.axes(analysis_settings.front_view_slider_axes[1])

slider_r = Slider(
    ax_slider_r, "Right Arm Angle", theta_down_r, theta_up_r, valinit=theta0
)
slider_l = Slider(
    ax_slider_l, "Left Arm Angle", theta_down_r, theta_up_r, valinit=theta0
)

# 固定在圖形上方，避免讀值被懸吊線條遮住。
state_text = fig.text(
    0.12, 0.96, "",
    va="top", ha="left", fontsize=10, fontfamily="monospace",
    linespacing=1.5,
    bbox=dict(boxstyle="round,pad=0.6", facecolor="#f5f7fa", edgecolor="#cbd5e1"),
)


def format_signed(value, decimals):
    # 消除浮點誤差造成的 -0.000，但保留實際有意義的正負號。
    if abs(value) < 0.5 * 10 ** (-decimals):
        value = 0.0
    return f"{value:+.{decimals}f}"


def export_points_csv():
    """匯出目前前視姿態；X 為橫向、Y 為高度，座標單位 mm。"""
    point_definitions = (
        ("cpb_rl", "下控制臂車身側接點"),
        ("cpb_ru", "上控制臂車身側接點"),
        ("kua", "上控制臂與羊角接點"),
        ("lak", "下控制臂與羊角接點"),
        ("kg", "主銷地面參考點"),
        ("gnd", "輪胎接地點"),
        ("ICo", "初始瞬心參考點"),
    )
    points = []
    for side in (left_sus, right_sus):
        for point_id, description in point_definitions:
            points.append(
                (side.name, point_id, description, getattr(side, point_id).pos)
            )
        # 平行控制臂沒有有限 IC，避免匯出求解器保留的上一個位置。
        ic_pos = sus_helper.line_intersection(
            side.cpb_rl, side.lak, side.cpb_ru, side.kua
        )
        points.append((side.name, "IC", "瞬時中心", ic_pos))
        for index, point in enumerate(side.wheel_pts):
            points.append(
                (side.name, f"wp{index}", f"輪胎外框點 {index}", point.pos)
            )
    points.extend((
        ("Center", "cg", "重心", cg_pt.pos),
        ("Center", "rc", "滾動中心", rc_pt.pos),
    ))

    # UTF-8 BOM 讓 Excel 能直接辨識中文；每次覆寫為當前姿態。
    with POINTS_CSV_PATH.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("side", "point", "description", "x_mm", "y_mm"))
        for side, point_id, description, position in points:
            coordinates = ("", "")
            if position is not None and np.isfinite(position).all():
                coordinates = tuple(float(value) * 1000.0 for value in position)
            writer.writerow((side, point_id, description, *coordinates))


def update(val):
    state = solver.solve_by_thetas(slider_l.val, slider_r.val)
    new_rc_pos = state["rc_pos"]

    if new_rc_pos is not None and np.isfinite(new_rc_pos).all():
        rc_pt.move(new_rc_pos)
        rc_x = format_signed(new_rc_pos[0] * 1000.0, 2)
        rc_y = format_signed(new_rc_pos[1] * 1000.0, 2)
        rc_readout = f"Roll Center: X {rc_x} mm   Y {rc_y} mm"
    else:
        # 無有限交點時隱藏 RC 點，避免留下上一個姿態的座標。
        rc_pt.move([np.nan, np.nan])
        rc_readout = "Roll Center: X N/A         Y N/A"

    state_text.set_text(
        f"Left Camber: {format_signed(state['camber_left_deg'], 3)} deg   "
        f"Right Camber: {format_signed(state['camber_right_deg'], 3)} deg\n"
        f"{rc_readout}\n"
        f"Roll: {format_signed(state['roll_deg'], 3)} deg   "
        f"Heave: {format_signed(state['heave_mm'], 2)} mm"
    )
    update_gnd_axis()
    scene.update()
    fig.canvas.draw_idle()
    try:
        export_points_csv()
    except OSError as exc:
        # CSV 被其他程式鎖定時仍可操作懸吊圖，並顯示寫入失敗原因。
        print(f"[CSV] Could not write {POINTS_CSV_PATH}: {exc}", file=sys.stderr)

slider_r.on_changed(update)
slider_l.on_changed(update)

# 圖表顯示設定
ax.set_aspect("equal")
ax.set_xlim(*analysis_settings.front_view_x_limits)
ax.set_ylim(*analysis_settings.front_view_y_limits)
ax.grid(True)
ax.set_title("Suspension Kinematics Front View")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")

# 開啟視窗時立即顯示初始姿態，不必先移動滑桿。
print(f"[CSV] Output path: {POINTS_CSV_PATH}")
update(None)

plt.show()
