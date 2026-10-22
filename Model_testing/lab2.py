import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import copy

from data_visualization.Dynamic_Visualizer_roll import RollCarVisualizer
from data_visualization.Simulation_Logger import SimulationLogger

from Parameter_set.Suspension import SuspensionLR
from Parameter_set.suspension_state import quarter_suspension_state,LR_suspension_state
from model import LR_model as sm
from road import LR_road as hr

current_dir = Path(__file__).resolve().parent

sus = SuspensionLR()
logger = SimulationLogger()

left = quarter_suspension_state()
right = quarter_suspension_state()

state = LR_suspension_state(copy.deepcopy(left),copy.deepcopy(right))# 兩個1/4組裝成1/2
# ===========================================================================================
# Simulation

t = np.arange(0,1,state.dt)


rc_list = np.arange(0.00, 0.081, 0.005)

peak_total = []
peak_geo = []
peak_spring = []

for h_rc in rc_list:

    # -------------------------
    # Reset simulation
    # -------------------------

    sus = SuspensionLR()
    sus.h_rc = h_rc

    logger = SimulationLogger()

    left = quarter_suspension_state()
    right = quarter_suspension_state()

    state = LR_suspension_state(
        copy.deepcopy(left),
        copy.deepcopy(right)
    )

    # -------------------------
    # Simulation
    # -------------------------

    for ti in np.arange(0, 1, state.dt):

        hr.Cornering_ramp_to_max(state)

        sm.Half_rollcenter_all_model(sus, state)

        state.Integration(sus)

        logger.add_state(state)

    # -------------------------
    # Result
    # -------------------------

    result = logger.result()

    t = np.asarray(result["t"])

    Fs_l = np.asarray(result["Fs_l"])
    Fs_r = np.asarray(result["Fs_r"])

    Ft_l = np.asarray(result["Ft_l"])
    Ft_r = np.asarray(result["Ft_r"])

    dF_roll_r = np.asarray(result["dF_roll_r"])

    dF_geo = 2*dF_roll_r
    dF_spring = Fs_r-Fs_l
    dF_total = Ft_r-Ft_l

    LT_rate_geo = np.gradient(dF_geo, t)
    LT_rate_spring = np.gradient(dF_spring, t)
    LT_rate_total = np.gradient(dF_total, t)

    peak_geo.append(np.max(LT_rate_geo))
    peak_spring.append(np.max(LT_rate_spring))
    peak_total.append(np.max(LT_rate_total))

    print(
        f"h_rc={h_rc:.3f} m | "
        f"Total={peak_total[-1]:8.1f} | "
        f"Geo={peak_geo[-1]:8.1f} | "
        f"Spring={peak_spring[-1]:8.1f}"
    )

plt.figure(figsize=(10,6))

# 找到 Total 最小值
idx_min = np.argmin(peak_total)

rc_best = rc_list[idx_min] * 1000   # mm
peak_best = peak_total[idx_min]

# 標示最佳點
plt.scatter(
    rc_best,
    peak_best,
    color='red',
    s=180,
    marker='*',
    zorder=5,
    label='Minimum Total'
)

plt.annotate(
    f"Best\nRC = {rc_best:.1f} mm\nPeak = {peak_best:.0f}",
    xy=(rc_best, peak_best),
    xytext=(15, -30),
    textcoords="offset points",
    fontsize=10,
    arrowprops=dict(arrowstyle="->", color="red")
)

plt.plot(rc_list*1000, peak_total, '-o', label="Total")
plt.plot(rc_list*1000, peak_geo, '--o', label="Geometry")
plt.plot(rc_list*1000, peak_spring, ':o', label="Elastic")

plt.xlabel("Roll Center Height (mm)")
plt.ylabel("Peak Load Transfer Rate (N/s)")
plt.title("Effect of Roll Center Height")

plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()