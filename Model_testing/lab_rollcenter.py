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

t = np.arange(0,0.25,state.dt)

sus.h_rc = 0.025


for ti in t:

    #hr.road_impulse_left(state)
    #hr.road_step_both(state)
    #hr.road_sin_left(state)
    #hr.road_sin_alternate(state)
    hr.Cornering_ramp_to_max(state)
    
    sm.Half_rollcenter_all_model(sus,state)

    state.Integration(sus)

    logger.add_state(state)

# =======================================================================
# show

#viewer = RollCarVisualizer(logger.result(),track_width=1.28)
#viewer.show()


result = logger.result()

t = np.asarray(result["t"])

Fs_l = np.asarray(result["Fs_l"])
Fs_r = np.asarray(result["Fs_r"])

Ft_l = np.asarray(result["Ft_l"])
Ft_r = np.asarray(result["Ft_r"])

dF_roll_l = np.asarray(result["dF_roll_l"])
dF_roll_r = np.asarray(result["dF_roll_r"])

zu_l = np.asarray(result["left_zu"])
zu_r = np.asarray(result["right_zu"])


# Geometry 造成的左右載荷差
dF_geo = 2 * dF_roll_r

# Elastic contribution
dF_spring = Fs_r- Fs_l

dF_tir = Ft_r-Ft_l


def trim_after_zero(y):
    y = y.copy()

    idx = np.where(y < 0)[0]
    if len(idx) > 0:
        y[idx[0]:] = 0.0       # 或 np.nan
    return y

LT_rate_geo = trim_after_zero(np.gradient(dF_geo, t))
LT_rate_spring = trim_after_zero(np.gradient(dF_spring, t))

LT_rate_total = LT_rate_geo + LT_rate_spring

#＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝
plt.figure(figsize=(10,6))

plt.axhline(0, color='k', linewidth=1)

signals = {
    "Total": LT_rate_total,
    "Geometry": LT_rate_geo,
    "Elastic": LT_rate_spring,
}

styles = {
    "Total": "-",
    "Geometry": "--",
    "Elastic": ":",
}

for name, y in signals.items():

    # 正峰值
    idx = np.argmax(y)
    peak = y[idx]
    t_peak = t[idx]

    plt.plot(t, y,
             linestyle=styles[name],
             linewidth=2,
             label=name)

    plt.scatter(t_peak, peak,
                s=80,
                marker='o')

    plt.annotate(
        f"peak : {peak:.0f}, t : {t_peak:.3f}s",
        xy=(t_peak, peak),
        xytext=(10,10),
        textcoords="offset points",
        fontsize=9,
        arrowprops=dict(arrowstyle="->")
    )

plt.xlabel("Time (s)")
plt.ylabel("Load Transfer Rate (N/s)")
plt.title("Transient Load Transfer Rate")

plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()


# print
#========================

signals = {
    "Total": LT_rate_total,
    "Geometry": LT_rate_geo,
    "Elastic": LT_rate_spring,
}

print("\n========== Load Transfer Rate Peak ==========")
print(f"{'Signal':<12}{'Peak (N/s)':>15}{'Time (s)':>12}")
print("-"*40)

for name, signal in signals.items():

    idx = np.argmax(signal)

    print(
        f"{name:<12}"
        f"{signal[idx]:15.2f}"
        f"{t[idx]:12.6f}"
    )


