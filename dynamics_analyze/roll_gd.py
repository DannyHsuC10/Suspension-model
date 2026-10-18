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

ay = -2

t = np.arange(0,5,state.dt)

for ti in t:

    hr.Cornering_left(state,ay = ay)
    
    sm.Half_rollcenter_all_model(sus,state)

    state.Integration(sus)

    logger.add_state(state)

# =======================================================================
# show

result = logger.result()

roll = np.rad2deg(np.asarray(result["roll"]))

roll_last = roll[len(roll)-1]

print(roll)
print(roll_last)
