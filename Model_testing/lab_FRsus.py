import numpy as np
from pathlib import Path

from data_visualization.Dynamic_Visualizer_quarter import Visualizer
from data_visualization.Dynamic_Visualizer_pitch import PitchCarVisualizer
from data_visualization.Simulation_Logger import SimulationLogger

from Parameter_set.Suspension import SuspensionFR
from Parameter_set.suspension_state import quarter_suspension_state,FR_suspension_state
from model import FR_model as sm
from road import FR_road as hr

current_dir = Path(__file__).resolve().parent

sus = SuspensionFR()
logger = SimulationLogger()
logger_f = SimulationLogger()
logger_r = SimulationLogger()
front = quarter_suspension_state()
rear = quarter_suspension_state()
state = FR_suspension_state(front,rear)# 兩個1/4組裝成1/2

# ===========================================================================================
# Simulation

t = np.arange(0,2,state.dt)


for ti in t:

    #hr.road_sine_front_rear(state)
    #hr.road_impulse_rear(state)
    hr.breaking(state)
    #hr.breaking(state)

    #sm.Half_suspension_2DOF_model(sus,state)
    sm.Half_suspension_model(sus,state)

    state.Integration(sus)
    logger.add_state(state)

    logger_f.add_state(state.front)
    logger_r.add_state(state.rear)

result = logger.result()


# =======================================================================
# show

viewer = PitchCarVisualizer(logger.result(),wheelbase=1.53)
viewer.show()

#viewer_r = Visualizer(logger_r.result())
#viewer_r.show()

#viewer_l = Visualizer(logger_l.result())
#viewer_l.show()
