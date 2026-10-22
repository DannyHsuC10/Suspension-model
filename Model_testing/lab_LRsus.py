import numpy as np
from pathlib import Path

from data_visualization.Dynamic_Visualizer_quarter import Visualizer
from data_visualization.Dynamic_Visualizer_roll import RollCarVisualizer
from data_visualization.Simulation_Logger import SimulationLogger

from Parameter_set.Suspension import SuspensionLR
from Parameter_set.suspension_state import quarter_suspension_state,LR_suspension_state
from model import LR_model as sm
from road import LR_road as hr

current_dir = Path(__file__).resolve().parent

sus = SuspensionLR()
logger = SimulationLogger()
logger_l = SimulationLogger()
logger_r = SimulationLogger()
left = quarter_suspension_state()
right = quarter_suspension_state()
state = LR_suspension_state(left,right)# 兩個1/4組裝成1/2

# ===========================================================================================
# Simulation

t = np.arange(0,1,state.dt)


for ti in t:

    #hr.road_impulse_left(state)
    #hr.road_step_both(state)
    #hr.road_sin_left(state)
    #hr.road_sin_alternate(state)
    hr.Cornering_left(state)
    
    #sm.Half_suspension_2DOF_model(sus,state)
    sm.Half_suspension_model(sus,state)
    #sm.two_mass_suspension_model(sus,state)


    state.Integration(sus)
    logger.add_state(state)

    logger_r.add_state(state.right)
    logger_l.add_state(state.left)

# =======================================================================
# show

viewer = RollCarVisualizer(logger.result(),track_width=1.28)
viewer.show()

#viewer_r = Visualizer(logger_r.result())
#viewer_r.show()

#viewer_l = Visualizer(logger_l.result())
#viewer_l.show()
