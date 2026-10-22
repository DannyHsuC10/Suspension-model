import numpy as np
from pathlib import Path

from data_visualization.Dynamic_Visualizer_quarter import Visualizer
from data_visualization.Simulation_Logger import SimulationLogger

from Parameter_set.Suspension import SuspensionQuarter
from road import quarter_road as qr
from model import quarter_model as sm
from Parameter_set.suspension_state import quarter_suspension_state

sus = SuspensionQuarter()
randomroad = qr.RandomRoad()# 定義隨機路面
logger = SimulationLogger()
state = quarter_suspension_state()

current_dir = Path(__file__).resolve().parent

# ===========================================================================================
# Simulation
t = np.arange(0, 2, state.dt)

for ti in t:
    # Time

    # Road input
    zr = qr.road_bump(state ,height=0.005)# 凸起路面
    #zr = qr.road_step(state)
    #zr = qr.road_impulse(state)# 凸起路面
    #zr = qr.road_sine(state)# 凸起路面
    #zr = randomroad(state)# 隨機路面
    
    # Suspension model
    #sm.single_spring_model(sus,state)
    #sm.one_eighth_suspension_model(sus,state)
    #sm.Double_spring_model(sus,state)
    sm.Quarter_suspension_model(sus,state)

    # Integration
    state.Integration(sus)
    # Logging
    logger.add_state(state)


# =======================================================================
# show
path = current_dir / "twe_bump.gif"
viewer = Visualizer(logger.result(),save_filename=path)
viewer.show()#(save=True)

