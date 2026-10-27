import numpy as np

from data_visualization.Simulation_Logger import SimulationLogger
from data_visualization.Dynamic_Visualizer_dashboard import SuspensionDashboard

from suspension_set.Suspension import Suspension_A
from suspension_set.suspension_state import full_suspension_state
from road import all_road
from model import Suspension_model_v13 as sm

logger = SimulationLogger()
sus = Suspension_A()
state = full_suspension_state(sus)

# ===========================================================================================
# Simulation
t = np.arange(0, 2, state.dt)

for ti in t:

    all_road.road_FL_bump(state)
    #all_road.road_D(state)
    sm.motion(state,sus)
       # Integration
    state.Integration(sus)    
    # Logging
    logger.add_state(state)

result = logger.result()


viewer = SuspensionDashboard(result)

viewer.show()


