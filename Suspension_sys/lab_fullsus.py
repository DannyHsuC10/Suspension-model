import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
current_dir = Path(__file__).resolve().parent

from data_visualization.Simulation_Logger import SimulationLogger
from data_visualization.Dynamic_Visualizer_quarter import wheel_monitor
from data_visualization.Dynamic_Visualizer_pitch import PitchCarVisualizer
from data_visualization.Dynamic_Visualizer_roll import RollCarVisualizer
from data_visualization.Dynamic_Visualizer_dashboard import SuspensionDashboard
from data_visualization.plot_results import plot_vehicle_results

from Suspension_LP03 import Suspension_A
from Vehicle_status import full_suspension_state
import road
from model import Suspension_model_v13 as sm

import Data_extraction as de

logger = SimulationLogger()
sus = Suspension_A()
state = full_suspension_state(sus)

state.load_state_json(current_dir / "static_state.json")



# ===========================================================================================
# Simulation
t = np.arange(0, 2, state.dt)

for ti in t:

    #road.road_FL_bump(state)
    road.road_D(state)
    sm.motion(state,sus)
       # Integration
    state.Integration(sus)    
    # Logging
    logger.add_state(state)

result = logger.result()


quarter_result = de.get_quarter_result(result, 0)
pitch_result = de.get_pitch_result(result)
f_result = de.get_roll_result(result,"F")

# Simulation end
state.save_state_json(
    current_dir / "final_state.json"
)

#plot_vehicle_results(result)


viewer = SuspensionDashboard(result)
#viewer = wheel_monitor(quarter_result)
#viewer = PitchCarVisualizer(pitch_result)
#viewer = RollCarVisualizer(f_result)
viewer.show()



logger.save(current_dir/"data.pkl")


