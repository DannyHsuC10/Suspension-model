import numpy as np
from pathlib import Path

from data_visualization.Simulation_Logger import SimulationLogger


from road import quarter_road,LR_road,FR_road
from model import quarter_model,LR_model,FR_model
from Parameter_set.Suspension import SuspensionQuarter,SuspensionLR,SuspensionFR
from Parameter_set.suspension_state import quarter_suspension_state,LR_suspension_state,FR_suspension_state
import copy

randomroad = quarter_road.RandomRoad()# 定義隨機路面
logger = SimulationLogger()

current_dir = Path(__file__).resolve().parent

ROADS_Q = {
    "step": quarter_road.road_step,
    "bump": quarter_road.road_bump,
    "impulse": quarter_road.road_impulse,
    "sine": quarter_road.road_sine,
    "random": randomroad,
}

MODELS_Q = {
    "quarter": quarter_model.Quarter_suspension_model,
    "double": quarter_model.Double_spring_model,
    "single": quarter_model.single_spring_model,
    "one_eighth": quarter_model.one_eighth_suspension_model,
}

DATA_Q = {
    "state": quarter_suspension_state(),
    "sus" : SuspensionQuarter(),
    "dir" : current_dir/ "data" / "quarter", 
}
# ==========================================================================
ROADS_LR = {
    "step_L": LR_road.road_step_left,
    "step": LR_road.road_step_both,
    "impulse_L": LR_road.road_impulse_left,
    "impulse": LR_road.road_impulse_both,
    "sin_L": LR_road.road_sin_left,
    "sin": LR_road.road_sin_both,
    "sin_A": LR_road.road_sin_alternate,
    "Corner": LR_road.Cornering_left,
}

MODELS_LR = {
    "2mass_2dof": LR_model.two_mass_suspension_2DOF_model,
    "2mass": LR_model.two_mass_suspension_model,
    "half_2dof": LR_model.Half_suspension_2DOF_model,
    "half": LR_model.Half_suspension_model,
}

DATA_LR = {
    "state": LR_suspension_state(quarter_suspension_state(),quarter_suspension_state()),
    "sus" : SuspensionLR(),
    "dir" : current_dir/ "data" / "LR" 
}
# ==========================================================================
ROADS_FR = {
    "impulse": FR_road.road_impulse_front_rear,
    "impulse_r": FR_road.road_impulse_rear,
    "impulse_D": FR_road.road_impulse_drive,
    "sin": FR_road.road_sine_front_rear,
    "ass": FR_road.accelerate,
    "brk": FR_road.breaking,
}

MODELS_FR = {
    "2dof": FR_model.Half_suspension_2DOF_model,
    "half": FR_model.Half_suspension_model,
}

DATA_FR = {
    "state": FR_suspension_state(quarter_suspension_state(),quarter_suspension_state()),
    "sus" : SuspensionFR(),
    "dir" : current_dir/ "data" / "FR" 
}

# ===========================================================================================
# Simulation

def run_simulation(data, road_func, model_func, save_path):
    data = copy.deepcopy(data)

    logger = SimulationLogger()
    state = data["state"]
    sus = data["sus"]

    t = np.arange(0, 2, state.dt)
    for ti in t:

        # Road
        road_func(state)

        # Suspension
        model_func(sus, state)

        # Integration
        state.Integration(sus)

        # Log
        logger.add_state(state)

    logger.save(save_path)
    
# ======================================================================
# 存檔

ROADS = ROADS_FR
MODELS = MODELS_FR
data = DATA_FR

for road_name, road_func in ROADS.items():

    for model_name, model_func in MODELS.items():

        filename = f"{model_name}_{road_name}.pkl"

        run_simulation(
            data,
            road_func,
            model_func,
            data["dir"] / filename
        )
        print("finish", filename)
