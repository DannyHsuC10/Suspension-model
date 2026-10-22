import sys
from pathlib import Path

from data_visualization.Dynamic_Visualizer_quarter import Visualizer
from data_visualization.Dynamic_Visualizer_roll import RollCarVisualizer
from data_visualization.Simulation_Logger import SimulationLogger

current_dir = Path(__file__).resolve().parent

"""
之前有改格式data需要重新建立
"""

#result_file = current_dir/ "data"/ "quarter" / "quarter_impulse.pkl"
result_file = current_dir/ "data"/ "LR" / "half_sin_L.pkl"

# =====================================================
# Load simulation result

logger = SimulationLogger.load(result_file)
result = logger.result()

# Quick check
print("Loaded result:")
for key, value in result.items():
    print(
        f"{key}: shape={value.shape}")


# =====================================================
# Visualization
viewer = RollCarVisualizer(result)# 1/2


#viewer = Visualizer(result)# 1/4

viewer.show()