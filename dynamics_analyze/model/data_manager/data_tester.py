from data_loader import DataLoader
from data_path import DataPath
# spring
spring_data = DataLoader.load_csv(
    "data/spring.csv"
)

print(len(spring_data))

spring_data.plot(
    xlabel="Displacement (m)",
    ylabel="Force (N)"
)
print("======spring data======")
print(spring_data.x)
print(spring_data.y)
# ===================================
# damper

damper_data = DataLoader.load_csv(
    "data/damper.csv"
)

damper_data.plot(
    xlabel="Velocity (m/s)",
    ylabel="Force (N)"
)

print("======damper data======")
print(damper_data.x)
print(damper_data.y)
# ====================================
# MR

mr_data = DataLoader.load_csv(
    "data/motion_ratio.csv"
)

mr_data.plot(
    xlabel="Displacement (m)",
    ylabel="Motion Ratio"
)

print("======MR data======")
print(mr_data.x)
print(mr_data.y)