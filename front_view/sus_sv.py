import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from car import Car

car = Car()

# ==========================================================
# Parameters
# ==========================================================

mechanical_trail = 15 / 1000     # [m]
caster_deg = 4                    # [deg]
caster = np.deg2rad(caster_deg)   # [rad]

wheel_radius = car.free_radius
load_radius = car.load_Radius[0]
kingpin_length = 0.6
# vehicle
h_cog = car.h_cog
l = car.l
cg_to_axle = car.tf

front_brake_bias = 0.7
# anti dive
target_anti = 0.15

svsa_distance = 3
print("svsa_distance",svsa_distance)

# ==========================================================
# SVSA Calculation
# ==========================================================

def calc_svsa_angle(
    anti_percent,
    front_brake_bias,
    cg_height,
    wheelbase):

    tan_phi = (anti_percent * cg_height / (front_brake_bias * wheelbase))
    phi = np.arctan(tan_phi)

    return phi
# ==========================================================
# Geometry
# ==========================================================

svsa_angle = calc_svsa_angle(
    target_anti,
    front_brake_bias,
    h_cog,
    l)

print("svsa_angle",np.rad2deg(svsa_angle))

# ----------------------------------------------------------
# Wheel
# ----------------------------------------------------------

contact_point = np.array(
    [
        0,
        0
    ]
)

wheel_center = np.array(
    [
        0,
        load_radius
    ]
)


# ----------------------------------------------------------
# SVIC
# ----------------------------------------------------------

svic = np.array(
    [
        svsa_distance,
        svsa_distance*np.sin(svsa_angle)
    ]
)

# ----------------------------------------------------------
# King pin axis
# ----------------------------------------------------------

# ground intersection of king pin

kingpin_ground = np.array(
    [
        -mechanical_trail,
        0
    ]
)

# caster tilts rearward
kingpin_direction = np.array(
    [
        np.sin(caster),
        np.cos(caster)
    ]
)

kingpin_top = (
    kingpin_ground
    +
    kingpin_direction
    *
    kingpin_length
)

# ==========================================================
# King pin intersection with wheel center height
# ==========================================================

t = (
    wheel_center[1] - kingpin_ground[1]
) / kingpin_direction[1]

kingpin_wc = (
    kingpin_ground
    + t * kingpin_direction
)

tu = (
    wheel_center[1]+car.Au_f - kingpin_ground[1]
) / kingpin_direction[1]

kingpin_Au = (
    kingpin_ground
    + tu * kingpin_direction
)

tl = (
    wheel_center[1]-car.Al_f - kingpin_ground[1]
) / kingpin_direction[1]

kingpin_Al = (
    kingpin_ground
    + tl * kingpin_direction
)

# ==========================================================
# Plot
# ==========================================================

fig, ax = plt.subplots(
    figsize=(8,8)
)


# Ground

ax.axhline(
    0,
    linewidth=2
)



# Wheel

wheel = Circle(
    wheel_center,
    wheel_radius,
    fill=False,
    linewidth=3,
    label="Wheel"
)

ax.add_patch(wheel)

# Wheel center
ax.scatter(
    wheel_center[0],
    wheel_center[1],
    color="red",
    s=60,
    zorder=10,
    label="Wheel Center"
)

ax.scatter(
    kingpin_wc[0],
    kingpin_wc[1],
    s=60,
    zorder=10,
    label="King Pin @ Wheel Center Height"
)
# ----------------------------------------------------------
# King pin
# ----------------------------------------------------------

ax.plot(
    [
        kingpin_ground[0],
        kingpin_top[0]
    ],
    [
        kingpin_ground[1],
        kingpin_top[1]
    ],
    linewidth=3,
    label="King pin axis"
)


# ----------------------------------------------------------
# SVSA
# ----------------------------------------------------------

ax.plot(
    [
        svic[0],
        contact_point[0]
    ],
    [
        svic[1],
        contact_point[1]
    ],
    linewidth=3,
    label="SVSA"
)


# ==========================================================
# Axis
# ==========================================================
ax.set_xlabel("Longitudinal X [m]")
ax.set_ylabel("Vertical Y [m]")
ax.set_aspect("equal")
ax.grid()
ax.legend()
plt.show()