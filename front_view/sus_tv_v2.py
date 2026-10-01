"""Top-view steering geometry with a vertically moving steering rack."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.widgets import Slider

import geometry as ge
from car import Car
from visualization import LinkPlot, PointPlot, PolygonPlot


# -----------------------------------------------------------------------------
# Editable geometry and plot parameters
# Coordinates use X = vehicle longitudinal direction, Y = vehicle track direction.
# -----------------------------------------------------------------------------
car = Car()

WHEEL_CENTER_X = 0.0
AA_LOCAL_X = 0.0
AA_LOCAL_Y = 0.5
P_TIE_LEFT = np.array([-0.30, 0.45])

RACK_LENGTH = 0.5
RACK_X = -0.4
RACK_TRAVEL_LIMIT = 0.025

FIGURE_SIZE = (8, 8)
PLOT_X_LIMITS = (-0.9, 0.45)
PLOT_Y_LIMITS = (-0.95, 0.95)
SLIDER_AXES = (0.20, 0.07, 0.62, 0.035)


class Scene:
    """Updateable Matplotlib scene, following the front/side-view scripts."""

    def __init__(self, ax):
        self.ax = ax
        self.objects = []

    def add(self, obj, **kwargs):
        if isinstance(obj, ge.Point):
            self.objects.append(PointPlot(self.ax, obj, **kwargs))
        elif isinstance(obj, ge.Link):
            self.objects.append(LinkPlot(self.ax, obj, **kwargs))
        elif isinstance(obj, ge.Polygon):
            self.objects.append(PolygonPlot(self.ax, obj, **kwargs))

    def update(self):
        for obj in self.objects:
            obj.update()


def circle_intersections(center_a, radius_a, center_b, radius_b):
    """Return intersections of two circles, or an empty list if unreachable."""
    delta = center_b - center_a
    distance = np.linalg.norm(delta)
    if distance < 1e-12 or distance > radius_a + radius_b + 1e-10:
        return []
    if distance < abs(radius_a - radius_b) - 1e-10:
        return []
    along = (radius_a**2 - radius_b**2 + distance**2) / (2.0 * distance)
    height_sq = radius_a**2 - along**2
    if height_sq < -1e-10:
        return []
    height = np.sqrt(max(0.0, height_sq))
    direction = delta / distance
    perpendicular = np.array([-direction[1], direction[0]])
    base = center_a + along * direction
    if height < 1e-10:
        return [base]
    return [base + height * perpendicular, base - height * perpendicular]


class SteeringAssembly:
    """Tire, A-arm reference, and tie pickup rotating as one rigid body."""

    def __init__(self, name, side, car, rack_point, tire_color):
        self.name = name
        self.side = side  # left is +y, right is -y
        self.car = car
        self.wheel_center_initial = np.array(
            [WHEEL_CENTER_X, side * car.tf / 2.0]
        )

        tie_position = np.array([P_TIE_LEFT[0], side * P_TIE_LEFT[1]])
        self.tie_local = tie_position - self.wheel_center_initial
        self.aa_local = np.array([AA_LOCAL_X, side * AA_LOCAL_Y])
        half_width = car.tire_width / 2.0
        half_diameter = car.free_radius
        wheel_local = [
            np.array([-half_width, -half_diameter]),
            np.array([half_width, -half_diameter]),
            np.array([half_width, half_diameter]),
            np.array([-half_width, half_diameter]),
        ]

        self.wheel_center = ge.Point(self.wheel_center_initial, f"{name} wheel center")
        self.tie_point = ge.Point(
            self.wheel_center_initial + self.tie_local, f"{name} P_tie"
        )
        self.aa_point = ge.Point(
            self.wheel_center_initial + self.aa_local, f"{name} AA point"
        )
        self.wheel_points = [
            ge.Point(self.wheel_center_initial + p, f"{name} tire corner {i}")
            for i, p in enumerate(wheel_local)
        ]
        self.tire = ge.Polygon(self.wheel_points, name=f"{name} tire")
        self.rigid_body = ge.RigidBody2D(
            [self.wheel_center, self.tie_point, self.aa_point, *self.wheel_points],
            reference_pair=(self.wheel_center, self.tie_point),
            name=f"{name} steering upright",
        )
        self.aa_link = ge.Link(
            self.wheel_center, self.aa_point, f"{name} AA reference"
        )
        self.tie_rod = ge.Link(self.tie_point, rack_point, f"{name} tie rod")
        self.tie_rod_length = self.tie_rod.length
        self.initial_tie_vector = self.tie_local.copy()
        self.body_angle = 0.0
        self.color = tire_color

    def add_to_scene(self, scene):
        scene.add(self.tire, facecolor="none", edgecolor=self.color, linewidth=2.2)
        scene.add(self.aa_link, color="tab:green", linewidth=2)
        scene.add(self.tie_rod, color="tab:orange", linewidth=2)
        scene.add(self.wheel_center, color=self.color, markersize=5)
        scene.add(self.tie_point, color="tab:orange", markersize=5)
        scene.add(self.aa_point, color="tab:green", markersize=5)

    def solve_for_rack_point(self, rack_position):
        candidates = circle_intersections(
            self.wheel_center_initial,
            np.linalg.norm(self.initial_tie_vector),
            rack_position,
            self.tie_rod_length,
        )
        if not candidates:
            return False

        def branch_cost(point):
            vector = point - self.wheel_center_initial
            angle = np.arctan2(vector[1], vector[0]) - np.arctan2(
                self.initial_tie_vector[1], self.initial_tie_vector[0]
            )
            return abs(np.arctan2(np.sin(angle - self.body_angle),
                                  np.cos(angle - self.body_angle)))

        tie_position = min(candidates, key=branch_cost)
        self.rigid_body.update_from_two_points(self.wheel_center_initial, tie_position)
        vector = tie_position - self.wheel_center_initial
        self.body_angle = np.arctan2(vector[1], vector[0]) - np.arctan2(
            self.initial_tie_vector[1], self.initial_tie_vector[0]
        )
        self.tie_rod.end.move(rack_position)
        return True

    @property
    def tire_angle_from_y_deg(self):
        rotated_y = ge.RigidBody2D.rotation_matrix(self.body_angle) @ np.array([0.0, 1.0])
        return np.degrees(np.arctan2(rotated_y[0], rotated_y[1]))


fig, ax = plt.subplots(figsize=FIGURE_SIZE)
fig.subplots_adjust(bottom=0.18)
scene = Scene(ax)

# Rack position leaves useful travel before the
# tie rods reach a fully extended, unsolvable configuration.
rack_left = ge.Point([RACK_X, RACK_LENGTH / 2.0], "Rack left end")
rack_right = ge.Point([RACK_X, -RACK_LENGTH / 2.0], "Rack right end")
rack = ge.Link(rack_right, rack_left, "Steering rack")
left = SteeringAssembly("Left", +1.0, car, rack_left, "tab:blue")
right = SteeringAssembly("Right", -1.0, car, rack_right, "tab:red")
left.add_to_scene(scene)
right.add_to_scene(scene)
scene.add(rack, color="black", linewidth=3)
scene.add(rack_left, color="black", markersize=6)
scene.add(rack_right, color="black", markersize=6)

ax.axvline(0.0, color="0.55", linestyle="--", linewidth=1, label="Vehicle y-axis")
ax.axhline(0.0, color="0.75", linewidth=0.8)
ax.set_aspect("equal", adjustable="box")
ax.set_xlim(*PLOT_X_LIMITS)
ax.set_ylim(*PLOT_Y_LIMITS)
ax.grid(True, linestyle=":")
ax.set_xlabel("Longitudinal X [m]")
ax.set_ylabel("Lateral Y [m]")
ax.set_title("Top-View Steering Geometry")
ax.legend(handles=[
    Line2D([0], [0], color="black", lw=3, label="Steering rack"),
    Line2D([0], [0], color="tab:orange", lw=2, label="Fixed-length tie rod"),
    Line2D([0], [0], color="tab:green", lw=2, label="Rigid A-arm reference"),
    Line2D([0], [0], color="tab:blue", lw=2, label="Left tire"),
    Line2D([0], [0], color="tab:red", lw=2, label="Right tire"),
], loc="upper right", fontsize=8)

slider_ax = fig.add_axes(SLIDER_AXES)
rack_slider = Slider(
    slider_ax,
    "Rack Y travel [m]",
    -RACK_TRAVEL_LIMIT,
    RACK_TRAVEL_LIMIT,
    valinit=0.0,
)


def update(_value):
    dy = rack_slider.val
    rack_left.move(np.array([RACK_X, RACK_LENGTH / 2.0 + dy]))
    rack_right.move(np.array([RACK_X, -RACK_LENGTH / 2.0 + dy]))
    left_ok = left.solve_for_rack_point(rack_left.pos)
    right_ok = right.solve_for_rack_point(rack_right.pos)
    scene.update()

    if left_ok and right_ok:
        angle_delta = left.tire_angle_from_y_deg - right.tire_angle_from_y_deg
        print(
            f"Rack Δy={dy:+.4f} m | "
            f"Left tire vs y-axis={left.tire_angle_from_y_deg:+.3f}° | "
            f"Right tire vs y-axis={right.tire_angle_from_y_deg:+.3f}° | "
            f"Left−right angle difference={angle_delta:+.3f}°"
        )
        ax.set_title(
            "Top-View Steering Geometry | "
            f"Left={left.tire_angle_from_y_deg:+.2f}°, "
            f"Right={right.tire_angle_from_y_deg:+.2f}°, "
            f"Δ={angle_delta:+.2f}°"
        )
    else:
        print(f"Rack Δy={dy:+.4f} m: no valid rigid-body solution")
        ax.set_title("Rack position is outside the tie-rod geometry range")
    fig.canvas.draw_idle()


rack_slider.on_changed(update)
update(0.0)
plt.show()
