"""Keep loaded wheel centers consistent across static tire and rim views."""

import unittest

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from static.analysis_settings import AnalysisSettings
from static.car import Car, SuspensionF, SuspensionR
from static.fv.interference_geometry import build_interference_geometry
from static.fv.kingpin_geometry import build_kingpin_geometry
from static.fv.sus_fv_v5 import FrontViewPlane, SideViewPlane
from static.fv.wheel_projection_geometry import build_wheel_projection_geometry


class WheelCenterTests(unittest.TestCase):
    def setUp(self):
        self.car = Car()
        # Deliberately different from each other and from the free radius.
        self.car.load_Radius = np.array([0.180, 0.160])
        self.settings = AnalysisSettings()
        front_sus, rear_sus = SuspensionF(), SuspensionR()
        self.front = FrontViewPlane(self.car, front_sus, "Front", 0.0, "blue")
        self.rear = FrontViewPlane(self.car, rear_sus, "Rear", -self.car.l, "red")
        self.sv = SideViewPlane(self.car, front_sus, rear_sus, self.settings)
        self.projections = build_wheel_projection_geometry(self.car, self.front, self.rear)

    def test_tire_and_rim_share_loaded_centers_and_preserve_dimensions(self):
        for corner, wheel in self.projections.items():
            axle = 0 if corner.startswith("F") else 1
            radius = self.car.load_Radius[axle]
            with self.subTest(corner=corner):
                self.assertAlmostEqual(wheel["center"][2], radius)
                for name, half_height in (("tire", self.car.free_radius),
                                          ("rim", self.car.rim_inner_diameter / 2)):
                    shape = wheel[name]
                    np.testing.assert_allclose(shape["vertices"].mean(axis=0), wheel["center"])
                    np.testing.assert_allclose(
                        [shape["vertices"][:, 2].min(), shape["vertices"][:, 2].max()],
                        [radius - half_height, radius + half_height],
                    )
                self.assertLess(wheel["tire"]["vertices"][:, 2].min(), 0.0)
        for prefix in ("F", "R"):
            left, right = (self.projections[prefix + side]["center"] for side in ("L", "R"))
            np.testing.assert_allclose(left, right * [1, -1, 1])

    def test_drawn_cylinders_have_loaded_center_and_free_radius(self):
        fig = plt.figure()
        self.addCleanup(plt.close, fig)
        ax = fig.add_subplot(111, projection="3d")
        for axle, plane in enumerate((self.front, self.rear)):
            plane.draw(ax, self.car, self.settings)
            radius = self.car.load_Radius[axle]
            for corner, artists in plane.tire_artists.items():
                with self.subTest(corner=corner):
                    for outline in artists["outlines"]:
                        x, y, z = map(np.asarray, outline.get_data_3d())
                        np.testing.assert_allclose([z.min(), z.max()],
                                                   radius + self.car.free_radius * np.array([-1, 1]))
                        np.testing.assert_allclose([x.min(), x.max()],
                                                   plane.longitudinal_x + self.car.free_radius * np.array([-1, 1]))
                        self.assertLess(z.min(), 0.0)

    def test_side_view_and_rim_clearance_use_the_same_loaded_centers(self):
        kingpins = build_kingpin_geometry(self.front, self.rear, self.sv)
        interference = build_interference_geometry(self.car, self.front, self.rear, kingpins)
        for corner, wheel in self.projections.items():
            axle = 0 if corner.startswith("F") else 1
            sv_side = self.sv.front if axle == 0 else self.sv.rear
            with self.subTest(corner=corner):
                self.assertAlmostEqual(sv_side.wheel_center[1], wheel["center"][2])
                clearance = interference[corner]
                np.testing.assert_allclose(clearance["tire_center"], wheel["center"])
                neutral_center = wheel["center"].copy()
                neutral_center[1] -= np.sign(neutral_center[1]) * self.car.tire_width / 2
                for pose, circle in clearance["circles"].items():
                    # The corrected inboard center must be used at every steer pose.
                    origin = clearance["axis_origin"]
                    unrotated = origin + circle["rotation_matrix"].T @ (circle["center"] - origin)
                    np.testing.assert_allclose(unrotated, neutral_center, atol=1e-12)
                    np.testing.assert_allclose(
                        np.linalg.norm(circle["points"] - circle["center"], axis=1),
                        clearance["radius"], atol=1e-12,
                    )


if __name__ == "__main__":
    unittest.main()
