"""Check camber signs against tire orientations and the moving ground line."""

import unittest

import numpy as np

from suspension_geo import geometry as ge
from suspension_geo.Geometric_Analysis import SuspensionSide, SusGeometryHelper
from suspension_geo.car import Car, SuspensionF


class CamberTests(unittest.TestCase):
    def setUp(self):
        helper = SusGeometryHelper(Car(), SuspensionF())
        self.right = SuspensionSide("Right")
        self.right.setup_right_side(helper.calc_static_points())
        self.left = SuspensionSide("Left", is_left=True)
        self.left.setup_from_right_template(self.right)

    def rotate_tire(self, side, angle_deg):
        rotation = ge.RigidBody2D.rotation_matrix(np.radians(angle_deg))
        for point in side.wheel_pts:
            point.move(rotation @ point.pos)

    def test_static_tires_have_zero_camber(self):
        for side in (self.left, self.right):
            with self.subTest(side=side.name):
                self.assertAlmostEqual(side.calc_camber_angle(), 0.0, places=12)

    def test_tops_tilted_outward_are_positive(self):
        # Left tire top moves toward -X; right tire top moves toward +X.
        self.rotate_tire(self.left, 5.0)
        self.rotate_tire(self.right, -5.0)
        self.assertAlmostEqual(self.left.calc_camber_angle(), 5.0)
        self.assertAlmostEqual(self.right.calc_camber_angle(), 5.0)

    def test_tops_tilted_inward_are_negative(self):
        self.rotate_tire(self.left, -5.0)
        self.rotate_tire(self.right, 5.0)
        self.assertAlmostEqual(self.left.calc_camber_angle(), -5.0)
        self.assertAlmostEqual(self.right.calc_camber_angle(), -5.0)

    def test_tires_aligned_with_tilted_ground_have_zero_camber(self):
        for ground_angle in (-12.0, 12.0):
            with self.subTest(ground_angle=ground_angle):
                rotation = ge.RigidBody2D.rotation_matrix(np.radians(ground_angle))
                ground = rotation @ np.array([1.0, 0.0])
                for side in (self.left, self.right):
                    self.rotate_tire(side, ground_angle)
                    self.assertAlmostEqual(side.calc_camber_angle(ground), 0.0)
                    self.rotate_tire(side, -ground_angle)

    def test_camber_is_relative_to_ground_on_both_sides(self):
        self.rotate_tire(self.left, 8.0)
        self.rotate_tire(self.right, 8.0)
        ground = np.array([np.cos(np.radians(3.0)), np.sin(np.radians(3.0))])
        self.assertAlmostEqual(self.left.calc_camber_angle(ground), 5.0)
        self.assertAlmostEqual(self.right.calc_camber_angle(ground), -5.0)

    def test_uniform_rotation_preserves_signed_camber(self):
        for ground_angle in (-25.0, 25.0):
            with self.subTest(ground_angle=ground_angle):
                rotation = ge.RigidBody2D.rotation_matrix(np.radians(ground_angle))
                ground = rotation @ np.array([1.0, 0.0])
                self.rotate_tire(self.left, ground_angle - 5.0)
                self.rotate_tire(self.right, ground_angle + 5.0)
                self.assertAlmostEqual(self.left.calc_camber_angle(ground), -5.0)
                self.assertAlmostEqual(self.right.calc_camber_angle(ground), -5.0)
                self.rotate_tire(self.left, -(ground_angle - 5.0))
                self.rotate_tire(self.right, -(ground_angle + 5.0))

    def test_ground_vector_length_does_not_change_camber(self):
        self.rotate_tire(self.left, 5.0)
        self.rotate_tire(self.right, -5.0)
        for length in (0.01, 1.0, 100.0):
            with self.subTest(length=length):
                ground = np.array([length, 0.0])
                self.assertAlmostEqual(self.left.calc_camber_angle(ground), 5.0)
                self.assertAlmostEqual(self.right.calc_camber_angle(ground), 5.0)

    def test_small_signed_angles_are_preserved(self):
        self.rotate_tire(self.left, -0.0001)
        self.rotate_tire(self.right, -0.0001)
        self.assertAlmostEqual(self.left.calc_camber_angle(), -0.0001, places=10)
        self.assertAlmostEqual(self.right.calc_camber_angle(), 0.0001, places=10)

    def test_upright_fallback_uses_the_same_sign_convention(self):
        for side in (self.left, self.right):
            side.wheel_pts = []
            side.lak = ge.Point([0.0, 0.0])
            outward = -1.0 if side.is_left else 1.0
            for camber in (-5.0, 5.0):
                with self.subTest(side=side.name, camber=camber):
                    angle = np.radians(camber)
                    side.kua = ge.Point([outward * np.sin(angle), np.cos(angle)])
                    self.assertAlmostEqual(side.calc_camber_angle(), camber)

    def test_zero_ground_vector_is_rejected(self):
        with self.assertRaises(ValueError):
            self.right.calc_camber_angle([0.0, 0.0])

    def test_zero_tire_vector_is_rejected(self):
        self.right.wheel_pts[1].move(self.right.wheel_pts[0].pos)
        with self.assertRaises(ValueError):
            self.right.calc_camber_angle()


if __name__ == "__main__":
    unittest.main()
