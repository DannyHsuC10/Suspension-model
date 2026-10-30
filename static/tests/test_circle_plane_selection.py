"""Regression checks for independent selection from both steering circles."""

import unittest

import numpy as np

from static.fv.circle_plane_intersections import build_circle_plane_intersections


def make_geometry(positive=(0.0, 4.0, 0.0), negative=(0.0, 0.2, 0.0)):
    """XY circles intersect an X=0 arm plane at known Y coordinates."""
    planes, wheels = {}, {}
    for corner in ("FL", "FR"):
        for arm, joint in (("upper", "UBJ"), ("lower", "LBJ")):
            planes[f"{corner}_{arm}"] = {
                "vertices": np.array([[0., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
                "normal": np.array([1., 0., 0.]),
                "joint": joint,
            }
        wheels[corner] = {
            "radius": 1.0,
            "circles": {
                pose: {"center": np.array(center, dtype=float),
                       "normal": np.array([0., 0., 1.]), "angle_deg": angle}
                for pose, center, angle in (
                    ("positive", positive, 36.0), ("negative", negative, -36.0),
                )
            },
        }
    return planes, wheels


class CirclePlaneSelectionTests(unittest.TestCase):
    def test_each_circle_contributes_its_nearest_point(self):
        results = build_circle_plane_intersections(*make_geometry())
        for key, arm in results.items():
            with self.subTest(arm=key):
                self.assertEqual(len(arm["candidates"]), 4)
                # Both globally nearest points belong to the negative circle.
                self.assertEqual([p["pose"] for p in arm["candidates"][:2]],
                                 ["negative", "negative"])
                selected = arm["selected"]
                self.assertEqual([p["pose"] for p in selected], ["positive", "negative"])
                self.assertEqual([p["angle_deg"] for p in selected], [36.0, -36.0])
                np.testing.assert_allclose([p["point"] for p in selected],
                                           [[0., 3., 0.], [0., -.8, 0.]])
                np.testing.assert_allclose([p["distance_to_xz"] for p in selected], [3., .8])

    def test_missing_pose_is_not_replaced_by_other_circle(self):
        for missing in ("positive", "negative"):
            centers = {missing: (2.0, 4.0, 0.0)}
            for key, arm in build_circle_plane_intersections(*make_geometry(**centers)).items():
                with self.subTest(missing=missing, arm=key):
                    self.assertEqual(arm["by_pose"][missing]["status"], "none")
                    self.assertEqual(len(arm["candidates"]), 2)
                    self.assertEqual(len(arm["selected"]), 1)
                    expected_pose = "negative" if missing == "positive" else "positive"
                    self.assertEqual(arm["selected"][0]["pose"], expected_pose)
                    expected_y = -.8 if missing == "positive" else 3.
                    np.testing.assert_allclose(arm["selected"][0]["point"], [0., expected_y, 0.])

    def test_coincident_poses_keep_independent_source_records(self):
        planes, wheels = make_geometry(positive=(0., .2, 0.))
        for wheel in wheels.values():
            for circle in wheel["circles"].values():
                circle["angle_deg"] = 0.0
        for key, arm in build_circle_plane_intersections(planes, wheels).items():
            with self.subTest(arm=key):
                selected = arm["selected"]
                self.assertEqual([p["pose"] for p in selected], ["positive", "negative"])
                self.assertEqual([p["angle_deg"] for p in selected], [0., 0.])
                self.assertEqual(len(arm["candidates"]), 2)
                self.assertTrue(all(p["poses"] == ["positive", "negative"]
                                    for p in arm["candidates"]))
                np.testing.assert_allclose([p["point"] for p in selected],
                                           [[0., -.8, 0.], [0., -.8, 0.]])
                self.assertFalse(np.shares_memory(selected[0]["point"], selected[1]["point"]))
                selected[0]["point"][1] = 99.
                np.testing.assert_allclose(selected[1]["point"], [0., -.8, 0.])
                np.testing.assert_allclose(arm["candidates"][0]["point"], [0., -.8, 0.])
        for wheel in wheels.values():
            for circle in wheel["circles"].values():
                np.testing.assert_array_equal(circle["center"], [0., .2, 0.])

    def test_tangent_pose_contributes_its_single_point(self):
        for key, arm in build_circle_plane_intersections(
                *make_geometry(positive=(1., 4., 0.))).items():
            with self.subTest(arm=key):
                self.assertEqual(arm["by_pose"]["positive"]["status"], "tangent")
                self.assertEqual(len(arm["selected"]), 2)
                np.testing.assert_allclose(arm["selected"][0]["point"], [0., 4., 0.])


if __name__ == "__main__":
    unittest.main()
