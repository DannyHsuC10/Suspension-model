"""Regression checks for KC force signs, damping ratios and replacement."""

import copy
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

from dynamics_analyze.model import model
from dynamics_analyze.model.KC_sys import LinearKC, LookupKC, NullKC, Ohlins, SeriesKC
from dynamics_analyze.road import quarter_road
from dynamics_analyze.suspension_set.Suspension import (
    SuspensionFR, SuspensionLR, SuspensionQuarter, SuspensionQuarteravg,
    SuspensionQuarterheave, SuspensionQuartertender,
)
from dynamics_analyze.suspension_set.suspension_state import (
    FR_suspension_state, LR_suspension_state, quarter_suspension_state,
)


QUARTER_CASES = (
    (SuspensionQuarter, 70000),
    (SuspensionQuartertender, 23800),
    (SuspensionQuarteravg, 43000),
    (SuspensionQuarterheave, 70000),
)


def random_corner(rng):
    state = quarter_suspension_state()
    state.zs, state.zu, state.zr = rng.uniform(-0.03, 0.03, 3)
    state.vzs, state.vzu, state.vzr = rng.uniform(-0.3, 0.3, 3)
    return state


class ForceProbe:
    """A force-only component: models must not reconstruct k*x + c*v."""

    def __init__(self, value):
        self.value = value
        self.calls = []

    def force(self, displacement, velocity):
        self.calls.append((displacement, velocity))
        return self.value, {}


class KCModelTests(unittest.TestCase):
    def assert_kc(self, system, k, c):
        self.assertEqual(system.spring.stiffness(0.0), k)
        self.assertAlmostEqual(system.damper.damping(0.0), c)
        self.assertEqual(system.motion_ratio.ratio(0.0), 1.0)

    def assert_damping_ratio(self, system, mass):
        k = system.spring.stiffness(0.0)
        c = system.damper.damping(0.0)
        self.assertAlmostEqual(c / (2 * np.sqrt(k * mass)), 0.7)

    def test_stiffness_and_damping_ratios(self):
        for factory, k in QUARTER_CASES:
            with self.subTest(factory=factory.__name__):
                sus = factory()
                self.assert_kc(sus.corner, k, 1.4 * np.sqrt(k * 13))
                self.assert_kc(sus.tire, 80000, 1.4 * np.sqrt(80000 * 13))
                self.assertEqual(sus.zeta, 0.7)
                for system in (sus.corner, sus.tire):
                    self.assert_damping_ratio(system, sus.mu)
                self.assertAlmostEqual(sus.ride_rate, k * 80000 / (k + 80000))
                self.assertEqual((sus.ms, sus.mu), (67, 13))

        lr = SuspensionLR()
        self.assertEqual(lr.zeta, 0.7)
        self.assert_kc(lr.heave, 18000, 1.4 * np.sqrt(18000 * 15))
        self.assert_kc(lr.roll, 50000, 1.4 * np.sqrt(50000 * 15))
        self.assert_kc(lr.frame, 500000, 1.4 * np.sqrt(500000 * 65))
        self.assert_damping_ratio(lr.frame, lr.m_frame)
        for system in (lr.heave, lr.roll, lr.tire_left, lr.tire_right):
            self.assert_damping_ratio(system, lr.mu)
        for tire in (lr.tire_left, lr.tire_right):
            self.assert_kc(tire, 800000, 1.4 * np.sqrt(800000 * 15))
        for corner in (lr.corner_left, lr.corner_right):
            self.assertEqual(corner.force(0.03, 0.2)[0], 0.0)

        fr = SuspensionFR()
        self.assertEqual(fr.zeta, 0.7)
        self.assert_kc(fr.heave_f, 30000, 1.4 * np.sqrt(30000 * 15))
        self.assert_kc(fr.heave_r, 28000, 1.4 * np.sqrt(28000 * 15))
        for system in (fr.heave_f, fr.heave_r, fr.tire_front, fr.tire_rear):
            self.assert_damping_ratio(system, fr.mu)
        for tire in (fr.tire_front, fr.tire_rear):
            self.assert_kc(tire, 80000, 1.4 * np.sqrt(80000 * 15))
        for corner in (fr.corner_front, fr.corner_rear):
            self.assertEqual(corner.force(0.03, 0.2)[0], 0.0)

    def test_objects_are_independent(self):
        first, second = SuspensionLR(), SuspensionLR()
        first.tire_left.spring.k = 12345
        self.assertEqual(first.tire_right.spring.k, 800000)
        self.assertEqual(second.tire_left.spring.k, 800000)
        first.heave.damper.c = 234
        self.assertAlmostEqual(second.heave.damper.c, 1.4 * np.sqrt(18000 * 15))

    def test_quarter_matches_linear_formula(self):
        rng = np.random.default_rng(20)
        for factory, k in QUARTER_CASES:
            sus = factory()
            for _ in range(30):
                state = random_corner(rng)
                fs = k * (state.zs - state.zu) + 1.4 * np.sqrt(k * 13) * (state.vzs - state.vzu)
                ft = 80000 * (state.zu - state.zr) + 1.4 * np.sqrt(80000 * 13) * (state.vzu - state.vzr)
                model.Quarter_suspension_model(sus, state)
                np.testing.assert_allclose(
                    [state.Fs, state.Ft, state.azs, state.azu],
                    [fs, ft, -fs / 67, (fs - ft) / 13], atol=1e-10,
                )

    def test_lr_matches_matrix_equations(self):
        rng = np.random.default_rng(21)
        sus = SuspensionLR()
        sus.h_rc = 0.025
        # Independent modal-to-corner stiffness/damping matrices.
        stiffness = np.array([[34000, -16000], [-16000, 34000]])
        c_heave, c_roll = 1.4 * np.sqrt(np.array([18000, 50000]) * 15)
        damping = np.array([[c_heave + c_roll, c_heave - c_roll],
                            [c_heave - c_roll, c_heave + c_roll]]) / 2
        c_tire = 1.4 * np.sqrt(800000 * 15)
        c_frame = 1.4 * np.sqrt(500000 * 65)
        for _ in range(30):
            state = LR_suspension_state(random_corner(rng), random_corner(rng))
            state.ay = rng.uniform(-10, 10)
            state.d_frame, state.v_frame = rng.uniform(-0.005, 0.005, 2)
            corners = (state.left, state.right)
            ds = np.array([s.zs - s.zu for s in corners])
            dv = np.array([s.vzs - s.vzu for s in corners])
            fs = stiffness @ ds + damping @ dv
            ft = np.array([800000 * (s.zu - s.zr) + c_tire * (s.vzu - s.vzr) for s in corners])
            frame = 500000 * state.d_frame + c_frame * state.v_frame
            moment_rc = -state.ay * 160 * 0.025
            moment = (fs[1] - fs[0]) * 0.625 + state.ay * 160 * 0.3 + moment_rc
            body_a = -fs.sum() / 130
            alpha = moment / 30
            model.Half_rollcenter_LR_model(sus, state)
            np.testing.assert_allclose([state.Fs_l, state.Fs_r], fs, atol=1e-10)
            np.testing.assert_allclose([state.Ft_l, state.Ft_r], ft, atol=1e-10)
            np.testing.assert_allclose(
                [state.left.azs, state.right.azs, state.left.azu, state.right.azu, state.a_frame],
                [body_a + 0.625 * alpha, body_a - 0.625 * alpha,
                 (fs[0] - ft[0] - frame) / 15, (fs[1] - ft[1] + frame) / 15,
                 (moment_rc / 1.25 - frame) / 65], atol=1e-10,
            )
            self.assertAlmostEqual(state.F_heave, 18000 * ds.sum() + c_heave * dv.sum())
            self.assertAlmostEqual(state.F_roll, 50000 * (ds[1] - ds[0]) + c_roll * (dv[1] - dv[0]))

    def test_fr_equations_with_dissipative_tire_damping(self):
        rng = np.random.default_rng(22)
        sus = SuspensionFR()
        sus.anti_dive_front, sus.anti_squat_rear = 0.12, 0.18
        for _ in range(30):
            state = FR_suspension_state(random_corner(rng), random_corner(rng))
            state.ax = rng.uniform(-10, 10)
            state.Fx_front, state.Fx_rear = rng.uniform(-1000, 1000, 2)
            corners = (state.front, state.rear)
            fs = np.array([30000, 28000]) * [s.zs - s.zu for s in corners]
            fs += 1.4 * np.sqrt(np.array([30000, 28000]) * 15) * [s.vzs - s.vzu for s in corners]
            c_tire = 1.4 * np.sqrt(80000 * 15)
            net = fs + np.array([-80000 * (s.zu - s.zr) - c_tire * (s.vzu - s.vzr) for s in corners])
            net += np.array([state.Fx_front, -state.Fx_rear]) * 0.3 / 1.53
            inertia = state.ax * 160 * 0.3
            anti = 0.12 if state.ax > 0 else 0.18
            moment = (fs[1] - fs[0]) * 0.765 + inertia * (1 - anti)
            body_a = -fs.sum() / 130
            alpha = moment / 50
            model.Half_suspension_model(sus, state)
            np.testing.assert_allclose([state.front.Fs, state.rear.Fs], fs, atol=1e-10)
            np.testing.assert_allclose([state.front.Ft, state.rear.Ft], net, atol=1e-10)
            np.testing.assert_allclose(
                [state.front.azs, state.rear.azs, state.front.azu, state.rear.azu, state.My],
                [body_a + 0.765 * alpha, body_a - 0.765 * alpha, net[0] / 15, net[1] / 15, moment],
                atol=1e-10,
            )

    def test_models_use_each_component_force(self):
        sus = SuspensionQuarter()
        sus.corner, sus.tire = ForceProbe(-100), ForceProbe(25)
        state = quarter_suspension_state()
        state.zs, state.zu, state.zr = 0.03, 0.01, -0.01
        state.vzs, state.vzu, state.vzr = 0.3, 0.1, -0.1
        model.Quarter_suspension_model(sus, state)
        self.assertEqual((state.Fs, state.Ft), (100, -25))
        np.testing.assert_allclose(sus.corner.calls, [(0.02, 0.2)])
        np.testing.assert_allclose(sus.tire.calls, [(0.02, 0.2)])

        lr = SuspensionLR()
        fields = ("heave", "roll", "corner_left", "corner_right", "tire_left", "tire_right", "frame")
        for field, value in zip(fields, (-100, -60, -10, -20, -200, -300, -40)):
            setattr(lr, field, ForceProbe(value))
        state = LR_suspension_state(copy.deepcopy(state), quarter_suspension_state())
        model.Half_rollcenter_LR_model(lr, state)
        self.assertEqual((state.Fs_l, state.Fs_r, state.Ft_l, state.Ft_r, state.dF_roll), (30, 100, 200, 300, 40))
        expected_inputs = ((0.02, 0.2), (-0.02, -0.2), (0.02, 0.2), (0, 0), (0.02, 0.2), (0, 0), (0, 0))
        for field, expected in zip(fields, expected_inputs):
            np.testing.assert_allclose(getattr(lr, field).calls, [expected], atol=1e-15)

        fr = SuspensionFR()
        fields = ("heave_f", "heave_r", "corner_front", "corner_rear", "tire_front", "tire_rear")
        for field, value in zip(fields, (-100, -60, -10, -20, -200, -300)):
            setattr(fr, field, ForceProbe(value))
        state = FR_suspension_state(state.left, state.right)
        model.Half_suspension_model(fr, state)
        self.assertEqual((state.front.Fs, state.rear.Fs, state.front.Ft, state.rear.Ft), (110, 80, -90, -220))
        for field, expected in zip(fields, ((0.02, 0.2), (0, 0), (0.02, 0.2), (0, 0), (0.02, 0.2), (0, 0))):
            np.testing.assert_allclose(getattr(fr, field).calls, [expected], atol=1e-15)

    def test_nonlinear_systems_can_replace_quarter_components(self):
        systems = (
            NullKC(),
            SeriesKC(main=50000, low=15000, travel=0.03, low_comp=1800,
                     high_comp=700, low_reb=2500, high_reb=1000, transition=0.05),
            Ohlins(k=30000, low_comp=1800, high_comp=700, low_reb=2500,
                   high_reb=1000, transition=0.05, mr_value=0.8),
            LookupKC(spring_displacement=[-0.1, 0, 0.1], spring_force=[-3000, 0, 3000],
                     damper_velocity=[-1, 0, 1], damper_force=[-1500, 0, 1500],
                     mr_displacement=[-0.1, 0, 0.1], mr_ratio=[0.8, 1, 1.2]),
        )
        for system in systems:
            sus = SuspensionQuarter()
            sus.corner = system
            sus.tire = LinearKC(80000, 50, mr_value=0.9)
            for displacement in (-0.05, 0.02):
                state = quarter_suspension_state()
                state.zs, state.vzs = displacement, 0.3
                state.zu, state.vzu = 0.003, -0.1
                force = system.force(state.zs - state.zu, 0.4)[0]
                tire_force = sus.tire.force(0.003, -0.1)[0]
                model.Quarter_suspension_model(sus, state)
                self.assertAlmostEqual(state.Fs, -force)
                self.assertAlmostEqual(state.azu, (-force + tire_force) / 13)

    def test_motion_ratio_and_ride_rate_follow_replacement(self):
        sus = SuspensionQuarter()
        sus.corner = LinearKC(k=30000, c=1000, mr_value=0.8)
        self.assertAlmostEqual(sus.ride_rate, 19200 * 80000 / 99200)
        state = quarter_suspension_state()
        state.zs, state.vzs = 0.01, 0.2
        model.Quarter_suspension_model(sus, state)
        self.assertAlmostEqual(state.Fs, 320)
        sus.corner = NullKC()
        self.assertEqual(sus.ride_rate, 0)
        sus.tire = NullKC()
        self.assertEqual(sus.ride_rate, 0)

    def test_quarter_bump_trajectories_match_linear_reference(self):
        for factory, k in QUARTER_CASES:
            sus = factory()
            state, reference = quarter_suspension_state(), quarter_suspension_state()
            c = 1.4 * np.sqrt(k * 13)
            for _ in range(2000):
                quarter_road.road_bump(state, height=0.02)
                quarter_road.road_bump(reference, height=0.02)
                model.Quarter_suspension_model(sus, state)
                fs = k * (reference.zs - reference.zu) + c * (reference.vzs - reference.vzu)
                ft = 80000 * (reference.zu - reference.zr) + 1.4 * np.sqrt(80000 * 13) * (reference.vzu - reference.vzr)
                reference.azs, reference.azu = -fs / 67, (fs - ft) / 13
                state.Integration(sus)
                reference.Integration(sus)
                np.testing.assert_allclose(
                    [state.zs, state.zu, state.vzs, state.vzu],
                    [reference.zs, reference.zu, reference.vzs, reference.vzu], atol=1e-12,
                )

    def test_half_car_simulations_are_finite(self):
        lr, fr = SuspensionLR(), SuspensionFR()
        lr.h_rc = 0.025
        roll = LR_suspension_state(quarter_suspension_state(), quarter_suspension_state())
        pitch = FR_suspension_state(quarter_suspension_state(), quarter_suspension_state())
        for _ in range(2000):
            phase = 2 * np.pi * roll.t
            roll.left.zr = 0.01 * np.sin(phase)
            roll.left.vzr = 0.02 * np.pi * np.cos(phase)
            roll.ay = -3 if roll.t > 0.1 else 0
            pitch.front.zr = 0.01 * np.sin(phase)
            pitch.front.vzr = 0.02 * np.pi * np.cos(phase)
            pitch.ax = -3 if pitch.t > 0.1 else 0
            pitch.Fx_front = pitch.Fx_rear = pitch.ax * 80
            model.Half_rollcenter_LR_model(lr, roll)
            model.Half_suspension_model(fr, pitch)
            roll.Integration(lr)
            pitch.Integration(fr)
            values = [roll.roll, roll.d_frame, pitch.pitch]
            for corner in (roll.left, roll.right, pitch.front, pitch.rear):
                values.extend([corner.zs, corner.zu, corner.vzs, corner.vzu, corner.Fs, corner.Ft])
            self.assertTrue(np.all(np.isfinite(values)))
        self.assertGreater(abs(roll.roll), 1e-6)
        self.assertGreater(abs(pitch.pitch), 1e-6)

    def test_imports_from_experiment_directory(self):
        directory = Path(__file__).resolve().parents[1]
        result = subprocess.run(
            [sys.executable, "-c",
             "from suspension_set.Suspension import SuspensionQuarter; "
             "from model.KC_sys import KC_library; "
             "from model import model; "
             "import lab_tender; "
             "assert SuspensionQuarter().tire.force(0.01, 0)[0] == -800"],
            cwd=directory, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
