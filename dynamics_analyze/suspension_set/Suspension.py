"""Vehicle parameters and replaceable KC systems for the dynamics models."""

import numpy as np

if __package__ == "suspension_set":
    from model.KC_sys import LinearKC, NullKC
else:
    from ..model.KC_sys import LinearKC, NullKC


class SuspensionQuarter:
    def __init__(self, k=70000):
        self.ms = 67      # sprung mass
        self.mu = 13      # unsprung mass
        self.m_corner = self.ms + self.mu

        # Keep the original Quarter convention: use unsprung mass as the
        # reference mass for suspension/tire component damping ratios.
        self.zeta = 0.7
        self.corner = LinearKC(
            k=k,
            c=2 * self.zeta * np.sqrt(k * self.mu),
            mr_value=1.0,
            name="corner",
        )
        k_tire = 80000
        self.tire = LinearKC(
            k=k_tire,
            c=2 * self.zeta * np.sqrt(k_tire * self.mu),
            name="tire",
        )

    @property
    def ride_rate(self):
        """Wheel/tire equivalent stiffness at zero displacement (N/m)."""
        kw = self.corner.spring.stiffness(0.0) * self.corner.motion_ratio.ratio(0.0)**2
        kt = self.tire.spring.stiffness(0.0) * self.tire.motion_ratio.ratio(0.0)**2
        return kw * kt / (kw + kt) if kw + kt != 0 else 0.0


class SuspensionQuartertender(SuspensionQuarter):
    def __init__(self):
        # Preserve the original linear tender approximation.
        super().__init__(k=70000 * 0.34)


class SuspensionQuarteravg(SuspensionQuarter):
    def __init__(self):
        super().__init__(k=(36000 + 50000) / 2)


class SuspensionQuarterheave(SuspensionQuarter):
    def __init__(self):
        super().__init__(k=70000)


class SuspensionLR:
    def __init__(self):
        self.track = 1.25
        self.Ix = 30
        self.ms = 65      # sprung mass per corner
        self.mu = 15      # unsprung mass per corner
        self.m_corner = self.ms + self.mu

        self.h_rc = 0.0
        self.h_cg = 0.3
        self.tau_rc = 0.005

        self.zeta = 0.7
        k_heave = 18000
        k_roll = 50000
        self.heave = LinearKC(
            k=k_heave,
            c=2 * self.zeta * np.sqrt(k_heave * self.mu),
            mr_value=1.0,
            name="heave",
        )
        self.roll = LinearKC(
            k=k_roll,
            c=2 * self.zeta * np.sqrt(k_roll * self.mu)*0,
            mr_value=1.0,
            name="roll",
        )

        # No direct corner shocks in the original setup. These can be replaced
        # independently with LinearKC, SeriesKC, Ohlins or LookupKC objects.
        self.corner_left = NullKC()
        self.corner_right = NullKC()

        k_tire = 800000
        self.tire_left = LinearKC(
            k=k_tire,
            c=2 * self.zeta * np.sqrt(k_tire * self.mu),
            name="tire_left",
        )
        self.tire_right = LinearKC(
            k=k_tire,
            c=2 * self.zeta * np.sqrt(k_tire * self.mu),
            name="tire_right",
        )

        self.m_frame = 65     # virtual frame mass (kg)
        k_frame = k_roll * 10
        self.frame = LinearKC(
            k=k_frame,
            c=2 * self.zeta * np.sqrt(k_frame * self.m_frame),
            name="frame",
        )


class SuspensionFR:
    def __init__(self):
        self.Iy = 50
        self.ms = 65      # sprung mass per corner
        self.mu = 15      # unsprung mass per corner
        self.m_corner = self.ms + self.mu

        self.lf = 0.765
        self.lr = 0.765
        self.h_cg = 0.3
        self.l = self.lf + self.lr

        self.anti_dive_front = 0
        self.anti_squat_rear = 0

        self.zeta = 0.7
        k_heave_f = 30000
        k_heave_r = 28000
        self.heave_f = LinearKC(
            k=k_heave_f,
            c=2 * self.zeta * np.sqrt(k_heave_f * self.mu),
            mr_value=1.0,
            name="heave_f",
        )
        self.heave_r = LinearKC(
            k=k_heave_r,
            c=2 * self.zeta * np.sqrt(k_heave_r * self.mu),
            mr_value=1.0,
            name="heave_r",
        )

        self.corner_front = NullKC()
        self.corner_rear = NullKC()

        k_tire = 80000
        self.tire_front = LinearKC(
            k=k_tire,
            c=2 * self.zeta * np.sqrt(k_tire * self.mu),
            name="tire_front",
        )
        self.tire_rear = LinearKC(
            k=k_tire,
            c=2 * self.zeta * np.sqrt(k_tire * self.mu),
            name="tire_rear",
        )
