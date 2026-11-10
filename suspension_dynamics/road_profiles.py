"""Synthetic longitudinal road profiles based on ISO 8608:2016 Annexes B and C."""

import numpy as np


ISO_ROUGHNESS = {"iso_a": 16e-6, "iso_b": 64e-6, "iso_c": 256e-6}
ISO_REFERENCE_FREQUENCY = .1  # cycles/m
ISO_FREQUENCY_BAND = (.011, 2.83)  # cycles/m
PROFILE_LENGTH = 4096.  # m, spatial synthesis period
PROFILE_SAMPLE_COUNT = 2**18


class IsoRoadProfile:
    """One deterministic track, sampled in metres independently of vehicle speed."""

    def __init__(self, roughness, seed):
        self.spacing = PROFILE_LENGTH / PROFILE_SAMPLE_COUNT
        frequencies = np.fft.rfftfreq(PROFILE_SAMPLE_COUNT, self.spacing)
        lower, upper = ISO_FREQUENCY_BAND
        active = (frequencies >= lower) & (frequencies <= upper)
        n = frequencies[active]
        density = roughness * (n / ISO_REFERENCE_FREQUENCY) ** -2
        phase = np.random.default_rng(int(seed)).uniform(0., 2 * np.pi, n.size)
        # A one-sided PSD bin contributes G(n) Δn to mean square height.
        spectrum = np.zeros(frequencies.size, dtype=complex)
        spectrum[active] = PROFILE_SAMPLE_COUNT * np.sqrt(density / (2 * PROFILE_LENGTH)) * np.exp(1j * phase)
        self.height = np.fft.irfft(spectrum, n=PROFILE_SAMPLE_COUNT)
        self.height -= self.height[0]
        self.slope = np.fft.irfft(2j * np.pi * frequencies * spectrum, n=PROFILE_SAMPLE_COUNT)

    def at(self, distance):
        """Return height and spatial slope using the same cubic interpolant."""
        position = (distance % PROFILE_LENGTH) / self.spacing
        u = position % 1
        index = int(position) % PROFILE_SAMPLE_COUNT
        next_index = (index + 1) % PROFILE_SAMPLE_COUNT
        height = self.height[index]
        change = self.height[next_index] - height
        left_slope = self.slope[index] * self.spacing
        right_slope = self.slope[next_index] * self.spacing
        quadratic = 3 * change - 2 * left_slope - right_slope
        cubic = -2 * change + left_slope + right_slope
        value = height + u * (left_slope + u * (quadratic + u * cubic))
        slope = (left_slope + u * (2 * quadratic + u * 3 * cubic)) / self.spacing
        return value, slope
