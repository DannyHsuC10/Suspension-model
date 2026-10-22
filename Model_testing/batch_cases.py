"""Explicit model/input compatibility catalog. Edit scenario parameters here."""

from dataclasses import dataclass
from typing import Callable

from model import quarter_model, LR_model, FR_model
from road import quarter_road, LR_road, FR_road
from Parameter_set.Suspension import SuspensionQuarter, SuspensionLR, SuspensionFR
from Parameter_set.suspension_state import (
    quarter_suspension_state, LR_suspension_state, FR_suspension_state,
)

INPUT_START = 0.1
SIMULATION_TIME = 1
DEFAULT_DT = 0.001


@dataclass(frozen=True)
class ModelSpec:
    function: Callable
    supports_cornering: bool = False


@dataclass(frozen=True)
class ScenarioSpec:
    function: Callable
    parameters: dict
    requires_cornering: bool = False


@dataclass(frozen=True)
class FamilySpec:
    suspension_factory: Callable
    state_factory: Callable
    models: dict[str, ModelSpec]
    scenarios: dict[str, ScenarioSpec]


def lr_state(dt):
    return LR_suspension_state(quarter_suspension_state(), quarter_suspension_state(), dt)


def fr_state(dt):
    return FR_suspension_state(quarter_suspension_state(), quarter_suspension_state(), dt)


FAMILIES = {
    "quarter": FamilySpec(
        SuspensionQuarter, quarter_suspension_state,
        {
            "single": ModelSpec(quarter_model.single_spring_model),
            "one_eighth": ModelSpec(quarter_model.one_eighth_suspension_model),
            "double": ModelSpec(quarter_model.Double_spring_model),
            "quarter": ModelSpec(quarter_model.Quarter_suspension_model),
        },
        {
            "step": ScenarioSpec(quarter_road.road_step, {"start": INPUT_START, "h": 0.01}),
            "bump": ScenarioSpec(quarter_road.road_bump, {
                "start": INPUT_START, "height": 0.01, "duration": 0.2,
            }),
            "sine": ScenarioSpec(quarter_road.road_sine, {
                "start": INPUT_START, "amplitude": 0.01, "frequency": 1,
            }),
            "random": ScenarioSpec(quarter_road.RandomRoad, {
                "start": INPUT_START, "amplitude": 0.005, "dt_update": 0.01, "seed": 0,
            }),
        },
    ),
    "LR": FamilySpec(
        SuspensionLR, lr_state,
        {
            "2mass_2dof": ModelSpec(LR_model.two_mass_suspension_2DOF_model),
            "2mass": ModelSpec(LR_model.two_mass_suspension_model),
            "half_2dof": ModelSpec(LR_model.Half_suspension_2DOF_model, True),
            "half": ModelSpec(LR_model.Half_suspension_model, True),
            "rollcenter_all": ModelSpec(LR_model.Half_rollcenter_all_model, True),
        },
        {
            "bump_left": ScenarioSpec(LR_road.road_bump_left, {
                "start": INPUT_START, "height": 0.02, "duration": 0.3,
            }),
            "bump_both": ScenarioSpec(LR_road.road_bump_both, {
                "start": INPUT_START, "height": 0.02, "duration": 0.3,
            }),
            "sine_left": ScenarioSpec(LR_road.road_sin_left, {
                "t0": INPUT_START, "amplitude": 0.01, "freq": 1,
            }),
            "cornering": ScenarioSpec(LR_road.Cornering_left, {
                "t0": INPUT_START, "ay": -2,
            }, True),
        },
    ),
    "FR": FamilySpec(
        SuspensionFR, fr_state,
        {
            "half_2dof": ModelSpec(FR_model.Half_suspension_2DOF_model),
            "half": ModelSpec(FR_model.Half_suspension_model),
        },
        {
            # l is resolved from the current suspension.
            "impulse_drive": ScenarioSpec(FR_road.road_impulse_drive, {
                "t0": INPUT_START, "speed": 5.0, "h": 0.05, "duration": 0.02,
            }),
            "accelerate": ScenarioSpec(FR_road.accelerate, {"t0": INPUT_START, "ax": 2}),
        },
    ),
}


def compatibility_error(family, model, scenario):
    """Reject cross-topology names and inputs ignored by a model."""
    spec = FAMILIES[family]
    if model not in spec.models:
        return f"{model!r} is not a {family} model"
    if scenario not in spec.scenarios:
        return f"{scenario!r} is not a {family} input"
    if spec.scenarios[scenario].requires_cornering and not spec.models[model].supports_cornering:
        return "model has no lateral-acceleration/body-roll dynamics"
    return None
