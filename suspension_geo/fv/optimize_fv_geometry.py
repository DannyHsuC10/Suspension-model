"""Optimize FV geometry on a 1 mm RC-height / 50 mm FVSA lattice.

Run: python -m suspension_geo.fv.optimize_fv_geometry
SciPy integer differential evolution starts at (35 mm, 1000 mm). By default,
scipy.optimize.brute then verifies every allowed discrete design. Every cost
evaluation covers the same theta grid as sus_fv_maping.py, including both wheels.
The RC cost target is fixed at (0, y_best=33 mm), independent of the starting
height and every candidate design's static RC height.
Radial RC migration, pure RC height error and camber are normalized separately
against the initial design. RC distance:height importance defaults to 1:5.
The height target is independent of horizontal migration and radial penalties.

Reference: https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.differential_evolution.html
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import sys
from time import perf_counter

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import brute, differential_evolution

from suspension_geo import Geometric_Analysis as ga
from suspension_geo.car import Car, SuspensionF


HEIGHT_BOUNDS_MM = (-5, 50)
FVSA_BOUNDS_MM = (750, 2500)
FVSA_STEP_MM = 50
GRID_SIZE = (HEIGHT_BOUNDS_MM[1] - HEIGHT_BOUNDS_MM[0] + 1) * (
    (FVSA_BOUNDS_MM[1] - FVSA_BOUNDS_MM[0]) // FVSA_STEP_MM + 1
)
START_HEIGHT_MM = 0.0
START_FVSA_MM = 1000
Y_BEST_MM = 30.0  # Fixed RC cost target; never an optimization variable.
RC_LIMIT_MM = 50.0
CAMBER_LIMIT_DEG = 2.0
PENALTY_FACTOR = 3.0
INVALID_COST = 1e12
NORMALIZATION_EPS = 1e-12
OUTPUT_DIR = Path(__file__).resolve().parent / "optimization_results"


@dataclass(frozen=True)
class Settings:
    steps: int = SuspensionF().front_view_mapping_steps
    rc_weight: float = 1.0
    camber_weight: float = 1.0
    camber_penalty: str = "global"
    rc_distance_weight: float = 1.0
    rc_height_weight: float = 5.0

    def __post_init__(self):
        if self.steps < 2:
            raise ValueError("Mapping needs at least two samples per wheel.")
        if (not np.isfinite([self.rc_weight, self.camber_weight]).all()
                or min(self.rc_weight, self.camber_weight) < 0
                or self.rc_weight + self.camber_weight == 0):
            raise ValueError("Cost weights must be finite, nonnegative and not both zero.")
        if self.camber_penalty not in ("point", "global"):
            raise ValueError("Camber penalty must be point or global.")
        if (not np.isfinite([self.rc_distance_weight, self.rc_height_weight]).all()
                or min(self.rc_distance_weight, self.rc_height_weight) < 0
                or self.rc_distance_weight + self.rc_height_weight == 0):
            raise ValueError("RC component weights must be finite, nonnegative and not both zero.")

    @property
    def normalized_weights(self):
        total = self.rc_weight + self.camber_weight
        return self.rc_weight / total, self.camber_weight / total

    @property
    def normalized_rc_component_weights(self):
        total = self.rc_distance_weight + self.rc_height_weight
        return self.rc_distance_weight / total, self.rc_height_weight / total


@dataclass(frozen=True)
class CostScales:
    """Fixed positive divisors, computed once from the initial design."""

    rc_distance_mm2: float
    rc_height_mm2: float
    camber_deg2: float

    def __post_init__(self):
        values = [self.rc_distance_mm2, self.rc_height_mm2, self.camber_deg2]
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("Cost normalization scales must be finite and positive.")

    @classmethod
    def from_initial(cls, metrics):
        distance = metrics["rc_cost_raw_mm2"]
        height = metrics["rc_height_cost_raw_mm2"]
        camber = metrics["camber_cost_raw_deg2"]
        # A zero initial error is already ideal. Use its physical tolerance as
        # a fallback scale instead of dividing by zero or a tiny epsilon.
        return cls(
            distance if distance > NORMALIZATION_EPS else RC_LIMIT_MM ** 2,
            height if height > NORMALIZATION_EPS else RC_LIMIT_MM ** 2,
            camber if camber > NORMALIZATION_EPS else CAMBER_LIMIT_DEG ** 2,
        )


def cross2d(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def build_mapping(height_mm: int, fvsa_mm: int, steps: int) -> dict[str, np.ndarray]:
    """Sample each independent wheel once, then combine all steps**2 poses.

    Mirroring and broadcasting avoid repeating each single-wheel solve steps
    times. Geometry, theta limits and signed-camber convention match FV mapping.
    Invalid kinematics or nonfinite IC/RC intersections reject the whole design.
    """
    car = Car()
    suspension = SuspensionF()
    suspension.h_rc = height_mm / 1000.0
    suspension.fvsa = fvsa_mm / 1000.0
    helper = ga.SusGeometryHelper(car, suspension)
    points = helper.calc_static_points()
    if any(value is None or not np.isfinite(value).all() for value in points.values()):
        raise ValueError("Invalid static geometry or parallel arm lines.")
    right = ga.SuspensionSide("Right")
    right.setup_right_side(points)
    ground_y0 = right.gnd.y
    theta_down, theta_up = ga.calculate_theta_limits(
        right, suspension.front_view_target_travel
    )
    thetas = np.linspace(theta_down, theta_up, steps)
    grounds, instant_centers, tire_bottoms = [], [], []
    for theta in thetas:
        result = ga.simulate_side_theta(right, theta)
        if result is None:
            raise ValueError("A mapping pose has no valid arm/upright intersection.")
        kua, lak = result
        right.upright_body.update_from_two_points(lak, kua)
        ic = helper.line_intersection(
            right.lower_arm.start, right.lower_arm.end,
            right.upper_arm.start, right.upper_arm.end,
        )
        if ic is None or not np.isfinite(ic).all():
            raise ValueError("A mapping pose has no finite instant center.")
        grounds.append(right.gnd.pos.copy())
        instant_centers.append(ic)
        tire_bottoms.append(right.wheel_pts[1].pos - right.wheel_pts[0].pos)

    ground_r = np.asarray(grounds)[None, :, :]
    ic_r = np.asarray(instant_centers)[None, :, :]
    bottom_r = np.asarray(tire_bottoms)[None, :, :]
    mirror = np.array([-1.0, 1.0])
    ground_l = np.asarray(grounds)[:, None, :] * mirror
    ic_l = np.asarray(instant_centers)[:, None, :] * mirror
    # Reverse the mirrored left bottom edge to point in the +X convention.
    bottom_l = -np.asarray(tire_bottoms)[:, None, :] * mirror
    ground = ground_r - ground_l
    force_l = ic_l - ground_l
    force_r = ic_r - ground_r
    denominator = cross2d(force_l, force_r)
    if np.isclose(denominator, 0.0).any():
        raise ValueError("A mapping pose has parallel RC force lines.")
    rc_mm = (ground_l + (cross2d(ground, force_r) / denominator)[..., None] * force_l) * 1000.0
    camber_l = np.degrees(np.arctan2(cross2d(ground, bottom_l), np.sum(ground * bottom_l, axis=-1)))
    camber_r = -np.degrees(np.arctan2(cross2d(ground, bottom_r), np.sum(ground * bottom_r, axis=-1)))
    shape = (steps, steps)
    mapping = {
        "rc_x_mm": rc_mm[..., 0],
        "rc_y_mm": rc_mm[..., 1],
        "camber_left_deg": camber_l,
        "camber_right_deg": camber_r,
        "travel_left_mm": np.broadcast_to((ground_l[..., 1] - ground_y0) * 1000.0, shape),
        "travel_right_mm": np.broadcast_to((ground_r[..., 1] - ground_y0) * 1000.0, shape),
        "roll_deg": np.degrees(np.arctan2(ground[..., 1], ground[..., 0])),
    }
    if not all(np.isfinite(values).all() for values in mapping.values()):
        raise ValueError("Nonfinite mapping result.")
    return mapping


def summarize_raw_cost(mapping, settings: Settings) -> dict:
    """Penalty-weighted errors in physical squared units, before normalization.

    RC: mean(k_rc * (x**2 + (y-y_best)**2)), y_best=33 mm.
    Camber: mean(k_camber * camber**2), over BOTH wheels.
    RC squared error is tripled outside RC_LIMIT_MM, independently of scale.
    By default, any camber exceedance triples the entire camber term.
    """
    x_error_sq = mapping["rc_x_mm"] ** 2
    y_error_sq = (mapping["rc_y_mm"] - Y_BEST_MM) ** 2
    radius_sq = x_error_sq + y_error_sq
    rc_outside = radius_sq > RC_LIMIT_MM ** 2
    rc_penalty = np.where(rc_outside, PENALTY_FACTOR, 1.0)
    rc_x_cost = float(np.mean(rc_penalty * x_error_sq))
    rc_y_cost = float(np.mean(rc_penalty * y_error_sq))
    cambers = np.stack((mapping["camber_left_deg"], mapping["camber_right_deg"]))
    camber_outside = np.abs(cambers) > CAMBER_LIMIT_DEG
    if settings.camber_penalty == "global":
        factor = PENALTY_FACTOR if camber_outside.any() else 1.0
    else:
        factor = np.where(camber_outside, PENALTY_FACTOR, 1.0)
    camber_cost = float(np.mean(factor * cambers ** 2))
    return {
        "rc_cost_raw_mm2": rc_x_cost + rc_y_cost,
        "rc_x_cost_raw_mm2": rc_x_cost,
        "rc_y_cost_raw_mm2": rc_y_cost,
        # A separate pure-height target cannot improve merely because an X
        # migration change makes fewer poses trigger the radial penalty.
        "rc_height_cost_raw_mm2": float(np.mean(y_error_sq)),
        "camber_cost_raw_deg2": camber_cost,
        "rc_rms_distance_mm": float(np.sqrt(np.mean(radius_sq))),
        "rc_max_distance_mm": float(np.sqrt(radius_sq.max())),
        "rc_x_rms_mm": float(np.sqrt(np.mean(x_error_sq))),
        "rc_y_error_rms_mm": float(np.sqrt(np.mean(y_error_sq))),
        "rc_y_mean_mm": float(np.mean(mapping["rc_y_mm"])),
        "rc_y_min_mm": float(np.min(mapping["rc_y_mm"])),
        "rc_y_max_mm": float(np.max(mapping["rc_y_mm"])),
        "camber_rms_deg": float(np.sqrt(np.mean(cambers ** 2))),
        "camber_max_abs_deg": float(np.abs(cambers).max()),
        "rc_penalty_points": int(rc_outside.sum()),
        "camber_penalty_wheel_points": int(camber_outside.sum()),
    }


def normalize_cost(metrics, settings: Settings, scales: CostScales) -> dict:
    distance_cost = metrics["rc_cost_raw_mm2"] / scales.rc_distance_mm2
    height_cost = metrics["rc_height_cost_raw_mm2"] / scales.rc_height_mm2
    distance_weight, height_weight = settings.normalized_rc_component_weights
    rc_cost = distance_weight * distance_cost + height_weight * height_cost
    camber_cost = metrics["camber_cost_raw_deg2"] / scales.camber_deg2
    rc_weight, camber_weight = settings.normalized_weights
    return {
        **metrics,
        "rc_cost": rc_cost,
        "rc_distance_cost": distance_cost,
        "rc_height_cost": height_cost,
        "camber_cost": camber_cost,
        "rc_contribution": rc_weight * rc_cost,
        "rc_distance_contribution": rc_weight * distance_weight * distance_cost,
        "rc_height_contribution": rc_weight * height_weight * height_cost,
        "camber_contribution": camber_weight * camber_cost,
        "total_cost": rc_weight * rc_cost + camber_weight * camber_cost,
    }


def summarize_cost(mapping, settings: Settings, scales: CostScales) -> dict:
    return normalize_cost(summarize_raw_cost(mapping, settings), settings, scales)


class GeometryObjective:
    def __init__(self, settings: Settings):
        self.settings = settings
        # Never normalize against the current candidate, the best-so-far value,
        # or the first arbitrary evaluation: all candidates share this baseline.
        initial_mapping = build_mapping(START_HEIGHT_MM, START_FVSA_MM, settings.steps)
        self.initial_raw = summarize_raw_cost(initial_mapping, settings)
        self.scales = CostScales.from_initial(self.initial_raw)
        self.cache = {}
        self.history = []
        self.best = None
        self.phase = "initial"
        self.calls = 0

    def __call__(self, coordinates):
        self.calls += 1
        height_mm = int(np.rint(coordinates[0]))
        fvsa_mm = int(np.rint(coordinates[1])) * FVSA_STEP_MM
        key = (height_mm, fvsa_mm)
        if not (HEIGHT_BOUNDS_MM[0] <= height_mm <= HEIGHT_BOUNDS_MM[1]
                and FVSA_BOUNDS_MM[0] <= fvsa_mm <= FVSA_BOUNDS_MM[1]):
            raise ValueError("Optimizer coordinates are outside the design bounds.")
        if key in self.cache:
            return self.cache[key]["total_cost"]
        metrics = {"height_mm": height_mm, "fvsa_mm": fvsa_mm, "valid": True, "reason": ""}
        try:
            with np.errstate(divide="raise", invalid="raise", over="raise"):
                if key == (START_HEIGHT_MM, START_FVSA_MM):
                    metrics.update(normalize_cost(self.initial_raw, self.settings, self.scales))
                else:
                    mapping = build_mapping(height_mm, fvsa_mm, self.settings.steps)
                    metrics.update(summarize_cost(mapping, self.settings, self.scales))
        except (ValueError, FloatingPointError, ZeroDivisionError) as exc:
            metrics.update(valid=False, reason=str(exc), total_cost=INVALID_COST,
                           rc_cost=None, camber_cost=None)
        self.cache[key] = metrics
        if metrics["valid"] and (self.best is None or metrics["total_cost"] < self.best["total_cost"]):
            self.best = metrics
        self.history.append({
            "evaluation": len(self.history) + 1, "phase": self.phase, **metrics,
            "best_total_cost": self.best["total_cost"] if self.best else INVALID_COST,
            "best_rc_cost": self.best["rc_cost"] if self.best else None,
            "best_rc_height_contribution": self.best["rc_height_contribution"] if self.best else None,
            "best_camber_cost": self.best["camber_cost"] if self.best else None,
            "best_height_mm": self.best["height_mm"] if self.best else None,
            "best_fvsa_mm": self.best["fvsa_mm"] if self.best else None,
        })
        if len(self.history) % 100 == 0:
            best = self.best
            print(f"[{self.phase}] {len(self.history)} unique designs; "
                  f"best RC={best['height_mm']} mm, FVSA={best['fvsa_mm']} mm, "
                  f"cost={best['total_cost']:.6f}" if best else "No valid design yet.", flush=True)
        return metrics["total_cost"]


def optimize(settings: Settings, *, maxiter=60, seed=42, verify_grid=True):
    objective = GeometryObjective(settings)
    start = [START_HEIGHT_MM, START_FVSA_MM // FVSA_STEP_MM]
    objective(start)
    initial = objective.cache[(START_HEIGHT_MM, START_FVSA_MM)]
    if not initial["valid"]:
        raise ValueError(f"Initial design is invalid: {initial['reason']}")
    objective.phase = "differential_evolution"
    result = differential_evolution(
        objective,
        bounds=[HEIGHT_BOUNDS_MM, (FVSA_BOUNDS_MM[0] // FVSA_STEP_MM, FVSA_BOUNDS_MM[1] // FVSA_STEP_MM)],
        x0=start, integrality=[True, True], polish=False,
        maxiter=maxiter, popsize=12, tol=1e-5, atol=1e-9, rng=seed,
    )
    if verify_grid:
        objective.phase = "grid_verification"
        brute(objective, (
            slice(HEIGHT_BOUNDS_MM[0], HEIGHT_BOUNDS_MM[1] + 1, 1),
            slice(FVSA_BOUNDS_MM[0] // FVSA_STEP_MM, FVSA_BOUNDS_MM[1] // FVSA_STEP_MM + 1, 1),
        ), finish=None)
    return objective, initial, result


def write_csv(path: Path, rows: list[dict]):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def plot_history(objective: GeometryObjective, path: Path):
    history = objective.history
    x = np.array([row["evaluation"] for row in history])
    valid = np.array([row["valid"] for row in history])
    fig, (cost_ax, parameter_ax, search_ax) = plt.subplots(3, 1, figsize=(11, 10))
    grid_start = next((row["evaluation"] for row in history if row["phase"] == "grid_verification"), None)
    # Keep the DE convergence readable even when grid verification has extreme
    # costs near singular geometry or many more evaluations than the DE stage.
    de_history = [row for row in history if row["phase"] != "grid_verification"]
    de_x = np.array([row["evaluation"] for row in de_history])
    rc_weight, camber_weight = objective.settings.normalized_weights
    for key, label in [("best_total_cost", "Best total cost"), ("best_rc_cost", "RC contribution"), ("best_camber_cost", "Camber contribution")]:
        weight = rc_weight if key == "best_rc_cost" else camber_weight if key == "best_camber_cost" else 1.0
        cost_ax.plot(de_x, [row[key] * weight for row in de_history], label=label, linewidth=1.6)
    cost_ax.plot(de_x, [row["best_rc_height_contribution"] for row in de_history],
                 label="RC height contribution", linestyle="--", color="tab:red", linewidth=1.3)
    cost_ax.set_title("Differential evolution: best-cost convergence")
    cost_ax.set_ylabel("Baseline-normalized cost")
    cost_ax.set_xlabel("Unique design evaluations during differential evolution")
    cost_ax.grid(True, alpha=0.3)
    cost_ax.legend(fontsize=9)
    parameter_ax.step(de_x, [row["best_height_mm"] for row in de_history], where="post", color="tab:blue", label="RC height")
    parameter_ax.set_ylabel("Best RC height [mm]", color="tab:blue")
    fvsa_ax = parameter_ax.twinx()
    fvsa_ax.step(de_x, [row["best_fvsa_mm"] for row in de_history], where="post", color="tab:orange", label="FVSA")
    fvsa_ax.set_ylabel("Best FVSA [mm]", color="tab:orange")
    parameter_ax.set_xlabel("Unique design evaluations during differential evolution")
    parameter_ax.grid(True, alpha=0.3)
    raw = np.array([row["total_cost"] for row in history])
    search_ax.scatter(x[valid], raw[valid], color="0.6", s=8, alpha=0.3, label="Evaluated design cost")
    search_ax.plot(x, [row["best_total_cost"] for row in history], label="Best total cost", linewidth=1.6)
    search_ax.set_yscale("symlog", linthresh=1e-4)
    search_ax.set_ylabel("Baseline-normalized cost (log scale)")
    search_ax.set_xlabel("All unique design evaluations (initial design = 1)")
    search_ax.set_title("Full search, including discrete grid verification")
    search_ax.grid(True, alpha=0.3)
    search_ax.legend(fontsize=9)
    if grid_start:
        search_ax.axvline(grid_start, color="tab:purple", linestyle="--", alpha=0.7)
        search_ax.text(grid_start, 0.98, " Grid verification", transform=search_ax.get_xaxis_transform(), va="top", color="tab:purple")
    best = objective.best
    fig.suptitle(
        f"FV geometry optimization | fixed RC target (0, {Y_BEST_MM:g}) mm\n"
        f"Best RC height {best['height_mm']} mm | FVSA {best['fvsa_mm']} mm"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=SuspensionF().front_view_mapping_steps)
    parser.add_argument("--camber-penalty", choices=("point", "global"), default="global")
    parser.add_argument("--rc-weight", type=float, default=1.0, help="RC relative weight after baseline normalization")
    parser.add_argument("--camber-weight", type=float, default=1.0, help="Camber-angle relative weight after baseline normalization")
    parser.add_argument("--rc-distance-weight", type=float, default=1.0, help="Radial migration weight inside the RC term")
    parser.add_argument("--rc-height-weight", type=float, default=5.0, help="Pure height-target weight inside the RC term (default: 5)")
    parser.add_argument("--maxiter", type=int, default=60)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--no-grid-check", action="store_true", help=f"Skip the exhaustive {GRID_SIZE}-design verification")
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DIR)
    args = parser.parse_args()
    if args.maxiter < 1:
        parser.error("--maxiter must be positive")
    settings = Settings(
        steps=args.steps, rc_weight=args.rc_weight, camber_weight=args.camber_weight,
        camber_penalty=args.camber_penalty,
        rc_distance_weight=args.rc_distance_weight, rc_height_weight=args.rc_height_weight,
    )
    print(f"Mapping: {settings.steps} x {settings.steps}; start RC=35 mm / FVSA=1000 mm", flush=True)
    print(f"Fixed RC target: (0, {Y_BEST_MM:g}) mm; camber penalty: {settings.camber_penalty}; "
          f"weights RC/camber={settings.rc_weight:g}/{settings.camber_weight:g}", flush=True)
    rc_weight, camber_weight = settings.normalized_weights
    distance_weight, height_weight = settings.normalized_rc_component_weights
    print(f"Baseline normalization; effective weights RC/camber={rc_weight:.1%}/{camber_weight:.1%}; "
          f"RC penalty radius={RC_LIMIT_MM:g} mm", flush=True)
    print(f"RC distance/height normalized separately; weights={distance_weight:.1%}/{height_weight:.1%}", flush=True)
    started = perf_counter()
    objective, initial, result = optimize(settings, maxiter=args.maxiter, seed=args.seed, verify_grid=not args.no_grid_check)
    elapsed = perf_counter() - started
    best = objective.best
    improvement = 100.0 * (initial["total_cost"] - best["total_cost"]) / initial["total_cost"] if initial["total_cost"] else 0.0
    summary = {
        "method": "SciPy differential_evolution, integer variables, polish=False",
        "grid_verified": not args.no_grid_check,
        "de_success": bool(result.success), "de_message": str(result.message),
        "de_generations": int(result.nit), "optimizer_calls": objective.calls,
        "unique_designs": len(objective.cache),
        "invalid_designs": sum(not row["valid"] for row in objective.cache.values()),
        "mapping_steps_per_wheel": settings.steps,
        "settings": {"rc_height_bounds_mm": HEIGHT_BOUNDS_MM, "fvsa_bounds_mm": FVSA_BOUNDS_MM,
                     "rc_height_step_mm": 1, "fvsa_step_mm": FVSA_STEP_MM,
                     "rc_reference": "fixed_target", "y_best_mm": Y_BEST_MM,
                     "rc_reference_xy_mm": [0.0, Y_BEST_MM],
                     "rc_penalty_radius_mm": RC_LIMIT_MM, "camber_penalty_limit_deg": CAMBER_LIMIT_DEG,
                     "normalization": {
                         "method": "fixed_initial_design_radial_height_and_camber_cost",
                         "baseline_design": {"height_mm": START_HEIGHT_MM, "fvsa_mm": START_FVSA_MM},
                         "rc_distance_scale_mm2": objective.scales.rc_distance_mm2,
                         "rc_height_scale_mm2": objective.scales.rc_height_mm2,
                         "camber_scale_deg2": objective.scales.camber_deg2,
                         "rc_distance_zero_baseline_fallback": objective.initial_raw["rc_cost_raw_mm2"] <= NORMALIZATION_EPS,
                         "rc_height_zero_baseline_fallback": objective.initial_raw["rc_height_cost_raw_mm2"] <= NORMALIZATION_EPS,
                         "camber_zero_baseline_fallback": objective.initial_raw["camber_cost_raw_deg2"] <= NORMALIZATION_EPS,
                     },
                     "penalty_factor": PENALTY_FACTOR, "camber_penalty": settings.camber_penalty,
                     "rc_weight": settings.rc_weight, "camber_weight": settings.camber_weight,
                     "effective_rc_weight": rc_weight, "effective_camber_weight": camber_weight,
                     "rc_distance_weight": settings.rc_distance_weight, "rc_height_weight": settings.rc_height_weight,
                     "effective_rc_distance_weight": distance_weight, "effective_rc_height_weight": height_weight,
                     "seed": args.seed},
        "initial": initial, "best": best, "cost_improvement_percent": improvement,
        "elapsed_seconds": elapsed,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "optimization_history.csv", objective.history)
    (args.output_dir / "optimization_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    mapping = build_mapping(best["height_mm"], best["fvsa_mm"], settings.steps)
    write_csv(args.output_dir / "optimal_mapping.csv", [dict(zip(mapping, values)) for values in zip(*(array.ravel().tolist() for array in mapping.values()))])
    fig = plot_history(objective, args.output_dir / "cost_history.png")
    print("\n=== FV optimization result ===")
    print(f"DE: success={result.success}; {result.message}")
    print(f"Final status: all {GRID_SIZE} discrete designs verified." if not args.no_grid_check else "Final status: DE search only; discrete global optimum not verified.")
    print(f"RC height = {best['height_mm']} mm (step 1 mm)")
    print(f"FVSA      = {best['fvsa_mm']} mm (step 50 mm)")
    print(f"Fixed normalization scales: RC distance={objective.scales.rc_distance_mm2:.8f} mm^2, "
          f"RC height={objective.scales.rc_height_mm2:.8f} mm^2, "
          f"camber={objective.scales.camber_deg2:.8f} deg^2")
    print(f"Total cost: {initial['total_cost']:.8f} -> {best['total_cost']:.8f} ({improvement:.2f}% improvement)")
    for key in ("rc_cost_raw_mm2", "rc_height_cost_raw_mm2", "camber_cost_raw_deg2", "rc_distance_cost", "rc_height_cost", "rc_cost", "camber_cost", "rc_contribution", "camber_contribution", "rc_height_contribution", "rc_rms_distance_mm", "rc_max_distance_mm", "rc_x_rms_mm", "rc_y_error_rms_mm", "rc_y_mean_mm", "rc_y_min_mm", "rc_y_max_mm", "camber_rms_deg", "camber_max_abs_deg", "rc_penalty_points", "camber_penalty_wheel_points"):
        print(f"{key}: {initial[key]:.6f} -> {best[key]:.6f}")
    print(f"Unique designs: {len(objective.cache)}; elapsed: {elapsed:.2f} s")
    print(f"Outputs: {args.output_dir.resolve()}")
    if not args.no_show and plt.get_backend().lower() != "agg":
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()
