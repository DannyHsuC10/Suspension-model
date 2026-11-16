"""Shared engine for the single data_creator.py batch entry point."""

import argparse
from contextlib import redirect_stdout
import csv
from datetime import datetime, timezone
from functools import partial
import hashlib
import json
import os
from pathlib import Path
import traceback

import numpy as np

from batch_cases import (
    FAMILIES, INPUT_START, SIMULATION_TIME, DEFAULT_DT, compatibility_error,
)
from data_visualization.Simulation_Logger import SimulationLogger
from road.quarter_road import RandomRoad

HERE = Path(__file__).resolve().parent


def build_plan(families, models=None, scenarios=None):
    for names, available, label in (
        (models, {n for f in families for n in FAMILIES[f].models}, "models"),
        (scenarios, {n for f in families for n in FAMILIES[f].scenarios}, "scenarios"),
    ):
        unknown = set(names or ()) - available
        if unknown:
            raise ValueError(f"Unknown {label}: {sorted(unknown)}; available: {sorted(available)}")
    cases, skipped = [], []
    for family in families:
        spec = FAMILIES[family]
        for model in spec.models:
            for scenario in spec.scenarios:
                if (models and model not in models) or (scenarios and scenario not in scenarios):
                    continue
                reason = compatibility_error(family, model, scenario)
                case = {"family": family, "model": model, "scenario": scenario}
                if reason:
                    skipped.append({**case, "reason": reason})
                else:
                    cases.append(case)
    if not cases:
        raise ValueError("No compatible cases selected; use --list to inspect the catalog")
    return cases, skipped


def source_hashes(family):
    model_file = {"quarter": "quarter_model.py", "LR": "LR_model.py", "FR": "FR_model.py"}[family]
    road_file = {"quarter": "quarter_road.py", "LR": "LR_road.py", "FR": "FR_road.py"}[family]
    files = [
        HERE / "model" / model_file, HERE / "road" / road_file,
        HERE / "Parameter_set" / "Suspension.py",
        HERE / "Parameter_set" / "suspension_state.py",
    ]
    return {str(p.relative_to(HERE)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def prepare_case(family, model, scenario, dt=DEFAULT_DT, seed=0):
    reason = compatibility_error(family, model, scenario)
    if reason:
        raise ValueError(reason)
    spec = FAMILIES[family]
    # Some existing constructors print debugging values.
    with open(os.devnull, "w") as sink, redirect_stdout(sink):
        suspension = spec.suspension_factory()
    state = spec.state_factory(dt)
    condition = spec.scenarios[scenario]
    parameters = dict(condition.parameters)
    if condition.function is RandomRoad:
        parameters["seed"] = seed
        road = condition.function(**parameters)
    else:
        if family == "FR" and scenario == "impulse_drive":
            parameters["l"] = suspension.l
        if family == "FR" and scenario == "accelerate":
            parameters["m"] = 2 * suspension.m_corner
        road = partial(condition.function, **parameters)
    config = {
        "schema_version": 1,
        "family": family, "model": model, "scenario": scenario,
        "dt": dt, "input_start_s": INPUT_START, "duration_s": SIMULATION_TIME,
        "model_function": spec.models[model].function.__name__,
        "input_function": condition.function.__name__,
        "input_parameters": parameters,
        "suspension_parameters": {
            key: value.item() if isinstance(value, np.generic) else value
            for key, value in vars(suspension).items()
        },
        "source_hashes": source_hashes(family),
    }
    return suspension, state, road, spec.models[model].function, config


def simulate_case(suspension, state, road, model, config):
    """Log the initial sample and the exact end point; inputs align with t."""
    logger = SimulationLogger()
    steps = int(round(SIMULATION_TIME / state.dt))
    with open(os.devnull, "w") as sink, redirect_stdout(sink), np.errstate(
        over="raise", invalid="raise", divide="raise"
    ):
        for i in range(steps + 1):
            # Assign grid times instead of accumulating floating-point drift.
            state.t = i * state.dt
            for name in ("left", "right", "front", "rear"):
                if hasattr(state, name):
                    getattr(state, name).t = state.t
            road(state)
            model(suspension, state)
            logger.add_state(state)
            if i < steps:
                state.Integration(suspension)
    for key, values in logger.result().items():
        if not np.isfinite(values).all():
            raise FloatingPointError(f"Non-finite {key}: model may be numerically unstable")
    logger.set_metadata(
        config=config, created_utc=datetime.now(timezone.utc).isoformat(),
        time_convention="input and model evaluated at t; initial and final samples included",
    )
    return logger


def save_animation(logger, path, fps=10, dpi=70):
    # No GUI windows or external ffmpeg installation are required.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from PIL import Image
    from data_visualization.Dynamic_Visualizer_quarter import Visualizer
    from data_visualization.Dynamic_Visualizer_roll import RollCarVisualizer
    from data_visualization.Dynamic_Visualizer_pitch import PitchCarVisualizer

    config = logger.metadata["config"]
    data = logger.result()
    family = config["family"]
    parameters = config["suspension_parameters"]
    fig = None
    try:
        if family == "quarter":
            viewer = Visualizer(data)
        elif family == "LR":
            # The uncoupled two-mass models do not integrate a body roll DOF.
            # Display their corner-derived roll, while preserving raw PKL fields.
            if not FAMILIES[family].models[config["model"]].supports_cornering:
                data["roll"] = (data["left_zs"] - data["right_zs"]) / parameters["track"]
                data["roll_rate"] = (data["left_vzs"] - data["right_vzs"]) / parameters["track"]
            viewer = RollCarVisualizer(data, track_width=parameters["track"])
        else:
            viewer = PitchCarVisualizer(data, wheelbase=parameters["l"])
        fig = viewer.fig
        road_lines = []
        road_channels = []
        if family in {"LR", "FR"}:
            if family == "LR":
                x_positions = [-parameters["track"] / 2, parameters["track"] / 2]
                road_channels = ["left_zr", "right_zr"]
                road_offset = -0.025
            else:
                x_positions = [parameters["l"] / 2, -parameters["l"] / 2]
                road_channels = ["front_zr", "rear_zr"]
                road_offset = -0.2
            for x in x_positions:
                line, = viewer.ax.plot([x - 0.15, x + 0.15], [road_offset] * 2,
                                       color="tab:orange", linewidth=3)
                road_lines.append(line)
        display_axis = viewer.ax_state if family == "quarter" else viewer.ax
        time_label = display_axis.text(0.02, 0.95, "", transform=display_axis.transAxes)

        def update_frame(index):
            artists = list(viewer.update(index))
            for line, channel in zip(road_lines, road_channels):
                y = data[channel][index] + road_offset
                line.set_ydata([y, y])
            time_label.set_text(f"t = {data['t'][index]:.3f} s")
            return artists + road_lines + [time_label]

        fig.suptitle(
            f"{family} / {config['model']} / {config['scenario']} | "
            f"input starts {INPUT_START:.2f} s | {SIMULATION_TIME:.1f} s simulation"
        )
        fig.subplots_adjust(top=0.89, bottom=0.08, hspace=0.65)
        for axis in fig.axes:
            if axis.get_legend() is not None:
                axis.legend(loc="upper right", fontsize=7)
        # Keep all numeric samples in PKL; only the animation is downsampled.
        count = max(2, round(SIMULATION_TIME * fps))
        frames = np.linspace(0, len(data["t"]) - 1, count, dtype=int)
        # All signal curves are static. Cache them once instead of recomputing
        # axes/legends/annotations for every GIF frame.
        artists = update_frame(0)
        for artist in artists:
            artist.set_animated(True)
        fig.set_dpi(dpi)
        fig.canvas.draw()
        background = fig.canvas.copy_from_bbox(fig.bbox)
        images = []
        for index in frames:
            fig.canvas.restore_region(background)
            for artist in update_frame(index):
                artist.axes.draw_artist(artist)
            images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba()).copy()).convert("RGB"))
        temp = path.with_name("response.partial.gif")
        try:
            images[0].save(temp, save_all=True, append_images=images[1:],
                           duration=round(1000 / fps), loop=0)
            temp.replace(path)
        finally:
            temp.unlink(missing_ok=True)
    finally:
        if fig is not None:
            plt.close(fig)


def write_manifest(root, manifest):
    temp = root / "manifest.partial.json"
    temp.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    temp.replace(root / "manifest.json")
    columns = ["family", "model", "scenario", "status", "pkl", "gif", "error"]
    with (root / "summary.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(manifest["cases"])


def main(families=None, default_fps=10, default_dpi=70):
    families = list(families or FAMILIES)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", nargs="+", help="Optional model-name filter")
    parser.add_argument("--scenarios", nargs="+", help="Optional input-name filter")
    parser.add_argument("--output", type=Path, help="Output directory; default: Model_testing/batch_results")
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--fps", type=int, default=default_fps)
    parser.add_argument("--dpi", type=int, default=default_dpi)
    parser.add_argument("--list", action="store_true", help="Show compatible cases without running")
    parser.add_argument("--resume", action="store_true", help="Reuse matching PKL and existing GIF files")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--no-gif", action="store_true", help="Only simulate and save PKL")
    modes.add_argument("--gif-only", action="store_true", help="Render GIF from matching saved PKL")
    args = parser.parse_args()
    if not np.isfinite(args.dt) or args.dt <= 0 or args.fps < 1 or args.dpi < 1:
        parser.error("dt, fps and dpi must be positive and finite")
    if not all(np.isclose(t / args.dt, round(t / args.dt), rtol=0, atol=1e-8)
               for t in (INPUT_START, SIMULATION_TIME)):
        parser.error("dt must divide both 0.25 s and 2 s exactly")
    try:
        cases, skipped = build_plan(families, args.models, args.scenarios)
    except ValueError as exc:
        parser.error(str(exc))
    if args.list:
        for case in cases:
            print(" / ".join(case.values()))
        for case in skipped:
            print(f"SKIP {case['family']}/{case['model']}/{case['scenario']}: {case['reason']}")
        print(f"{len(cases)} compatible cases; {len(skipped)} incompatible cases excluded")
        return 0

    root = (args.output or HERE / "batch_results").resolve()
    resume = args.resume or args.gif_only
    root.mkdir(parents=True, exist_ok=True)
    manifest = {"input_start_s": INPUT_START, "duration_s": SIMULATION_TIME,
                "dt": args.dt, "fps": args.fps, "dpi": args.dpi,
                "skipped_incompatible": skipped, "cases": []}
    # Preserve only current, unselected cases in the shared output directory.
    previous = root / "manifest.json"
    if previous.exists():
        old = json.loads(previous.read_text(encoding="utf-8"))
        current_cases, current_skipped = build_plan(FAMILIES)
        valid = {(c["family"], c["model"], c["scenario"]) for c in current_cases}
        incompatible = {(c["family"], c["model"], c["scenario"]) for c in current_skipped}
        selected = {(c["family"], c["model"], c["scenario"]) for c in cases}
        manifest["cases"] = [c for c in old.get("cases", [])
                             if (c["family"], c["model"], c["scenario"]) in valid - selected]
        seen = {(c["family"], c["model"], c["scenario"]) for c in skipped}
        manifest["skipped_incompatible"].extend(
            c for c in old.get("skipped_incompatible", [])
            if (c["family"], c["model"], c["scenario"]) in incompatible - seen
        )
    print(f"Output: {root}\nCases: {len(cases)}, excluded incompatible: {len(skipped)}", flush=True)
    failures = 0
    for index, case in enumerate(cases, 1):
        directory = root / case["family"] / case["model"] / case["scenario"]
        directory.mkdir(parents=True, exist_ok=True)
        pkl, gif = directory / "result.pkl", directory / "response.gif"
        row = {**case, "status": "pending", "pkl": "", "gif": "", "error": ""}
        manifest["cases"].append(row)
        print(f"[{index}/{len(cases)}] {case['family']}/{case['model']}/{case['scenario']}", flush=True)
        try:
            sus, state, road, model, config = prepare_case(**case, dt=args.dt, seed=args.seed)
            if resume and pkl.exists():
                logger = SimulationLogger.load(pkl)
                if logger.metadata.get("config") != config:
                    raise ValueError("Saved configuration/source differs; select a new --output directory")
            elif args.gif_only:
                raise FileNotFoundError(f"No saved PKL: {pkl}")
            else:
                logger = simulate_case(sus, state, road, model, config)
                temp = pkl.with_suffix(".partial.pkl")
                logger.save(temp)
                temp.replace(pkl)
                # A rerun invalidates animation files from the previous data.
                gif.unlink(missing_ok=True)
                (directory / "animation.json").unlink(missing_ok=True)
            row["pkl"] = pkl.relative_to(root).as_posix()
            row["status"] = "data_only"
            if not args.no_gif:
                renderer_version = 3 if case["family"] == "quarter" else 2
                animation_config = {"renderer_version": renderer_version, "fps": args.fps, "dpi": args.dpi,
                                    "data_sha256": hashlib.sha256(pkl.read_bytes()).hexdigest()}
                animation_info = directory / "animation.json"
                cached = json.loads(animation_info.read_text(encoding="utf-8")) if animation_info.exists() else None
                if not (resume and gif.exists() and cached == animation_config):
                    save_animation(logger, gif, args.fps, args.dpi)
                    animation_info.write_text(json.dumps(animation_config, indent=2), encoding="utf-8")
                row["gif"] = gif.relative_to(root).as_posix()
                row["status"] = "complete"
            (directory / "error.txt").unlink(missing_ok=True)
        except Exception as exc:
            failures += 1
            row["status"] = "failed"
            row["error"] = f"{type(exc).__name__}: {exc}"
            (directory / "error.txt").write_text(traceback.format_exc(), encoding="utf-8")
            print(f"  FAILED: {row['error']}", flush=True)
        write_manifest(root, manifest)
    print(f"Finished {len(cases)} cases; {failures} failed. Summary: {root / 'summary.csv'}", flush=True)
    return 1 if failures else 0


