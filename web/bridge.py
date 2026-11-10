"""Browser I/O adapter. Model inputs, simulation and result formats stay shared."""
from dataclasses import asdict, replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from suspension_dynamics.catalog import FAMILIES, MODELS, scenarios, targets
from suspension_dynamics.engine import Result, simulate
from suspension_dynamics.project import (
    FIELDS, INPUT_FIELDS, LINKED_LOCAL_FIELDS, PARAMETER_SOURCES, default_project,
    migrate_parameters, parameters_for_analysis, prepare_project, save_project, validate,
)
from suspension_dynamics.project_import import fill_import_project, read_import_project
from suspension_dynamics.storage import (
    fingerprint, format_stat, save_result, statistics, validate_result, write_csv,
)
from web.english import (
    FAMILY_LABELS, GROUP_LABELS, INPUT_LABELS, MODEL_LABELS, PROJECT_NAME, RESPONSE_NOTE,
    SCENARIO_LABELS, TARGET_LABELS, WARNING_LABELS, error_message, parameter_label, unit_label,
)


def bootstrap():
    project = default_project()
    project["name"] = PROJECT_NAME
    return dict(
        project=project, families=FAMILY_LABELS, models=MODEL_LABELS, scenarios=SCENARIO_LABELS,
        family_scenarios={family: {model: scenarios(family, model) for model in models}
                          for family, models in MODELS.items()},
        targets={family: {key: TARGET_LABELS[key] for key in targets(family)} for family in FAMILIES},
        sources={family: (name, {key: TARGET_LABELS[key] for key in choices})
                 for family, (name, choices) in PARAMETER_SOURCES.items()},
        inputs={key: (INPUT_LABELS[key], unit_label(unit)) for key, (_, unit) in INPUT_FIELDS.items()},
        fields={family: {key: (parameter_label(family, key, label), unit_label(unit))
                         for key, (label, unit, *_) in fields.items()}
                for family, fields in FIELDS.items()},
        local=LINKED_LOCAL_FIELDS, response_note=RESPONSE_NOTE, group_labels=GROUP_LABELS, warning_labels=WARNING_LABELS,
    )


def resolved_project(project):
    validate(project)
    effective = {family: parameters_for_analysis(project, family) for family in FAMILIES}
    for family, values in effective.items():
        if family in LINKED_LOCAL_FIELDS and not all(np.isfinite(v) for v in values.values()):
            raise ValueError("參數換算結果超出數值範圍。")
    if project.get("auto_damping", {}).get("quarter"):
        project["parameters"]["quarter"]["cs"] = effective["quarter"]["cs"]
    return dict(project=validate(project), effective=effective)


def pack_result(result):
    meta = {name: getattr(result, name) for name in
            ("family", "model", "units", "groups", "project", "info")}
    meta["comparison"] = fingerprint(result)
    meta["statistics"] = [["Not settled" if value == "未安定" else format_stat(value) for value in row]
                          for row in statistics(result)]
    return dict(meta=meta, time=result.time, signals=result.signals)


def localize_missing_field(field):
    if field.path == ("name",):
        return replace(field, label="Project name", default=PROJECT_NAME)
    _, family, key = field.path
    original = field.label.split(" · ", 1)[-1]
    label = f"{FAMILY_LABELS[family]} · {parameter_label(family, key, original)}"
    return replace(field, label=label, unit=unit_label(field.unit))


def file_action(action, args, data):
    with TemporaryDirectory() as directory:
        path = Path(directory) / "file"
        if action == "import-project":
            path.write_bytes(data.to_py())
            project, fields = read_import_project(path)
            if not fields:
                return dict(project=prepare_project(project))
            fields = [localize_missing_field(field) for field in fields]
            if "repairs" in args:
                return dict(project=fill_import_project(project, fields, args["repairs"]))
            return dict(missing=[asdict(field) for field in fields])
        if action == "save-project":
            save_project(path, resolved_project(args["project"])["project"])
            return path.read_bytes()

        meta = args["meta"]
        arrays = data.to_py()
        result = validate_result(Result(
            meta["family"], meta["model"], np.asarray(arrays["time"]),
            {name: np.asarray(values) for name, values in arrays["signals"].items()},
            meta["units"], meta["groups"], meta["project"], meta["info"],
        ))
        if args["format"] == "npz":
            save_result(path, result)
        elif args["format"] == "csv":
            write_csv(path, result)
        else:
            raise ValueError("Unsupported export format.")
        return path.read_bytes()


def dispatch(action, args, data, notify):
    if action == "bootstrap":
        return bootstrap()
    if action == "parameters":
        return resolved_project(args["project"])
    if action == "run":
        return pack_result(simulate(args["project"], args["family"], progress=notify))
    if action == "restore":
        project, snapshot, family = args["project"], args["snapshot"], args["family"]
        values = snapshot["parameters"][family]
        if family in LINKED_LOCAL_FIELDS:
            project["parameters"][family] = {key: values[key] for key in LINKED_LOCAL_FIELDS[family]}
        else:
            project["parameters"][family] = migrate_parameters(family, values)
        project["analysis"][family] = snapshot["analysis"][family]
        if family == "quarter":
            project["auto_damping"] = snapshot.get("auto_damping", {})
        return resolved_project(project)["project"]
    return file_action(action, args, data)


def handle(action, payload, data=None, notify=None):
    try:
        return dispatch(action, json.loads(payload), data, notify)
    except ValueError as error:
        raise ValueError(error_message(str(error))) from None
