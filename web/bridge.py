"""Browser I/O adapter. Model inputs, simulation and result formats stay shared."""
from dataclasses import asdict
import json
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from suspension_dynamics.catalog import FAMILIES, MODELS, SCENARIOS, TARGETS, scenarios, targets
from suspension_dynamics.engine import Result, simulate
from suspension_dynamics.project import (
    FIELDS, INPUT_FIELDS, LINKED_LOCAL_FIELDS, PARAMETER_SOURCES, default_project,
    migrate_parameters, parameters_for_analysis, prepare_project, save_project, validate,
)
from suspension_dynamics.project_import import fill_import_project, read_import_project
from suspension_dynamics.storage import (
    RESPONSE_NOTE, fingerprint, format_stat, load_result, save_result, statistics,
    validate_result, write_csv, write_report,
)


def bootstrap():
    return dict(
        project=default_project(), families=FAMILIES, models=MODELS, scenarios=SCENARIOS,
        family_scenarios={family: {model: scenarios(family, model) for model in models}
                          for family, models in MODELS.items()},
        targets={family: {key: TARGETS[key] for key in targets(family)} for family in FAMILIES},
        sources=PARAMETER_SOURCES, inputs=INPUT_FIELDS, fields=FIELDS,
        local=LINKED_LOCAL_FIELDS, response_note=RESPONSE_NOTE,
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
            ("family", "model", "units", "groups", "project", "info", "legacy")}
    meta["comparison"] = fingerprint(result)
    meta["statistics"] = [[format_stat(value) for value in row] for row in statistics(result)]
    return dict(meta=meta, time=result.time, signals=result.signals)


def file_action(action, args, data):
    with TemporaryDirectory() as directory:
        path = Path(directory) / "file"
        if action == "import-project":
            path.write_bytes(data.to_py())
            project, fields = read_import_project(path)
            if not fields:
                return dict(project=prepare_project(project))
            if "repairs" in args:
                return dict(project=fill_import_project(project, fields, args["repairs"]))
            return dict(missing=[asdict(field) for field in fields])
        if action == "import-result":
            path.write_bytes(data.to_py())
            return pack_result(load_result(path))
        if action == "save-project":
            save_project(path, resolved_project(args["project"])["project"])
            return path.read_bytes()

        meta = args["meta"]
        arrays = data.to_py()
        result = validate_result(Result(
            meta["family"], meta["model"], np.asarray(arrays["time"]),
            {name: np.asarray(values) for name, values in arrays["signals"].items()},
            meta["units"], meta["groups"], meta["project"], meta["info"], meta["legacy"],
        ))
        if args["format"] == "npz":
            save_result(path, result)
        elif args["format"] == "csv":
            write_csv(path, result)
        elif args["format"] == "html":
            write_report(path, result, args["svg"])
        else:
            raise ValueError("不支援的匯出格式。")
        return path.read_bytes()


def handle(action, payload, data=None, notify=None):
    args = json.loads(payload)
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
