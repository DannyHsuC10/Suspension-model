"""Find missing project parameters before the desktop app validates the repair."""
import copy
from dataclasses import dataclass
import math

from .catalog import FAMILIES
from .project import (FIELDS, LEGACY_LR_MASS_FIELDS, blank, default_project, parameter_fields,
                      parameters_for_analysis, prepare_project, read_project, unsprung_masses)


@dataclass(frozen=True)
class MissingField:
    path: tuple[str, ...]
    label: str
    unit: str
    default: str | float

    @property
    def location(self):
        return ".".join(self.path)

    def parse(self, raw):
        raw = raw.strip()
        if not raw:
            return self.default
        if isinstance(self.default, str):
            return raw
        try:
            value = float(raw)
            if math.isfinite(value):
                return value
        except ValueError:
            pass
        raise ValueError(f"{self.label}（{self.location}）必須為有限數值。")


def _section(parent, key, location):
    values = parent.get(key)
    if blank(values):
        values = parent[key] = {}
    if not isinstance(values, dict):
        raise ValueError(f"{location} 必須為 JSON 物件。")
    return values


def read_import_project(path):
    project = read_project(path)
    defaults = default_project()
    groups = _section(project, "parameters", "parameters")
    compact = groups.keys() <= {"full"}
    missing = []
    if blank(project.get("name")):
        missing.append(MissingField(("name",), "專案名稱", "", defaults["name"]))

    families = ("full",) if compact else FAMILIES
    for family in families:
        group_path = ("parameters", family)
        values = _section(groups, family, ".".join(group_path))
        fields = parameter_fields(family, values, linked=True)
        baseline = parameters_for_analysis(defaults, family).copy()
        if family == "full":
            baseline.update({f"mu_{corner}": mass for corner, mass in unsprung_masses(baseline).items()})
            if compact:
                local = defaults["parameters"]
                fields = fields | {"zeta": FIELDS["quarter"]["zeta"]} | {
                    key: FIELDS["LR"][key] for key in local["LR"]
                }
                baseline.update(local["LR"], zeta=local["quarter"]["zeta"])
        elif family == "LR":
            baseline.update(dict.fromkeys(LEGACY_LR_MASS_FIELDS, baseline["mu"]))
        if family in ("LR", "FR"):
            baseline["zeta"] = defaults["parameters"]["quarter"]["zeta"]

        for key, (label, unit, *_) in fields.items():
            if blank(values.get(key)):
                missing.append(MissingField(group_path + (key,), f"{FAMILIES[family]} · {label}",
                                            unit, baseline[key]))
    return project, missing


def fill_import_project(project, fields, inputs):
    candidate = copy.deepcopy(project)
    for field, raw in zip(fields, inputs, strict=True):
        parent = candidate
        for key in field.path[:-1]:
            parent = parent[key]
        parent[field.path[-1]] = field.parse(raw)
    return prepare_project(candidate)
