"""JSON projects, editable source parameters and strict boundary validation."""
import copy
import json
import math
from pathlib import Path

from .catalog import CORNERS, FAMILIES, MODELS, TARGETS, scenarios, targets
from .road_profiles import ISO_FREQUENCY_BAND, ISO_ROUGHNESS

# (label, unit, lower bound, strictly positive)
def field(label, unit, minimum=0., positive=False):
    return label, unit, minimum, positive

COMMON = {
    "ms": field("每端簧上質量 ms", "kg", positive=True),
    "mu": field("每端簧下質量 mu", "kg", positive=True),
    "kt": field("輪胎剛性 kt", "N/m", positive=True), "ct": field("輪胎阻尼 ct", "N·s/m"),
    "zeta": field("自動估算用阻尼比 ζ", "1"),
}
LINKED_LOCAL_FIELDS = {"quarter": ("zeta", "ks", "cs", "mr"), "LR": ("k_frame", "c_frame", "m_frame"), "FR": ()}
SIZING_METADATA = (
    "tender_enabled", "tender_stiffness_N_per_m", "heave_configuration", "heave_stiffness_basis",
)
MODAL_CHANNELS = ("heave_f", "heave_r", "roll_f", "roll_r", "roll_c", "warp")
FULL_CHANNEL_LABELS = {name: name for name in MODAL_CHANNELS} | {
    "corner_f": "前軸直推 corner_f",
    "corner_r": "後軸直推 corner_r",
    "tire_f": "前軸輪胎 tire_f",
    "tire_r": "後軸輪胎 tire_r",
}
CHANNEL_UNITS = {"k": "N/m", "c": "N·s/m", "mr": "彈簧／輪端"}
LEGACY_WHEEL_KEYS = {
    f"{component}_{axle}_{kind}": tuple(f"{component}_{axle.upper()}{side}_{kind}" for side in "LR")
    for component in ("corner", "tire") for axle in "fr" for kind in CHANNEL_UNITS
}
PARAMETER_SOURCES = {
    "quarter": ("parameter_corner", {corner: TARGETS[corner] for corner in CORNERS}),
    "LR": ("parameter_axle", {"front": "前軸參數"}),
    "FR": ("parameter_axle", {"front": "前軸參數", "rear": "後軸參數"}),
}
LEGACY_FULL_MASS_FIELDS = {f"mu_{corner}": field(f"{corner} 簧下質量", "kg", positive=True) for corner in CORNERS}
LEGACY_LR_MASS_FIELDS = {f"mu_{side}": field(f"{label} 簧下質量", "kg", positive=True)
                         for side, label in (("left", "左輪"), ("right", "右輪"))}
LR_WHEEL_FIELDS = {
    f"{key}_{side}": field(f"{label} {name}", unit)
    for side, label in (("left", "左輪"), ("right", "右輪"))
    for key, name, unit in (("kt", "輪胎剛性", "N/m"), ("ct", "輪胎阻尼", "N·s/m"))
}
ANTI_FIELDS = {"anti_front": field("前軸 Anti 比例", "1=100%"),
               "anti_rear": field("後軸 Anti 比例", "1=100%")}
LEGACY_ANTI_KEYS = {
    "full": {"anti_front": "dive_front", "anti_rear": "lift_rear"},
    "FR": {"anti_front": "anti_dive_front", "anti_rear": "anti_squat_rear"},
}
LEGACY_ANTI_FIELDS = {
    "full": {
        "dive_front": field("前 Anti-dive", "1=100%"), "lift_front": field("前加速 Anti-lift", "1=100%"),
        "squat_rear": field("後 Anti-squat", "1=100%"), "lift_rear": field("後煞車 Anti-lift", "1=100%"),
        "drive_front": field("前軸驅動力比例", "0..1"), "brake_front": field("前軸煞車力比例", "0..1"),
    },
    "FR": {"anti_dive_front": field("前軸 Anti-dive", "1=100%"),
           "anti_squat_rear": field("後軸 Anti-squat", "1=100%")},
}
FIELDS = {
    "quarter": {**COMMON, "kt": field("輪胎剛性 kt", "N/m"), "ks": field("彈簧剛性 ks", "N/m", positive=True),
                "cs": field("阻尼係數 cs", "N·s/m"), "mr": field("MR 彈簧／輪端", "1", positive=True)},
    "LR": {**COMMON, "kt": field("輪胎剛性 kt", "N/m"),
           "track": field("輪距", "m", positive=True), "Ix": field("半車側傾慣量 Ix", "kg·m²", positive=True),
           "h_cg": field("重心高度", "m"), "h_rc": field("Roll center 高度", "m", -10.),
           "k_heave": field("heave spring", "N/m"), "c_heave": field("heave damper", "N·s/m"),
           "k_roll": field("roll spring", "N/m"), "c_roll": field("roll damper", "N·s/m"),
           **{f"{t}_{n}": field(f"{name} {t}", "N/m" if t == "k" else "N·s/m")
              for n, name in (("left", "左直推"), ("right", "右直推")) for t in ("k", "c")},
           "mr_heave": field("heave motion ratio", "1", positive=True),
           "mr_roll": field("roll motion ratio", "1", positive=True),
           "mr_corner": field("corner motion ratio", "1", positive=True),
           "k_frame": field("車架傳力剛性", "N/m", positive=True), "c_frame": field("車架傳力阻尼", "N·s/m"),
           "m_frame": field("車架傳力虛擬質量", "kg", positive=True)},
    "FR": {**COMMON, "kt": field("輪胎剛性 kt", "N/m"),
           "lf": field("重心至前軸 lf", "m", positive=True), "lr": field("重心至後軸 lr", "m", positive=True),
           "Iy": field("半車俯仰慣量 Iy", "kg·m²", positive=True), "h_cg": field("重心高度", "m"),
           **ANTI_FIELDS,
           "k_heave_f": field("front heave spring", "N/m"), "c_heave_f": field("front heave damper", "N·s/m"),
           "k_heave_r": field("rear heave spring", "N/m"), "c_heave_r": field("rear heave damper", "N·s/m"),
           **{f"{t}_{n}": field(f"{name} {t}", "N/m" if t == "k" else "N·s/m")
              for n, name in (("front", "前直推"), ("rear", "後直推")) for t in ("k", "c")},
           "mr_front": field("front heave motion ratio", "1", positive=True),
           "mr_rear": field("rear heave motion ratio", "1", positive=True),
           "mr_corner": field("corner motion ratio", "1", positive=True)},
    "full": {
        "m": field("整車總質量（含簧下）", "kg", positive=True),
        "mu_f": field("前輪簧下質量（每輪）", "kg", positive=True),
        "mu_r": field("後輪簧下質量（每輪）", "kg", positive=True),
        "lf": field("簧上重心至前軸 lf", "m", positive=True), "lr": field("簧上重心至後軸 lr", "m", positive=True),
        "tf": field("前輪距", "m", positive=True), "tr": field("後輪距", "m", positive=True),
        "h_cg": field("等效慣性力高度", "m"), "Ixx": field("全車簧上側傾慣量", "kg·m²", positive=True),
        "Iyy": field("簧上俯仰慣量", "kg·m²", positive=True),
        "h_rc_f": field("前 RC 高度", "m", -10.), "h_rc_r": field("後 RC 高度", "m", -10.),
        **ANTI_FIELDS,
        "body_twist_k": field("車架扭轉剛性", "N·m/rad"), "body_twist_c": field("車架扭轉阻尼", "N·m·s/rad"),
        **{f"{name}_{kind}": field(f"{label} {kind}", unit, positive=kind == "mr")
           for name, label in FULL_CHANNEL_LABELS.items() for kind, unit in CHANNEL_UNITS.items()},
    },
}
LEGACY_WHEEL_FIELDS = {
    old: (old, *FIELDS["full"][key][1:])
    for key, old_keys in LEGACY_WHEEL_KEYS.items() for old in old_keys
}
ESTIMATED_DAMPING = {
    "quarter": ("cs",),
    "LR": ("c_heave", "c_roll", "c_left", "c_right"),
    "FR": ("c_heave_f", "c_heave_r", "c_front", "c_rear"),
}
INPUT_FIELDS = {
    "duration": ("模擬時間", "s"), "dt": ("輸出時間步長", "s"), "start": ("輸入開始", "s"),
    "height": ("路面高度／振幅", "m"), "period": ("凸塊持續時間", "s"), "frequency": ("正弦頻率", "Hz"),
    "speed": ("車速", "m/s"), "ax": ("縱向加速度", "m/s²"), "ay": ("側向加速度 ay", "m/s²"),
    "ramp": ("加速度上升時間", "s"), "seed": ("隨機種子", "整數"), "random_interval": ("隨機路面節點間隔", "s"),
}


def default_project():
    project = json.loads(Path(__file__).with_name("default_project.json").read_text(encoding="utf-8"))
    return link_full_parameters(project)


def link_full_parameters(project):
    """Migrate editable projects while keeping archived result parameters intact."""
    full = project["parameters"]["full"]
    if "mu_f" not in full:
        for axle in ("F", "R"):
            left = full.pop(f"mu_{axle}L")
            right = full.pop(f"mu_{axle}R")
            full[f"mu_{axle.lower()}"] = (left + right) / 2
    for family, (key, choices) in PARAMETER_SOURCES.items():
        project["analysis"][family].setdefault(key, next(iter(choices)))
    return project


def migrate_axle_anti(family, parameters):
    legacy = LEGACY_ANTI_FIELDS.get(family)
    if legacy is None or not isinstance(parameters, dict) or not legacy.keys() & parameters.keys():
        return parameters
    values = {key: value for key, value in parameters.items() if key not in legacy}
    for key, old_key in LEGACY_ANTI_KEYS[family].items():
        if old_key in parameters:
            values.setdefault(key, parameters[old_key])
    return values


def migrate_parameters(family, parameters):
    parameters = migrate_axle_anti(family, parameters)
    if family != "full" or not isinstance(parameters, dict) or not LEGACY_WHEEL_FIELDS.keys() & parameters.keys():
        return parameters
    values = {key: value for key, value in parameters.items() if key not in LEGACY_WHEEL_FIELDS}
    for key, old_keys in LEGACY_WHEEL_KEYS.items():
        if not blank(values.get(key)):
            continue
        present = [parameters[old] for old in old_keys if not blank(parameters.get(old))]
        if len(present) == 1:
            values[key] = present[0]
        elif len(present) == 2:
            minimum, positive = FIELDS["full"][key][2:]
            invalid = [v for v in present if not valid_field_value(v, minimum, positive)]
            if invalid:
                values[key] = invalid[0]  # Let validation report the invalid input.
            else:
                left, right = present
                values[key] = left if left == right else left / 2 + right / 2
    return values


def blank(value):
    return value is None or (isinstance(value, str) and not value.strip())


def migrate_project_parameters(project):
    """Convert editable inputs while leaving archived result snapshots intact."""
    groups = project.get("parameters") if isinstance(project, dict) else None
    if not isinstance(groups, dict):
        return project
    return {**project, "parameters": {family: migrate_parameters(family, values)
                                      for family, values in groups.items()}}


def unsprung_masses(full):
    if "mu_f" in full:
        return {corner: full[f"mu_{corner[0].lower()}"] for corner in CORNERS}
    return {corner: full[f"mu_{corner}"] for corner in CORNERS}


def full_mass_distribution(full):
    masses = unsprung_masses(full)
    sprung_mass = full["m"] - sum(masses.values())
    front_share = full["lr"] / (full["lf"] + full["lr"])
    return sprung_mass, front_share, masses


def wheel_rates(full, channel):
    mr_squared = full[f"{channel}_mr"] ** 2
    return full[f"{channel}_k"] * mr_squared, full[f"{channel}_c"] * mr_squared


def corner_rates(full, component, corner):
    channel = f"{component}_{corner[0].lower()}"
    if f"{channel}_mr" not in full:  # Archived NPZ results can have different values for each wheel.
        channel = f"{component}_{corner}"
    return wheel_rates(full, channel)


def parameters_for_analysis(project, family):
    """Reduce full-vehicle parameters to the selected wheel or half-car coordinates."""
    p = project["parameters"][family]
    if family not in PARAMETER_SOURCES:
        return p
    source_key, _ = PARAMETER_SOURCES[family]
    source = project["analysis"][family].get(source_key)
    if source is None:
        return p

    p = p.copy()
    full = project["parameters"]["full"]
    sprung_mass, front_share, masses = full_mass_distribution(full)
    if family == "quarter":
        share = front_share if source.startswith("F") else 1 - front_share
        kt, ct = corner_rates(full, "tire", source)
        p.update(ms=sprung_mass * share / 2, mu=masses[source], kt=kt, ct=ct)
        if project.get("auto_damping", {}).get(family):
            p = estimate_damping(family, p)
        return p

    suffix = source[0]
    corners = (suffix.upper() + "L", suffix.upper() + "R")
    mu = [masses[corner] for corner in corners]
    tire = [corner_rates(full, "tire", corner) for corner in corners]
    direct = [corner_rates(full, "corner", corner) for corner in corners]
    heave = f"heave_{suffix}"
    p.update(h_cg=full["h_cg"], mu=sum(mu) / 2,
             kt=(tire[0][0] + tire[1][0]) / 2, ct=(tire[0][1] + tire[1][1]) / 2,
             mr_corner=1.)
    if family == "LR":
        p.update(ms=sprung_mass * front_share / 2, Ix=full["Ixx"] * front_share,
                 track=full["tf"], h_rc=full["h_rc_f"])
        # The lateral model uses normalized sum/difference coordinates.
        for mode in ("heave", "roll"):
            channel = f"{mode}_f"
            p[f"k_{mode}"] = 2 * full[f"{channel}_k"]
            p[f"c_{mode}"] = 2 * full[f"{channel}_c"]
            p[f"mr_{mode}"] = full[f"{channel}_mr"]
        for side, (kt, ct), (ks, cs) in zip(("left", "right"), tire, direct):
            p.pop(f"mu_{side}", None)
            p.update({f"kt_{side}": kt, f"ct_{side}": ct, f"k_{side}": ks, f"c_{side}": cs})
    else:
        p.update(ms=sprung_mass / 4, Iy=full["Iyy"] / 2, lf=full["lf"], lr=full["lr"],
                 anti_front=full["anti_front"], anti_rear=full["anti_rear"])
        # In symmetric axle heave, each side carries half the full axle force.
        for end, index in (("front", "f"), ("rear", "r")):
            p[f"k_heave_{index}"] = 2 * full[f"{heave}_k"]
            p[f"c_heave_{index}"] = 2 * full[f"{heave}_c"]
            p[f"mr_{end}"] = full[f"{heave}_mr"]
            p[f"k_{end}"] = (direct[0][0] + direct[1][0]) / 2
            p[f"c_{end}"] = (direct[0][1] + direct[1][1]) / 2
    return p


def parameter_fields(family, parameters, *, linked=False):
    """Return the field definitions for the parameter layout indicated by its keys."""
    fields = FIELDS[family]
    if linked and family in LINKED_LOCAL_FIELDS and parameters.keys() <= set(LINKED_LOCAL_FIELDS[family]):
        return {key: fields[key] for key in LINKED_LOCAL_FIELDS[family]}
    if family in ("LR", "FR") and "zeta" not in parameters:
        fields = {key: value for key, value in fields.items() if key != "zeta"}
    if family == "LR":
        for extra_fields in (LR_WHEEL_FIELDS, LEGACY_LR_MASS_FIELDS):
            if extra_fields.keys() & parameters.keys():
                fields = fields | extra_fields
    elif family == "full":
        if LEGACY_FULL_MASS_FIELDS.keys() & parameters.keys():
            fields = {key: value for key, value in fields.items() if key not in ("mu_f", "mu_r")}
            fields.update(LEGACY_FULL_MASS_FIELDS)
        if LEGACY_WHEEL_FIELDS.keys() & parameters.keys():
            fields = {key: value for key, value in fields.items() if key not in LEGACY_WHEEL_KEYS} | LEGACY_WHEEL_FIELDS
    legacy_anti = LEGACY_ANTI_FIELDS.get(family, {})
    if legacy_anti.keys() & parameters.keys():
        fields = {key: value for key, value in fields.items() if key not in ANTI_FIELDS} | legacy_anti
    return fields


def valid_field_value(value, minimum, positive):
    return (not isinstance(value, bool) and isinstance(value, (float, int)) and math.isfinite(value)
            and (value > minimum if positive else value >= minimum))


def validate_parameters(family, parameters, *, linked=False):
    fields = parameter_fields(family, parameters, linked=linked) if isinstance(parameters, dict) else None
    if fields is None or parameters.keys() != fields.keys():
        raise ValueError(f"{FAMILIES[family]} 參數不完整或含未知欄位。")
    for key, (label, unit, minimum, positive) in fields.items():
        value = parameters[key]
        if not valid_field_value(value, minimum, positive):
            raise ValueError(f"{FAMILIES[family]} · {label} ({unit}) 必須為有限數，且 {'>' if positive else '≥'} {minimum:g}。")
    if family == "full":
        if parameters["m"] <= sum(unsprung_masses(parameters).values()):
            raise ValueError("整車總質量必須大於四個簧下質量之和。")
        if parameters.get("drive_front", 0) > 1 or parameters.get("brake_front", 0) > 1:
            raise ValueError("驅動／煞車前軸比例必須介於 0 與 1。")


def validate(project):
    if not isinstance(project, dict) or project.get("application") != "SuspensionDynamicsLab" or project.get("schema_version") != 1:
        raise ValueError("請開啟 SuspensionDynamicsLab 的版本 1 專案 JSON。")
    if not isinstance(project.get("name"), str) or not project["name"].strip():
        raise ValueError("專案名稱不可空白。")
    auto_damping = project.get("auto_damping", {})
    if not isinstance(auto_damping, dict) or any(
        family not in ESTIMATED_DAMPING or not isinstance(enabled, bool)
        for family, enabled in auto_damping.items()
    ):
        raise ValueError("阻尼自動估算設定無效。")
    for family in FAMILIES:
        options = validate_options(family, project.get("analysis", {}).get(family, {}))
        source = PARAMETER_SOURCES.get(family)
        validate_parameters(family, project.get("parameters", {}).get(family),
                            linked=source is not None and source[0] in options)
    return project


def validate_options(family, opt):
    if opt.get("model") not in MODELS[family]:
        raise ValueError("模型類型不適用於目前分類。")
    if opt.get("scenario") not in scenarios(family, opt["model"]):
        raise ValueError("此模型不支援所選輸入。")
    if opt.get("target") not in targets(family):
        raise ValueError("路面作用輪位不適用於此模型。")
    source_key, choices = PARAMETER_SOURCES.get(family, (None, ()))
    for key in ("parameter_axle", "parameter_corner"):
        if key in opt and (key != source_key or opt[key] not in tuple(choices)):
            raise ValueError("全車參數的來源輪位或車軸不適用於此模型。")
    for key in INPUT_FIELDS:
        v = opt.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f"{INPUT_FIELDS[key][0]} 必須為有限數值。")
    if not .00001 <= opt["dt"] <= .1 or not 0 < opt["duration"] <= 120 or opt["duration"] / opt["dt"] > 200000:
        raise ValueError("時間步長需在 0.00001～0.1 s，模擬時間 0～120 s，輸出不可超過 200,000 步。")
    if not 0 <= opt["start"] < opt["duration"]:
        raise ValueError("輸入開始時間需介於 0 與模擬終止時間之間。")
    for key in ("period", "frequency", "speed", "ramp", "random_interval"):
        if opt[key] <= 0:
            raise ValueError(f"{INPUT_FIELDS[key][0]} 必須大於 0。")
    if opt["seed"] != int(opt["seed"]) or not 0 <= opt["seed"] <= 2**32-1:
        raise ValueError("隨機種子必須為 0～4294967295 的整數。")
    if opt["scenario"] in ISO_ROUGHNESS and opt["speed"] >= .5 / (ISO_FREQUENCY_BAND[1] * opt["dt"]):
        limit = .5 / ISO_FREQUENCY_BAND[1] / opt["speed"]
        raise ValueError(f"以目前車速取樣 ISO 路面時，輸出步長須小於 {limit:.6g} s。")
    if opt["scenario"] == "random" and opt["duration"] / opt["random_interval"] > 200000:
        raise ValueError("隨機路面節點過多，請增加節點間隔。")
    return opt


def _normalize_sizing_export(project):
    sections = {}
    for name in ("dynamics", "geometry", "vehicle"):
        values = project.get(name)
        if blank(values):
            values = {}
        if not isinstance(values, dict):
            raise ValueError(f"{name} 必須為 JSON 物件。")
        sections[name] = values

    full = sections["vehicle"] | sections["dynamics"]
    if project["schema_version"] == 3:
        full["heave_stiffness_basis"] = "main_spring"
    geometry = sections["geometry"].copy()
    full.update({key: geometry.pop(key) for key in ("h_rc_f", "h_rc_r") if key in geometry})
    geometry["vehicle"] = {key: full.pop(key) for key in ("gravity", "free_radius") if key in full}
    if "clearance_steer_deg" in geometry:
        geometry["vehicle"]["clearance_steer_deg"] = geometry.pop("clearance_steer_deg")
    for section, fields in (
        ("front", {"fvsa": "fvsa_f", "kpi_deg": "kpi_f_deg", "caster_deg": "caster_f_deg"}),
        ("rear", {"fvsa": "fvsa_r", "kpi_deg": "kpi_r_deg", "caster_deg": "caster_r_deg"}),
        ("analysis", {key: key for key in ("bump_mm", "droop_mm", "steer_deg")}),
    ):
        values = {key: geometry.pop(source) for key, source in fields.items() if source in geometry}
        if values:
            geometry[section] = values
    return {
        "application": "SuspensionDynamicsLab", "schema_version": 1, "name": project.get("name"),
        "parameters": {"full": full}, "geometry": geometry,
    }


def read_project(path):
    project = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if (isinstance(project, dict) and project.get("application") == "VehicleInitialSizingLab-DynamicsExport"
            and project.get("schema_version") in (2, 3)):
        project = _normalize_sizing_export(project)
    if (not isinstance(project, dict) or project.get("schema_version") != 1
            or project.get("application") not in ("SuspensionDynamicsLab", "VehicleInitialSizingLab-DynamicsExport")):
        raise ValueError("請開啟 SuspensionDynamicsLab 專案或 Initial Sizing 的全車參數 JSON。")
    return migrate_project_parameters(project)


def load_project(path):
    return prepare_project(read_project(path))


def prepare_project(project):
    """Normalize and validate a freshly read project in place."""
    defaults = default_project()
    groups = project.get("parameters")
    project["analysis"] = defaults["analysis"]
    if isinstance(groups, dict) and groups.keys() == {"full"} and isinstance(groups["full"], dict):
        project["application"] = "SuspensionDynamicsLab"
        full = groups["full"]
        metadata = {key: full.pop(key) for key in SIZING_METADATA if key in full}
        if metadata:
            project["initial_sizing"] = metadata
        quarter = defaults["parameters"]["quarter"]
        quarter["zeta"] = full.pop("zeta", quarter["zeta"])
        frame = {key: full.pop(key, value) for key, value in defaults["parameters"]["LR"].items()}
        groups.update(quarter=quarter, LR=frame, FR={})
        validate(project)
        groups["quarter"] = estimate_quarter_from_full(project, metadata)
    return link_full_parameters(validate(upgrade_project(project)))


def estimate_quarter_from_full(project, metadata):
    full = project["parameters"]["full"]
    tender = metadata.get("tender_stiffness_N_per_m")
    enabled = metadata.get("tender_enabled", tender is not None)
    if not isinstance(enabled, bool):
        raise ValueError("Tender 啟用狀態須為布林值。")
    if tender is not None and (type(tender) not in (int, float) or not math.isfinite(tender) or tender <= 0):
        raise ValueError("Tender 剛性須為正的有限數值，或留空。")
    heave = full["heave_f_k"]
    if metadata.get("heave_stiffness_basis") != "main_spring":
        # Legacy exports stored the front main-spring rate divided by four.
        heave *= 4
    if enabled and tender is not None and heave > 0:
        heave = 1 / (1 / heave + 1 / tender)
    quarter = parameters_for_analysis(project, "quarter")
    quarter.update(ks=(heave + full["roll_f_k"]) / 4, mr=full["corner_f_mr"])
    validate_parameters("quarter", quarter)
    quarter = estimate_damping("quarter", quarter)
    return {key: quarter[key] for key in LINKED_LOCAL_FIELDS["quarter"]}


def upgrade_project(project):
    # Early v1 previews shared the heave MR with direct-push channels.
    if isinstance(project,dict) and project.get('application')=='SuspensionDynamicsLab' and project.get('schema_version')==1:
        fr=project.get('parameters',{}).get('FR')
        if isinstance(fr,dict) and fr:fr.setdefault('mr_corner',1.)
    return project


def save_project(path, project):
    project = migrate_project_parameters(project)
    validate(project)
    groups = project["parameters"]
    full = {**groups["full"], "zeta": groups["quarter"]["zeta"]}
    full.update({key: groups["LR"][key] for key in LINKED_LOCAL_FIELDS["LR"]})
    metadata = project.get("initial_sizing", {})
    tender = metadata.get("tender", {})
    if tender:
        full.update(tender_enabled=tender.get("enabled"), tender_stiffness_N_per_m=tender.get("stiffness_N_per_m"))
    full.update({key: metadata[key] for key in SIZING_METADATA if key in metadata})
    compact = {key: value for key, value in project.items()
               if key not in ("analysis", "parameters", "initial_sizing")}
    compact["parameters"] = {"full": full}
    Path(path).write_text(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def estimate_damping(family, values):
    """Estimate supported suspension damping coefficients from the damping ratio."""
    p = copy.deepcopy(values)
    if family == "full":
        return p
    z = p["zeta"]
    if family == "quarter":
        p["cs"] = 2*z*math.sqrt(p["ks"]*p["ms"])/p["mr"]
    elif family == "LR":
        p["c_heave"] = 2*z*math.sqrt(p["k_heave"]*p["ms"])/p["mr_heave"]
        p["c_roll"] = 2*z*math.sqrt(p["k_roll"]*2*p["Ix"]/p["track"]**2)/p["mr_roll"]
        m_eff = 1/(1/(2*p["ms"])+(p["track"]/2)**2/p["Ix"])
        for n in ("left", "right"):
            p[f"c_{n}"] = 2*z*math.sqrt(p[f"k_{n}"]*m_eff)/p["mr_corner"]
    else:
        for n, arm, mr in (("heave_f", "lf", "front"), ("heave_r", "lr", "rear"), ("front", "lf", "corner"), ("rear", "lr", "corner")):
            m_eff = 1/(1/(2*p["ms"])+p[arm]**2/p["Iy"])
            p[f"c_{n}"] = 2*z*math.sqrt(p[f"k_{n}"]*m_eff)/p[f"mr_{mr}"]
    return p
