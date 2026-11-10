"""English presentation for the browser adapter. Solver keys stay unchanged."""
import re

from suspension_dynamics.catalog import FAMILIES, SCENARIOS
from suspension_dynamics.project import FIELDS, INPUT_FIELDS


PROJECT_NAME = "Spring Damping Analysis"
FAMILY_LABELS = {
    "quarter": "Quarter Car", "LR": "Lateral Half Car",
    "FR": "Longitudinal Half Car", "full": "Full Car",
}
MODEL_LABELS = {
    "quarter": {
        "single": "1 DOF · Equivalent spring",
        "one_eighth": "1 DOF · Equivalent spring and damper",
        "double": "2 DOF · Two springs",
        "quarter": "2 DOF · Damped two-mass model",
    },
    "LR": {
        "2mass_2dof": "2 DOF · Two masses, no wheel DOF",
        "2mass": "4 DOF · Two masses with tyres",
        "half_2dof": "2 DOF · Body heave and roll",
        "half": "4 DOF · Body, tyres and roll centre",
        "rollcenter_all": "5 DOF · Dynamic frame transfer",
    },
    "FR": {
        "half_2dof": "2 DOF · Body heave and pitch",
        "half": "4 DOF · Body, tyres and anti geometry",
    },
    "full": {
        "rigid": "7 DOF · Rigid body",
        "flex": "8 DOF · Front/rear roll and chassis twist",
    },
}
SCENARIO_LABELS = {
    **SCENARIOS,  # The ISO class names are already in English.
    "bump": "Smooth road bump", "step": "Road step", "sine": "Sinusoidal road",
    "random": "Random road", "cornering": "Lateral acceleration",
    "accelerate": "Longitudinal acceleration",
    "bump_cornering": "Bump + lateral acceleration",
    "combined": "Bump + longitudinal and lateral acceleration",
}
TARGET_LABELS = {
    "all": "All wheels", "left": "Left side", "right": "Right side",
    "front": "Front axle", "rear": "Rear axle", "FL": "Front left (FL)",
    "FR": "Front right (FR)", "RL": "Rear left (RL)", "RR": "Rear right (RR)",
}
GROUP_LABELS = {
    "位移": "Displacement", "懸吊行程": "Suspension travel", "速度": "Velocity",
    "加速度": "Acceleration", "懸吊力": "Suspension force", "輪荷": "Wheel load",
    "車身姿態": "Body attitude", "升沉": "Heave", "輸入加速度": "Input acceleration",
    "模態通道力": "Modal channel force", "車架變形": "Frame deflection",
    "車架傳力": "Frame force", "能量": "Energy",
}
WARNING_LABELS = {
    "出現負輪荷：線性輪胎已超出接地假設，未啟用離地／再接觸求解。":
        "Negative wheel load detected. The linear tyre model assumes continuous road contact.",
    "此版本省略輪胎自由度；動畫輪位跟隨路面，不提供動態輪荷。":
        "This model omits wheel degrees of freedom. The animated wheels follow the road profile.",
    "系統含零頻模態，部分方向沒有回復剛性。":
        "The system has a zero-frequency mode with no restoring stiffness in that direction.",
}
RESPONSE_NOTE = (
    "Settling time is measured from the start of the input, with settling evaluated after the input finishes. "
    "The 2% band is based on the change in steady-state value, or on the maximum deviation when the "
    "signal returns to its initial equilibrium. A dash indicates an inapplicable value or no fixed equilibrium."
)

INPUT_LABELS = {
    "duration": "Simulation duration", "dt": "Output time step", "start": "Input start time",
    "height": "Road height / amplitude", "period": "Bump duration", "frequency": "Sine frequency",
    "speed": "Vehicle speed", "ax": "Longitudinal acceleration", "ay": "Lateral acceleration",
    "ramp": "Acceleration ramp time", "seed": "Random seed", "random_interval": "Random road sample interval",
}
FIELD_LABELS = {
    "ms": "Sprung mass per end (ms)", "mu": "Unsprung mass per end (mu)",
    "kt": "Tyre stiffness (kt)", "ct": "Tyre damping (ct)", "zeta": "Damping ratio (ζ)",
    "ks": "Spring stiffness (ks)", "cs": "Damping coefficient (cs)", "mr": "Motion ratio",
    "track": "Track width", "Ix": "Half-car roll inertia (Ix)", "Iy": "Half-car pitch inertia (Iy)",
    "h_cg": "CG height", "h_rc": "Roll centre height", "k_frame": "Frame transfer stiffness",
    "c_frame": "Frame transfer damping", "m_frame": "Effective frame transfer mass",
    "lf": "CG to front axle (lf)", "lr": "CG to rear axle (lr)",
    "anti_front": "Front anti ratio", "anti_rear": "Rear anti ratio",
    "m": "Total vehicle mass (including unsprung mass)", "mu_f": "Front unsprung mass per wheel",
    "mu_r": "Rear unsprung mass per wheel", "tf": "Front track width", "tr": "Rear track width",
    "Ixx": "Sprung-body roll inertia (Ixx)", "Iyy": "Sprung-body pitch inertia (Iyy)",
    "h_rc_f": "Front roll centre height", "h_rc_r": "Rear roll centre height",
    "body_twist_k": "Chassis torsional stiffness", "body_twist_c": "Chassis torsional damping",
    "dive_front": "Front anti-dive", "lift_front": "Front anti-lift", "squat_rear": "Rear anti-squat",
    "lift_rear": "Rear anti-lift", "drive_front": "Front drive force fraction",
    "brake_front": "Front braking force fraction", "anti_dive_front": "Front anti-dive",
    "anti_squat_rear": "Rear anti-squat",
}
CHANNEL_LABELS = {
    "heave_f": "Front heave", "heave_r": "Rear heave", "roll_f": "Front roll", "roll_r": "Rear roll",
    "roll_c": "Centre roll", "warp": "Warp", "corner_f": "Front direct spring/damper",
    "corner_r": "Rear direct spring/damper", "tire_f": "Front tyre", "tire_r": "Rear tyre",
}
UNIT_LABELS = {"整數": "integer", "彈簧／輪端": "spring / wheel", "1=100%": "1 = 100%"}


def parameter_label(family, key, original=None):
    if family == "full" and key == "h_cg":
        return "Effective inertia force height"
    if key in FIELD_LABELS:
        return FIELD_LABELS[key]
    channel, _, component = key.rpartition("_")
    if channel in CHANNEL_LABELS:
        kind = {"k": "stiffness", "c": "damping", "mr": "motion ratio"}[component]
        return f"{CHANNEL_LABELS[channel]} {kind}"
    side = {"left": "Left", "right": "Right", "front": "Front", "rear": "Rear"}.get(component)
    if side and channel in ("k", "c", "kt", "ct", "mu"):
        kind = {"k": "direct stiffness", "c": "direct damping", "kt": "tyre stiffness",
                "ct": "tyre damping", "mu": "unsprung mass"}[channel]
        return f"{side} {kind}"
    if key.startswith("mu_"):
        return f"{key[3:]} unsprung mass"
    return original.capitalize() if original and original.isascii() else key


def unit_label(unit):
    return UNIT_LABELS.get(unit, unit)


ERROR_LABELS = {
    "未知模型分類。": "Unknown model family.",
    "此組參數產生不穩定系統，請檢查彈簧、阻尼及車架設定。":
        "These parameters produce an unstable system. Check the spring, damper and chassis settings.",
    "初始狀態維度或數值錯誤。": "The initial state has an invalid size or contains invalid values.",
    "模擬數值發散，請檢查參數與輸入。": "The simulation diverged. Check the parameters and inputs.",
    "整車總質量必須大於四個簧下質量之和。": "Total vehicle mass must exceed the sum of the four unsprung masses.",
    "驅動／煞車前軸比例必須介於 0 與 1。": "Front drive and braking fractions must be between 0 and 1.",
    "請開啟 SuspensionDynamicsLab 的版本 1 專案 JSON。": "Open a version 1 SuspensionDynamicsLab project JSON file.",
    "專案名稱不可空白。": "Enter a project name.",
    "阻尼自動估算設定無效。": "The automatic damping settings are invalid.",
    "模型類型不適用於目前分類。": "The selected model does not belong to this model family.",
    "此模型不支援所選輸入。": "This model does not support the selected input.",
    "路面作用輪位不適用於此模型。": "The selected road input location is unavailable for this model.",
    "全車參數的來源輪位或車軸不適用於此模型。": "The selected source wheel or axle is unavailable for this model.",
    "時間步長需在 0.00001～0.1 s，模擬時間 0～120 s，輸出不可超過 200,000 步。":
        "Use an output time step of 0.00001 to 0.1 s, a duration above 0 and up to 120 s, and no more than 200,000 output steps.",
    "輸入開始時間需介於 0 與模擬終止時間之間。": "The input start time must be at least 0 and before the end of the simulation.",
    "隨機種子必須為 0～4294967295 的整數。": "The random seed must be an integer from 0 to 4,294,967,295.",
    "隨機路面節點過多，請增加節點間隔。": "There are too many random road samples. Increase the sample interval.",
    "請開啟 SuspensionDynamicsLab 專案或 Initial Sizing 的全車參數 JSON。":
        "Open a SuspensionDynamicsLab project or an Initial Sizing full-car parameter JSON file.",
    "Tender 啟用狀態須為布林值。": "The tender spring enabled flag must be true or false.",
    "Tender 剛性須為正的有限數值，或留空。": "Tender spring stiffness must be a positive finite number or left blank.",
    "結果檔必須包含嚴格遞增、有限的時間序列。": "The result must contain finite, strictly increasing times.",
    "結果檔沒有訊號。": "The result contains no signals.",
}
FAMILY_KEYS = {label: key for key, label in FAMILIES.items()}
INPUT_NAMES = {label: INPUT_LABELS[key] for key, (label, _) in INPUT_FIELDS.items()}


def error_message(message):
    if message in ERROR_LABELS:
        return ERROR_LABELS[message]
    if message.startswith("參數需要超過 750,000"):
        adjustments = "shorten the duration, lower the vehicle speed, or reduce stiffness or damping"
        if "降低車速" not in message:
            adjustments = "shorten the duration or reduce stiffness or damping"
        return f"This analysis needs more than 750,000 integration steps. Please {adjustments}."
    match = re.fullmatch(r"(.+) 參數不完整或含未知欄位。", message)
    if match:
        family = FAMILY_LABELS[FAMILY_KEYS[match[1]]]
        return f"{family} parameters are incomplete or contain unknown fields."
    match = re.fullmatch(r"(.+) · (.+) \((.*)\) 必須為有限數，且 ([>≥]) (.+)。", message)
    if match:
        family = FAMILY_KEYS[match[1]]
        key = next((key for key, (label, *_) in FIELDS[family].items() if label == match[2]), None)
        label = parameter_label(family, key, match[2]) if key else "Parameter"
        return f"{FAMILY_LABELS[family]} · {label} ({unit_label(match[3])}) must be finite and {match[4]} {match[5]}."
    match = re.fullmatch(r"(.+) 必須為有限數值。", message)
    if match:
        return f"{INPUT_NAMES.get(match[1], 'The input')} must be a finite number."
    match = re.fullmatch(r".+（(.+)）必須為有限數值。", message)
    if match:
        return f"Enter a finite number for {match[1]}."
    match = re.fullmatch(r"(.+) 必須大於 0。", message)
    if match:
        return f"{INPUT_NAMES.get(match[1], 'The input')} must be greater than zero."
    match = re.fullmatch(r"以目前車速取樣 ISO 路面時，輸出步長須小於 (.+) s。", message)
    if match:
        return f"At this vehicle speed, the ISO road requires an output time step below {match[1]} s."
    match = re.fullmatch(r"(.+) 必須為 JSON 物件。", message)
    if match:
        return f"{match[1]} must be a JSON object."
    match = re.fullmatch(r"結果訊號 (.+) 的長度、數值或單位錯誤。", message)
    if match:
        return f"Signal {match[1]} has an invalid length, value or unit."
    return "Check the inputs or project file and try again." if re.search(r"[\u3400-\u9fff]", message) else message
