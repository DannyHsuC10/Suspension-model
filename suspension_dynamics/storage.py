"""Portable result bundles, exports, and data-only legacy NumPy pickle import."""
import csv
import hashlib
import html
import io
import json
import pickle
from pathlib import Path

import numpy as np

from .engine import Result, steady_response
from .project import default_project, parameters_for_analysis, validate, upgrade_project


def fingerprint(result):
    if result.legacy: return "legacy:" + result.info.get("source", "")
    family = result.family
    options = dict(result.project["analysis"][family]); options.pop("model", None)
    return json.dumps([family,result.project["parameters"][family],options],sort_keys=True)


RESPONSE_NOTE = (
    "安定時間自輸入開始計時，且不早於輸入完成。2% 以穩態變化量為基準，"
    "回到原平衡點時以最大偏差為基準。— 表示不適用或無可求的固定平衡點。"
)


def format_stat(value):
    if value is None:
        return "—"
    return f"{value:.6g}" if isinstance(value, float) else str(value)


def statistics(result):
    steady = steady_response(result)
    if steady is not None:
        ready, equilibrium, initial_jump = steady
        start = result.project["analysis"][result.family]["start"]
        first = int(np.searchsorted(result.time, start))
        eligible = int(np.searchsorted(result.time, ready))
    rows = []
    for name, data in result.signals.items():
        rms = float(np.sqrt(np.sum((data[1:]**2+data[:-1]**2)*np.diff(result.time)/2)/(result.time[-1]-result.time[0]))) if len(data)>1 else abs(float(data[0]))
        peak = float(np.max(np.abs(data)))
        final = settling = overshoot = None
        if steady is not None and name in equilibrium:
            final = equilibrium[name]
            settling = "未安定"
            if first < len(data):
                baseline = float(data[max(0, first - 1)]) - initial_jump.get(name, 0.)
                change = final - baseline
                deviation = data[first:] - final
                roundoff = 64 * np.finfo(float).eps * max(1., peak, abs(final), abs(baseline))
                if abs(change) > roundoff:
                    overshoot = 100 * max(0., float(np.max(deviation / change)))
                    band = .02 * abs(change)
                else:
                    band = .02 * float(np.max(np.abs(deviation)))
                if eligible < len(data) and abs(deviation[-1]) <= max(band, roundoff):
                    outside = np.flatnonzero(np.abs(data[eligible:] - final) > max(band, roundoff))
                    sample = eligible + (int(outside[-1]) + 1 if outside.size else 0)
                    settling = float(result.time[sample] - start)
        rows.append((name, result.units[name], final, settling, overshoot,
                     float(np.min(data)), float(np.max(data)), rms, peak))
    return rows


def save_result(path, result):
    meta = dict(family=result.family,model=result.model,project=result.project,units=result.units,groups=result.groups,
                info=result.info,legacy=result.legacy,names=list(result.signals),format="SuspensionDynamicsLab-result-v1")
    arrays = {f"s{i}": values for i,values in enumerate(result.signals.values())}
    with Path(path).open("wb") as stream:
        np.savez_compressed(stream,time=result.time,metadata=np.array(json.dumps(meta,ensure_ascii=False,allow_nan=False)),**arrays)


def load_result(path):
    with np.load(path,allow_pickle=False) as data:
        meta = json.loads(str(data["metadata"]))
        if meta.get("format") != "SuspensionDynamicsLab-result-v1":
            raise ValueError("結果檔格式不支援。")
        validate(upgrade_project(meta["project"]))
        signals = {name:data[f"s{i}"].copy() for i,name in enumerate(meta["names"])}
        result = Result(meta["family"],meta["model"],data["time"].copy(),signals,meta["units"],meta["groups"],meta["project"],meta["info"],meta["legacy"])
    return validate_result(result)


def validate_result(result):
    t = result.time
    if t.ndim != 1 or len(t)<2 or not np.isfinite(t).all() or not np.all(np.diff(t)>0):
        raise ValueError("結果檔必須包含嚴格遞增、有限的時間序列。")
    if not result.signals:
        raise ValueError("結果檔沒有訊號。")
    for key, values in result.signals.items():
        if values.shape != t.shape or not np.isfinite(values).all() or key not in result.units:
            raise ValueError(f"結果訊號 {key} 的長度、數值或單位錯誤。")
    return result


def write_csv(path, result):
    with Path(path).open("w",newline="",encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        names = list(result.signals)
        writer.writerow(["Time [s]", *[f"{key} [{result.units[key]}]" for key in names]])
        writer.writerows(zip(result.time,*(result.signals[key] for key in names)))


def write_report(path, result, svg=""):
    esc = html.escape
    rows = "".join("<tr>"+"".join(f"<td>{esc(format_stat(v))}</td>" for v in row)+"</tr>" for row in statistics(result))
    warnings = "".join(f"<li>{esc(w)}</li>" for w in result.info.get("warnings",[]))
    source = "舊資料匯入（使用原始結果，未重新計算）" if result.legacy else result.info["engine"]
    text = f"""<!doctype html><html lang="zh-Hant"><meta charset="utf-8"><title>懸吊模型分析報告</title>
<style>body{{font:15px 'Microsoft JhengHei',sans-serif;margin:36px;color:#172334}}table{{border-collapse:collapse;width:100%}}td,th{{border:1px solid #ccd4df;padding:6px}}pre{{white-space:pre-wrap;background:#f1f5f9;padding:16px}}svg{{max-width:100%;height:auto}}@media print{{body{{margin:12px}}}}</style>
<h1>懸吊模型分析報告</h1><p>{esc(result.project['name'])} · {esc(result.family)} / {esc(result.model)} · {esc(source)}</p>
<p>{len(result.time)} samples · {result.time[0]:g}–{result.time[-1]:g} s</p><ul>{warnings}</ul>{svg}
<h2>訊號統計</h2><p>{esc(RESPONSE_NOTE)}</p>
<table><tr><th>Signal</th><th>Unit</th><th>穩態平衡點</th><th>2% 安定時間 (s)</th><th>超越量 (%)</th><th>Min</th><th>Max</th><th>RMS</th><th>Peak |value|</th></tr>{rows}</table>
<h2>計算資訊</h2><pre>{esc(json.dumps(result.info,ensure_ascii=False,indent=2))}</pre>
<h2>輸入快照</h2><pre>{esc(json.dumps(result.project,ensure_ascii=False,indent=2))}</pre></html>"""
    Path(path).write_text(text,encoding="utf-8")


class NumpyDataUnpickler(pickle.Unpickler):
    """Legacy logger contains only dicts and NumPy numeric arrays, never models."""
    def find_class(self, module, name):
        allowed = {("numpy", "ndarray"), ("numpy", "dtype"),
                   ("numpy.core.multiarray", "_reconstruct"), ("numpy._core.multiarray", "_reconstruct"),
                   ("numpy.core.multiarray", "scalar"), ("numpy._core.multiarray", "scalar"),
                   ("numpy.core.numeric", "_frombuffer"), ("numpy._core.numeric", "_frombuffer")}
        if (module,name) not in allowed:
            raise ValueError(f"舊檔含非數值物件，無法匯入：{module}.{name}")
        return super().find_class(module,name)


def import_legacy(path):
    path = Path(path)
    content = path.read_bytes()
    package = NumpyDataUnpickler(io.BytesIO(content)).load()
    data, metadata = package["data"], package.get("metadata",{})
    config = metadata.get("config",{})
    family = config.get("family")
    if family not in ("quarter","LR","FR","full"):
        family = "full" if "theta" in data else "LR" if "left_zs" in data else "FR" if "front_zs" in data else "quarter"
    project = default_project()
    project["parameters"][family] = parameters_for_analysis(project, family)
    signals, units, groups = {}, {}, {}
    def add(name, values, unit, group):
        signals[name] = np.asarray(values,dtype=float)
        units[name] = unit
        groups.setdefault(group,[]).append(name)
    if family == "full":
        prefixes = [(c,i) for i,c in enumerate(("FL","FR","RL","RR"))]
        for c,index in prefixes:
            for key,label in (("zs","Body"),("zu","Wheel"),("zr","Road")):
                if key in data: add(f"{label} {c}",np.asarray(data[key])[:,index]*1000,"mm","位移")
    else:
        prefixes = {"quarter":[("Q","")],"LR":[("L","left_"),("R","right_")],"FR":[("F","front_"),("R","rear_")]}[family]
        for c,prefix in prefixes:
            for key,label,unit,group,scale in (("zs","Body","mm","位移",1000),("zu","Wheel","mm","位移",1000),("zr","Road","mm","位移",1000),
                                                ("azs","Body acceleration","m/s²","加速度",1),("Fs","Fs (original)","N","原始力量",1),("Ft","Ft (original)","N","原始力量",1)):
                if prefix+key in data: add(f"{label} {c}",np.asarray(data[prefix+key])*scale,unit,group)
    for key in ("roll","pitch"):
        if key in data: add(key+" (original)",np.rad2deg(data[key]),"deg","原始姿態")
    # Keep every numeric raw channel available, without guessing its units.
    for key, values in data.items():
        arr = np.asarray(values)
        if key == "t" or arr.dtype.kind not in "fiu": continue
        if arr.ndim == 1: add("raw:"+key,arr,"原始單位","原始訊號")
        elif arr.ndim == 2:
            for i in range(arr.shape[1]): add(f"raw:{key}[{i}]",arr[:,i],"原始單位","原始訊號")
    info = {"source":str(path.resolve()),"sha256":hashlib.sha256(content).hexdigest(),"original_metadata":metadata,
            "warnings":["舊資料使用原模型與原始符號；全車重算與舊核心可能不同。","舊檔參數保存在計算資訊；此處預設專案不是舊資料的重算設定。"],
            "model_label":config.get("model","legacy"),"dof":"舊資料","samples":len(data["t"])}
    # Metadata written by the supplied logger contains ordinary JSON values.
    info = json.loads(json.dumps(info,ensure_ascii=False,default=lambda x: x.tolist() if isinstance(x,np.ndarray) else x.item() if isinstance(x,np.generic) else str(x)))
    return validate_result(Result(family,config.get("model","legacy"),np.asarray(data["t"],dtype=float),signals,units,groups,project,info,True))
