"""Four-family native desktop workbench; computation workers never call Tk."""
import copy
import json
from pathlib import Path
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np

from .catalog import FAMILIES, MODELS, SCENARIOS, TARGETS, NOTES, scenarios, targets
from .project import (FIELDS, FULL_CHANNEL_LABELS, LINKED_LOCAL_FIELDS, INPUT_FIELDS, ESTIMATED_DAMPING,
                      PARAMETER_SOURCES, default_project, link_full_parameters, migrate_parameters, migrate_project_parameters,
                      parameters_for_analysis, prepare_project, save_project, validate, validate_options,
                      validate_parameters)
from .project_import import fill_import_project, read_import_project
from .engine import simulate, Cancelled
from .road_profiles import ISO_ROUGHNESS
from .storage import (save_result, load_result, import_legacy, write_csv, write_report,
                      statistics, format_stat, fingerprint, RESPONSE_NOTE)
from .charts import ResponseChart, MotionView, COLORS
from .dashboard import FullVehicleDashboard
from .widgets import ScrollFrame

ROOT = Path(__file__).resolve().parent.parent
PROJECTS = ROOT / "projects"
MODEL_NAMES = {family:{label:key for key,label in models.items()} for family,models in MODELS.items()}
SCENARIO_KEYS = {value:key for key,value in SCENARIOS.items()}
TARGET_KEYS = {value:key for key,value in TARGETS.items()}


class ImportRepairDialog(tk.Toplevel):
    def __init__(self, app, project, fields):
        super().__init__(app)
        self.project = project
        self.fields = fields
        self.repaired = None
        self.title("補齊專案缺失欄位")
        self.geometry(f"820x{min(620, 195 + 48 * len(fields))}")
        self.minsize(700, 240)
        self.transient(app)
        self.bind("<Escape>", lambda event: self.destroy())
        header = ttk.Frame(self, padding=(14, 10, 14, 4))
        header.pack(fill="x")
        ttk.Button(header, text="取消", command=self.destroy).pack(side="right")
        ttk.Button(header, text="補齊並匯入", command=self.apply).pack(side="right", padx=(0, 8))
        ttk.Label(header, text=f"找到 {len(fields)} 個缺失欄位",
                  font=("Microsoft JhengHei UI", 13, "bold")).pack(side="left")
        ttk.Label(self, text="可逐欄自行輸入，留空就使用該欄的預設值。已有資料會保留。",
                  padding=(14, 0, 14, 10)).pack(anchor="w")

        scroll = ScrollFrame(self, padding=(14, 0))
        scroll.pack(fill="both", expand=True)
        self.canvas = scroll.canvas
        table = scroll.content
        table.columnconfigure(0, weight=1)
        for column, heading in enumerate(("缺失欄位／JSON 路徑", "預設值", "自行輸入（可留空）")):
            ttk.Label(table, text=heading, padding=6).grid(row=0, column=column, sticky="w")
        self.inputs = []
        for row, field in enumerate(fields, start=1):
            label = f"{field.label} ({field.unit})" if field.unit else field.label
            ttk.Label(table, text=f"{label}\n{field.location}", padding=(6, 4)).grid(row=row, column=0, sticky="w")
            ttk.Label(table, text=str(field.default), padding=6).grid(row=row, column=1, sticky="w")
            entry = ttk.Entry(table, width=24)
            entry.grid(row=row, column=2, padx=6, sticky="ew")
            entry.bind("<FocusIn>", self.show_focused_input)
            self.inputs.append(entry)

        self.error = ttk.Label(self, foreground="#9a3412", wraplength=650, padding=(14, 4))
        self.error.pack(fill="x")
        self.wait_visibility()
        self.grab_set()
        self.inputs[0].focus_set()

    def show_focused_input(self, event):
        bounds = self.canvas.bbox("all")
        if not bounds:
            return
        top = event.widget.winfo_y()
        bottom = top + event.widget.winfo_height()
        visible_top = self.canvas.canvasy(0)
        height = self.canvas.winfo_height()
        if top < visible_top:
            self.canvas.yview_moveto(top / bounds[3])
        elif bottom > visible_top + height:
            self.canvas.yview_moveto((bottom - height) / bounds[3])

    def apply(self):
        try:
            self.repaired = fill_import_project(self.project, self.fields, [entry.get() for entry in self.inputs])
        except (ValueError, ArithmeticError) as exc:
            self.error.configure(text=str(exc))
            return
        self.destroy()


class ParameterEditor(tk.Toplevel):
    def __init__(self,app):
        super().__init__(app)
        self.app=app
        self.title("車輛／彈簧阻尼設定 · Suspension Dynamics Lab")
        self.geometry("1000x790");self.minsize(880,620)
        self.transient(app);self.grab_set()
        self.entries={}
        self.damping_fields = {}
        self.auto_damping = {}
        self.error = tk.StringVar(master=self)
        self.original=copy.deepcopy(app.project)
        self.sources = {family: app.panels[family].selected_parameter_source() for family in ("quarter", "FR")}
        ttk.Label(self,text="灰底欄位由全車設定換算。MR = 彈簧行程 / 輪端行程，k、c 依 MR² 換算。",padding=12).pack(fill="x")
        row=ttk.Frame(self,padding=(12,0,12,8));row.pack(fill="x")
        ttk.Label(row,text="專案名稱").pack(side="left")
        self.name=tk.StringVar(value=app.project["name"])
        ttk.Entry(row,textvariable=self.name).pack(side="left",fill="x",expand=True,padx=10)
        self.notebook=ttk.Notebook(self);self.notebook.pack(fill="both",expand=True,padx=12)
        for family in FAMILIES:
            scroll=ScrollFrame(self.notebook)
            self.notebook.add(scroll,text=FAMILIES[family])
            frame=scroll.content
            toolbar=ttk.Frame(frame,padding=10);toolbar.pack(fill="x")
            if family in ("LR", "FR"):
                _, choices = PARAMETER_SOURCES[family]
                axle = choices[self.sources.get(family, "front")]
                detail = ("在前後模型的「參數來源」可切換車軸。" if family == "FR"
                          else "車架傳力模型的 k、c、m 可在本頁設定。")
                ttk.Label(toolbar,text=f"灰底欄位由全車換算，懸吊與輪胎採{axle}。{detail}",wraplength=640).pack(side="left",padx=10)
            elif family in ESTIMATED_DAMPING:
                enabled = self.original.get("auto_damping", {}).get(family, False)
                self.auto_damping[family] = tk.BooleanVar(master=self, value=enabled)
                ttk.Checkbutton(toolbar, text="依阻尼比 ζ 自動估算阻尼係數",
                                variable=self.auto_damping[family]).pack(side="left", padx=8)
                ttk.Label(toolbar, text="勾選後自動更新灰底欄位，取消勾選可手動輸入。",
                          wraplength=400).pack(side="left")
            else:
                ttk.Label(toolbar,text="車架扭轉 k 使用 N·m/rad。直推與輪胎各有前、後軸設定，同軸左右輪共用每輪參數。",
                          wraplength=640).pack(side="left",padx=10)
            if family == "quarter":
                _, choices = PARAMETER_SOURCES[family]
                corner = choices[self.sources[family]]
                ttk.Label(frame, text=f"簧上／簧下質量與輪胎 k、c 採全車的{corner}。可在 1/4 模型的「參數來源」切換輪位。",
                          padding=(12, 0), wraplength=780).pack(fill="x")
            grid=ttk.Frame(frame,padding=10);grid.pack(fill="x")
            fields=FIELDS[family]
            if family in ("LR", "FR"):
                fields = {key: field for key, field in fields.items() if key != "zeta"}
            if family == "LR":
                fields = {key: field for key, field in fields.items() if key not in ("k_right", "c_right")}
                fields["k_left"] = ("前軸直推 k（每輪）", *fields["k_left"][1:])
                fields["c_left"] = ("前軸直推 c（每輪）", *fields["c_left"][1:])
            basic=[k for k in fields if not (family=="full" and k.rsplit("_", 1)[0] in FULL_CHANNEL_LABELS)]
            for i,key in enumerate(basic):
                row,col=divmod(i,2)
                label,unit,*_=fields[key]
                ttk.Label(grid,text=f"{label} ({unit})",wraplength=290).grid(row=row,column=col*2,sticky="w",padx=(0,10),pady=5)
                value = app.project["parameters"][family].get(key)
                var=tk.StringVar(value="" if value is None else f"{value:.12g}")
                estimated = family in self.auto_damping and key in ESTIMATED_DAMPING[family]
                inherited = family in LINKED_LOCAL_FIELDS and key not in LINKED_LOCAL_FIELDS[family]
                entry = ttk.Entry(grid, textvariable=var, width=17,
                                  style="AutoDamping.TEntry" if estimated or inherited else "TEntry",
                                  state="readonly" if inherited else "normal")
                entry.grid(row=row,column=col*2+1,sticky="ew",padx=(0,22),pady=5)
                self.entries[family,key]=var
                if estimated:
                    self.damping_fields[family, key] = entry
            grid.columnconfigure(0,weight=1);grid.columnconfigure(2,weight=1)
            if family=="full":
                table=ttk.LabelFrame(frame,text="元件參數（元件端 k、c；MR 為彈簧／輪端）",padding=12);table.pack(fill="x",padx=10,pady=8)
                for col,text in enumerate(("通道","k (N/m)","c (N·s/m)","MR")):
                    ttk.Label(table,text=text).grid(row=0,column=col,sticky="w",padx=8,pady=5)
                for row,(n,label) in enumerate(FULL_CHANNEL_LABELS.items(),1):
                    ttk.Label(table,text=label).grid(row=row,column=0,sticky="w",padx=8,pady=4)
                    for col,typ in enumerate(("k","c","mr"),1):
                        key=f"{n}_{typ}";var=tk.StringVar(value=f"{app.project['parameters'][family][key]:.12g}")
                        ttk.Entry(table,textvariable=var,width=20).grid(row=row,column=col,sticky="ew",padx=8,pady=4)
                        self.entries[family,key]=var
                for col in range(1,4):table.columnconfigure(col,weight=1)
            ttk.Label(frame,text="數值以靜態平衡為基準。按「套用全部設定」才更新專案；既有結果保留其計算快照。",padding=12).pack(fill="x")
        self.notebook.select(list(FAMILIES).index(app.current_panel().family))
        bottom=ttk.Frame(self,padding=12);bottom.pack(fill="x")
        ttk.Label(bottom,textvariable=self.error,foreground="#b91c1c",wraplength=650).pack(side="left",fill="x",expand=True)
        ttk.Button(bottom,text="取消",command=self.destroy).pack(side="right")
        ttk.Button(bottom,text="套用全部設定",command=self.apply).pack(side="right",padx=8)
        for family, enabled in self.auto_damping.items():
            enabled.trace_add("write", lambda *args, f=family: self.update_damping_mode(f))
            for key in LINKED_LOCAL_FIELDS[family]:
                if key not in ESTIMATED_DAMPING[family]:
                    self.entries[family, key].trace_add("write", lambda *args, f=family: self.refresh_damping(f))
            self.update_damping_mode(family)
        for key in FIELDS["full"]:
            self.entries["full", key].trace_add("write", self.refresh_linked_previews)
        self.refresh_linked_previews()

    def values(self,family):
        original = self.original["parameters"][family]
        values = original.copy()
        automatic = family in self.auto_damping and self.auto_damping[family].get()
        for key in LINKED_LOCAL_FIELDS.get(family, FIELDS[family]):
            old=original[key]
            if automatic and key in ESTIMATED_DAMPING[family]:
                continue
            text=self.entries[family,key].get().strip()
            try:values[key]=old if text==f"{old:.12g}" else float(text)
            except ValueError as exc:raise ValueError(f"{FIELDS[family][key][0]} 需要數值。") from exc
        if automatic:
            validate_parameters(family, values, linked=True)
            try:
                draft = self.linked_draft()
                draft["parameters"][family] = values
                draft["auto_damping"] = {family: True}
                values["cs"] = parameters_for_analysis(draft, family)["cs"]
            except (OverflowError, ZeroDivisionError) as exc:
                raise ValueError(f"{FAMILIES[family]} 無法估算阻尼係數，請檢查輸入範圍。") from exc
            validate_parameters(family, values, linked=True)
        return values

    def linked_draft(self):
        full = self.values("full")
        validate_parameters("full", full)
        draft = copy.deepcopy(self.original)
        draft["parameters"]["full"] = full
        for family, source in self.sources.items():
            key, _ = PARAMETER_SOURCES[family]
            draft["analysis"][family][key] = source
        return draft

    def refresh_linked_previews(self, *args):
        try:
            draft = self.linked_draft()
            draft.pop("auto_damping", None)
            previews = {family: parameters_for_analysis(draft, family) for family in LINKED_LOCAL_FIELDS}
        except (ValueError, OverflowError):
            return
        for family, values in previews.items():
            for key, value in values.items():
                entry = self.entries.get((family, key))
                if entry is not None and key not in LINKED_LOCAL_FIELDS[family]:
                    entry.set(f"{value:.12g}")
        self.refresh_damping("quarter")

    def update_damping_mode(self, family):
        enabled = self.auto_damping[family].get()
        for key in ESTIMATED_DAMPING[family]:
            self.damping_fields[family, key].configure(state="readonly" if enabled else "normal")
        if enabled:
            self.refresh_damping(family)
        else:
            self.error.set("")

    def refresh_damping(self, family):
        if not self.auto_damping[family].get():
            return
        try:
            values = self.values(family)
        except ValueError as exc:
            self.error.set(str(exc))
            return
        for key in ESTIMATED_DAMPING[family]:
            self.entries[family, key].set(f"{values[key]:.12g}")
        self.error.set("")

    def apply(self):
        try:
            candidate=copy.deepcopy(self.app.project)
            candidate["name"]=self.name.get().strip()
            for family in FAMILIES:candidate["parameters"][family]=self.values(family)
            modes = {family: True for family, enabled in self.auto_damping.items() if enabled.get()}
            if modes:
                candidate["auto_damping"] = modes
            else:
                candidate.pop("auto_damping", None)
            validate(candidate)
        except ValueError as exc:self.error.set(str(exc));return
        self.app.project=candidate;self.app.mark_dirty();self.app.status.set("參數已套用。重新計算可更新結果；現有結果保留原快照。")
        for panel in self.app.panels.values():panel.mark_stale()
        self.destroy()


class ModelPanel(ttk.Frame):
    def __init__(self,master,app,family):
        super().__init__(master)
        self.app,self.family=app,family
        self.records=[];self.result=None;self.play_timer=None;self.playing=False;self.suppress=False
        self.counter=0;self.current_index=0
        outer=ttk.Panedwindow(self,orient="horizontal");outer.pack(fill="both",expand=True)
        controls=ttk.Frame(outer);outer.add(controls,weight=0)
        self.control_tabs=ttk.Notebook(controls);self.control_tabs.pack(fill="both",expand=True)
        input_scroll=ScrollFrame(self.control_tabs);self.control_tabs.add(input_scroll,text="模型與輸入")
        signal_scroll=ScrollFrame(self.control_tabs);self.control_tabs.add(signal_scroll,text="曲線選擇／比較")
        left=input_scroll.content;signal_controls=signal_scroll.content
        ttk.Label(left,text="模型與輸入",font=("Microsoft JhengHei UI",12,"bold"),padding=(10,10)).pack(anchor="w")
        self.model=tk.StringVar();self.scenario=tk.StringVar();self.target=tk.StringVar()
        self.model_combo=self.combo(left,"模型",self.model,list(MODELS[family].values()),self.model_changed)
        self.scenario_combo=self.combo(left,"分析輸入",self.scenario,[],self.input_changed)
        if family in ("quarter", "FR"):
            _, choices = PARAMETER_SOURCES[family]
            self.source_choice = tk.StringVar()
            self.source_combo = self.combo(left, "參數來源（全車）", self.source_choice,
                                           tuple(choices.values()), lambda event: self.changed())
        self.target_combo=self.combo(left,"路面作用輪位",self.target,[TARGETS[k] for k in targets(family)],lambda e:self.changed())
        self.delay=tk.BooleanVar()
        if family in ("FR","full"):
            ttk.Checkbutton(left,text="同一輪跡延遲至後軸（軸距／車速）",variable=self.delay,command=self.changed).pack(anchor="w",padx=10,pady=5)
        self.vars={};self.input_widgets={}
        for key,(label,unit) in INPUT_FIELDS.items():
            row=ttk.Frame(left,padding=(10,2));row.pack(fill="x")
            ttk.Label(row,text=f"{label} ({unit})",wraplength=180).pack(side="left",fill="x",expand=True)
            var=tk.StringVar();self.vars[key]=var
            entry=ttk.Entry(row,textvariable=var,width=11);entry.pack(side="right",padx=(6,0))
            self.input_widgets[key]=entry
            var.trace_add("write",lambda *args:self.changed())
        input_hint = "階躍不施加速度脈衝。ISO 路面依車速換算行駛距離，隨機與 ISO 路面都可用種子重現。"
        ttk.Label(left,text=input_hint + "只有啟用的輸入欄位會影響該情境。",wraplength=270,padding=10,foreground="#64748b").pack(fill="x")
        buttons=ttk.Frame(left,padding=10);buttons.pack(fill="x")
        self.run_button=ttk.Button(buttons,text="計算此模型  F5",command=self.run);self.run_button.pack(fill="x",pady=2)
        self.all_button=ttk.Button(buttons,text="計算本類全部模型",command=lambda:self.run(True));self.all_button.pack(fill="x",pady=2)
        buttons.pack_configure(before=self.scenario_combo.master)
        ttk.Label(signal_controls,text="顯示訊號",font=("Microsoft JhengHei UI",11,"bold"),padding=(10,8)).pack(anchor="w")
        self.group=tk.StringVar();self.group_combo=self.combo(signal_controls,"結果種類",self.group,[],self.fill_signals)
        self.signal_list=tk.Listbox(signal_controls,selectmode="extended",exportselection=False,height=16,font=("Segoe UI",9),highlightthickness=0)
        self.signal_list.pack(fill="x",padx=10,pady=4)
        self.signal_list.bind("<<ListboxSelect>>",lambda e:self.draw_chart())
        ttk.Label(signal_controls,text="Ctrl／Shift 可多選曲線。",padding=(10,2),foreground="#64748b").pack(anchor="w")
        self.compare=tk.BooleanVar()
        ttk.Checkbutton(signal_controls,text="疊圖比較相同參數與輸入的模型",variable=self.compare,command=self.draw_chart).pack(anchor="w",padx=10,pady=8)
        self.compare_info=tk.StringVar()
        ttk.Label(signal_controls,textvariable=self.compare_info,wraplength=270,padding=(10,0,10,12),foreground="#64748b").pack(fill="x")
        right=ttk.Frame(outer);outer.add(right,weight=1)
        history=ttk.Frame(right,padding=(8,8));history.pack(fill="x")
        ttk.Label(history,text="顯示結果").pack(side="left")
        self.history=tk.StringVar()
        self.history_combo=ttk.Combobox(history,textvariable=self.history,state="readonly",width=62)
        self.history_combo.pack(side="left",fill="x",expand=True,padx=8)
        self.history_combo.bind("<<ComboboxSelected>>",self.choose_history)
        ttk.Button(history,text="清除本類結果",command=self.clear_results).pack(side="right")
        apply_label = "套用分析輸入" if family in LINKED_LOCAL_FIELDS else "套用結果輸入"
        self.apply_result_button=ttk.Button(history,text=apply_label,command=self.apply_result_inputs,state="disabled")
        self.apply_result_button.pack(side="right",padx=4)
        self.result_status=tk.StringVar(value="設定參數並按「計算此模型」。")
        ttk.Label(right,textvariable=self.result_status,wraplength=1000,padding=(10,0,10,5),foreground="#475569").pack(fill="x")
        self.tabs=ttk.Notebook(right);self.tabs.pack(fill="both",expand=True,padx=8,pady=(0,8))
        visual = ttk.Frame(self.tabs)
        self.tabs.add(visual, text="曲線與動態儀表板" if family == "full" else "曲線與模型動畫")
        split = ttk.Panedwindow(visual, orient="horizontal")
        motion_frame = ttk.Frame(split)
        split.add(motion_frame, weight=3 if family == "full" else 4)
        signal_frame = ttk.Frame(split)
        split.add(signal_frame, weight=2 if family == "full" else 5)
        plot_actions = ttk.Frame(signal_frame, padding=5)
        plot_actions.pack(fill="x")
        ttk.Button(plot_actions, text="重設曲線視角", command=lambda: self.chart.reset()).pack(side="left")
        ttk.Label(plot_actions, text="滾輪縮放 · 拖曳平移").pack(side="left", padx=10)
        self.chart = ResponseChart(signal_frame)
        self.chart.pack(fill="both", expand=True)
        if family == "full":
            self.motion = FullVehicleDashboard(motion_frame)
        else:
            motion_tools = ttk.Frame(motion_frame, padding=4)
            motion_tools.pack(fill="x")
            self.projection = tk.StringVar(value="Front" if family == "LR" else "Side")
            ttk.Label(motion_tools, text="動畫視角").pack(side="left")
            self.projection_control = ttk.Label(motion_tools, text=f"{self.projection.get()} View")
            self.projection_control.pack(side="left", padx=5)
            self.gain = tk.StringVar(value="1")
            ttk.Label(motion_tools, text="位移放大").pack(side="left", padx=(10, 0))
            gain = ttk.Combobox(motion_tools, textvariable=self.gain, values=("1", "2", "5", "10", "20"), width=4, state="readonly")
            gain.pack(side="left", padx=5)
            gain.bind("<<ComboboxSelected>>", lambda event: self.motion_options())
            ttk.Button(motion_tools, text="重設動畫視角", command=lambda: self.motion.reset()).pack(side="right")
            self.motion = MotionView(motion_frame)
            self.motion.projection = self.projection.get()
        self.motion.pack(fill="both", expand=True)
        playback=ttk.Frame(visual,padding=5);playback.pack(fill="x",side="bottom")
        split.pack(fill="both",expand=True)
        ttk.Button(playback,text="|◀",width=4,command=lambda:self.seek(0)).pack(side="left")
        self.play_button=ttk.Button(playback,text="播放",width=7,command=self.toggle_play);self.play_button.pack(side="left",padx=3)
        ttk.Button(playback,text="◀",width=3,command=lambda:self.seek(self.current_index-1)).pack(side="left")
        ttk.Button(playback,text="▶",width=3,command=lambda:self.seek(self.current_index+1)).pack(side="left")
        self.speed=tk.StringVar(value="1")
        ttk.Combobox(playback,textvariable=self.speed,values=("0.25","0.5","1","2","4"),state="readonly",width=5).pack(side="left",padx=4)
        ttk.Label(playback,text="×").pack(side="left")
        self.time_var=tk.DoubleVar(value=0.)
        self.scale=ttk.Scale(playback,from_=0,to=1,variable=self.time_var,command=self.scrub);self.scale.pack(side="left",fill="x",expand=True,padx=8)
        self.time_label=ttk.Label(playback,text="0.000 s",width=16);self.time_label.pack(side="right")
        results=ttk.Frame(self.tabs);self.tabs.add(results,text="數值結果")
        ttk.Label(results, text=RESPONSE_NOTE, wraplength=720, padding=(8, 6)).pack(fill="x")
        table = ttk.Frame(results)
        table.pack(fill="both", expand=True)
        columns = (("unit", "Unit", 60), ("equilibrium", "穩態平衡點", 112),
                   ("settle", "2% 安定時間 (s)", 130), ("overshoot", "超越量 (%)", 105),
                   ("min", "Min", 90), ("max", "Max", 90), ("rms", "RMS", 90), ("peak", "Peak |value|", 100))
        self.summary = ttk.Treeview(table, columns=[key for key, _, _ in columns], show="tree headings")
        self.summary.heading("#0", text="Signal")
        self.summary.column("#0", width=205, minwidth=170, stretch=False)
        for key, label, width in columns:
            self.summary.heading(key, text=label)
            self.summary.column(key, width=width, anchor="e", stretch=False)
        ybar = ttk.Scrollbar(table, orient="vertical", command=self.summary.yview)
        xbar = ttk.Scrollbar(table, orient="horizontal", command=self.summary.xview)
        self.summary.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        self.summary.grid(row=0, column=0, sticky="nsew")
        ybar.grid(row=0, column=1, sticky="ns")
        xbar.grid(row=1, column=0, sticky="ew")
        table.rowconfigure(0, weight=1)
        table.columnconfigure(0, weight=1)
        info=ttk.Frame(self.tabs);self.tabs.add(info,text="模型說明／計算快照")
        self.info=tk.Text(info,wrap="word",font=("Microsoft JhengHei UI",10),padx=14,pady=14)
        bar=ttk.Scrollbar(info,orient="vertical",command=self.info.yview);bar.pack(side="right",fill="y")
        self.info.configure(yscrollcommand=bar.set);self.info.pack(fill="both",expand=True)
        self.info.insert("1.0",NOTES);self.info.configure(state="disabled")
        self.load_options(app.project["analysis"][family])
        outer.sashpos(0,315)
        self.layout_ready=False
        def initialize_split(event=None):
            if not self.layout_ready and split.winfo_ismapped() and split.winfo_width() > 700:
                self.layout_ready = True
                split.sashpos(0, int(split.winfo_width() * (.58 if family == "full" else .46)))
        split.bind('<Configure>',initialize_split)
        split.bind('<Map>',initialize_split)

    @staticmethod
    def combo(parent,label,var,values,command):
        frame=ttk.Frame(parent,padding=(10,3));frame.pack(fill="x")
        ttk.Label(frame,text=label).pack(anchor="w")
        widget=ttk.Combobox(frame,textvariable=var,values=values,state="readonly",width=32);widget.pack(fill="x",pady=2)
        widget.bind("<<ComboboxSelected>>",command)
        return widget

    def load_options(self,opt):
        self.suppress=True
        self.model.set(MODELS[self.family][opt["model"]]);self.scenario.set(SCENARIOS[opt["scenario"]]);self.target.set(TARGETS[opt["target"]])
        self.delay.set(opt.get("rear_delay",False))
        if self.family in ("quarter", "FR"):
            key, choices = PARAMETER_SOURCES[self.family]
            self.source_choice.set(choices[opt[key]])
        for key in INPUT_FIELDS:self.vars[key].set(f"{opt[key]:.12g}")
        self.suppress=False;self.update_choices()

    def update_choices(self):
        key=MODEL_NAMES[self.family][self.model.get()]
        available=[SCENARIOS[s] for s in scenarios(self.family,key)]
        self.scenario_combo.configure(values=available)
        if self.scenario.get() not in available:self.scenario.set(available[0])
        self.input_changed(mark=False)

    def model_changed(self,event=None):
        self.update_choices();self.changed()
        model=MODEL_NAMES[self.family][self.model.get()]
        for label,result in reversed(self.records):
            if result.model==model and not result.legacy:self.history.set(label);self.show_result(result);break

    def input_changed(self,event=None,mark=True):
        scenario=SCENARIO_KEYS.get(self.scenario.get(),"bump")
        common={"duration","dt","start"}
        extra = {
            "bump": {"height", "period"},
            "step": {"height"},
            "sine": {"height", "frequency"},
            "random": {"height", "seed", "random_interval"},
            "cornering": {"ay", "ramp"},
            "accelerate": {"ax", "ramp"},
            "bump_cornering": {"height", "period", "ay", "ramp"},
            "combined": {"height", "period", "ax", "ay", "ramp", "speed"},
            **dict.fromkeys(ISO_ROUGHNESS, {"speed", "seed"}),
        }[scenario]
        if self.family in ("FR","full") and scenario not in ("cornering","accelerate"):extra=extra|{"speed"}
        for key,widget in self.input_widgets.items():widget.configure(state="normal" if key in common|extra else "disabled")
        self.target_combo.configure(state="disabled" if scenario in ("cornering","accelerate") else "readonly")
        if mark:self.changed()

    def changed(self,*args):
        if self.suppress:return
        self.app.mark_dirty();self.mark_stale()

    def mark_stale(self):
        if self.result:self.result_status.set("設定可能已變更；目前顯示已保存的結果快照。按 F5 重新計算。")

    def options(self):
        opt=copy.deepcopy(self.app.project["analysis"][self.family])
        opt.update(model=MODEL_NAMES[self.family][self.model.get()],scenario=SCENARIO_KEYS[self.scenario.get()],target=TARGET_KEYS[self.target.get()],rear_delay=self.delay.get())
        if self.family in ("quarter", "FR"):
            key, _ = PARAMETER_SOURCES[self.family]
            opt[key] = self.selected_parameter_source()
        for key,var in self.vars.items():
            old=opt[key];text=var.get().strip()
            try:opt[key]=old if text==f"{old:.12g}" else float(text)
            except ValueError as exc:raise ValueError(f"{INPUT_FIELDS[key][0]} 需要數值。") from exc
        return validate_options(self.family,opt)

    def selected_parameter_source(self):
        _, choices = PARAMETER_SOURCES[self.family]
        return next(key for key, label in choices.items() if label == self.source_choice.get())

    def run(self,all_models=False):
        self.pause();self.app.calculate(self,all_models)

    def add_result(self,result):
        self.counter+=1
        scenario="舊資料" if result.legacy else SCENARIOS[result.project["analysis"][self.family]["scenario"]]
        label=f"{self.counter:02d} · {result.info.get('model_label',result.model)} · {scenario}"
        if self.family in ("quarter", "FR"):
            key, choices = PARAMETER_SOURCES[self.family]
            if result.info.get(key) in choices:
                label += f" · {choices[result.info[key]]}"
        self.records.append((label,result));self.records=self.records[-20:]
        self.history_combo.configure(values=[x[0] for x in self.records]);self.history.set(label)
        self.show_result(result)

    def choose_history(self,event=None):
        for label,result in self.records:
            if label==self.history.get():self.show_result(result);return

    def show_result(self,result):
        self.pause();self.result=result
        self.apply_result_button.configure(state="disabled" if result.legacy else "normal")
        available=list(result.groups)
        self.group_combo.configure(values=available)
        if self.group.get() not in available:self.group.set(available[0])
        self.fill_signals()
        self.summary.delete(*self.summary.get_children())
        for name, *values in statistics(result):
            self.summary.insert("", "end", text=name, values=[format_stat(value) for value in values])
        self.info.configure(state="normal");self.info.delete("1.0","end")
        self.info.insert("1.0",NOTES+"\n\n本次計算資訊\n"+json.dumps(result.info,ensure_ascii=False,indent=2)+"\n\n本次輸入快照\n"+json.dumps(result.project,ensure_ascii=False,indent=2));self.info.configure(state="disabled")
        warning=" / ".join(result.info.get("warnings",[]))
        self.result_status.set(f"結果快照：{result.info.get('model_label',result.model)} · {len(result.time):,} samples · {result.time[-1]:g} s"+(" · "+warning if warning else ""))
        self.scale.configure(from_=float(result.time[0]),to=float(result.time[-1]));self.motion.set_result(result);self.seek(0)

    def apply_result_inputs(self):
        if self.result is None or self.result.legacy:return
        parameters = self.result.project['parameters'][self.family]
        if self.family in LINKED_LOCAL_FIELDS:
            for key in LINKED_LOCAL_FIELDS[self.family]:
                self.app.project['parameters'][self.family][key] = parameters[key]
        else:
            self.app.project['parameters'][self.family] = migrate_parameters(self.family, copy.deepcopy(parameters))
        self.app.project['analysis'][self.family]=copy.deepcopy(self.result.project['analysis'][self.family])
        link_full_parameters(self.app.project)
        modes = self.app.project.get("auto_damping", {}).copy()
        if self.family == "quarter" and self.result.project.get("auto_damping", {}).get(self.family):
            modes[self.family] = True
        else:
            modes.pop(self.family, None)
        if modes:
            self.app.project["auto_damping"] = modes
        else:
            self.app.project.pop("auto_damping", None)
        self.load_options(self.app.project['analysis'][self.family]);self.app.mark_dirty()
        if self.app.editor is not None and self.app.editor.winfo_exists():self.app.editor.destroy()
        if self.family in LINKED_LOCAL_FIELDS:
            self.app.status.set("已套用分析輸入與本類專用設定，重算時會使用目前的全車參數。")
            self.mark_stale()
        else:
            self.app.status.set("已將目前結果的參數與分析輸入套用至本類，可再修改或重算。")
        if self.family == "full":
            for family in LINKED_LOCAL_FIELDS:
                self.app.panels[family].mark_stale()

    def fill_signals(self,event=None):
        if not self.result:return
        old={self.signal_list.get(i) for i in self.signal_list.curselection()}
        names=self.result.groups.get(self.group.get(),[])
        self.signal_list.delete(0,"end")
        for name in names:self.signal_list.insert("end",name)
        chosen=[i for i,n in enumerate(names) if n in old]
        if not chosen:chosen=list(range(min(4,len(names))))
        for i in chosen:self.signal_list.selection_set(i)
        self.draw_chart()

    def draw_chart(self):
        if not self.result:return
        names=[self.signal_list.get(i) for i in self.signal_list.curselection()]
        # Raw legacy signals have unknown units; preserve the original values.
        chosen=[];seen=set()
        if self.compare.get() and not self.result.legacy:
            for _,result in reversed(self.records):
                if result.model in seen or fingerprint(result)!=fingerprint(self.result):continue
                chosen.append(result);seen.add(result.model)
            chosen.reverse()
        if not chosen:chosen=[self.result]
        series=[]
        for result in chosen:
            for name in names:
                if name not in result.signals:continue
                label=f"{name} · {result.model}" if len(chosen)>1 else name
                series.append(dict(name=label,x=result.time,y=result.signals[name]))
        self.compare_info.set(f"顯示 {len(chosen)} 個模型、{len(series)} 條曲線；比較只使用同類且參數與輸入完全相同的結果。")
        unit=self.result.units[names[0]] if names else ""
        self.chart.set(dict(title=f"{self.group.get()} · {self.result.model}",xlabel="Time (s)",ylabel=unit,series=series))
        if self.result:self.chart.cursor(float(self.result.time[self.current_index if self.current_index<len(self.result.time) else 0]))

    def motion_options(self):
        self.motion.projection=self.projection.get();self.motion.gain=float(self.gain.get());self.motion.reset()

    def seek(self,index,pause=True):
        if pause:self.pause()
        if not self.result:return
        self.current_index=max(0,min(int(index),len(self.result.time)-1))
        t=float(self.result.time[self.current_index]);self.suppress=True;self.time_var.set(t);self.suppress=False
        self.time_label.configure(text=f"{t:.3f} / {self.result.time[-1]:.2f} s")
        self.motion.update_frame(self.current_index);self.chart.cursor(t)

    def scrub(self,value):
        if self.suppress or self.result is None:return
        index=int(np.searchsorted(self.result.time,float(value)))
        self.seek(index)

    def toggle_play(self):
        if self.playing:self.pause();return
        if not self.result:return
        if self.current_index>=len(self.result.time)-1:self.seek(0)
        self.playing=True;self.play_button.configure(text="暫停");self.last_tick=time.perf_counter()
        self.play_time=float(self.result.time[self.current_index]);self.tick()

    def tick(self):
        self.play_timer=None
        if not self.playing or self.result is None:return
        now=time.perf_counter();self.play_time+=(now-self.last_tick)*float(self.speed.get());self.last_tick=now
        index=int(np.searchsorted(self.result.time,self.play_time,side="right"))-1
        self.seek(index,pause=False)
        if self.play_time>=self.result.time[-1]:self.pause();return
        self.play_timer=self.after(16,self.tick)

    def pause(self):
        self.playing=False
        if self.play_timer is not None:self.after_cancel(self.play_timer);self.play_timer=None
        self.play_button.configure(text="播放")

    def clear_results(self):
        self.pause();self.records.clear();self.result=None;self.history.set("");self.history_combo.configure(values=[])
        self.chart.set(None);self.motion.set_result(None);self.signal_list.delete(0,"end");self.summary.delete(*self.summary.get_children())
        self.result_status.set("已清除本類記憶體結果。")
        self.apply_result_button.configure(state="disabled")


class App(tk.Tk):
    def __init__(self,project=None):
        super().__init__()
        project = migrate_project_parameters(copy.deepcopy(project or default_project()))
        self.project=link_full_parameters(validate(project))
        self.project_path=None;self.dirty=False;self.editor=None;self.busy=False
        self.events=queue.Queue();self.cancel_event=threading.Event();self.poll_timer=None
        self.title("Suspension Dynamics Lab · 懸吊模型分析")
        self.geometry("1440x940");self.minsize(1160,760)
        style=ttk.Style(self);style.theme_use("clam")
        style.map("AutoDamping.TEntry", fieldbackground=[("readonly", "#e2e8f0")],
                  foreground=[("readonly", "#334155")])
        style.configure(".",font=("Microsoft JhengHei UI",10));style.configure("TNotebook.Tab",padding=(16,8))
        style.configure("TButton",padding=(7,4));style.configure("TFrame",background="#f1f5f9");style.configure("TLabel",background="#f1f5f9")
        header=ttk.Frame(self,padding=(14,10));header.pack(fill="x")
        ttk.Label(header,text="Suspension Dynamics Lab",font=("Segoe UI",18,"bold"),foreground="#0f172a").pack(side="left")
        ttk.Label(header,text="懸吊彈簧阻尼模型分析",padding=(14,0)).pack(side="left")
        toolbar=ttk.Frame(self,padding=(12,0,12,8));toolbar.pack(fill="x")
        for label, command in (
            ("匯入", self.open),
            ("儲存", self.save),
            ("另存", lambda: self.save(True)),
            ("車輛／彈簧阻尼設定", self.parameters),
        ):
            ttk.Button(toolbar, text=label, command=command).pack(side="left", padx=3)
        export=ttk.Menubutton(toolbar,text="匯出結果 ▾");export.pack(side="right",padx=3)
        menu=tk.Menu(export,tearoff=False);export.configure(menu=menu)
        for key,label in (("npz","完整結果 .npz"),("csv","全部訊號 CSV"),("svg","目前曲線 SVG"),("png","目前曲線 PNG"),("html","HTML 報告")):
            menu.add_command(label=label,command=lambda k=key:self.export(k))
        self.book=ttk.Notebook(self)
        self.panels={}
        for family,label in FAMILIES.items():
            panel=ModelPanel(self.book,self,family);self.panels[family]=panel;self.book.add(panel,text=label)
        self.book.bind("<<NotebookTabChanged>>",self.tab_changed)
        footer=ttk.Frame(self,padding=(12,6));footer.pack(fill="x",side="bottom")
        self.book.pack(fill="both",expand=True,padx=12)
        self.status=tk.StringVar(value="就緒 · 選擇模型、設定輸入，按 F5 計算。")
        ttk.Label(footer,textvariable=self.status,wraplength=1000).pack(side="left",fill="x",expand=True)
        self.progress=ttk.Progressbar(footer,maximum=1.,length=150);self.progress.pack(side="left",padx=10)
        self.stop_button=ttk.Button(footer,text="停止計算",command=self.cancel_event.set,state="disabled");self.stop_button.pack(side="right")
        self.bind("<F5>",lambda e:self.current_panel().run())
        self.bind("<Control-s>",lambda e:self.save());self.bind("<Control-o>",lambda e:self.open())
        self.protocol("WM_DELETE_WINDOW",self.close)
        self.dirty=False;self.update_title();self.poll_timer=self.after(70,self.poll)

    def current_panel(self):
        return self.panels[list(FAMILIES)[self.book.index("current")]]

    def tab_changed(self,event=None):
        for panel in self.panels.values():panel.pause()

    def mark_dirty(self):
        self.dirty=True;self.update_title()

    def update_title(self):
        self.title(f"{'* ' if self.dirty else ''}{self.project['name']} · Suspension Dynamics Lab")

    def sync_inputs(self):
        candidate=copy.deepcopy(self.project)
        for family,panel in self.panels.items():candidate["analysis"][family]=panel.options()
        self.project=validate(candidate)

    def parameters(self):
        if self.editor is not None and self.editor.winfo_exists():self.editor.lift();self.editor.focus_force();return
        self.editor=ParameterEditor(self)

    def discard_ok(self):
        return not self.dirty or messagebox.askyesno("尚未儲存","專案設定尚未儲存，確定要捨棄變更？",parent=self)

    def accept_project(self,project,path=None):
        self.project=link_full_parameters(project);self.project_path=path
        for family,panel in self.panels.items():panel.clear_results();panel.load_options(project["analysis"][family])
        if self.editor is not None and self.editor.winfo_exists():self.editor.destroy()
        self.dirty=False;self.update_title();self.status.set("專案已載入。按 F5 計算。")

    def project_dialog_dir(self):
        if self.project_path is not None:
            return self.project_path.parent.resolve()
        return PROJECTS if PROJECTS.is_dir() else ROOT

    def open(self):
        if not self.discard_ok():
            return
        path = filedialog.askopenfilename(
            parent=self, title="開啟懸吊動態專案",
            initialdir=self.project_dialog_dir(), filetypes=[("專案 JSON", "*.json")],
        )
        if path:
            self.import_project(path)

    def import_project(self, path):
        if self.busy:
            messagebox.showinfo("計算中", "請先停止計算再切換專案。", parent=self)
            return
        try:
            project, missing = read_import_project(path)
            if not missing:
                self.accept_project(prepare_project(project), Path(path))
                return
            dialog = ImportRepairDialog(self, project, missing)
            self.wait_window(dialog)
            if dialog.repaired is not None:
                self.accept_project(dialog.repaired)
                self.mark_dirty()
                self.status.set(f"已補齊 {len(missing)} 個缺失欄位並匯入。按 F5 計算，或另存修補後的專案。")
        except Exception as exc:
            messagebox.showerror("開啟失敗", str(exc), parent=self)

    def save(self, as_new=False):
        path = self.project_path
        if as_new or path is None:
            path = filedialog.asksaveasfilename(
                parent=self, title="儲存專案", initialdir=self.project_dialog_dir(),
                initialfile=path.name if path is not None else "suspension_dynamics.json",
                defaultextension=".json", filetypes=[("專案 JSON", "*.json")],
            )
        if not path:
            return
        try:
            save_project(path, self.project)
            self.project_path = Path(path)
            self.dirty = False
            self.update_title()
            self.status.set(f"專案已儲存：{self.project_path.name}")
        except Exception as exc:
            messagebox.showerror("儲存失敗", str(exc), parent=self)

    def calculate(self,panel,all_models=False):
        if self.busy:return
        try:self.sync_inputs()
        except ValueError as exc:messagebox.showerror("輸入錯誤",str(exc),parent=self);return
        family=panel.family;snapshot=copy.deepcopy(self.project)
        models=list(MODELS[family]) if all_models else [snapshot["analysis"][family]["model"]]
        scenario=snapshot["analysis"][family]["scenario"]
        models=[m for m in models if scenario in scenarios(family,m)]
        self.busy=True;self.cancel_event.clear();self.progress["value"]=0
        self.stop_button.configure(state="normal")
        for p in self.panels.values():p.run_button.configure(state="disabled");p.all_button.configure(state="disabled")
        self.status.set(f"正在計算 {FAMILIES[family]} · {len(models)} 個模型…")
        def worker():
            try:
                for i,model in enumerate(models):
                    candidate=copy.deepcopy(snapshot);candidate["analysis"][family]["model"]=model
                    result=simulate(candidate,family,cancel=self.cancel_event.is_set,progress=lambda value,i=i:self.events.put(("progress",(i+value)/len(models))))
                    self.events.put(("result",panel,result))
                self.events.put(("done",f"完成 {len(models)} 個模型。可切換結果、疊圖比較或播放。"))
            except Cancelled:self.events.put(("done","計算已停止，已完成的模型結果仍保留。"))
            except Exception as exc:self.events.put(("error",str(exc)))
        threading.Thread(target=worker,daemon=True,name="suspension-dynamics").start()

    def poll(self):
        try:
            while True:
                event=self.events.get_nowait()
                if event[0]=="progress":self.progress["value"]=event[1]
                elif event[0]=="result":event[1].add_result(event[2])
                else:
                    self.busy=False;self.stop_button.configure(state="disabled")
                    for p in self.panels.values():p.run_button.configure(state="normal");p.all_button.configure(state="normal")
                    self.status.set(event[1])
                    if event[0]=="error":messagebox.showerror("計算失敗",event[1],parent=self)
        except queue.Empty:pass
        self.poll_timer=self.after(70,self.poll)

    def open_result(self,path=None):
        if path is None:
            initial=ROOT.parent/"new"/"Model_testing"/"batch_results"
            path=filedialog.askopenfilename(parent=self,title="開啟結果（NPZ 或原始數值 PKL）",initialdir=initial if initial.exists() else ROOT,filetypes=[("結果檔","*.npz *.pkl"),("新結果","*.npz"),("舊結果","*.pkl")])
        if not path:return
        try:
            result=import_legacy(path) if Path(path).suffix.lower()==".pkl" else load_result(path)
            panel=self.panels[result.family];self.book.select(panel);panel.add_result(result)
            self.status.set(f"已開啟結果：{Path(path).name}。當前專案設定保持原值，結果快照可在說明頁查看。")
        except Exception as exc:messagebox.showerror("結果讀取失敗",str(exc),parent=self)

    def batch_browser(self,path=None):
        default=ROOT.parent/"new"/"Model_testing"/"batch_results"/"manifest.json"
        if path is None:path=default if default.exists() else filedialog.askopenfilename(parent=self,title="選擇舊批次 manifest.json",filetypes=[("JSON","*.json")])
        if not path:return
        try:
            root=Path(path).resolve().parent
            cases=json.loads(Path(path).read_text(encoding="utf-8"))["cases"]
        except Exception as exc:messagebox.showerror("讀取失敗",str(exc),parent=self);return
        window=tk.Toplevel(self);window.title("舊批次結果 · 不需重新分析");window.geometry("760x560");window.transient(self)
        ttk.Label(window,text=str(root),padding=10,wraplength=720).pack(fill="x")
        tree=ttk.Treeview(window,columns=("family","model","scenario"),show="headings")
        for key in ("family","model","scenario"):tree.heading(key,text=key);tree.column(key,width=200)
        tree.pack(fill="both",expand=True,padx=10)
        paths={}
        for case in cases:
            if case.get("status")!="complete" or not case.get("pkl"):continue
            file=(root/case["pkl"]).resolve()
            if not file.is_relative_to(root) or not file.is_file():continue
            identity=tree.insert("","end",values=(case["family"],case["model"],case["scenario"]));paths[identity]=file
        def choose(event=None):
            selection=tree.selection()
            if selection:self.open_result(paths[selection[0]])
        tree.bind("<Double-1>",choose)
        row=ttk.Frame(window,padding=10);row.pack(fill="x")
        ttk.Button(row,text="開啟選取結果",command=choose).pack(side="right")
        ttk.Button(row,text="切換批次資料夾",command=lambda:(window.destroy(),self.choose_batch_manifest())).pack(side="left")

    def choose_batch_manifest(self):
        path=filedialog.askopenfilename(parent=self,title="選擇舊批次 manifest.json",filetypes=[("批次索引 JSON","*.json")])
        if path:self.batch_browser(path)

    def export(self,kind):
        panel=self.current_panel();result=panel.result
        if result is None:messagebox.showinfo("尚無結果","請先計算或開啟結果。",parent=self);return
        path=filedialog.asksaveasfilename(parent=self,title="匯出目前顯示結果",initialfile=f"{result.family}_{result.model}.{kind}",defaultextension="."+kind,filetypes=[(kind.upper(),"*."+kind)])
        if not path:return
        try:
            if kind=="npz":save_result(path,result)
            elif kind=="csv":write_csv(path,result)
            elif kind=="svg":Path(path).write_text(panel.chart.svg(),encoding="utf-8")
            elif kind=="png":panel.chart.png(path)
            elif kind=="html":write_report(path,result,panel.chart.svg())
            self.status.set(f"已匯出：{Path(path).name}")
        except Exception as exc:messagebox.showerror("匯出失敗",str(exc),parent=self)

    def close(self):
        if not self.discard_ok():return
        self.cancel_event.set()
        for panel in self.panels.values():panel.pause()
        if self.poll_timer is not None:self.after_cancel(self.poll_timer)
        self.destroy()


def main():
    import argparse
    parser=argparse.ArgumentParser(description="懸吊彈簧阻尼模型分析")
    parser.add_argument("--project",type=Path)
    parser.add_argument("--result",type=Path)
    args=parser.parse_args()
    app=App()

    def open_startup_files():
        if args.project:
            app.import_project(args.project)
        if args.result:
            app.open_result(args.result)

    if args.project or args.result:
        app.wait_visibility()
        app.after_idle(open_startup_files)
    app.mainloop()
