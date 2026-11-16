"""Small desktop viewer for locally generated batch_results (no simulation runs)."""

import argparse
import json
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from PIL import Image, ImageTk

from data_visualization.Simulation_Logger import SimulationLogger


HERE = Path(__file__).resolve().parent


def read_cases(root):
    """Prefer the runner's status records; also support folders without a manifest."""
    manifest = root / "manifest.json"
    if manifest.exists():
        return json.loads(manifest.read_text(encoding="utf-8-sig"))["cases"]
    return [
        dict(family=p.parts[0], model=p.parts[1], scenario=p.parts[2],
             status="complete" if (root / p.parent / "response.gif").exists() else "data_only",
             pkl=p.as_posix(), gif=(p.parent / "response.gif").as_posix(), error="")
        for p in sorted(path.relative_to(root) for path in root.glob("*/*/*/result.pkl"))
    ]


def read_result(root, case):
    path = case.get("pkl")
    if not path:
        raise ValueError(case.get("error") or "This case has no saved PKL data.")
    logger = SimulationLogger.load(root / path)
    data = logger.result()
    time = np.asarray(data.get("t"), dtype=float)
    if time.ndim != 1 or not len(time) or not np.isfinite(time).all():
        raise ValueError("Result needs a nonempty, finite, one-dimensional t channel.")
    channels = [key for key, value in data.items()
                if key != "t" and not key.endswith("_t")
                and value.shape == time.shape and np.issubdtype(value.dtype, np.number)]
    if not channels:
        raise ValueError("No numeric signal channels found.")
    return data, logger.metadata, channels


def signal_unit(channel):
    name = channel.split("_")[-1]
    if channel.startswith("alpha_"):
        return "rad/s²"
    if channel in {"roll", "pitch"}:
        return "rad"
    if channel in {"roll_rate", "pitch_rate"}:
        return "rad/s"
    if channel in {"Mx", "My", "F_roll"}:
        return "N·m"
    if channel.startswith(("F", "dF_")) or name in {"Fs", "Ft"}:
        return "N"
    if name in {"zs", "zu", "zr"} or channel in {"heave", "d_frame"}:
        return "m"
    if name in {"vzs", "vzu", "vzr"} or channel in {"heave_rate", "v_frame"}:
        return "m/s"
    if name in {"azs", "azu"} or channel in {"az_body", "ax", "ay", "a_frame"}:
        return "m/s²"
    return "raw"


class BatchViewer:
    def __init__(self, window, root):
        self.window, self.root = window, root
        self.cases, self.data, self.metadata = [], {}, {}
        self.gif, self.timer, self.photo = None, None, None
        window.title("Batch Results Viewer")
        window.geometry("1100x800")
        window.minsize(800, 600)
        window.protocol("WM_DELETE_WINDOW", self.close)

        top = ttk.Frame(window, padding=10)
        top.pack(fill="x")
        self.folder = ttk.Label(top)
        self.folder.pack(side="left", fill="x", expand=True)
        ttk.Button(top, text="Open folder", command=self.open_folder).pack(side="left", padx=5)
        ttk.Button(top, text="Refresh", command=self.refresh).pack(side="left")
        filters = ttk.Frame(window, padding=(10, 0, 10, 8))
        filters.pack(fill="x")
        self.selectors = {}
        for name, callback in (("family", self.family_changed), ("scenario", self.scenario_changed),
                               ("model", self.load_case)):
            ttk.Label(filters, text=name.title()).pack(side="left", padx=(0, 5))
            box = ttk.Combobox(filters, state="readonly", width=22)
            box.pack(side="left", padx=(0, 15))
            box.bind("<<ComboboxSelected>>", callback)
            self.selectors[name] = box
        self.status = ttk.Label(window, padding=(10, 0, 10, 8), wraplength=1000)
        self.status.pack(fill="x")
        self.tabs = ttk.Notebook(window)
        self.tabs.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        plot_tab, gif_tab, config_tab = (ttk.Frame(self.tabs) for _ in range(3))
        for frame, title in ((plot_tab, "Signals"), (gif_tab, "Animation"), (config_tab, "Metadata")):
            self.tabs.add(frame, text=title)
        controls = ttk.Frame(plot_tab, padding=5)
        controls.pack(fill="x")
        ttk.Label(controls, text="Signal").pack(side="left", padx=5)
        self.signal = ttk.Combobox(controls, state="readonly", width=28)
        self.signal.pack(side="left")
        self.signal.bind("<<ComboboxSelected>>", self.plot)
        self.compare = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls, text="Compare models (same family / scenario)",
                        variable=self.compare, command=self.plot).pack(side="left", padx=15)
        self.figure = Figure(figsize=(9, 5), dpi=100, layout="constrained")
        self.axis = self.figure.add_subplot()
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_tab)
        NavigationToolbar2Tk(self.canvas, plot_tab).update()
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.plot_info = ttk.Label(plot_tab, padding=5, wraplength=1000)
        self.plot_info.pack(fill="x")
        self.animation = ttk.Label(gif_tab, text="No GIF available", anchor="center")
        self.animation.pack(fill="both", expand=True)
        self.config = tk.Text(config_tab, wrap="word", padx=12, pady=12)
        scrollbar = ttk.Scrollbar(config_tab, command=self.config.yview)
        scrollbar.pack(side="right", fill="y")
        self.config.configure(yscrollcommand=scrollbar.set)
        self.config.pack(fill="both", expand=True)
        self.tabs.bind("<<NotebookTabChanged>>", self.tab_changed)
        self.refresh()

    def set_options(self, name, values):
        box = self.selectors[name]
        previous = box.get()
        box["values"] = sorted(set(values))
        box.set(previous if previous in box["values"] else (box["values"][0] if values else ""))

    def open_folder(self):
        folder = filedialog.askdirectory(initialdir=self.root, title="Select batch_results folder")
        if folder:
            self.root = Path(folder)
            self.refresh()

    def refresh(self):
        self.folder.configure(text=str(self.root))
        try:
            self.cases = read_cases(self.root)
        except (OSError, ValueError, KeyError) as exc:
            self.cases = []
            messagebox.showerror("Cannot read batch results", str(exc))
        self.set_options("family", [c["family"] for c in self.cases])
        self.family_changed()

    def family_changed(self, event=None):
        family = self.selectors["family"].get()
        self.set_options("scenario", [c["scenario"] for c in self.cases if c["family"] == family])
        self.scenario_changed()

    def matching_cases(self):
        return [c for c in self.cases if c["family"] == self.selectors["family"].get()
                and c["scenario"] == self.selectors["scenario"].get()]

    def scenario_changed(self, event=None):
        self.set_options("model", [c["model"] for c in self.matching_cases()])
        self.load_case()

    def stop_animation(self):
        if self.timer is not None:
            self.window.after_cancel(self.timer)
            self.timer = None
        if self.gif is not None:
            self.gif.close()
            self.gif = None
        self.photo = None
        self.animation.configure(image="", text="No GIF available")

    def load_case(self, event=None):
        self.stop_animation()
        self.data, self.metadata = {}, {}
        case = next((c for c in self.matching_cases()
                     if c["model"] == self.selectors["model"].get()), None)
        channels, details = [], ""
        if case:
            self.status.configure(text=f"{len(self.cases)} cases | {case['status']} | {case.get('error', '')}")
            try:
                self.data, self.metadata, channels = read_result(self.root, case)
                details = json.dumps(self.metadata, indent=2, ensure_ascii=False)
            except Exception as exc:
                details = f"Cannot load result: {exc}"
                self.status.configure(text=f"{case['status']} | {details}")
            gif = self.root / case["gif"] if case.get("gif") else None
            if gif and gif.is_file():
                try:
                    self.gif = Image.open(gif)
                except OSError as exc:
                    self.animation.configure(text=f"Cannot open GIF: {exc}")
        else:
            self.status.configure(text="No results found. Select a batch_results folder or run data_creator.py first.")
        previous = self.signal.get()
        self.signal["values"] = channels
        default = next((c for c in ("zs", "left_zs", "front_zs") if c in channels), "")
        self.signal.set(previous if previous in channels else (default or (channels[0] if channels else "")))
        self.config.configure(state="normal")
        self.config.delete("1.0", "end")
        self.config.insert("1.0", details)
        self.config.configure(state="disabled")
        self.plot()
        self.tab_changed()

    def plot(self, event=None):
        self.axis.clear()
        channel = self.signal.get()
        notes = []
        if channel and self.data:
            selected = self.selectors["model"].get()
            for case in self.matching_cases():
                if not self.compare.get() and case["model"] != selected:
                    continue
                try:
                    data = self.data if case["model"] == selected else read_result(self.root, case)[0]
                    values = data[channel]
                    self.axis.plot(data["t"], values, label=case["model"],
                                   linewidth=2 if case["model"] == selected else 1.3)
                    finite = values[np.isfinite(values)]
                    if finite.size:
                        notes.append(f"{case['model']}: min={finite.min():.4g}, max={finite.max():.4g}")
                except Exception as exc:
                    notes.append(f"{case['model']}: unavailable ({exc})")
            start = self.metadata.get("config", {}).get("input_start_s")
            if start is not None:
                self.axis.axvline(start, color="gray", linestyle="--", alpha=0.6, label="Input start")
            self.axis.legend(loc="best")
        self.axis.set(title=channel or "Select a case and signal", xlabel="Time (s)",
                      ylabel=f"{channel} ({signal_unit(channel)})" if channel else "")
        self.axis.grid(True, alpha=0.25)
        self.plot_info.configure(text=" | ".join(notes))
        self.canvas.draw_idle()

    def tab_changed(self, event=None):
        if self.timer is not None:
            self.window.after_cancel(self.timer)
            self.timer = None
        if self.tabs.index("current") == 1 and self.gif is not None:
            self.animate()

    def animate(self):
        self.timer = None
        frame = self.gif.convert("RGB")
        frame.thumbnail((max(100, self.animation.winfo_width()), max(100, self.animation.winfo_height())))
        self.photo = ImageTk.PhotoImage(frame)
        self.animation.configure(image=self.photo, text="")
        delay = max(20, self.gif.info.get("duration", 100))
        try:
            self.gif.seek(self.gif.tell() + 1)
        except EOFError:
            self.gif.seek(0)
        self.timer = self.window.after(delay, self.animate)

    def close(self):
        self.stop_animation()
        self.window.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=HERE / "batch_results")
    args = parser.parse_args()
    window = tk.Tk()
    BatchViewer(window, args.results.resolve())
    window.mainloop()


if __name__ == "__main__":
    main()
