"""Full vehicle result dashboard with retained Tk Canvas playback items."""
import tkinter as tk
from tkinter import ttk

import numpy as np

from .chart_base import BG, COLORS
from .engine import CHANNEL_NAMES, CORNERS


MODAL_LABELS = ("前升沉", "後升沉", "前側傾", "後側傾", "中央", "扭曲", *CORNERS)


def axle_attitude(front, rear, pitch, yaw=None):
    values = {"Roll": front, "Pitch": pitch, "Yaw": yaw}
    if front is not None and rear is not None:
        values.update(Roll=(front + rear) / 2, Twist=front - rear)
        values.update({"Roll front": front, "Roll rear": rear})
    return {name: samples for name, samples in values.items() if samples is not None}


def dashboard_data(result):
    signals = result.signals
    if result.legacy:
        loads = [signals.get(f"raw:N[{i}]") for i in range(len(CORNERS))]
        modal = [signals.get(f"raw:F_{channel}") for channel in CHANNEL_NAMES[:6]]
        modal += [signals.get(f"raw:F_corner[{i}]") for i in range(len(CORNERS))]
        samples = [signals.get(f"raw:theta[{i}]") for i in range(4)]
        degrees = [np.rad2deg(values) if values is not None else None for values in samples]
        return loads, modal, axle_attitude(*degrees)

    loads = [signals.get(f"Normal load {corner}") for corner in CORNERS]
    modal = [signals.get(f"Channel {channel}") for channel in CHANNEL_NAMES]
    angles = axle_attitude(signals.get("Roll front"), signals.get("Roll rear"), signals.get("Pitch"))
    for name in ("Roll", "Twist"):
        if name in signals:
            angles[name] = signals[name]
    return loads, modal, angles


class FullVehicleDashboard(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=0, height=290)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda event: self.draw())
        self.result = None
        self.index = 0
        self.time_label = None
        self.bars = []
        self.readouts = []

    def set_result(self, result):
        self.result = result
        self.index = 0
        if result is not None:
            self.loads, self.modal, self.angles = dashboard_data(result)
        else:
            self.loads, self.modal, self.angles = [], [], {}
        self.draw()

    def draw(self):
        self.canvas.delete("all")
        self.bars.clear()
        self.readouts.clear()
        self.time_label = None
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        title = "全車動態儀表板"
        if self.result is not None and self.result.legacy:
            title += " · 舊資料"
        self.canvas.create_text(12, 12, text=title, anchor="w", fill="#e2e8f0", font=("Microsoft JhengHei UI", 10, "bold"))
        if self.result is None or width < 420 or height < 240:
            message = "計算後顯示輪荷、模態力與姿態" if self.result is None else "拖曳分隔線放大儀表板"
            self.canvas.create_text(width / 2, height / 2, text=message, fill="#94a3b8")
            return
        self.time_label = self.canvas.create_text(width - 12, 12, anchor="e", fill="#e2e8f0", font=("Segoe UI", 9))
        gap = 6
        left, right, top = 8, width - 8, 24
        readout_height = 68
        row_height = (height - top - 3 * gap - readout_height) / 2
        # Give all ten force channels the pane's full width.
        self.draw_bars((left, top, right, top + row_height), "四輪法向輪荷 (N)", CORNERS, self.loads)
        top += row_height + gap
        self.draw_bars((left, top, right, top + row_height), "模態與直推力 (N)", MODAL_LABELS, self.modal)
        top += row_height + gap
        self.draw_readouts((left, top, right, height - gap))
        self.update_frame(self.index)

    def panel(self, box, title):
        left, top, right, bottom = box
        self.canvas.create_rectangle(*box, fill="#162237", outline="#293750")
        self.canvas.create_text(left + 7, top + 9, text=title, anchor="w", fill="#e2e8f0", font=("Microsoft JhengHei UI", 8, "bold"))

    def draw_bars(self, box, title, labels, samples):
        self.panel(box, title)
        left, top, right, bottom = box
        present = [values for values in samples if values is not None]
        if not present:
            self.canvas.create_text((left + right) / 2, (top + bottom) / 2 + 6, text="無資料", fill="#94a3b8")
            return
        low = min(0., *(float(np.min(values)) for values in present))
        high = max(0., *(float(np.max(values)) for values in present))
        if low < 0:
            high = max(-low, high)
            low = -high
        high = max(high, 1.) * 1.1
        scale = max(1., bottom - top - 46) / (high - low)
        zero = top + 31 + high * scale
        self.canvas.create_line(left + 5, zero, right - 5, zero, fill="#64748b")
        slot = (right - left - 12) / len(labels)
        for i, (label, values) in enumerate(zip(labels, samples)):
            x = left + 6 + (i + .5) * slot
            color = COLORS[i % len(COLORS)]
            self.canvas.create_text(x, bottom - 7, text=label, fill="#cbd5e1", font=("Microsoft JhengHei UI", 7))
            number = self.canvas.create_text(x, top + 24, text="—", fill=color, font=("Segoe UI", 8))
            if values is not None:
                bar = self.canvas.create_rectangle(0, 0, 0, 0, fill=color, outline="", tags=(f"bar:{label}",))
                half_width = min(18., slot * .32)
                self.bars.append((bar, number, values, x - half_width, x + half_width, zero, scale))

    def draw_readouts(self, box):
        self.panel(box, "目前姿態 (°)")
        left, top, right, bottom = box
        order = (("Roll", "側傾"), ("Pitch", "俯仰"), ("Twist", "扭轉"),
                 ("Roll front", "前側傾"), ("Roll rear", "後側傾"), ("Yaw", "偏航"))
        available = [(label, self.angles[name]) for name, label in order if name in self.angles]
        for index, (label, samples) in enumerate(available):
            row, column = divmod(index, 3)
            x = left + 7 + column * (right - left) / 3
            item = self.canvas.create_text(x, top + 28 + row * 20, anchor="w", fill="#cbd5e1",
                                           font=("Microsoft JhengHei UI", 9))
            self.readouts.append((item, label, samples))

    def update_frame(self, index):
        if self.result is None:
            return
        self.index = max(0, min(int(index), len(self.result.time) - 1))
        if self.time_label is None:
            return
        time = self.result.time[self.index]
        self.canvas.itemconfigure(self.time_label, text=f"{time:.3f} s")
        for bar, number, samples, left, right, zero, scale in self.bars:
            value = samples[self.index]
            end = zero - value * scale
            self.canvas.coords(bar, left, min(zero, end), right, max(zero, end))
            self.canvas.itemconfigure(number, text=f"{value:.0f}")
        for item, label, samples in self.readouts:
            self.canvas.itemconfigure(item, text=f"{label} {samples[self.index]:.3f}")
