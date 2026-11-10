"""Local Tk widgets for SuspensionDynamicsLab."""
import tkinter as tk
from tkinter import ttk


class ScrollFrame(ttk.Frame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        canvas = tk.Canvas(self, highlightthickness=0, bg="#f1f5f9", width=300)
        scrollbar = ttk.Scrollbar(self, orient="vertical", command=canvas.yview)
        self.content = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=self.content, anchor="nw")
        self.content.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        self.canvas = canvas
        self.winfo_toplevel().bind("<MouseWheel>", self._wheel, add="+")

    def _wheel(self,event):
        try:
            widget=self.winfo_containing(event.x_root,event.y_root)
            if widget is not None and str(widget).startswith(str(self)+"."):
                self.canvas.yview_scroll(-int(event.delta/120),"units")
        except tk.TclError:
            pass
