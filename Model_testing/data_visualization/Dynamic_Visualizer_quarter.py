import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

class Visualizer:
    
    def __init__(self, result, save_filename="quarter_suspension.gif"):
        self.save_filename = save_filename

        # =====================================================
        # Data
        # =====================================================

        self.time = np.asarray(result["t"])

        self.zr = np.asarray(result["zr"])

        self.zs = np.asarray(result["zs"])
        self.zu = np.asarray(result["zu"])

        self.vzs = np.asarray(result["vzs"])
        self.vzu = np.asarray(result["vzu"])

        self.azs = np.asarray(result["azs"])
        self.azu = np.asarray(result["azu"])

        self.Fs = np.asarray(result["Fs"])
        self.Ft = np.asarray(result["Ft"])


        self.window = 2.0

        # 動畫偏移
        self.ms_offset = 0.2
        self.zr_offset = 0.0595

        self.fig = plt.figure(figsize=(14,8))


        gs = self.fig.add_gridspec(
            4,
            2,
            width_ratios=[1.2,1],
            hspace=0.4
        )


        # 左動畫
        self.ax_state = self.fig.add_subplot(gs[:,0])


        # 右邊四張圖
        self.ax_pos = self.fig.add_subplot(gs[0,1])
        self.ax_vel = self.fig.add_subplot(gs[1,1],sharex=self.ax_pos)
        self.ax_acc = self.fig.add_subplot(gs[2,1],sharex=self.ax_pos)
        self.ax_force = self.fig.add_subplot(gs[3,1],sharex=self.ax_pos)

        self._create_signal_plot(
            self.ax_pos,
            "Displacement",
            [
                (self.zr,"Road"),
                (self.zu,"Unsprung"),
                (self.zs,"Sprung")
            ],
            "m")

        self._create_signal_plot(
            self.ax_vel,
            "Velocity",
            [
                (self.vzu,"vzu"),
                (self.vzs,"vzs")
            ],
            "m/s")

        self._create_signal_plot(
            self.ax_acc,
            "Acceleration",
            [
                (self.azu,"azu"),
                (self.azs,"azs")
            ],
            "m/s²")

        self._create_signal_plot(
            self.ax_force,
            "Force",
            [
                (self.Ft,"Tire Force"),
                (self.Fs,"Suspension Force")
            ],
            "N")

        self.vline_pos = self._create_cursor(self.ax_pos)
        self.vline_vel = self._create_cursor(self.ax_vel)
        self.vline_acc = self._create_cursor(self.ax_acc)
        self.vline_force = self._create_cursor(self.ax_force)

        # =====================================================
        # Left Animation
        # =====================================================

        ymin = min(
            self.zu.min() + self.zr_offset,
            self.zr.min() + self.zr_offset,
            self.zs.min() + self.ms_offset)

        ymax = max(
            self.zr.max() + self.zr_offset,
            self.zu.max() + self.zr_offset,
            self.zs.max() + self.ms_offset)

        self.ax_state.set_xlim(-self.window, self.window)
        self.ax_state.set_ylim(ymin-0.3, ymax+0.3)

        self.ax_state.set_title("Suspension Animation")
        self.ax_state.set_xlabel("Relative Position")
        self.ax_state.set_ylabel("Height")

        self.ax_state.axvline(
            0,
            color='gray',
            linestyle='--')

        # ---------------- Road ----------------

        self.wave_artist, = self.ax_state.plot(
            [],
            [],
            color='black',
            lw=2,
            label="Road")

        # ---------------- Wheel ----------------

        self.wheel_artist, = self.ax_state.plot(
            [],
            [],
            marker='o',
            color='black',
            markersize=70,
            label='Wheel'
        )

        # ---------------- Unsprung ----------------

        self.mu_artist, = self.ax_state.plot(
            [],
            [],
            marker='o',
            color='gray',
            markersize=40,
            label='mu'
        )

        # ---------------- Sprung ----------------

        self.ms_artist, = self.ax_state.plot(
            [],
            [],
            marker='s',
            color='blue',
            markersize=70,
            label='ms'
        )

    def _create_signal_plot(
        self,
        ax,
        title,
        signals,
        ylabel=None
    ):

        ax.set_title(title)

        for data, label in signals:
            line, = ax.plot(
                self.time,
                data,
                label=label
            )
            self._annotate_extrema(ax, data, label, line.get_color())

        if ylabel:
            ax.set_ylabel(ylabel)

        ax.legend()
        ax.grid()

    def _annotate_extrema(self, ax, data, label, color):
        data = np.asarray(data)
        mask = np.isfinite(data)

        if not np.any(mask):
            return

        valid_index = np.where(mask)[0]
        valid_data = data[mask]

        max_i = valid_index[np.argmax(valid_data)]
        min_i = valid_index[np.argmin(valid_data)]

        points = [
            ("max", max_i, -20),
        ]

        if min_i != max_i:
            points.append(("min", min_i, 20))
        else:
            points[0] = ("max=min", max_i, -20)

        for name, i, y_offset in points:
            x = self.time[i]
            y = data[i]

            ax.scatter(
                x,
                y,
                color=color,
                s=25,
                zorder=5
            )

            ax.annotate(
                f"{label} {name}: {y:.3g}",
                xy=(x, y),
                xytext=(8, y_offset),
                textcoords="offset points",
                fontsize=8,
                color=color,
                arrowprops={
                    "arrowstyle": "->",
                    "color": color,
                    "lw": 0.8
                }
            )

    def _create_cursor(self, ax):
        
        return ax.axvline(
            self.time[0],
            color="k",
            linestyle="--"
        )

    def update(self, frame):
        
        t_now = self.time[frame]

        # ----------------------
        # Road wave
        # ----------------------

        x = self.time - t_now
        mask = np.abs(x) < self.window

        self.wave_artist.set_data(x[mask], self.zr[mask])


        # ----------------------
        # Animation
        # ----------------------

        self.wheel_artist.set_data([0],[self.zr[frame]+self.zr_offset])
        self.mu_artist.set_data([0],[self.zu[frame]+self.zr_offset])
        self.ms_artist.set_data([0],[self.zs[frame]+self.ms_offset])


        # ----------------------
        # Cursor
        # ----------------------

        self.vline_pos.set_xdata([t_now,t_now])
        self.vline_vel.set_xdata([t_now,t_now])
        self.vline_acc.set_xdata([t_now,t_now])
        self.vline_force.set_xdata([t_now,t_now])


        return (
            self.wave_artist,
            self.wheel_artist,
            self.mu_artist,
            self.ms_artist,
            self.vline_pos,
            self.vline_vel,
            self.vline_acc,
            self.vline_force)

    def get_animation_frames(self, fps=30):
        
        total_time = self.time[-1]-self.time[0]

        target_frames = max(
            1,
            int(total_time * fps)
        )

        step = max(
            1,
            len(self.time)//target_frames
        )

        return range(
            0,
            len(self.time),
            step
        )

    def save_gif(self, fps=30, filename=None):
        save_path = Path(filename or self.save_filename).expanduser()

        if save_path.suffix.lower() != ".gif":
            save_path = save_path.with_suffix(".gif")

        save_path = save_path.resolve()

        if save_path.parent != Path("."):
            save_path.parent.mkdir(parents=True, exist_ok=True)

        print(f"Saving animation to: {save_path}", flush=True)

        self.ani.save(
            save_path,
            writer="pillow",
            fps=fps
        )

        print(f"Saved animation: {save_path}", flush=True)

    def show(self, fps=30, save=False):
        
        frames = list(self.get_animation_frames(fps))

        self.ani = FuncAnimation(
            self.fig,
            self.update,
            frames=frames,
            interval=1000/fps,
            blit=True,
            repeat=True
        )

        if save:
            self.save_gif(fps=fps)

        plt.show()
