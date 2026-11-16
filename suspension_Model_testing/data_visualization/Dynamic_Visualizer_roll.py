import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import numpy as np
from pathlib import Path

class RollCarVisualizer:

    def __init__(
        self,
        result,
        track_width=1.25,
        save_filename="roll_suspension.gif"
    ):

        self.track_width = track_width
        self.save_filename = save_filename
        self.body_offset = 0.2


        self.fig = plt.figure(figsize=(14, 8))

        gs = self.fig.add_gridspec(
            4,
            2,
            width_ratios=[1.2, 1],
            hspace=0.4
        )

        self.ax = self.fig.add_subplot(gs[:3, 0])

        self.ax_heave = self.fig.add_subplot(gs[0, 1])
        self.ax_roll = self.fig.add_subplot(gs[1, 1], sharex=self.ax_heave)
        self.ax_Rollrate = self.fig.add_subplot(gs[2, 1], sharex=self.ax_heave)
        self.ax_Heaverate = self.fig.add_subplot(gs[3, 1], sharex=self.ax_heave)
        self.ax_force = self.fig.add_subplot(gs[3:, 0], sharex=self.ax_heave)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-track_width,track_width)
        self.ax.set_ylim(-0.3,1)


        # 訊號計算

        self.time = result["t"]

        self.zsl = result["left_zs"]
        self.zul = result["left_zu"]

        self.vzsl = result["left_vzs"]

        self.zsr = result["right_zs"]
        self.zur = result["right_zu"]

        self.vzsr = result["right_vzs"]

        self.left_Ft = result["left_Ft"]
        self.right_Ft = result["right_Ft"]

        self.F_heave = result["F_heave"]
        self.F_roll = result["F_roll"]

        self.heave = result["heave"]

        self.roll = np.rad2deg(result["roll"])
        self.heave_rate =  result["heave_rate"]

        self.roll_rate = np.rad2deg(result["roll_rate"])

        # =====================
        # 圖片建立
        # =====================
        self._create_signal_plot(
            self.ax_heave,
            "Heave",
            [
                (self.heave, "Heave")
            ],
            "m"
        )

        self._create_signal_plot(
            self.ax_roll,
            "Roll",
            [
                (self.roll, "Roll")
            ],
            "deg"
        )

        self._create_signal_plot(
            self.ax_Heaverate,
            "Normal force",
            [
                (self.left_Ft, "N left"),
                (self.right_Ft, "N right")
            ],
            ""
        )

        self._create_signal_plot(
            self.ax_Rollrate,
            "Modal Velocity Roll",
            [
                (self.roll_rate, "Roll")
            ],
            ""
        )


        self._create_signal_plot(
            self.ax_force,
            "Modal Force",
            [
                (self.F_heave, "Heave"),
                (self.F_roll, "Roll"),
                (self.left_Ft+self.right_Ft, "total tire force")
            ],
            "N"
        )

        self.vline_heave = self._create_cursor(self.ax_heave)
        self.vline_roll = self._create_cursor(self.ax_roll)
        self.vline_Rollrate = self._create_cursor(self.ax_Rollrate)
        self.vline_Heaverate = self._create_cursor(self.ax_Heaverate)
        self.vline_force = self._create_cursor(self.ax_force)



        # =====================
        # 模擬建立
        # =====================

        self.ax.axhline(
        0.2,
        color='red',
        linestyle='--')

        self.ax.axvline(
        0,
        color='gray',
        linestyle='--')
        self.body, = self.ax.plot(
            [],
            [],
            lw=5
        )



        self.sus_l, = self.ax.plot(
            [],
            [],
            lw=3
        )

        self.sus_r, = self.ax.plot(
            [],
            [],
            lw=3
        )


        # =====================
        # Unsprung mass
        # =====================

        self.wheel_l = patches.Rectangle(
            (-0.1,0),
            0.2,
            0.4,
            color = "black"
        )

        self.wheel_r = patches.Rectangle(
            (-0.1,0),
            0.2,
            0.4,
            color = "black"
        )


        self.ax.add_patch(self.wheel_l)
        self.ax.add_patch(self.wheel_r)

    def _create_signal_plot(
        self,
        ax,
        title,
        signals,
        ylabel=None
    ):

        ax.set_title(title)

        for data, label in signals:
            ax.plot(
                self.time,
                data,
                label=label
            )

        if ylabel:
            ax.set_ylabel(ylabel)

        ax.legend()
        ax.grid()


    def _create_cursor(self, ax):
        
        return ax.axvline(
            self.time[0],
            color="k",
            linestyle="--"
        )

    def update(self, i):
        t = self.time[i]

        self.vline_heave.set_xdata([t, t])
        self.vline_roll.set_xdata([t, t])
        self.vline_Rollrate.set_xdata([t, t])
        self.vline_Heaverate.set_xdata([t, t])
        self.vline_force.set_xdata([t, t])

        zsl = self.zsl[i]
        zul = self.zul[i]

        zsr = self.zsr[i]
        zur = self.zur[i]

        xl = -self.track_width / 2
        xr =  self.track_width / 2

        self.body.set_data(
            [xl, xr],
            [
                zsl + self.body_offset,
                zsr + self.body_offset
            ]
        )

        self.sus_l.set_data(
            [xl, xl],
            [zul + self.body_offset, zsl + self.body_offset]
        )

        self.sus_r.set_data(
            [xr, xr],
            [zur + self.body_offset, zsr + self.body_offset]
        )

        self.wheel_l.set_xy(
            (
                xl - 0.1,
                zul - 0.025
            )
        )

        self.wheel_r.set_xy(
            (
                xr - 0.1,
                zur - 0.025
            )
        )

        return (
            self.body,
            self.sus_l,
            self.sus_r,
            self.wheel_l,
            self.wheel_r,
            self.vline_heave,
            self.vline_roll,
            self.vline_Rollrate,
            self.vline_Heaverate,
            self.vline_force,
        )


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
            blit=False,
            repeat=True
        )

        if save:
            self.save_gif(fps=fps)

        plt.show()
