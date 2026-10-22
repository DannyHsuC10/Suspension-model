import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import numpy as np
from pathlib import Path

class PitchCarVisualizer:

    def __init__(
        self,
        result,
        wheelbase=1.25,
        save_filename="pitch_suspension.gif"
    ):

        self.wheelbase = wheelbase
        self.save_filename = save_filename
        self.body_offset = 0.0


        self.fig = plt.figure(figsize=(14, 8))

        gs = self.fig.add_gridspec(
            4,
            2,
            width_ratios=[1.2, 1],
            hspace=0.4
        )

        self.ax = self.fig.add_subplot(gs[:3, 0])

        self.ax_heave = self.fig.add_subplot(gs[0, 1])
        self.ax_pitch = self.fig.add_subplot(gs[1, 1], sharex=self.ax_heave)
        self.ax_pitchrate = self.fig.add_subplot(gs[2, 1], sharex=self.ax_heave)
        self.ax_Heaverate = self.fig.add_subplot(gs[3, 1], sharex=self.ax_heave)
        self.ax_force = self.fig.add_subplot(gs[3:, 0], sharex=self.ax_heave)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-wheelbase,wheelbase)
        self.ax.set_ylim(-0.7,0.7)


        # 訊號計算

        self.time = result["t"]

        self.zsf = result["front_zs"]
        self.zuf = result["front_zu"]

        self.vzsf = result["front_vzs"]


        self.zsr = result["rear_zs"]
        self.zur = result["rear_zu"]

        self.vzsr = result["rear_vzs"]

        self.front_Ft = result["front_Ft"]
        self.rear_Ft = result["rear_Ft"]

        self.heave = result["heave"]

        self.pitch = np.rad2deg(result["pitch"])
        self.heave_rate = result["heave_rate"]

        self.pitch_rate = np.rad2deg(result["pitch_rate"])

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
            self.ax_pitch,
            "pitch",
            [
                (self.pitch, "pitch")
            ],
            "deg"
        )

        self._create_signal_plot(
            self.ax_Heaverate,
            "Modal Velocity Heave",
            [
                (self.heave_rate, "Heave")
            ],
            ""
        )

        self._create_signal_plot(
            self.ax_pitchrate,
            "Modal Velocity pitch",
            [
                (self.pitch_rate, "pitch")
            ],
            ""
        )


        self._create_signal_plot(
            self.ax_force,
            "Modal Force",
            [
                (self.front_Ft, "front_Ft"),
                (self.rear_Ft, "rear_Ft"),
                (self.front_Ft+self.rear_Ft, "total")
            ],
            "N"
        )

        self.vline_heave = self._create_cursor(self.ax_heave)
        self.vline_pitch = self._create_cursor(self.ax_pitch)
        self.vline_pitchrate = self._create_cursor(self.ax_pitchrate)
        self.vline_Heaverate = self._create_cursor(self.ax_Heaverate)
        self.vline_force = self._create_cursor(self.ax_force)



        # =====================
        # 模擬建立
        # =====================

        self.ax.axhline(
        self.body_offset,
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


        self.wheel_f = patches.Circle(
            (wheelbase/2,0),
            0.2,
            color = "black"
        )

        self.wheel_r = patches.Circle(
            (-wheelbase/2,0),
            0.2,
            color = "black"
        )


        self.ax.add_patch(self.wheel_f)
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
        self.vline_pitch.set_xdata([t, t])
        self.vline_pitchrate.set_xdata([t, t])
        self.vline_Heaverate.set_xdata([t, t])
        self.vline_force.set_xdata([t, t])

        zsf = self.zsf[i]
        zuf = self.zuf[i]

        zsr = self.zsr[i]
        zur = self.zur[i]

        xf = self.wheelbase / 2
        xr =  -self.wheelbase / 2

        self.body.set_data(
            [xf, xr],
            [
                zsf + self.body_offset,
                zsr + self.body_offset
            ]
        )

        self.sus_l.set_data(
            [xf, xf],
            [zuf + self.body_offset, zsf + self.body_offset]
        )

        self.sus_r.set_data(
            [xr, xr],
            [zur + self.body_offset, zsr + self.body_offset]
        )

        self.wheel_f.set_center ((xf,zuf))

        self.wheel_r.set_center ((xr,zur))

        return (
            self.body,
            self.sus_l,
            self.sus_r,
            self.wheel_f,
            self.wheel_r,
            self.vline_heave,
            self.vline_pitch,
            self.vline_pitchrate,
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
