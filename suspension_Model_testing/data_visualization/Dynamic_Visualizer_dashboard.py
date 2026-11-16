import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class SuspensionDashboard:
    """
    Dashboard for full vehicle suspension simulation.

    Expected attitude definition:

        theta = [front_roll, rear_roll, pitch, yaw]
        omega = [front_roll_rate, rear_roll_rate, pitch_rate, yaw_rate]
        alpha = [front_roll_acc, rear_roll_acc, pitch_acc, yaw_acc]

    Dashboard roll definition:

        Roll  = (Front Roll + Rear Roll) / 2
        Twist = Front Roll - Rear Roll
    """

    def __init__(self, result):

        # =====================================================
        # Time
        # =====================================================

        self.time = np.asarray(result["t"])

        # =====================================================
        # Wheel normal force
        # =====================================================

        self.Fz = np.asarray(result["N"])

        if self.Fz.shape[0] == 4:
            self.Fz = self.Fz.T

        # =====================================================
        # Corner force
        # =====================================================

        self.F_corner = np.asarray(result["F_corner"])

        if self.F_corner.shape[0] == 4:
            self.F_corner = self.F_corner.T

        # =====================================================
        # Modal force
        # =====================================================

        self.modal = np.column_stack([
            result["F_heave_f"],
            result["F_heave_r"],
            result["F_roll_f"],
            result["F_roll_r"],
            result["F_roll_c"],
            result["F_warp"],
            self.F_corner,
        ])

        # =====================================================
        # Angular data
        # =====================================================

        theta = np.asarray(result["theta"])
        omega = np.asarray(result["omega"])
        alpha = np.asarray(result["alpha"])

        # -----------------------------------------------------
        # Original DOF
        #
        # [front_roll, rear_roll, pitch, yaw]
        # -----------------------------------------------------

        self.front_roll = np.rad2deg(theta[:, 0])
        self.rear_roll = np.rad2deg(theta[:, 1])

        self.front_roll_rate = np.rad2deg(omega[:, 0])
        self.rear_roll_rate = np.rad2deg(omega[:, 1])

        self.front_roll_acc = np.rad2deg(alpha[:, 0])
        self.rear_roll_acc = np.rad2deg(alpha[:, 1])

        # -----------------------------------------------------
        # Vehicle roll = front/rear average
        # -----------------------------------------------------

        self.roll = (
            self.front_roll
            + self.rear_roll
        ) / 2.0

        self.roll_rate = (
            self.front_roll_rate
            + self.rear_roll_rate
        ) / 2.0

        self.roll_acc = (
            self.front_roll_acc
            + self.rear_roll_acc
        ) / 2.0

        # -----------------------------------------------------
        # Chassis twist
        #
        # positive:
        # front roll > rear roll
        # -----------------------------------------------------

        self.twist = (
            self.front_roll
            - self.rear_roll
        )

        self.twist_rate = (
            self.front_roll_rate
            - self.rear_roll_rate
        )

        self.twist_acc = (
            self.front_roll_acc
            - self.rear_roll_acc
        )

        # -----------------------------------------------------
        # Pitch / Yaw
        # -----------------------------------------------------

        self.pitch = np.rad2deg(theta[:, 2])
        self.yaw = np.rad2deg(theta[:, 3])

        self.pitch_rate = np.rad2deg(omega[:, 2])
        self.yaw_rate = np.rad2deg(omega[:, 3])

        self.pitch_acc = np.rad2deg(alpha[:, 2])
        self.yaw_acc = np.rad2deg(alpha[:, 3])

        # =====================================================
        # Figure
        # =====================================================

        self.fig = plt.figure(figsize=(16, 9))

        gs = self.fig.add_gridspec(
            3, 2,
            width_ratios=[1, 1],
            hspace=0.45
        )

        self.ax_fz = self.fig.add_subplot(gs[0, 0])
        self.ax_modal = self.fig.add_subplot(gs[1, 0])
        self.ax_info = self.fig.add_subplot(gs[2:, 0])

        self.ax_theta = self.fig.add_subplot(gs[0, 1])
        self.ax_omega = self.fig.add_subplot(
            gs[1, 1],
            sharex=self.ax_theta
        )

        self.ax_alpha = self.fig.add_subplot(
            gs[2:, 1],
            sharex=self.ax_theta
        )

        # =====================================================
        # Bar plots
        # =====================================================

        self._create_barplots()

        # =====================================================
        # Attitude
        # =====================================================

        self._create_signal_plot(

            self.ax_theta,

            "Attitude (deg)",

            [
                (self.roll, "Roll"),
                (self.twist, "Twist"),
                (self.pitch, "Pitch"),
                (self.yaw, "Yaw"),
            ],

        )

        # =====================================================
        # Angular velocity
        # =====================================================

        self._create_signal_plot(

            self.ax_omega,

            "Angular Velocity (deg/s)",

            [
                (self.roll_rate, "Roll"),
                (self.twist_rate, "Twist"),
                (self.pitch_rate, "Pitch"),
                (self.yaw_rate, "Yaw"),
            ],

        )

        # =====================================================
        # Angular acceleration
        # =====================================================

        self._create_signal_plot(

            self.ax_alpha,

            "Angular Acceleration (deg/s²)",

            [
                (self.roll_acc, "Roll"),
                (self.twist_acc, "Twist"),
                (self.pitch_acc, "Pitch"),
                (self.yaw_acc, "Yaw"),
            ],

        )

        # =====================================================
        # Cursor
        # =====================================================

        self.cursor_theta = self.ax_theta.axvline(
            self.time[0],
            ls="--",
            c="k"
        )

        self.cursor_omega = self.ax_omega.axvline(
            self.time[0],
            ls="--",
            c="k"
        )

        self.cursor_alpha = self.ax_alpha.axvline(
            self.time[0],
            ls="--",
            c="k"
        )

        # =====================================================
        # Information panel
        # =====================================================

        self.ax_info.axis("off")

        self.info = self.ax_info.text(
            0.02,
            0.98,
            "",
            va="top",
            family="monospace",
            fontsize=8
        )

    # ==========================================================
    # Signal plot
    # ==========================================================

    def _create_signal_plot(self, ax, title, signals):

        ax.set_title(title)

        for data, label in signals:
            ax.plot(
                self.time,
                data,
                label=label
            )

        ax.grid(True)
        ax.legend()

    # ==========================================================
    # Bar plots
    # ==========================================================

    def _create_barplots(self):

        labels = [
            "FL",
            "FR",
            "RL",
            "RR"
        ]

        self.bar_fz = self.ax_fz.bar(
            labels,
            self.Fz[0]
        )

        self.ax_fz.set_title(
            "Wheel Normal Force"
        )

        self.ax_fz.set_ylabel("N")

        self.ax_fz.set_ylim(
            0,
            2500
        )

        # ---------------------------------------------
        # Modal
        # ---------------------------------------------

        mlabels = [
            "Heave F",
            "Heave R",
            "Roll F",
            "Roll R",
            "Center",
            "Warp",
            "FL",
            "FR",
            "RL",
            "RR"
        ]

        self.bar_modal = self.ax_modal.bar(
            mlabels,
            self.modal[0]
        )

        self.ax_modal.tick_params(
            axis="x",
            rotation=20
        )

        self.ax_modal.set_title(
            "Modal Force"
        )

        self.ax_modal.set_ylim(
            0,
            2500
        )

    # ==========================================================
    # Update
    # ==========================================================

    def update(self, frame):

        # =====================================================
        # Wheel load
        # =====================================================

        for bar, value in zip(
            self.bar_fz,
            self.Fz[frame]
        ):
            bar.set_height(value)

        # =====================================================
        # Modal force
        # =====================================================

        for bar, value in zip(
            self.bar_modal,
            self.modal[frame]
        ):
            bar.set_height(value)

        # =====================================================
        # Time cursor
        # =====================================================

        t = self.time[frame]

        self.cursor_theta.set_xdata(
            [t, t]
        )

        self.cursor_omega.set_xdata(
            [t, t]
        )

        self.cursor_alpha.set_xdata(
            [t, t]
        )

        # =====================================================
        # Information text
        # =====================================================

        txt = f"""
Time : {t:7.3f} s

Roll
θ : {self.roll[frame]:8.3f}  ω : {self.roll_rate[frame]:8.3f}  α : {self.roll_acc[frame]:8.3f}

Front Roll
θ : {self.front_roll[frame]:8.3f}  ω : {self.front_roll_rate[frame]:8.3f}  α : {self.front_roll_acc[frame]:8.3f}

Rear Roll
θ : {self.rear_roll[frame]:8.3f}  ω : {self.rear_roll_rate[frame]:8.3f}  α : {self.rear_roll_acc[frame]:8.3f}

Twist
θ : {self.twist[frame]:8.3f}  ω : {self.twist_rate[frame]:8.3f}  α : {self.twist_acc[frame]:8.3f}

Pitch
θ : {self.pitch[frame]:8.3f}  ω : {self.pitch_rate[frame]:8.3f}  α : {self.pitch_acc[frame]:8.3f}

Yaw
θ : {self.yaw[frame]:8.3f}  ω : {self.yaw_rate[frame]:8.3f}  α : {self.yaw_acc[frame]:8.3f}

Wheel Load (N)
FL : {self.Fz[frame,0]:8.1f}  FR : {self.Fz[frame,1]:8.1f}  RL : {self.Fz[frame,2]:8.1f}  RR : {self.Fz[frame,3]:8.1f}
"""

        self.info.set_text(txt)

        return (
            *self.bar_fz,
            *self.bar_modal,
            self.cursor_theta,
            self.cursor_omega,
            self.cursor_alpha,
            self.info
        )

    # ==========================================================
    # Animation frames
    # ==========================================================

    def get_animation_frames(self, fps=30):

        total = (
            self.time[-1]
            - self.time[0]
        )

        target = max(
            1,
            int(total * fps)
        )

        step = max(
            1,
            len(self.time) // target
        )

        return range(
            0,
            len(self.time),
            step
        )

    # ==========================================================
    # Show
    # ==========================================================

    def show(self, fps=30):

        self.ani = FuncAnimation(
            self.fig,
            self.update,
            frames=self.get_animation_frames(fps),
            interval=1000 / fps,
            blit=True
        )

        plt.show()