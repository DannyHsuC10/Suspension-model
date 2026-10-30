import numpy as np
from . import geometry as ge
from .car import SuspensionR


# ==============================================================================
# 靜態懸吊幾何工具 Static Geometry Helper
# ==============================================================================
class SusGeometryHelper:
    """靜態懸吊幾何建構與直線交點工具。"""

    def __init__(self, car, suspension_f):
        """接受前軸或後軸懸吊，套用對應的輪距與負載輪胎半徑。"""
        self.car = car
        self.suspension_f = suspension_f
        self.tire_width = car.tire_width
        self.free_radius = car.free_radius
        self.h_cog = car.h_cog
        self.bottom = car.bottom

        is_rear = isinstance(suspension_f, SuspensionR)
        self.t = car.tr if is_rear else car.tf
        self.load_Radius = car.load_Radius[1 if is_rear else 0]
        self.scrub_radius = suspension_f.scrub_radius
        self.kpi = suspension_f.kpi
        self.h_RC = suspension_f.h_RC
        self.fvsa = suspension_f.fvsa
        self.lower_mount_x = float(suspension_f.lower_mount_x)
        self.upper_mount_x = float(suspension_f.upper_mount_x)
        mount_x = [self.lower_mount_x, self.upper_mount_x]
        if not np.isfinite(mount_x).all() or min(mount_x) < 0:
            raise ValueError("Mount X coordinates must be finite, nonnegative distances [m].")
        self.Au = suspension_f.Au
        self.Al = suspension_f.Al

    @staticmethod
    def line_intersection(p1, p2, q1, q2):
        if isinstance(p1, ge.Point):
            p1 = p1.pos
        if isinstance(p2, ge.Point):
            p2 = p2.pos
        if isinstance(q1, ge.Point):
            q1 = q1.pos
        if isinstance(q2, ge.Point):
            q2 = q2.pos

        p1 = np.asarray(p1, dtype=float)
        p2 = np.asarray(p2, dtype=float)
        q1 = np.asarray(q1, dtype=float)
        q2 = np.asarray(q2, dtype=float)

        r = p2 - p1
        s = q2 - q1
        denom = r[0] * s[1] - r[1] * s[0]

        if np.isclose(denom, 0.0):
            return None

        t = ((q1 - p1)[0] * s[1] - (q1 - p1)[1] * s[0]) / denom
        return p1 + t * r

    def calc_static_points(self):
        """計算靜態狀態下的幾何座標點 (右側為基準)"""
        lu = (self.load_Radius + self.Au) / np.cos(self.kpi)
        kg = np.array([self.t / 2 - self.scrub_radius, 0.0])
        theta_kpi = np.array(
            [np.cos(np.pi / 2 + self.kpi), np.sin(np.pi / 2 + self.kpi)]
        )
        kua = kg + lu * theta_kpi

        # 唯一的固定設計瞬心，不再由控制臂交點重新求解。
        IC = np.array(
            [self.t / 2 - self.fvsa, self.fvsa / (self.t / 2) * self.h_RC]
        )

        ll = (self.load_Radius - self.Al) / np.cos(self.kpi)
        lak = kg + ll * theta_kpi

        gnd = np.array([self.t / 2, 0.0])
        cg = np.array([0.0, self.h_cog])
        RC = np.array([0.0, self.h_RC])

        # 輪心以負載半徑定位；外形仍使用自由半徑，因此胎底會低於地面。
        wheel_center = gnd + np.array([0.0, self.load_Radius])

        def mount_at_x(x, upright):
            # 固定 X，直接由羊角接點到設計瞬心的斜率解 Y。
            direction = IC - upright
            if np.isclose(direction[0], 0.0):
                raise ValueError("Cannot solve mount Y from a vertical upright-to-IC line.")
            y = upright[1] + (x - upright[0]) * direction[1] / direction[0]
            return np.array([x, y])

        cpb_rl = mount_at_x(self.lower_mount_x, lak)
        cpb_ru = mount_at_x(self.upper_mount_x, kua)

        return {
            "kg": kg,
            "kua": kua,
            "lak": lak,
            "gnd": gnd,
            "cg": cg,
            "RC": RC,
            "cpb_rl": cpb_rl,
            "cpb_ru": cpb_ru,
            "IC": IC,
            "wheel_center": wheel_center,
            "tire_radius": self.free_radius,
            "tire_width": self.tire_width,
        }

    def calc_roll_center(self, left_sus, right_sus):
        """以固定 IC 與接地點的左右力線計算靜態滾動中心。"""
        return self.line_intersection(
            left_sus.gnd, left_sus.IC, right_sus.gnd, right_sus.IC
        )


# ==============================================================================
# 單側懸吊類別 SuspensionSide (核心模組化封裝)
# ==============================================================================
class SuspensionSide:

    def __init__(self, name, is_left=False):
        self.name = name
        self.is_left = is_left

        # Points
        self.kg = None
        self.kua = None
        self.lak = None
        self.gnd = None
        self.IC = None
        self.cpb_rl = None
        self.cpb_ru = None
        self.wheel_center = None

        # Links
        self.kingpin = None
        self.lower_arm = None
        self.upper_arm = None
        self.upright = None
        self.body_fix = None

        # Force Lines
        self.upper_force = None
        self.lower_force = None
        self.wheel_force = None

        # Static cylindrical tire dimensions and transverse axis in the FV plane.
        self.tire_radius = None
        self.tire_width = None
        self.wheel_axis = None

    def setup_right_side(self, pts_dict):
        """根據靜態計算座標建立右側懸吊」"""
        self.kg = ge.Point(pts_dict["kg"], f"kg_{self.name}")
        self.kua = ge.Point(pts_dict["kua"], f"kua_{self.name}")
        self.lak = ge.Point(pts_dict["lak"], f"lak_{self.name}")
        self.gnd = ge.Point(pts_dict["gnd"], f"gnd_{self.name}")
        self.IC = ge.Point(pts_dict["IC"], f"IC_{self.name}")
        self.cpb_rl = ge.Point(pts_dict["cpb_rl"], f"cpb_rl_{self.name}")
        self.cpb_ru = ge.Point(pts_dict["cpb_ru"], f"cpb_ru_{self.name}")

        self.wheel_center = ge.Point(pts_dict["wheel_center"], f"wheel_center_{self.name}")
        self.tire_radius = float(pts_dict["tire_radius"])
        self.tire_width = float(pts_dict["tire_width"])
        self.wheel_axis = np.array([1.0, 0.0])

        self._build_links()

    def setup_from_right_template(self, right_side):
        """從右側懸吊自動鏡像生成左側懸吊物件"""
        if not self.is_left:
            return

        self.kg = right_side.kg.mirror_x()
        self.kua = right_side.kua.mirror_x()
        self.lak = right_side.lak.mirror_x()
        self.gnd = right_side.gnd.mirror_x()
        self.IC = right_side.IC.mirror_x()
        self.cpb_rl = right_side.cpb_rl.mirror_x()
        self.cpb_ru = right_side.cpb_ru.mirror_x()

        self.wheel_center = right_side.wheel_center.mirror_x()
        self.tire_radius = right_side.tire_radius
        self.tire_width = right_side.tire_width
        self.wheel_axis = right_side.wheel_axis * np.array([-1.0, 1.0])

        self._build_links()

    def _build_links(self):
        """建立固定連桿與力線。"""
        self.kingpin = ge.Link(self.kg, self.kua, f"Kingpin {self.name}")
        self.lower_arm = ge.Link(
            self.cpb_rl, self.lak, f"Lower Arm {self.name}"
        )
        self.upper_arm = ge.Link(
            self.cpb_ru, self.kua, f"Upper Arm {self.name}"
        )
        self.upright = ge.Link(self.kua, self.lak, f"Upright {self.name}")
        self.body_fix = ge.Link(
            self.cpb_rl, self.cpb_ru, f"Body Fix {self.name}"
        )

        self.upper_force = ge.Link(
            self.kua, self.IC, f"Upper Force {self.name}"
        )
        self.lower_force = ge.Link(
            self.lak, self.IC, f"Lower Force {self.name}"
        )
        self.wheel_force = ge.Link(
            self.gnd, self.IC, f"Wheel Force {self.name}"
        )

    def add_to_scene(self, scene):
        """將該側物件統一渲染進 Scene"""
        points = [
            self.kg,
            self.kua,
            self.lak,
            self.IC,
            self.gnd,
            self.cpb_rl,
            self.cpb_ru,
        ]
        links = [
            self.kingpin,
            self.lower_arm,
            self.upper_arm,
            self.upright,
            self.body_fix,
        ]
        forces = [self.upper_force, self.lower_force, self.wheel_force]

        for p in points:
            scene.add(p, color="red")
        for l in links:
            scene.add(l, color="green")
        for f in forces:
            scene.add(f, color="blue")

    def calc_camber_angle(self, gnd_line_vec=np.array([1.0, 0.0])):
        """
        靜態輪軸相對左右接地點連線的有號 camber，單位 deg。

        地面向量由左輪指向右輪；輪胎上端向車外傾為正、向車內傾為負。
        左輪的輪軸方向經過鏡像，先反轉方向再計算。
        """
        gnd_line_vec = np.asarray(gnd_line_vec, dtype=float)
        wheel_vec = np.asarray(self.wheel_axis, dtype=float)
        if self.is_left:
            wheel_vec = -wheel_vec

        if (np.linalg.norm(wheel_vec) < 1e-12
                or np.linalg.norm(gnd_line_vec) < 1e-12):
            raise ValueError("Camber requires nonzero tire and ground vectors.")

        # atan2 保留地面轉到輪胎底邊的方向，並避免 arccos 在 0 度附近的誤差。
        cross_product = (
            gnd_line_vec[0] * wheel_vec[1]
            - gnd_line_vec[1] * wheel_vec[0]
        )
        dot_product = np.dot(wheel_vec, gnd_line_vec)
        angle_deg = np.degrees(np.arctan2(cross_product, dot_product))

        # 同樣的旋轉方向，在左右輪代表相反的內傾／外傾。
        return float(angle_deg if self.is_left else -angle_deg)
