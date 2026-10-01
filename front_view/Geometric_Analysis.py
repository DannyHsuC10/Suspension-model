import numpy as np
import geometry as ge
from scipy.optimize import fsolve

# 幾何運算函式 (旋轉、求解、極限計算)
def rotate_point_about(center, length, theta):# 旋轉點
    return center + length * np.array([np.cos(theta), np.sin(theta)])

def circle_intersection(c1, r1, c2, r2):# 解析幾何2圓焦點
    dvec = c2 - c1
    d = np.linalg.norm(dvec)
    if d > r1 + r2 or d < abs(r1 - r2):
        return None

    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = np.sqrt(max(0, r1 * r1 - a * a))
    p = c1 + a * dvec / d
    perp = np.array([-dvec[1], dvec[0]]) / d

    return p + h * perp, p - h * perp

def simulate_side_theta(sus, theta):
    """
    根據傳入的單側懸吊物件 (sus) 與輸入角度 (theta) 計算更新後的關節點位置 (kua_new, lak_new)
    """
    actual_theta = (np.pi - theta) if sus.is_left else theta

    # 計算下臂旋轉後的位置
    lak_new = rotate_point_about(sus.cpb_rl.pos, sus.lower_arm.length, actual_theta)

    # 計算上臂與羊角連桿交點
    sol = circle_intersection(
        sus.cpb_ru.pos, sus.upper_arm.length, lak_new, sus.upright.length
    )
    if sol is None:
        return None

    # 選擇距離原本 kua 位置最近的解
    kua_new = min(sol, key=lambda x: np.linalg.norm(x - sus.kua.pos))
    return kua_new, lak_new

def update_upright(sus, kua_new, lak_new):
    """
    根據傳入的單側懸吊物件 (sus) 以及新的 (kua_new, lak_new)，計算羊角剛體更新後的接地點 gnd 座標
    """
    old_vec = sus.kua.pos - sus.lak.pos
    new_vec = kua_new - lak_new

    # 計算羊角旋轉角
    dtheta = np.arctan2(new_vec[1], new_vec[0]) - np.arctan2(
        old_vec[1], old_vec[0]
    )

    c, s = np.cos(dtheta), np.sin(dtheta)
    R = np.array([[c, -s], [s, c]])

    # 旋轉並平移得到新的 gnd 位置
    return lak_new + R @ (sus.gnd.pos - sus.lak.pos)

def calculate_theta_limits(sus, target_travel=0.025):
    """
    根據傳入的單側懸吊物件 (sus) 與目標行程 (target_travel)，計算下臂可用的角度極限 (theta_down, theta_up)
    """
    theta0 = sus.lower_arm.angle
    theta_list = np.linspace(theta0 - 0.3, theta0 + 0.3, 500)
    travel = []

    for theta in theta_list:
        # 直接傳入 sus 物件與當前 theta
        result = simulate_side_theta(sus, theta)

        if result is None:
            travel.append(np.nan)
            continue

        kua_test, lak_test = result

        # 直接傳入 sus 物件與新關節位置計算 gnd
        gnd_test = update_upright(sus, kua_test, lak_test)
        travel.append(gnd_test[1] - sus.gnd.pos[1])

    travel = np.array(travel)
    theta_up = np.interp(target_travel, travel, theta_list)
    theta_down = np.interp(-target_travel, travel, theta_list)

    if np.isnan(theta_up) or np.isnan(theta_down):
        theta_up, theta_down = theta0 + 0.3, theta0 - 0.3

    return min(theta_down, theta_up), max(theta_down, theta_up)

# ==============================================================================
# 懸吊幾何求解器 Geometry & Kinematics Helper
# ==============================================================================
class SusGeometryHelper:
    """懸吊幾何與運動學數值求解工具箱"""

    def __init__(self, car, suspension_f):
        self.car = car
        self.suspension_f = suspension_f
        self.tire_width = car.tire_width
        self.free_radius = car.free_radius
        self.h_cog = car.h_cog
        self.bottom = car.bottom

        self.t = car.tf
        self.load_Radius = car.load_Radius[0]
        self.scrub_radius = suspension_f.scrub_radius
        self.kpi = suspension_f.kpi
        self.h_rc = suspension_f.h_rc
        self.fvsa = suspension_f.fvsa
        self.body_face_f = car.body_face_f
        self.sus_contact = suspension_f.sus_contact
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

        ICo = np.array(
            [self.t / 2 - self.fvsa, self.fvsa / (self.t / 2) * self.h_rc]
        )

        ll = (self.load_Radius - self.Al) / np.cos(self.kpi)
        lak = kg + ll * theta_kpi

        gnd = np.array([self.t / 2, 0.0])
        cg = np.array([0.0, self.h_cog])
        rc = np.array([0.0, self.h_rc])

        wheel_p1 = np.array([gnd[0] - self.tire_width / 2, gnd[1]])
        wheel_p2 = np.array([gnd[0] + self.tire_width / 2, gnd[1]])
        wheel_p3 = np.array(
            [gnd[0] + self.tire_width / 2, gnd[1] + self.free_radius * 2]
        )
        wheel_p4 = np.array(
            [gnd[0] - self.tire_width / 2, gnd[1] + self.free_radius * 2]
        )

        rl, ll_p, lu_p, ru = self.sus_contact
        cpb_rl = self.line_intersection(rl, ru, lak, ICo)
        cpb_ru = self.line_intersection(rl, ru, kua, ICo)

        IC = self.line_intersection(cpb_rl, lak, cpb_ru, kua)

        return {
            "kg": kg,
            "kua": kua,
            "ICo": ICo,
            "lak": lak,
            "gnd": gnd,
            "cg": cg,
            "rc": rc,
            "cpb_rl": cpb_rl,
            "cpb_ru": cpb_ru,
            "IC": IC,
            "wheel_pts": [wheel_p1, wheel_p2, wheel_p3, wheel_p4],
        }

    def update_IC(self, IC_point, lower_arm, upper_arm):
        new_IC = self.line_intersection(
            lower_arm.start, lower_arm.end, upper_arm.start, upper_arm.end
        )
        if new_IC is not None:
            IC_point.pos = new_IC

    def calc_roll_center(self, left_sus, right_sus):
            """計算左右兩側 Wheel Force Line 的交點，即為動態 Roll Center (RC)"""
            rc_pos = self.line_intersection(
                left_sus.gnd, left_sus.IC, right_sus.gnd, right_sus.IC
            )
            print("rc_pos",rc_pos)
            return rc_pos


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
        self.ICo = None
        self.IC = None
        self.cpb_rl = None
        self.cpb_ru = None
        self.wheel_pts = []

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

        # Rigid Bodies & Visuals
        self.wheel = None
        self.upright_body = None

    def setup_right_side(self, pts_dict):
        """根據靜態計算座標建立右側懸吊」"""
        self.kg = ge.Point(pts_dict["kg"], f"kg_{self.name}")
        self.kua = ge.Point(pts_dict["kua"], f"kua_{self.name}")
        self.lak = ge.Point(pts_dict["lak"], f"lak_{self.name}")
        self.gnd = ge.Point(pts_dict["gnd"], f"gnd_{self.name}")
        self.ICo = ge.Point(pts_dict["ICo"], f"ICo_{self.name}")
        self.IC = ge.Point(pts_dict["IC"], f"IC_{self.name}")
        self.cpb_rl = ge.Point(pts_dict["cpb_rl"], f"cpb_rl_{self.name}")
        self.cpb_ru = ge.Point(pts_dict["cpb_ru"], f"cpb_ru_{self.name}")

        self.wheel_pts = [
            ge.Point(p, f"wp{i}_{self.name}")
            for i, p in enumerate(pts_dict["wheel_pts"])
        ]

        self._build_links_and_bodies()

    def setup_from_right_template(self, right_side):
        """從右側懸吊自動鏡像生成左側懸吊物件"""
        if not self.is_left:
            return

        self.kg = right_side.kg.mirror_x()
        self.kua = right_side.kua.mirror_x()
        self.lak = right_side.lak.mirror_x()
        self.gnd = right_side.gnd.mirror_x()
        self.ICo = right_side.ICo.mirror_x()
        self.IC = right_side.IC.mirror_x()
        self.cpb_rl = right_side.cpb_rl.mirror_x()
        self.cpb_ru = right_side.cpb_ru.mirror_x()

        self.wheel_pts = [p.mirror_x() for p in right_side.wheel_pts]

        self._build_links_and_bodies()

    def _build_links_and_bodies(self):
        """建立連桿與剛體綁定關係"""
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

        self.wheel = ge.Wheel(self.wheel_pts, name=f"{self.name} Wheel")

        self.upright_body = ge.RigidBody2D(
            [self.kua, self.lak, self.gnd, self.kg] + self.wheel_pts,
            reference_pair=(self.lak, self.kua),
            name=f"{self.name} Upright",
        )

    def add_to_scene(self, scene):# for動畫新增
        """將該側物件統一渲染進 Scene"""
        points = [
            self.kg,
            self.kua,
            self.lak,
            self.IC,
            self.ICo,
            self.gnd,
            self.cpb_rl,
            self.cpb_ru,
        ] + self.wheel_pts
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

        scene.add(self.wheel, linewidth=2, edgecolor="black", facecolor="gray")

    def calc_camber_angle(self, gnd_line_vec=np.array([1.0, 0.0])):
        """
        利用 wheel_pts[0] 與 wheel_pts[1] 組成的輪胎軸線/中心線向量來計算
        """
        if len(self.wheel_pts) >= 2:
            p1 = self.wheel_pts[0].pos
            p2 = self.wheel_pts[1].pos
            wheel_vec = p2 - p1  # 輪胎邊界/中軸向量
        else:
            # 備用方案：使用 upright 向量 (kua - lak)
            wheel_vec = self.kua.pos - self.lak.pos

        # 兩向量夾角 (rad)
        dot_product = np.dot(wheel_vec, gnd_line_vec)
        norms = np.linalg.norm(wheel_vec) * np.linalg.norm(gnd_line_vec)
        cos_angle = np.clip(dot_product / (norms + 1e-9), -1.0, 1.0)
        angle_rad = np.arccos(cos_angle)

        # 轉為度數 (deg)
        angle_deg = np.degrees(angle_rad)

        # 判斷外傾角正負號 (以左/右側輪幾何為準)
        if self.is_left:
            return (180-angle_deg)
        else:
            return angle_deg

    def update_kinematics(self, theta, helper):
        """根據角度更新單側運動學"""
        res = simulate_side_theta(
            sus=self,theta=theta)

        if res is not None:
            kua_new, lak_new = res
            # 1. 更新剛體（連帶更新轉向節上所有頂點與輪胎點）
            self.upright_body.update_from_two_points(lak_new, kua_new)

            # 2. 更新瞬心 IC
            helper.update_IC(self.IC, self.lower_arm, self.upper_arm)

            # 3. 更新力線端點
            self.upper_force.p2 = self.IC
            self.lower_force.p2 = self.IC
            self.wheel_force.p2 = self.IC

class SuspensionDirectSolver:

    def __init__(self, sus_helper, left_sus, right_sus):
        self.sus_helper = sus_helper
        self.left_sus = left_sus
        self.right_sus = right_sus
        self.gnd_y0_l = left_sus.gnd.pos[1]
        self.gnd_y0_r = right_sus.gnd.pos[1]

    def solve_by_thetas(self, theta_l, theta_r):
        """方法 1：已知左右下臂角度 (theta_l, theta_r)，直接計算精確的懸吊姿態與 RC。"""
        # 1. 更新懸吊運動學
        self.left_sus.update_kinematics(theta_l, self.sus_helper)
        self.right_sus.update_kinematics(theta_r, self.sus_helper)

        # 2. 計算單輪行程
        l_travel = self.left_sus.gnd.pos[1] - self.gnd_y0_l
        r_travel = self.right_sus.gnd.pos[1] - self.gnd_y0_r

        # 3. 計算 Heave 與 Roll
        heave = (l_travel + r_travel) / 2.0
        gnd_vec = self.right_sus.gnd.pos - self.left_sus.gnd.pos
        roll_angle_deg = np.degrees(np.arctan2(gnd_vec[1], gnd_vec[0]))

        # 4. 計算 Camber 角度
        gnd_norm_vec = gnd_vec / np.linalg.norm(gnd_vec)
        camber_l = self.left_sus.calc_camber_angle(gnd_norm_vec)
        camber_r = self.right_sus.calc_camber_angle(gnd_norm_vec)

        # 5. 計算 Roll Center
        rc_pos = self.sus_helper.calc_roll_center(self.left_sus, self.right_sus)

        return {
            "heave_mm": heave * 1000.0,
            "roll_deg": roll_angle_deg,
            "camber_left_deg": camber_l,
            "camber_right_deg": camber_r,
            "rc_pos": rc_pos,  # np.array([x, y])
            "travel_left_mm": l_travel * 1000.0,
            "travel_right_mm": r_travel * 1000.0,
            "theta_left": theta_l,
            "theta_right": theta_r,
        }

    def solve_by_travels(self, l_travel_m, r_travel_m):
        """方法 2：已知左右輪懸吊行程 (l_travel_m, r_travel_m，單位: 米)，
        自動反算對應的 theta 角度，並回傳精確的 Camber 與 Roll Center 位置。

        :param l_travel_m: 左輪目標行程 (m)，例如 0.015 代表上壓 15mm
        :param r_travel_m: 右輪目標行程 (m)
        :return: 包含精確狀態資訊的字典
        """

        # 定義殘差目標函數 (Residual Function)
        def travel_residuals(thetas):
            th_l, th_r = thetas
            self.left_sus.update_kinematics(th_l, self.sus_helper)
            self.right_sus.update_kinematics(th_r, self.sus_helper)

            # 計算當前的行程
            curr_l_travel = self.left_sus.gnd.pos[1] - self.gnd_y0_l
            curr_r_travel = self.right_sus.gnd.pos[1] - self.gnd_y0_r

            # 殘差：目標行程 - 當前行程
            return [
                curr_l_travel - l_travel_m,
                curr_r_travel - r_travel_m,
            ]

        # 初始猜測值取當前角度
        guess = [self.left_sus.lower_arm.angle, self.right_sus.lower_arm.angle]

        # 執行非線性求解
        solved_thetas, info, ier, mesg = fsolve(
            travel_residuals, guess, full_output=True
        )

        if ier != 1:
            raise RuntimeError(f"行程反算求解失敗：{mesg}")

        # 利用解出的角度計算最終狀態
        return self.solve_by_thetas(solved_thetas[0], solved_thetas[1])

    def solve_by_pose(self, target_heave_m, target_roll_deg):
        """方法 3：已知車身姿態 (Heave, Roll)，使用數值根求解器反推 theta 並傳回精確數據。"""
        target_roll_rad = np.radians(target_roll_deg)

        def pose_residuals(thetas):
            th_l, th_r = thetas
            self.left_sus.update_kinematics(th_l, self.sus_helper)
            self.right_sus.update_kinematics(th_r, self.sus_helper)

            l_travel = self.left_sus.gnd.pos[1] - self.gnd_y0_l
            r_travel = self.right_sus.gnd.pos[1] - self.gnd_y0_r

            current_heave = (l_travel + r_travel) / 2.0
            gnd_vec = self.right_sus.gnd.pos - self.left_sus.gnd.pos
            current_roll = np.arctan2(gnd_vec[1], gnd_vec[0])

            return [
                current_heave - target_heave_m,
                current_roll - target_roll_rad,
            ]

        guess = [self.left_sus.lower_arm.angle, self.right_sus.lower_arm.angle]

        solved_thetas, info, ier, mesg = fsolve(
            pose_residuals, guess, full_output=True
        )

        if ier != 1:
            raise RuntimeError(f"姿態反算求解失敗：{mesg}")

        return self.solve_by_thetas(solved_thetas[0], solved_thetas[1])


