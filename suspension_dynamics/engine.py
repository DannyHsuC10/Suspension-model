"""Work-conjugate linear mechanical assembly and cancellable RK4 simulation.

No Tk, Matplotlib, legacy imports, global parameters, or filesystem side effects.
"""
from dataclasses import dataclass, field
import copy
import math

import numpy as np

from .catalog import CORNERS, MODELS
from .project import (MODAL_CHANNELS, PARAMETER_SOURCES, corner_rates, full_mass_distribution,
                      migrate_project_parameters, parameters_for_analysis, validate, validate_parameters, wheel_rates)
from .road_profiles import ISO_FREQUENCY_BAND, ISO_REFERENCE_FREQUENCY, ISO_ROUGHNESS, IsoRoadProfile

G = 9.81
BUMP_SCENARIOS = ("bump", "bump_cornering", "combined")
LONGITUDINAL_SCENARIOS = ("accelerate", "combined")
LATERAL_SCENARIOS = ("cornering", "bump_cornering", "combined")
FULL_T = np.array([[1,1,0,0], [0,0,1,1], [1,-1,0,0], [0,0,1,-1],
                   [1,-1,1,-1], [1,-1,-1,1], *np.eye(4)], dtype=float)
CHANNEL_NAMES = (*MODAL_CHANNELS, *(f"corner_{corner}" for corner in CORNERS))


class Cancelled(Exception):
    pass


@dataclass
class System:
    family: str
    model: str
    p: dict
    labels: tuple
    B: np.ndarray
    masses: np.ndarray
    ks: np.ndarray
    cs: np.ndarray
    kt: np.ndarray
    ct: np.ndarray
    static_load: np.ndarray
    body_dof: int
    tire_dof: bool
    K: np.ndarray = field(default=None)
    C: np.ndarray = field(default=None)
    D: np.ndarray = field(default=None)
    T: np.ndarray = field(default=None)
    channel_k: np.ndarray = field(default=None)
    channel_c: np.ndarray = field(default=None)

    @property
    def dof(self):
        return len(self.masses)

    def forces(self, time, road):
        """External generalized forces, including exact road z and dz/dt."""
        return self.input_force(*road.at(time))

    def _longitudinal_anti(self, ax, moment):
        p = self.p
        transfer = moment / (p["lf"] + p["lr"])
        if "anti_front" in p:
            return transfer * p["anti_front"], -transfer * p["anti_rear"]

        # Archived results use their original formula for equilibrium statistics.
        if self.family == "FR":
            anti = p["anti_squat_rear"] if ax >= 0 else p["anti_dive_front"]
            return transfer * anti, -transfer * anti
        drive = ax >= 0
        ratio = p["drive_front" if drive else "brake_front"]
        anti_f = p["lift_front" if drive else "dive_front"]
        anti_r = p["squat_rear" if drive else "lift_rear"]
        return moment * ratio * anti_f / p["lf"], -moment * (1 - ratio) * anti_r / p["lr"]

    def input_force(self, zr, vr, ax, ay):
        force = np.zeros(self.dof)
        if self.tire_dof:
            force[self.body_dof:self.body_dof+len(self.labels)] = self.kt*zr + self.ct*vr
        else:
            force[:self.body_dof] = self.B.T @ (self.ks @ zr + self.cs @ vr)
        p = self.p
        geo = np.zeros(len(self.labels))
        if self.family == "LR":
            lateral_force = 2 * (p["ms"] + p["mu"]) * ay
            moment = lateral_force * p["h_cg"]
            if self.model in ("2mass", "2mass_2dof"):
                # Virtual work: M d(roll) = M d(zL - zR) / track.
                force[0] += moment / p["track"]
                force[1] -= moment / p["track"]
            else:
                force[1] += moment
                if self.tire_dof:
                    command = lateral_force * p["h_rc"] / p["track"]
                    if self.model == "rollcenter_all":
                        force[-1] = command
                    else:
                        geo[:] = [command, -command]
        elif self.family == "FR":
            moment = 2*(p["ms"]+p["mu"])*ax*p["h_cg"]
            force[1] += moment
            if self.tire_dof:
                geo[:] = self._longitudinal_anti(ax, moment)
        elif self.family == "full":
            share = p["lr"]/(p["lf"]+p["lr"])
            mx = p["m"]*ay*p["h_cg"]
            my = p["m"]*ax*p["h_cg"]
            if self.model == "rigid":
                force[1:3] += [mx, my]
            else:
                force[1:4] += [mx*share, mx*(1-share), my]
            gf = p["m"]*ay*share*p["h_rc_f"]/p["tf"]
            gr = p["m"]*ay*(1-share)*p["h_rc_r"]/p["tr"]
            geo[:] = [gf, -gf, gr, -gr]
            gf, gr = self._longitudinal_anti(ax, my)
            geo += np.array([gf, gf, gr, gr])/2
        if self.tire_dof:
            force[:self.body_dof] -= self.B.T @ geo
            force[self.body_dof:self.body_dof+len(self.labels)] += geo
        return force


def assemble(family, model, p):
    """Assemble M diagonal and symmetric passive K/C via virtual work."""
    tire = model not in ("single", "one_eighth", "2mass_2dof", "half_2dof")
    T = None
    if family == "quarter":
        labels, B = ("Q",), np.ones((1, 1))
        if model in ("single", "one_eighth"):
            kw = p["ks"]*p["mr"]**2
            ks = np.array([[kw*p["kt"]/(kw+p["kt"])]])
            cs = np.array([[p["cs"]*p["mr"]**2 if model == "one_eighth" else 0.]])
            masses = [p["ms"]+p["mu"]]
        else:
            ks = np.array([[p["ks"]*p["mr"]**2]])
            cs = np.array([[p["cs"]*p["mr"]**2 if model == "quarter" else 0.]])
            masses = [p["ms"]]
        kt, ct = [p["kt"]], [p["ct"] if model == "quarter" else 0.]
        static = [(p["ms"]+p["mu"])*G]
    elif family == "LR":
        labels = ("L", "R")
        independent = model in ("2mass", "2mass_2dof")
        B = np.eye(2) if independent else np.array([[1., p["track"]/2], [1., -p["track"]/2]])
        wheel_masses = [p.get(f"mu_{side}", p["mu"]) for side in ("left", "right")]
        masses = [p["ms"] + (0 if tire else mu) for mu in wheel_masses] if independent else [2*p["ms"], p["Ix"]]
        T = np.array([[1.,1.], [1.,-1.]])/math.sqrt(2)
        ks = T.T @ np.diag([p["k_heave"]*p["mr_heave"]**2, p["k_roll"]*p["mr_roll"]**2]) @ T
        cs = T.T @ np.diag([p["c_heave"]*p["mr_heave"]**2, p["c_roll"]*p["mr_roll"]**2]) @ T
        ks += np.diag([p["k_left"], p["k_right"]])*p["mr_corner"]**2
        cs += np.diag([p["c_left"], p["c_right"]])*p["mr_corner"]**2
        kt = [p.get(f"kt_{side}", p["kt"]) for side in ("left", "right")]
        ct = [p.get(f"ct_{side}", p["ct"]) for side in ("left", "right")]
        static = [(p["ms"] + mu) * G for mu in wheel_masses]
    elif family == "FR":
        labels = ("F", "R")
        B = np.array([[1., p["lf"]], [1., -p["lr"]]])
        masses = [2*p["ms"], p["Iy"]]
        ks = np.diag([p["k_heave_f"]*p["mr_front"]**2+p["k_front"]*p["mr_corner"]**2,
                      p["k_heave_r"]*p["mr_rear"]**2+p["k_rear"]*p["mr_corner"]**2])
        cs = np.diag([p["c_heave_f"]*p["mr_front"]**2+p["c_front"]*p["mr_corner"]**2,
                      p["c_heave_r"]*p["mr_rear"]**2+p["c_rear"]*p["mr_corner"]**2])
        kt, ct = [p["kt"]]*2, [p["ct"]]*2
        static = [2*p["ms"]*G*p["lr"]/(p["lf"]+p["lr"])+p["mu"]*G,
                  2*p["ms"]*G*p["lf"]/(p["lf"]+p["lr"])+p["mu"]*G]
    elif family == "full":
        labels = CORNERS
        ms, share, corner_masses = full_mass_distribution(p)
        wheel_masses = [corner_masses[corner] for corner in CORNERS]
        if model == "rigid":
            B = np.array([[1,p["tf"]/2,p["lf"]], [1,-p["tf"]/2,p["lf"]],
                          [1,p["tr"]/2,-p["lr"]], [1,-p["tr"]/2,-p["lr"]]])
            masses = [ms, p["Ixx"], p["Iyy"]]
        else:
            B = np.array([[1,p["tf"]/2,0,p["lf"]], [1,-p["tf"]/2,0,p["lf"]],
                          [1,0,p["tr"]/2,-p["lr"]], [1,0,-p["tr"]/2,-p["lr"]]])
            masses = [ms, p["Ixx"]*share, p["Ixx"]*(1-share), p["Iyy"]]
        T = FULL_T
        rates = [wheel_rates(p, channel) for channel in MODAL_CHANNELS]
        rates.extend(corner_rates(p, "corner", corner) for corner in CORNERS)
        kvals, cvals = np.asarray(rates).T
        ks, cs = T.T @ np.diag(kvals) @ T, T.T @ np.diag(cvals) @ T
        kt, ct = zip(*(corner_rates(p, "tire", corner) for corner in CORNERS))
        static = [(ms * (share if corner.startswith("F") else 1 - share) / 2 + corner_masses[corner]) * G
                  for corner in CORNERS]
    else:
        raise ValueError("未知模型分類。")
    nb = len(masses)
    n = len(labels)
    if tire:
        if family in ("LR", "full"):
            masses += wheel_masses
        else:
            masses += [p["mu"]]*n
    frame = family == "LR" and model == "rollcenter_all"
    if frame:
        masses += [p["m_frame"]]
    s = System(family, model, copy.deepcopy(p), labels, B, np.array(masses), ks, cs,
               np.array(kt), np.array(ct), np.array(static), nb, tire, T=T)
    if family == "full":
        s.channel_k, s.channel_c = kvals, cvals
    D = np.zeros((n, len(masses)))
    D[:, :nb] = B
    if tire:
        D[:, nb:nb+n] = -np.eye(n)
    s.D = D
    s.K = D.T @ ks @ D
    s.C = D.T @ cs @ D
    if tire:
        s.K[nb:nb+n, nb:nb+n] += np.diag(kt)
        s.C[nb:nb+n, nb:nb+n] += np.diag(ct)
    if family == "full" and model == "flex":
        twist = np.zeros(len(masses)); twist[1:3] = [1, -1]
        s.K += np.outer(twist, twist)*p["body_twist_k"]
        s.C += np.outer(twist, twist)*p["body_twist_c"]
    if frame:
        # Transmitted force reacts equally/oppositely on body and wheels.
        direction = np.array([1., -1.])
        coupling = np.zeros(len(masses))
        coupling[:nb] = -B.T @ direction
        coupling[nb:nb+n] = direction
        s.K[:, -1] -= coupling*p["k_frame"]
        s.C[:, -1] -= coupling*p["c_frame"]
        s.K[-1, -1] = p["k_frame"]
        s.C[-1, -1] = p["c_frame"]
    return s


class Road:
    def __init__(self, system, options):
        self.system, self.o = system, options
        self.delays = np.zeros(len(system.labels))
        if options.get("rear_delay") and system.family in ("FR", "full"):
            delay = (system.p["lf"]+system.p["lr"])/options["speed"]
            self.delays[-(2 if system.family == "full" else 1):] = delay
        target = options["target"]
        self.mask = np.array([self.matches(label, target, system.family) for label in system.labels], dtype=float)
        scenario = options["scenario"]
        events = {0., options["duration"], options["start"]}
        if scenario == "random":
            rng = np.random.default_rng(int(options["seed"]))
            count = math.ceil(options["duration"] / options["random_interval"]) + 3
            self.random = rng.normal(0., options["height"], count)
            self.random[0] = 0.
        elif scenario in ISO_ROUGHNESS:
            self.iso_profile = IsoRoadProfile(ISO_ROUGHNESS[scenario], options["seed"])
            starts = options["start"] + self.delays
            # Wheels with the same onset share one profile sample.
            self.iso_onsets = [(start, starts == start) for start in np.unique(starts[self.mask != 0])]
            events.update(start for start, _ in self.iso_onsets)
        if scenario not in ISO_ROUGHNESS:
            events.add(options["start"]+options["ramp"])
        if scenario in (*BUMP_SCENARIOS, "step", "sine", "random"):
            for delay in self.delays:
                start = options["start"]+delay
                events.update((start, start+options["period"]))
                if scenario == "random":
                    events.update(start+i*options["random_interval"] for i in range(count))
        self.events = np.array(sorted(t for t in events if 0 <= t <= options["duration"]))

    @staticmethod
    def matches(label, target, family):
        if target == "all": return True
        if family == "LR": return label == ("L" if target == "left" else "R")
        if family == "FR": return label == ("F" if target == "front" else "R")
        if target == "left": return label.endswith("L")
        if target == "right": return label.endswith("R")
        if target == "front": return label.startswith("F")
        if target == "rear": return label.startswith("R")
        return label == target

    def at(self, time):
        o = self.o
        t = time-o["start"]-self.delays
        n = len(t)
        z, v = np.zeros(n), np.zeros(n)
        scenario = o["scenario"]
        active = t >= 0
        if scenario == "step":
            z[active] = o["height"]
        elif scenario in BUMP_SCENARIOS:
            active &= t <= o["period"]
            phase = 2*np.pi*t[active]/o["period"]
            z[active] = o["height"]*(1-np.cos(phase))/2
            v[active] = o["height"]*np.pi/o["period"]*np.sin(phase)
        elif scenario == "sine":
            phase = 2*np.pi*o["frequency"]*t[active]
            z[active] = o["height"]*np.sin(phase)
            v[active] = o["height"]*2*np.pi*o["frequency"]*np.cos(phase)
        elif scenario == "random":
            pos = t[active]/o["random_interval"]
            index = np.minimum(pos.astype(int), len(self.random)-2)
            fraction = pos-index
            z[active] = self.random[index]*(1-fraction)+self.random[index+1]*fraction
            v[active] = (self.random[index+1]-self.random[index])/o["random_interval"]
        elif scenario in ISO_ROUGHNESS:
            speed = o["speed"]
            for start, wheels in self.iso_onsets:
                if time >= start:
                    height, slope = self.iso_profile.at(speed * (time - start))
                    z[wheels] = height
                    v[wheels] = speed * slope
        ramp_t = max(0., min(1., (time-o["start"])/o["ramp"]))
        ramp = (1-math.cos(math.pi*ramp_t))/2
        ax = o["ax"] * ramp if scenario in LONGITUDINAL_SCENARIOS else 0.
        ay = o["ay"] * ramp if scenario in LATERAL_SCENARIOS else 0.
        return z * self.mask, v * self.mask, ax, ay

    def terminal_input(self):
        """Return the time and values of the last constant input, if it exists."""
        options = self.o
        scenario = options["scenario"]
        height = options["height"]
        if scenario in ISO_ROUGHNESS or (height and scenario in ("sine", "random")):
            return None
        start = options["start"]
        ready = start
        if height and scenario in (*BUMP_SCENARIOS, "step"):
            ready += float(np.max(self.delays[self.mask != 0]))
            if scenario in BUMP_SCENARIOS:
                ready += options["period"]
        ax = options["ax"] if scenario in LONGITUDINAL_SCENARIOS else 0.
        ay = options["ay"] if scenario in LATERAL_SCENARIOS else 0.
        if ax or ay:
            ready = max(ready, start + options["ramp"])
        road_height = height * self.mask if scenario == "step" else np.zeros_like(self.mask)
        return ready, (road_height, np.zeros_like(self.mask), ax, ay)


@dataclass
class Result:
    family: str
    model: str
    time: np.ndarray
    signals: dict
    units: dict
    groups: dict
    project: dict
    info: dict
    legacy: bool = False


def simulate(project, family, cancel=lambda: False, progress=lambda fraction: None, initial=None):
    project = migrate_project_parameters(project)
    validate(project)
    options = project["analysis"][family]
    model = options["model"]
    parameters = parameters_for_analysis(project, family)
    if parameters is not project["parameters"][family]:
        validate_parameters(family, parameters)
    s = assemble(family, model, parameters)
    road = Road(s, options)
    n = s.dof
    A = np.block([[np.zeros((n,n)), np.eye(n)], [-s.K/s.masses[:,None], -s.C/s.masses[:,None]]])
    eigen = np.linalg.eigvals(A)
    if np.max(eigen.real) > 1e-6:
        raise ValueError("此組參數產生不穩定系統，請檢查彈簧、阻尼及車架設定。")
    max_step = min(options["dt"], .06/max(1., float(np.max(np.abs(eigen)))))
    if options["scenario"] == "sine":
        max_step = min(max_step, 1/(40*options["frequency"]))
    elif options["scenario"] in ISO_ROUGHNESS:
        highest_frequency = ISO_FREQUENCY_BAND[1] * options["speed"]
        max_step = min(max_step, 1 / (40 * highest_frequency))
    if options["scenario"] in BUMP_SCENARIOS:
        max_step = min(max_step, options["period"]/40)
    if options["duration"]/max_step > 750000:
        adjustment = "、降低車速" if options["scenario"] in ISO_ROUGHNESS else ""
        raise ValueError(f"參數需要超過 750,000 個內部積分步，請縮短模擬時間{adjustment}或降低剛性／阻尼。")
    time = np.arange(math.floor(options["duration"]/options["dt"])+1)*options["dt"]
    if time[-1] < options["duration"]-1e-12:
        time = np.append(time, options["duration"])
    else:
        time[-1] = options["duration"]
    state = np.zeros(2*n) if initial is None else np.asarray(initial, dtype=float).copy()
    if state.shape != (2*n,) or not np.isfinite(state).all():
        raise ValueError("初始狀態維度或數值錯誤。")
    history = np.empty((len(time), 2*n))
    count = 0

    def derivative(t, y):
        f = A @ y
        f[n:] += s.forces(t, road)/s.masses
        return f

    for i, t in enumerate(time):
        if cancel(): raise Cancelled()
        history[i] = state
        if i % 100 == 0: progress(i/max(1,len(time)-1))
        if i == len(time)-1: break
        end = time[i+1]
        boundaries = road.events[(road.events > t+1e-13) & (road.events < end-1e-13)]
        points = [float(t), *boundaries, float(end)]
        for a, b in zip(points, points[1:]):
            steps = max(1, math.ceil((b-a)/max_step-1e-10))
            h = (b-a)/steps
            for j in range(steps):
                if count % 128 == 0 and cancel(): raise Cancelled()
                now = a+j*h
                # Do not smear a discontinuity into the preceding interval.
                last_t = float(np.nextafter(now+h, now))
                k1 = derivative(now, state)
                k2 = derivative(now+h/2, state+h*k1/2)
                k3 = derivative(now+h/2, state+h*k2/2)
                k4 = derivative(last_t, state+h*k3)
                state += h*(k1+2*k2+2*k3+k4)/6
                count += 1
        if not np.isfinite(state).all() or np.max(np.abs(state)) > 1e10:
            raise ValueError("模擬數值發散，請檢查參數與輸入。")
    progress(1.)
    return make_result(s, road, time, history, project, eigen, count, max_step)


def result_signals(s, inputs, history):
    n, nb = s.dof, s.body_dof
    q, v = history[:, :n], history[:, n:]
    zr = np.array([x[0] for x in inputs])
    vr = np.array([x[1] for x in inputs])
    acc = np.array([(s.input_force(*sample) - s.K @ pos - s.C @ vel) / s.masses
                    for sample, pos, vel in zip(inputs, q, v)])
    body, body_v, body_a = q[:,:nb]@s.B.T, v[:,:nb]@s.B.T, acc[:,:nb]@s.B.T
    zu = q[:,nb:nb+len(s.labels)] if s.tire_dof else zr
    vu = v[:,nb:nb+len(s.labels)] if s.tire_dof else vr
    deformation, rate = body-zu, body_v-vu
    force = -(deformation@s.ks.T+rate@s.cs.T)
    signals, units, groups = {}, {}, {}

    def add(name, values, unit, group):
        signals[name] = np.asarray(values, dtype=float)
        units[name] = unit
        groups.setdefault(group, []).append(name)

    for i, label in enumerate(s.labels):
        add(f"Body {label}",body[:,i]*1000,"mm","位移")
        add(f"Road {label}",zr[:,i]*1000,"mm","位移")
        add(f"Travel {label}",-deformation[:,i]*1000,"mm","懸吊行程")
        add(f"Body velocity {label}",body_v[:,i],"m/s","速度")
        add(f"Body acceleration {label}",body_a[:,i],"m/s²","加速度")
        add(f"Suspension force {label}",force[:,i],"N","懸吊力")
        if s.tire_dof:
            add(f"Wheel {label}",zu[:,i]*1000,"mm","位移")
            add(f"Wheel velocity {label}",vu[:,i],"m/s","速度")
            add(f"Wheel acceleration {label}",acc[:,nb+i],"m/s²","加速度")
            load = s.static_load[i]-s.kt[i]*(zu[:,i]-zr[:,i])-s.ct[i]*(vu[:,i]-vr[:,i])
            add(f"Normal load {label}",load,"N","輪荷")
    family, model, p = s.family, s.model, s.p
    if family == "quarter":
        heave = q[:,0]
    elif family == "LR" and model in ("2mass", "2mass_2dof"):
        heave = np.mean(body, axis=1)
        add("Roll (derived)",np.rad2deg((body[:,0]-body[:,1])/p["track"]),"deg","車身姿態")
    else:
        heave = q[:,0]
        if family == "LR": add("Roll",np.rad2deg(q[:,1]),"deg","車身姿態")
        elif family == "FR": add("Pitch",np.rad2deg(q[:,1]),"deg","車身姿態")
        elif model == "rigid":
            add("Roll",np.rad2deg(q[:,1]),"deg","車身姿態")
            add("Pitch",np.rad2deg(q[:,2]),"deg","車身姿態")
        else:
            for name, values in (("Roll front",q[:,1]),("Roll rear",q[:,2]),("Twist",q[:,1]-q[:,2]),("Pitch",q[:,3])):
                add(name,np.rad2deg(values),"deg","車身姿態")
    add("CG heave",heave*1000,"mm","升沉")
    add("ax",[x[2] for x in inputs],"m/s²","輸入加速度")
    add("ay",[x[3] for x in inputs],"m/s²","輸入加速度")
    if family == "full":
        e, ev = deformation@s.T.T, rate@s.T.T
        for j,name in enumerate(CHANNEL_NAMES):
            add(f"Channel {name}",-(s.channel_k[j]*e[:,j]+s.channel_c[j]*ev[:,j]),"N","模態通道力")
    if model == "rollcenter_all":
        add("Frame deflection",q[:,-1]*1000,"mm","車架變形")
        add("Frame force",p["k_frame"]*q[:,-1]+p["c_frame"]*v[:,-1],"N","車架傳力")
    if model != "rollcenter_all":
        energy = .5*np.sum(v*v*s.masses,axis=1)+.5*np.einsum("ni,ij,nj->n",q,s.K,q)
        add("Mechanical energy",energy,"J","能量")
    return signals, units, groups


def steady_response(result):
    """Solve Kq=f for the final input and evaluate the normal output channels."""
    if result.legacy:
        return None
    family = result.family
    options = result.project["analysis"][family]
    system = assemble(family, result.model, result.project["parameters"][family])
    road = Road(system, options)
    terminal = road.terminal_input()
    if terminal is None or np.linalg.matrix_rank(system.K) < system.dof:
        return None
    ready, inputs = terminal
    force = system.input_force(*inputs)
    try:
        position = np.linalg.solve(system.K, force)
    except np.linalg.LinAlgError:
        return None
    state = np.concatenate((position, np.zeros(system.dof)))
    if not np.isfinite(state).all():
        return None
    signals, _, _ = result_signals(system, [inputs], state[None, :])
    # At a step starting at t=0 the first sample includes direct feedthrough.
    initial_jump = {}
    if options["scenario"] == "step" and options["start"] == result.time[0]:
        zero_road = np.zeros_like(road.mask)
        neutral = (zero_road, zero_road, 0., 0.)
        baseline, _, _ = result_signals(system, [neutral, road.at(result.time[0])], np.zeros((2, 2 * system.dof)))
        initial_jump = {name: float(values[1] - values[0]) for name, values in baseline.items()}
    return ready, {name: float(values[0]) for name, values in signals.items()}, initial_jump


def make_result(s, road, time, history, project, eigen, count, max_step):
    inputs = [road.at(float(t)) for t in time]
    signals, units, groups = result_signals(s, inputs, history)
    family, model = s.family, s.model
    warnings = []
    if any(np.min(values) < 0 for name,values in signals.items() if name.startswith("Normal load")):
        warnings.append("出現負輪荷：線性輪胎已超出接地假設，未啟用離地／再接觸求解。")
    if not s.tire_dof:
        warnings.append("此版本省略輪胎自由度；動畫輪位跟隨路面，不提供動態輪荷。")
    if min(abs(eigen)) < 1e-8:
        warnings.append("系統含零頻模態，部分方向沒有回復剛性。")
    frequencies = sorted(float(x.imag/(2*np.pi)) for x in eigen if x.imag > 1e-7)
    info = {"dof":s.dof,"internal_steps":count,"max_internal_dt":max_step,"damped_frequencies_Hz":frequencies,
            "warnings":warnings,"model_label":MODELS[family][model],"samples":len(time),"engine":"linear-work-conjugate-RK4-v1"}
    scenario = road.o["scenario"]
    if scenario in ISO_ROUGHNESS:
        info["iso_road"] = {
            "class": scenario[-1].upper(),
            "reference_psd_m3": ISO_ROUGHNESS[scenario],
            "reference_frequency_cycles_per_m": ISO_REFERENCE_FREQUENCY,
            "frequency_band_cycles_per_m": list(ISO_FREQUENCY_BAND),
            "speed_m_per_s": road.o["speed"],
            "distance_m": road.o["speed"] * (time[-1] - road.o["start"]),
        }
    snapshot = copy.deepcopy(project)
    snapshot["parameters"][family] = s.p
    if family in PARAMETER_SOURCES:
        source_key, _ = PARAMETER_SOURCES[family]
        if source_key in road.o:
            info.update(parameter_source="full", **{source_key: road.o[source_key]})
    return Result(family,model,time,signals,units,groups,snapshot,info)
