# ============================================================================================
#  RoboSim Control Center v9-0-dbg  Tool-Axis Motion + Fix Position + Deep Debug Log
# ============================================================================================

import tkinter as tk
from tkinter import ttk, filedialog
import socket, threading, time, math, json
import numpy as np
from dataclasses import dataclass
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from ik_rotosim import RotoSimIK, IKLimits

# World coordinates are in cm, UI step sizes are in mm
MM_TO_WORLD = 0.1  # 1 mm = 0.1 cm
UDP_HOST, UDP_PORT = "127.0.0.1", 9999
VIEW_LIMIT_XY = 70.0
VIEW_LIMIT_Z  = 95.0
GRIPPER_LEN   = 6.0
GRIPPER_MAX_GAP = 8.0   # 4 cm je Seite

# ------------------ Models ------------------
@dataclass
class Geometry:
    base_height: float = 24.0
    L1: float = 23.0
    L2: float = 25.0
    L_tool: float = 18.0

@dataclass
class Pose:
    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0
    A: float = 0.0   # Yaw
    B: float = 0.0   # Pitch
    C: float = 0.0   # Roll   NEU
    servo: float = 1000.0

# ------------------ Helpers ------------------
def _fmt(v, p=3):
    if isinstance(v, (list, tuple, np.ndarray)):
        return np.array2string(np.array(v, dtype=float), precision=p, suppress_small=False)
    return f"{float(v):.{p}f}"

def dir_to_yaw_tilt(t_world: np.ndarray):
    """Compute yaw/tilt (deg) from a world direction vector."""
    t = np.array(t_world, dtype=float)
    n = float(np.linalg.norm(t))
    if n < 1e-12:
        return 0.0, 0.0
    t /= n
    yaw = (math.degrees(math.atan2(t[1], t[0])) - 90.0) % 360.0
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, t[2]))))  # 0 = +Z
    return yaw, tilt

def rot_x(a):
    c,s = math.cos(a), math.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]],float)

def rot_y(a):
    c,s = math.cos(a), math.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]],float)

def rot_z(a):
    c,s = math.cos(a), math.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]],float)


def fk_points(geom: Geometry, pose: Pose):
    """
    Fully correct FK for Thor/6DOF:
      A = global Yaw (um Z)
      X = Schulter Pitch
      Y = Ellbogen Pitch
      Z = Wrist Pitch
      B = Wrist Roll
      C = Tool Roll
    """

    # Joint angles
    th_x = math.radians(pose.X)
    th_y = math.radians(pose.Y)
    th_z = math.radians(pose.Z)
    th_b = math.radians(pose.B)
    th_c = math.radians(pose.C)
    yaw  = math.radians(pose.A)

    # -----------------------------
    #  1) Basis + 1. Segment
    # -----------------------------
    P0 = np.array([0,0,0], float)
    P1 = np.array([0,0,geom.base_height], float)

    R1 = rot_y(th_x)                          # Schulter
    P2 = P1 + R1 @ np.array([0,0,geom.L1])

    # -----------------------------
    #  2) 2. Segment
    # -----------------------------
    R2 = R1 @ rot_y(th_y)                     # Ellbogen
    P3 = P2 + R2 @ np.array([0,0,geom.L2])

    # -----------------------------
    #  3) Wrist pitch (Z)
    # -----------------------------
    R3 = R2 @ rot_y(th_z)

    # -----------------------------
    #  4) Wrist roll (B)
    # -----------------------------
    R4 = R3 @ rot_z(th_b)

    # -----------------------------
    #  5) Tool roll (C)
    # -----------------------------
    R5 = R4 @ rot_z(th_c)

    # Toolvektor im lokalen Frame
    dir_local = R5 @ np.array([0,0,1], float)

    # Endpunkt
    P4 = P3 + dir_local * geom.L_tool

    # -----------------------------
    #  6) Globale Yaw-Rotation (A)
    # -----------------------------
    Rg = rot_z(yaw)

    pts = [Rg @ p for p in (P0, P1, P2, P3, P4)]
    dir_world = Rg @ dir_local

    return pts, dir_world


def fk_points_dh_rows(
    rows_cm_deg,
    pose: Pose,
    mirror_x: bool = False,
    joint_angle_offsets_deg=None,
    joint_angle_scales=None,
):
    """
    FK + joint points from DH chain.
    rows_cm_deg: list[{
        axis, alpha_deg, a_cm, d_cm, theta_offset_deg
    }]
    """
    if not rows_cm_deg:
        return fk_points(Geometry(), pose)

    joints_deg = {
        "A": float(pose.A),
        "X": float(pose.X),
        "Y": float(pose.Y),
        "Z": float(pose.Z),
        "B": float(pose.B),
        "C": float(pose.C),
    }

    def _T_dh(theta_deg, alpha_deg, a_cm, d_cm):
        th = math.radians(float(theta_deg))
        al = math.radians(float(alpha_deg))
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        return np.array(
            [
                [ct, -st * ca, st * sa, a_cm * ct],
                [st, ct * ca, -ct * sa, a_cm * st],
                [0.0, sa, ca, d_cm],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    sim_offsets = {}
    if isinstance(joint_angle_offsets_deg, dict):
        for raw_axis, raw_val in joint_angle_offsets_deg.items():
            axis = str(raw_axis).strip().upper()
            if not axis:
                continue
            try:
                sim_offsets[axis] = float(raw_val)
            except Exception:
                continue

    sim_scales = {}
    if isinstance(joint_angle_scales, dict):
        for raw_axis, raw_val in joint_angle_scales.items():
            axis = str(raw_axis).strip().upper()
            if not axis:
                continue
            try:
                sim_scales[axis] = float(raw_val)
            except Exception:
                continue

    T = np.eye(4, dtype=float)
    pts = [np.array([0.0, 0.0, 0.0], dtype=float)]
    for row in rows_cm_deg:
        axis = str(row.get("axis", "")).strip().upper()
        scale = float(sim_scales.get(axis, 1.0))
        theta = (
            float(joints_deg.get(axis, 0.0)) * scale
            + float(row.get("theta_offset_deg", 0.0))
            + float(sim_offsets.get(axis, 0.0))
        )
        Ti = _T_dh(theta, row.get("alpha_deg", 0.0), row.get("a_cm", 0.0), row.get("d_cm", 0.0))
        T = T @ Ti
        pts.append(np.array(T[:3, 3], dtype=float))

    dir_tool = np.array(T[:3, 2], dtype=float)
    n = np.linalg.norm(dir_tool)
    if n > 1e-12:
        dir_tool = dir_tool / n
    else:
        dir_tool = np.array([0.0, 0.0, 1.0], dtype=float)

    if mirror_x:
        pts = [np.array([-p[0], p[1], p[2]], dtype=float) for p in pts]
        dir_tool = np.array([-dir_tool[0], dir_tool[1], dir_tool[2]], dtype=float)

    return pts, dir_tool


def tcp_pose_from_points(pts, dir_tool):
    X, Y, Z = pts[-1]
    n = np.linalg.norm(dir_tool)
    if n < 1e-12:
        n = 1.0
    t = dir_tool / n
    tilt = math.degrees(math.acos(np.clip(np.dot(t, [0, 0, 1]), -1.0, 1.0)))
    yaw = math.degrees(math.atan2(Y, X))
    return round(float(X), 2), round(float(Y), 2), round(float(Z), 2), round(float(yaw), 2), round(float(tilt), 2)

def tcp_pose_5(geom, pose):
    pts, dir_tool = fk_points(geom, pose)
    return tcp_pose_from_points(pts, dir_tool)

# ------------------ UDP Listener ------------------
class UDPListener(threading.Thread):
    def __init__(self, cb):
        super().__init__(daemon=True)
        self.cb = cb
        self.start()
    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((UDP_HOST, UDP_PORT))
        while True:
            try:
                data,_=s.recvfrom(65535)
                line=data.decode("utf-8","replace").strip()
                if line:
                    self.cb(line)
            except Exception:
                # UDP noise ignorieren
                time.sleep(0.05)

# ------------------ Main GUI ------------------
class VisualizerApp(ttk.Frame):
    def __init__(self, master):
        super().__init__(master); self.pack(fill="both", expand=True)
        self.geom, self.pose = Geometry(), Pose()
        self.robot_profile_name = "Unknown"
        self.robot_joint_order = ["A", "X", "Y", "Z", "B", "C"]
        self.dh_rows_cm_deg = []
        self.post_transform = {"mirror_x": False, "sim_theta_offset_deg": {}, "sim_theta_scale": {}}
        self.ik = RotoSimIK(
            geom=self.geom,
            limits=IKLimits(
                A=(-180,180), X=(-120,120), Y=(-135,135), Z=(-180,180), B=(-180,180)
            ),
            prefer="down"
        )
        self.pose.servo = 1000.0
        self.gcode_lines = []
        self.path_fixed = []
        self.path_kin = []
        self.fixed_frame = None
        self._build_ui()
        UDPListener(self.on_udp)
        self.after(50, self.loop)

    def _apply_robot_profile(self, data: dict):
        try:
            geom_mm = data.get("geometry_mm", {}) if isinstance(data, dict) else {}
            base_cm = float(geom_mm.get("base_height_mm", self.geom.base_height * 10.0)) / 10.0
            l1_cm = float(geom_mm.get("L1_mm", self.geom.L1 * 10.0)) / 10.0
            l2_cm = float(geom_mm.get("L2_mm", self.geom.L2 * 10.0)) / 10.0
            tool_cm = float(geom_mm.get("L_tool_mm", self.geom.L_tool * 10.0)) / 10.0
            self.geom = Geometry(base_height=base_cm, L1=l1_cm, L2=l2_cm, L_tool=tool_cm)

            limits = data.get("limits_deg", {}) if isinstance(data, dict) else {}
            def _lim(ax, fallback):
                raw = limits.get(ax, fallback)
                if isinstance(raw, (list, tuple)) and len(raw) >= 2:
                    try:
                        return (float(raw[0]), float(raw[1]))
                    except Exception:
                        return fallback
                return fallback

            ik_limits = IKLimits(
                A=_lim("A", (-180.0, 180.0)),
                X=_lim("X", (-120.0, 120.0)),
                Y=_lim("Y", (-135.0, 135.0)),
                Z=_lim("Z", (-180.0, 180.0)),
                B=_lim("B", (-180.0, 180.0)),
            )
            self.ik = RotoSimIK(geom=self.geom, limits=ik_limits, prefer="down")

            raw_rows = data.get("dh_rows", []) if isinstance(data, dict) else []
            rows = []
            if isinstance(raw_rows, list):
                for row in raw_rows:
                    if not isinstance(row, dict):
                        continue
                    axis = str(row.get("axis", "")).strip().upper()
                    if not axis:
                        continue
                    try:
                        rows.append(
                            {
                                "axis": axis,
                                "alpha_deg": float(row.get("alpha_deg", 0.0)),
                                "a_cm": float(row.get("a_mm", 0.0)) * MM_TO_WORLD,
                                "d_cm": float(row.get("d_mm", 0.0)) * MM_TO_WORLD,
                                "theta_offset_deg": float(row.get("theta_offset_deg", 0.0)),
                            }
                        )
                    except Exception:
                        continue
            self.dh_rows_cm_deg = rows

            raw_post = data.get("post_transform", {}) if isinstance(data, dict) else {}
            post = {"mirror_x": False, "sim_theta_offset_deg": {}, "sim_theta_scale": {}}
            if isinstance(raw_post, dict):
                post["mirror_x"] = bool(raw_post.get("mirror_x", False))
                raw_sim = raw_post.get("sim_theta_offset_deg", {})
                if isinstance(raw_sim, dict):
                    clean_sim = {}
                    for raw_axis, raw_val in raw_sim.items():
                        axis = str(raw_axis).strip().upper()
                        if not axis:
                            continue
                        try:
                            clean_sim[axis] = float(raw_val)
                        except Exception:
                            continue
                    post["sim_theta_offset_deg"] = clean_sim
                raw_scale = raw_post.get("sim_theta_scale", {})
                if isinstance(raw_scale, dict):
                    clean_scale = {}
                    for raw_axis, raw_val in raw_scale.items():
                        axis = str(raw_axis).strip().upper()
                        if not axis:
                            continue
                        try:
                            clean_scale[axis] = float(raw_val)
                        except Exception:
                            continue
                    post["sim_theta_scale"] = clean_scale
            self.post_transform = post

            raw_order = data.get("joint_order", []) if isinstance(data, dict) else []
            order = []
            seen = set()
            if isinstance(raw_order, list):
                for raw_axis in raw_order:
                    axis = str(raw_axis).strip().upper()
                    if not axis or axis in seen:
                        continue
                    seen.add(axis)
                    order.append(axis)
            if order:
                self.robot_joint_order = order

            self.robot_profile_name = str(data.get("profile", self.robot_profile_name))
            self.log(
                f" Profil {self.robot_profile_name}: "
                f"base={self.geom.base_height:.2f}cm L1={self.geom.L1:.2f}cm "
                f"L2={self.geom.L2:.2f}cm Tool={self.geom.L_tool:.2f}cm"
            )
            if self.dh_rows_cm_deg:
                self.log(
                    f" DH chain active ({len(self.dh_rows_cm_deg)} joints): "
                    + " ".join(r['axis'] for r in self.dh_rows_cm_deg)
                )
        except Exception as e:
            self.log(f" Profil-Update fehlgeschlagen: {e}")

    def _current_fk(self):
        if self.dh_rows_cm_deg:
            return fk_points_dh_rows(
                self.dh_rows_cm_deg,
                self.pose,
                mirror_x=bool(self.post_transform.get("mirror_x")),
                joint_angle_offsets_deg=self.post_transform.get("sim_theta_offset_deg", {}),
                joint_angle_scales=self.post_transform.get("sim_theta_scale", {}),
            )
        return fk_points(self.geom, self.pose)

    def _current_tcp_pose(self):
        pts, dir_tool = self._current_fk()
        return tcp_pose_from_points(pts, dir_tool)

    # ------------------ UI ------------------
    def _build_ui(self):
        # ==========================================
        # Ansichten: YZ/XZ oben, XY/3D unten (kompakter)
        # ==========================================
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        # === UDP Receive Display ===
        self.udp_recv_var = tk.StringVar(value="UDP: ")
        ttk.Label(self, textvariable=self.udp_recv_var, font=("Consolas", 9)).pack(anchor="w", padx=6, pady=2)

        #  Reduced height: was (16, 9), now (16, 6.5)
        fig = Figure(figsize=(16, 6.5), dpi=100)

        # Top/bottom ratio slightly biased toward the lower row
        gs = fig.add_gridspec(
            2, 2,
            height_ratios=[0.8, 1.2],   # oben flacher, unten mehr Platz
            width_ratios=[1, 1.2]
        )

        # --- obere Reihe ---
        self.ax_yz = fig.add_subplot(gs[0, 0])
        self.ax_xz = fig.add_subplot(gs[0, 1])

        # --- untere Reihe ---
        self.ax_xy = fig.add_subplot(gs[1, 0])
        self.ax3d  = fig.add_subplot(gs[1, 1], projection="3d")

        # Aspect & Labels
        self.ax_yz.set_aspect(0.8)
        self.ax_xz.set_aspect(0.8)
        self.ax_xy.set_aspect("equal")

        for ax, title in [
            (self.ax_yz, "YZ View"),
            (self.ax_xz, "XZ View"),
            (self.ax_xy, "XY View")
        ]:
            ax.set_title(title)
            ax.grid(True, linestyle=":", color="#888", alpha=0.5)

        # Canvas einbetten
        self.canvas = FigureCanvasTkAgg(fig, master=self)
        self.canvas.draw()
        #  Less vertical expand so G-code area keeps enough space
        self.canvas.get_tk_widget().pack(fill="both", expand=False, padx=6, pady=4)

        # ==========================================
        # Bedienleiste darunter
        # ==========================================
        bar = ttk.Frame(self)
        bar.pack(fill="x", pady=4)

        ttk.Button(bar, text="Fix Position", command=self.fix_position).pack(side="left", padx=4)
        ttk.Label(bar, text="Tool (mm):").pack(side="left", padx=(10, 2))

        self.tool_step_var = tk.DoubleVar(value=10.0)
        ttk.Entry(bar, textvariable=self.tool_step_var, width=6).pack(side="left")

        ttk.Button(bar, text="+Tool", command=lambda: self.move_tool_axis(+self.tool_step_var.get())).pack(side="left", padx=4)
        ttk.Button(bar, text="Tool", command=lambda: self.move_tool_axis(-self.tool_step_var.get())).pack(side="left", padx=4)
        ttk.Button(bar, text="Clear Gcode", command=self.clear_gcode).pack(side="left", padx=4)
        ttk.Button(bar, text="Save Gcode", command=self.export_gcode).pack(side="left", padx=4)

        # ==========================================
        # Gcode + Log unten (bleibt sichtbar!)
        # ==========================================
        pan = ttk.PanedWindow(self, orient="vertical")
        pan.pack(fill="both", expand=True)
        self.gcode_text = tk.Text(pan, height=10, bg="#1e1e1e", fg="#00ff88", font=("Consolas", 9))
        self.log_text   = tk.Text(pan, height=8,  bg="#111",   fg="#ccc",    font=("Consolas", 9))
        pan.add(self.gcode_text)
        pan.add(self.log_text)

        # ==========================================
        # TCP-Zeile unten
        # ==========================================
        self.tcp_var = tk.StringVar(value="TCP  X=0 Y=0 Z=0 Yaw=0 Tilt=0")
        ttk.Label(self, textvariable=self.tcp_var, font=("Consolas", 10, "bold")).pack(anchor="w", padx=6, pady=3)

    # ------------------ Event handlers ------------------
    def on_udp(self, line):
        # Timestamp entfernen, falls vorhanden
        try:
            d = json.loads(line)
            if "timestamp" in d:
                d.pop("timestamp")
            clean = json.dumps(d)
            self.udp_recv_var.set(f"UDP: {clean}")
        except:
            self.udp_recv_var.set(f"UDP: {line}")
        try:
            data = json.loads(line)
            if data.get("type") == "path_fixed":
                pts = data.get("pts", [])
                parsed = []
                for p in pts:
                    if len(p) >= 4:
                        parsed.append((p[0] * MM_TO_WORLD, p[1] * MM_TO_WORLD, p[2] * MM_TO_WORLD, float(p[3])))
                    elif len(p) >= 3:
                        parsed.append((p[0] * MM_TO_WORLD, p[1] * MM_TO_WORLD, p[2] * MM_TO_WORLD, 0.0))
                self.path_fixed = parsed
                return
            if data.get("type") == "path_kin":
                pts = data.get("pts", [])
                parsed = []
                for p in pts:
                    if len(p) >= 4:
                        parsed.append((p[0] * MM_TO_WORLD, p[1] * MM_TO_WORLD, p[2] * MM_TO_WORLD, float(p[3])))
                    elif len(p) >= 3:
                        parsed.append((p[0] * MM_TO_WORLD, p[1] * MM_TO_WORLD, p[2] * MM_TO_WORLD, 0.0))
                self.path_kin = parsed
                return
            if data.get("type") == "fixed_frame":
                origin = data.get("origin", [0, 0, 0])
                x_axis = data.get("x", [1, 0, 0])
                y_axis = data.get("y", [0, 1, 0])
                z_axis = data.get("z", [0, 0, 1])
                self.fixed_frame = {
                    "origin": (origin[0] * MM_TO_WORLD, origin[1] * MM_TO_WORLD, origin[2] * MM_TO_WORLD),
                    "x": np.array(x_axis, dtype=float),
                    "y": np.array(y_axis, dtype=float),
                    "z": np.array(z_axis, dtype=float),
                }
                return
            if data.get("type") == "robot_profile":
                self._apply_robot_profile(data)
                return
            if data.get("type") == "abs":
                self.pose.X = float(data.get("X", 0))
                self.pose.Y = float(data.get("Y", 0))
                self.pose.Z = float(data.get("Z", 0))
                self.pose.A = float(data.get("A", 0))
                self.pose.B = float(data.get("B", 0))
                self.pose.C = float(data.get("C", 0))   # NEW
                if "S" in data:
                    self.pose.servo = float(data["S"])
        except Exception:
            pass  # no more logs for UDP noise

    def loop(self):
        self.redraw()
        self.after(50, self.loop)

    # ------------------ Tool-Axis Motion & Debug ------------------
    def fix_position(self):
        """Save current joint pose 1:1 from robot (G-code position)."""
        self.fixed_pose = {
            "A": self.pose.A,
            "X": self.pose.X,
            "Y": self.pose.Y,
            "Z": self.pose.Z,
            "B": self.pose.B
        }
        line = f"G1 A{self.pose.A:.3f} X{self.pose.X:.3f} Y{self.pose.Y:.3f} Z{self.pose.Z:.3f} B{self.pose.B:.3f} F6000"
        self.append_gcode(line)
        self.log(" Fixed position saved (directly from G-code)")
        self.log(line)

    def _workspace_probe(self, target_world: np.ndarray, t_world: np.ndarray, A_deg: float):
        """Replicate the 2R reachability check (logging only) to expose early root causes."""
        # in Vor-Yaw-Raum transformieren
        A = math.radians(A_deg)
        cy, sy = math.cos(A), math.sin(A)
        Rz_inv = np.array([[ cy, sy, 0],
                           [-sy, cy, 0],
                           [  0,  0, 1]])
        p4p = Rz_inv @ target_world
        tp  = Rz_inv @ (t_world / max(1e-12, np.linalg.norm(t_world)))
        p3p = p4p - self.geom.L_tool * tp
        yw = float(p3p[1])
        zw = float(p3p[2] - self.geom.base_height)
        L1, L2 = float(self.geom.L1), float(self.geom.L2)
        r2 = yw*yw + zw*zw
        cY = (r2 - L1*L1 - L2*L2) / (2.0*L1*L2)
        return cY, yw, zw

    def move_tool_axis(self, dist_mm: float):
        """Bewegt den TCP entlang der Toolachse (Welt), Orientierung gesperrt (A, B, Tilt)."""
        try:
            # Aktuelle TCP-Pos/Toolrichtung
            pts, dir_world = self._current_fk()
            p_tip = np.array(pts[-1], dtype=float)                      # cm
            t_world = dir_world / max(1e-12, np.linalg.norm(dir_world)) # Einheit

            # Schritt in cm
            dist_world = dist_mm * MM_TO_WORLD
            new_p_world = p_tip + t_world * dist_world
            Xn, Yn, Zn = map(float, new_p_world)

            # Lock orientation: A/B from pose, tilt from current tool direction
            # (Tilt = angle against +Z; robustly extracted from FK)
            _, _, _, _, tilt_now = self._current_tcp_pose()
            A_lock = float(self.pose.A)
            B_lock = float(self.pose.B)
            # --- NEW: relax tilt automatically if roll  0 ---
            tilt_w = 0.25 if abs(B_lock) > 1e-3 else 1.0

            # Debug
            self.log("=== TOOL AXIS MOVE DEBUG ===")
            self.log(f"Tool (mm)  : {dist_mm:+.3f}    (cm): {dist_world:+.3f}")
            self.log(f"Pose (deg)  : A={self.pose.A:.3f}  X={self.pose.X:.3f}  Y={self.pose.Y:.3f}  Z={self.pose.Z:.3f}  B={self.pose.B:.3f}")
            self.log(f"TCP world   : { _fmt(p_tip) }")
            self.log(f"Tool world  : { _fmt(t_world) }")
            self.log(f"new_p_world : { _fmt(new_p_world) }")
            self.log(f"Locks       : A={A_lock:.3f}, B={B_lock:.3f}, Tilt={tilt_now:.3f} (fix)")

            # 1. Versuch: harte Locks
            sol = self.ik.solve_from_pose_locked(
                Xn, Yn, Zn,
                Yawd=A_lock,         # Yaw = aktuelles A (fix)
                Tiltd=tilt_now,      # Tilt fix
                lock_A_deg=A_lock,
                lock_B_deg=B_lock,
                max_iters=300, pos_tol_mm=0.3, ang_tol_deg=0.6, lambd=3.0,
                debug=True, logger=self.log
            )

            # 2. Versuch (Fallback): A freigeben, B/ Tilt bleiben fix
            if not sol:
                self.log("... fallback: unlock A (B+Tilt bleiben fix)")
                sol = self.ik.solve_from_pose_locked(
                    Xn, Yn, Zn,
                    Yawd=self.pose.A,   # initial value
                    Tiltd=tilt_now,
                    lock_A_deg=None,    # A frei
                    lock_B_deg=B_lock,  # B fix
                    max_iters=350, pos_tol_mm=0.3, ang_tol_deg=0.6, lambd=4.0,
                    debug=True, logger=self.log
                )

            if not sol:
                self.log(" IK: no solution returned.")
                self.log("==============================")
                return

            # Apply pose
            self.pose.A = sol["A"]; self.pose.X = sol["X"]; self.pose.Y = sol["Y"]; self.pose.Z = sol["Z"]; self.pose.B = sol["B"]

            # === G-code from joint angles ===
            gline = (
                f"G1 A{sol['A']:.3f} "
                f"X{sol['X']:.3f} "
                f"Y{sol['Y']:.3f} "
                f"Z{sol['Z']:.3f} "
                f"B{sol['B']:.3f} F6000"
            )
            self.append_gcode(gline)
            self.log(f" Tool Move (joints) -> {gline}")
            self.log("==============================")

        except Exception as e:
            import traceback
            self.log(f" Ausnahme in move_tool_axis: {e}")
            for line in traceback.format_exc().splitlines():
                self.log(line)

    def retreat_to_fix(self):
        """Return from current TCP pose to saved fixed position (G-code output)."""
        if not hasattr(self, "fixed_pose"):
            self.log(" No fixed position saved.")
            return
        pos = self.fixed_pose["pos"]
        A, B = self.fixed_pose["A"], self.fixed_pose["B"]
        X, Y, Z = pos
        line = f"G1 X{X:.3f} Y{Y:.3f} Z{Z:.3f} A{A:.3f} B{B:.3f} F6000"
        self.append_gcode(line)
        self.log(f" Retreat  Fix-Position\n{line}")

    def add_gripper(self, state: str):
        """Insert gripper action (open/close)."""
        line = "M3 S1000" if state == "open" else "M5 S0"
        self.append_gcode(line)
        self.log(f" Gripper {state.upper()}  {line}")

    def add_delay(self, ms: int):
        """Insert delay as G4."""
        sec = ms / 1000.0
        line = f"G4 S{sec:.3f}"
        self.append_gcode(line)
        self.log(f" Delay {sec:.3f}s  {line}")

    # ------------------ Gcode / Log ------------------
    def clear_gcode(self):
        self.gcode_text.delete("1.0", tk.END)
        self.gcode_lines.clear()
        self.log(" G-code cleared")

    def append_gcode(self, line):
        self.gcode_lines.append(line + "\n")
        self.gcode_text.insert(tk.END, line + "\n")
        self.gcode_text.see(tk.END)

    def export_gcode(self):
        if not self.gcode_lines:
            self.log(" No G-code to save")
            return
        path = filedialog.asksaveasfilename(title="Save G-code", defaultextension=".gcode",
                                            filetypes=[("G-code","*.gcode"),("All files","*.*")])
        if not path: return
        with open(path,"w",encoding="utf-8") as f:
            f.writelines(self.gcode_lines)
        self.log(f" G-code saved: {path}")

    def log(self, msg:str):
        self.log_text.insert(tk.END, time.strftime("[%H:%M:%S] ") + str(msg) + "\n")
        self.log_text.see(tk.END)

    # ------------------ Draw ------------------
    def redraw(self):
        pts, dir_tool = self._current_fk()
        X, Y, Z, Yaw, Tilt = self._current_tcp_pose()
        self.tcp_var.set(f"TCP  X={X:.2f}  Y={Y:.2f}  Z={Z:.2f}  Yaw={Yaw:.2f}  Tilt={Tilt:.2f}")

        self.ax3d.clear()
        for i in range(len(pts) - 1):
            p1, p2 = pts[i], pts[i + 1]
            if i == 0:
                col = "#777777"
            elif i == (len(pts) - 2):
                col = "#cc3333"
            else:
                col = "#00aa55"
            self.ax3d.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                           "o-", color=col, lw=2)

        # === Axes ===
        L = 20
        self.ax3d.quiver(0, 0, 0, L, 0, 0, color="r")
        self.ax3d.quiver(0, 0, 0, 0, L, 0, color="g")
        self.ax3d.quiver(0, 0, 0, 0, 0, L, color="b")

        # === Tool-Vektor (rot) ===
        p_tcp = np.array(pts[-1])
        t = dir_tool / np.linalg.norm(dir_tool)
        self.ax3d.quiver(p_tcp[0], p_tcp[1], p_tcp[2],
                         t[0]*10, t[1]*10, t[2]*10, color="red")

        self._draw_path_3d(self.ax3d, self.path_kin, "#20c050")
        self._draw_path_3d(self.ax3d, self.path_fixed, "#1a9850")
        self._draw_fixed_frame_3d(self.ax3d)

        # === 2D-Ansichten im Weltkoordinatensystem ===
        pts_2d = [np.array(p, dtype=float) for p in pts]
        path_kin_2d = [(p[0], p[1], p[2]) for p in self.path_kin]
        path_fixed_2d = [(p[0], p[1], p[2]) for p in self.path_fixed]
        fixed_frame_2d = self.fixed_frame


        # === 2D-Ansichten des Arms ===
        # XY = Draufsicht
        self._draw_2d(self.ax_xy, pts_2d, (0, 1), "XY")
        self._draw_path_2d(self.ax_xy, path_kin_2d, (0, 1), "#20c050")
        self._draw_path_2d(self.ax_xy, path_fixed_2d, (0, 1), "#1a9850")
        self._draw_fixed_frame_2d(self.ax_xy, (0, 1), fixed_frame_2d)

        # YZ = Seitenansicht
        self._draw_2d(self.ax_yz, pts_2d, (1, 2), "YZ")
        self._draw_path_2d(self.ax_yz, path_kin_2d, (1, 2), "#20c050")
        self._draw_path_2d(self.ax_yz, path_fixed_2d, (1, 2), "#1a9850")
        self._draw_fixed_frame_2d(self.ax_yz, (1, 2), fixed_frame_2d)

        # XZ = Frontansicht
        self._draw_2d(self.ax_xz, pts_2d, (0, 2), "XZ")
        self._draw_path_2d(self.ax_xz, path_kin_2d, (0, 2), "#20c050")
        self._draw_path_2d(self.ax_xz, path_fixed_2d, (0, 2), "#1a9850")
        self._draw_fixed_frame_2d(self.ax_xz, (0, 2), fixed_frame_2d)


        # === Gripper (two parallel lines, rolls with A + B + C around tool axis) ===
        # === Greifer (Backen korrekt stabilisiert & alle Rotationen A/B/C integriert) ===
        try:
            servo_val = float(np.clip(self.pose.servo, 0.0, 1000.0))
            half_gap = np.interp(servo_val, [0.0, 1000.0],
                                 [GRIPPER_MAX_GAP / 2.0, 0.0])
            line_len = GRIPPER_LEN * 1.5

            P3 = np.array(pts[-2], dtype=float)
            P4 = np.array(pts[-1], dtype=float)
            p_tcp = P4

            # Toolrichtung
            t = P4 - P3
            t /= np.linalg.norm(t) + 1e-12

            # ---------------------------------------------------------
            # STABLE reference vector (singularity-safe)
            # ---------------------------------------------------------
            up = np.array([0.0, 0.0, 1.0])
            v0 = np.cross(up, t)

            # Fallback when tool ~ Z-axis -> use second reference vector
            if np.linalg.norm(v0) < 1e-6:
                up2 = np.array([1.0, 0.0, 0.0])
                v0 = np.cross(up2, t)

            v0 /= np.linalg.norm(v0) + 1e-12

            # ---------------------------------------------------------
            # OPTIONAL: Vorverdrehung um +90 im XY-Plane (dein Wunsch)
            # ---------------------------------------------------------
            def rotZ(v, ang):
                c = math.cos(ang)
                s = math.sin(ang)
                return np.array([c*v[0]-s*v[1], s*v[0]+c*v[1], v[2]])

            v0 = rotZ(v0, math.radians(90.0))

            # ---------------------------------------------------------
            # Wrist Roll B + Tool Roll C um die Toolachse
            # ---------------------------------------------------------
            def rot_axis(v, axis, ang):
                axis = axis / (np.linalg.norm(axis) + 1e-12)
                c = math.cos(ang)
                s = math.sin(ang)
                return v*c + np.cross(axis, v)*s + axis*np.dot(axis, v)*(1-c)

            v_side = v0
            # Achtung: B rollt "gegen" die Richtung, C normal
            v_side = rot_axis(v_side, t, -math.radians(self.pose.B))
            v_side = rot_axis(v_side, t,  math.radians(self.pose.C))

            # ---------------------------------------------------------
            # Greiferlinien errechnen
            # ---------------------------------------------------------
            pL0 = p_tcp - v_side * half_gap
            pL1 = pL0 + t * line_len
            pR0 = p_tcp + v_side * half_gap
            pR1 = pR0 + t * line_len

            # ---------------------------------------------------------
            # 3D Zeichnen
            # ---------------------------------------------------------
            for p1, p2 in ((pL0, pL1), (pR0, pR1)):
                self.ax3d.plot([p1[0], p2[0]],
                               [p1[1], p2[1]],
                               [p1[2], p2[2]], color="#00aaff", lw=5)

            # ---------------------------------------------------------
            # 2D Projektionen
            # ---------------------------------------------------------
            lw2d = 3.0
            for ax, (i, j) in (
                (self.ax_xy, (0, 1)),
                (self.ax_yz, (1, 2)),
                (self.ax_xz, (0, 2)),
            ):
                ax.plot([pL0[i], pL1[i]], [pL0[j], pL1[j]], color="#00aaff", lw=lw2d)
                ax.plot([pR0[i], pR1[i]], [pR0[j], pR1[j]], color="#00aaff", lw=lw2d)

        except Exception as e:
            print(" Greifer draw error:", e)





        # === Grenzen & Titel ===
        self.ax3d.set_xlim(-VIEW_LIMIT_XY, VIEW_LIMIT_XY)
        self.ax3d.set_ylim(-VIEW_LIMIT_XY, VIEW_LIMIT_XY)
        self.ax3d.set_zlim(0, VIEW_LIMIT_Z)
        self.ax3d.set_box_aspect([1, 1, VIEW_LIMIT_Z / VIEW_LIMIT_XY])
        self.ax3d.set_title("3D View")
        self.canvas.draw()

    def _draw_2d(self, ax, pts, idx, title):
        ax.clear()
        xs = [p[idx[0]] for p in pts]
        ys = [p[idx[1]] for p in pts]
        ax.plot(xs, ys, 'o-', lw=2)

        # Dynamische Bereiche pro Ansicht
        if title == "XY":
            ax.set_xlim(-80, 80)
            ax.set_ylim(-80, 80)
        elif title in ("YZ", "XZ"):
            ax.set_xlim(-80, 80)
            ax.set_ylim(0, 100)

        ax.set_aspect('equal')
        ax.set_title(title)
        ax.grid(True, linestyle=":", color="#999", alpha=0.4)

    def _draw_path_2d(self, ax, pts, idx, color):
        if not pts or len(pts) < 2:
            return
        xs = [p[idx[0]] for p in pts]
        ys = [p[idx[1]] for p in pts]
        ax.plot(xs, ys, '-', color=color, lw=2)
        ax.annotate(
            "",
            xy=(xs[-1], ys[-1]),
            xytext=(xs[-2], ys[-2]),
            arrowprops=dict(arrowstyle="->", color=color, lw=2),
        )

    def _draw_path_3d(self, ax, pts, color):
        if not pts or len(pts) < 2:
            return
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        zs = [p[2] for p in pts]
        ax.plot(xs, ys, zs, color=color, lw=2)
        dx = xs[-1] - xs[-2]
        dy = ys[-1] - ys[-2]
        dz = zs[-1] - zs[-2]
        ax.quiver(xs[-2], ys[-2], zs[-2], dx, dy, dz, color=color, linewidths=2)


    def _draw_fixed_frame_3d(self, ax, frame=None):
        if frame is None:
            frame = self.fixed_frame
        if not frame:
            return
        o = np.array(frame["origin"], dtype=float)
        x = np.array(frame["x"], dtype=float)
        y = np.array(frame["y"], dtype=float)
        z = np.array(frame["z"], dtype=float)
        for v in (x, y, z):
            n = np.linalg.norm(v)
            if n > 1e-9:
                v /= n
        L = 8.0
        ax.quiver(o[0], o[1], o[2], x[0]*L, x[1]*L, x[2]*L, color="#cc3333")
        ax.quiver(o[0], o[1], o[2], y[0]*L, y[1]*L, y[2]*L, color="#33aa33")
        ax.quiver(o[0], o[1], o[2], z[0]*L, z[1]*L, z[2]*L, color="#3366cc")

    def _draw_fixed_frame_2d(self, ax, idx, frame=None):
        if frame is None:
            frame = self.fixed_frame
        if not frame:
            return
        o = np.array(frame["origin"], dtype=float)
        x = np.array(frame["x"], dtype=float)
        y = np.array(frame["y"], dtype=float)
        z = np.array(frame["z"], dtype=float)
        for v in (x, y, z):
            n = np.linalg.norm(v)
            if n > 1e-9:
                v /= n
        L = 8.0
        for v, color in ((x, "#cc3333"), (y, "#33aa33"), (z, "#3366cc")):
            p0 = (o[idx[0]], o[idx[1]])
            p1 = (o[idx[0]] + v[idx[0]] * L, o[idx[1]] + v[idx[1]] * L)
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=color, lw=2)

# ------------------ Main ------------------
if __name__ == "__main__":
    root = tk.Tk()
    root.title("RoboSim Control Center v8.3")
    root.attributes("-topmost", True)
    root.geometry("750x550+1150+70") 
    app = VisualizerApp(root)
    root.mainloop()
