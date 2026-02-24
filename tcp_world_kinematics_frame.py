# tcp_kinematics_tabs.py
# =============================================================
# New TCP-based Kinematics / Sequence Generator
# Fully replacing world_kinematics_frame_v3
#
# Uses ONLY:
#   - TcpPosePanel (live TCP in mm/deg)
#   - Simple IK6 (Moveo-like, XYZ+Pitch+Roll)
# =============================================================

import tkinter as tk
from tkinter import ttk, messagebox
import math
import json
import os
from tcp_pose_module_v3 import (
    get_dh_model,
    get_dh_rows_mm_deg,
    get_dh_axes,
    get_post_transform,
    apply_joint_angle_post_transform,
    reload_dh_model,
)

# We reuse the same axis naming convention (joint letters):
AXES = ["A", "X", "Y", "B", "Z", "C"]


# =============================================================
#  IK6 SOLVER (minimal, Moveo-optimized)
# =============================================================

class IK6:
    def __init__(self, geom):
        self.L1 = geom["L1"]
        self.L2 = geom["L2"]
        self.L3 = geom["L3"]
        self.L4 = geom["L4"]

    @staticmethod
    def _wrap180(a):
        return ((a + 180.0) % 360.0) - 180.0

    def solve_xyz(self, X, Y, Z, pitch_deg=0.0, roll_deg=0.0):
        """
        Pure XYZ IK similar to your FK:
          - A = atan2(Y, X)
          - Planar IK in (r,Z) for shoulder+elbow

        pitch_deg: "Tilt" of the tool (Moveo-Konvention)
        roll_deg : wrist roll
        """
        if abs(X) + abs(Y) < 1e-6:
            A = 0.0
        else:
            A = math.degrees(math.atan2(Y, X))

        r = math.sqrt(X * X + Y * Y)
        phi = math.radians(180.0 - pitch_deg)
        xw = r - self.L4 * math.sin(phi)
        zw = Z - self.L1 - self.L4 * math.cos(phi)

        r2 = xw * xw + zw * zw
        L2, L3 = self.L2, self.L3
        c2 = max(-1.0, min(1.0, (r2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)))
        s2 = math.sqrt(max(0.0, 1.0 - c2 * c2))
        q2 = math.atan2(s2, c2)
        q1 = math.atan2(xw, zw) - math.atan2(L3 * s2, L2 + L3 * c2)

        # Wrist
        q3 = (math.pi - math.radians(pitch_deg)) - (q1 + q2)

        return {
            "A": -A,
            "X": math.degrees(q1),
            "Y": math.degrees(q2),
            "Z": math.degrees(q3),
            "B": IK6._wrap180(roll_deg + 180.0),
            "C": 0.0,
        }


# =============================================================
# MAIN TAB: TCP-COORDINATES
# =============================================================

class TcpKinematicsFrame(ttk.Frame):
    """
    Main UI tab:
       Uses TCP pose inputs (mask) in mm / deg
       "Pose -> Mask" copies live TCP from ExecuteApp
       User defines:
           - Retreat (D_ret)
           - Work depth (D_work)
           - Feed
           - Pitch override (deg)
           - Roll override (deg)
           - Anchor: Target / Zero / Retreat
           - Invert tool direction
           - Gripper with pauses before/after
           - Optional pre/post G-Code
       Generates a RET  TAR  RET sequence in joint space
       Uses IK6 to obtain joints for each position
       Full G-code preview with limit highlight
    """

    def __init__(self, master, execute_app, client):
        super().__init__(master)
        self.exec = execute_app
        self.client = client

        # Geometry from DH table:
        self.geom = self.exec.GEOM_DH  # must contain L1..L4
        self.ik = IK6(self.geom)

        # ---------- Parameters ----------
        self.D_ret = tk.DoubleVar(value=20.0)
        self.D_work = tk.DoubleVar(value=30.0)
        self.feed = tk.DoubleVar(value=3000.0)

        # Scale orientation error (rad) to mm for DLS balancing
        self.rot_weight_mm = 100.0

        # Overrides for IK (Pitch / Roll)
        self.pitch_override = tk.DoubleVar(value=0.0)   # Pitch (deg)
        self.roll_override = tk.DoubleVar(value=0.0)    # Roll (deg)

        # Anchor mode: zero / target / retreat
        # Default "retreat" avoids an unnecessary first positioning move when
        # the current TCP is copied into the mask and then executed directly.
        self.anchor_mode = tk.StringVar(value="retreat")

        self.invert_u = tk.BooleanVar(value=False)

        # Gripper + pauses
        self.use_gripper = tk.BooleanVar(value=False)
        self.grip_s = tk.IntVar(value=1000)
        self.grip_pause_before_ms = tk.IntVar(value=0)
        self.grip_pause_after_ms = tk.IntVar(value=0)

        # Pre/Post-GCode
        self.use_prepost = tk.BooleanVar(value=False)

        # TCP pose inputs (X,Y,Z,Roll,Pitch,Yaw) as StringVar (for formatting)
        self.tcp_x = tk.StringVar(value="0.00")
        self.tcp_y = tk.StringVar(value="0.00")
        self.tcp_z = tk.StringVar(value="0.00")
        self.tcp_roll = tk.StringVar(value="0.00")
        self.tcp_pitch = tk.StringVar(value="0.00")   # statt Tilt
        self.tcp_yaw = tk.StringVar(value="0.00")

        # ---------- UI ----------
        self._build_ui()

    # =========================================================
    # UI BUILD
    # =========================================================
    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        # =========================================================
        #  4-SPALTEN LAYOUT  OBERER BEREICH
        # =========================================================
        top = ttk.Frame(self)
        top.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)
        top.columnconfigure(0, weight=1)
        top.columnconfigure(1, weight=1)
        top.columnconfigure(2, weight=1)
        top.columnconfigure(3, weight=1)

        # =========================================================
        #  SPALTE 1  TCP POSE INPUT MASKE
        # =========================================================
        col_pose = ttk.LabelFrame(top, text="TCP Pose Input (mm / deg)")
        col_pose.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

        pose_fields = [
            ("X [mm]", self.tcp_x),
            ("Y [mm]", self.tcp_y),
            ("Z [mm]", self.tcp_z),
            ("Roll [deg]", self.tcp_roll),
            ("Pitch [deg]", self.tcp_pitch),
            ("Yaw [deg]", self.tcp_yaw),
        ]

        for i, (label, var) in enumerate(pose_fields):
            ttk.Label(col_pose, text=label).grid(
                row=i, column=0, sticky="e", padx=4, pady=1
            )
            e = ttk.Entry(col_pose, textvariable=var, width=10, justify="right")
            e.grid(row=i, column=1, sticky="w", padx=4, pady=1)

        # --- TCP -> Maske (Weltkoordinaten) + optional Solve&Execute ---

        def _fmt4(v):
            try:
                return f"{float(v):.4f}"
            except Exception:
                return "0.0000"

        def _pose_to_mask_only():
            """
            (1) Transfer only TCP pose -> mask (world coordinates / mm+deg)
            """
            try:
                tcp = self.exec.get_current_tcp_mm()
            except Exception:
                tcp = None
            if not tcp:
                return False

            self.tcp_x.set(_fmt4(tcp.get("X_mm", 0.0)))
            self.tcp_y.set(_fmt4(tcp.get("Y_mm", 0.0)))
            self.tcp_z.set(_fmt4(tcp.get("Z_mm", 0.0)))

            roll_deg = tcp.get("Roll_deg", 0.0)
            pitch_deg = tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0))
            yaw_deg = tcp.get("Yaw_deg", 0.0)

            self.tcp_roll.set(_fmt4(roll_deg))
            self.tcp_pitch.set(_fmt4(pitch_deg))
            self.tcp_yaw.set(_fmt4(yaw_deg))
            return True

        def _pose_to_mask_and_solve_execute():
            """
            (2) Transfer TCP pose -> mask, solve sequence (Generate), and execute immediately
            """
            ok = _pose_to_mask_only()
            if not ok:
                return
            # Sequenz generieren (IK solve)
            if hasattr(self, "_run"):
                self._run()
            # execute immediately
            if hasattr(self, "_execute_direct"):
                self._execute_direct()

        def _pose_to_mask_and_generate():
            """
            (3) Transfer TCP pose -> mask, solve sequence (Generate) without execution
            """
            ok = _pose_to_mask_only()
            if not ok:
                return
            if hasattr(self, "_run"):
                self._run()

        # Button: transfer only (your normal GUI button)
        ttk.Button(
            col_pose,
            text=" Pose  Maske",
            command=_pose_to_mask_only,
        ).grid(row=len(pose_fields), column=0, columnspan=2, pady=6)

        # Expose both functions as public methods (for gamepad proxies)
        self.pose_to_mask_only = _pose_to_mask_only
        self.pose_to_mask_and_solve_execute = _pose_to_mask_and_solve_execute
        self.pose_to_mask_and_generate = _pose_to_mask_and_generate


        # =========================================================
        #  SPALTE 2  PARAMETER & ANCHOR
        # =========================================================
        col_params = ttk.LabelFrame(top, text="Parameters")
        col_params.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)

        params = [
            ("Retreat D_ret [mm]", self.D_ret),
            ("Work D_work [mm]", self.D_work),
            ("Feed [mm/min]", self.feed),
            ("Pitch override [deg]", self.pitch_override),
            ("Roll override [deg]", self.roll_override),
        ]

        for i, (label, var) in enumerate(params):
            ttk.Label(col_params, text=label).grid(
                row=i, column=0, sticky="e", padx=4, pady=2
            )
            ttk.Entry(col_params, textvariable=var, width=12, justify="right").grid(
                row=i, column=1, sticky="w", padx=4, pady=2
            )

        ttk.Checkbutton(
            col_params,
            text="Invert tool axis",
            variable=self.invert_u,
        ).grid(row=len(params), column=0, columnspan=2, sticky="w", padx=4, pady=4)

        # Anchor-Auswahl
        anchor_frame = ttk.LabelFrame(col_params, text="Anchor (mask is ...)")
        anchor_frame.grid(
            row=len(params) + 1,
            column=0,
            columnspan=2,
            sticky="ew",
            padx=2,
            pady=(4, 2),
        )

        ttk.Radiobutton(
            anchor_frame, text="Target", value="target", variable=self.anchor_mode
        ).pack(side=tk.LEFT, padx=3)
        ttk.Radiobutton(
            anchor_frame, text="Zero", value="zero", variable=self.anchor_mode
        ).pack(side=tk.LEFT, padx=3)
        ttk.Radiobutton(
            anchor_frame, text="Retreat", value="retreat", variable=self.anchor_mode
        ).pack(side=tk.LEFT, padx=3)

        # =========================================================
        #  SPALTE 3  GRIPPER / SEQUENZ-OPTIONEN
        # =========================================================
        col_grip = ttk.LabelFrame(top, text="Gripper / Sequence")
        col_grip.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)

        ttk.Checkbutton(
            col_grip,
            text="Use gripper",
            variable=self.use_gripper,
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=4, pady=2)

        ttk.Label(col_grip, text="S-value").grid(
            row=1, column=0, sticky="e", padx=4, pady=2
        )
        ttk.Entry(col_grip, textvariable=self.grip_s, width=10, justify="right").grid(
            row=1, column=1, sticky="w", padx=4, pady=2
        )

        ttk.Label(col_grip, text="Pause before [ms]").grid(
            row=2, column=0, sticky="e", padx=4, pady=2
        )
        ttk.Entry(
            col_grip,
            textvariable=self.grip_pause_before_ms,
            width=10,
            justify="right",
        ).grid(row=2, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(col_grip, text="Pause after [ms]").grid(
            row=3, column=0, sticky="e", padx=4, pady=2
        )
        ttk.Entry(
            col_grip,
            textvariable=self.grip_pause_after_ms,
            width=10,
            justify="right",
        ).grid(row=3, column=1, sticky="w", padx=4, pady=2)

        # =========================================================
        #  SPALTE 4  DH PARAMETERS (Moveo)
        # =========================================================
        col_dh = ttk.LabelFrame(top, text="DH Parameters (deg / mm)")
        col_dh.grid(row=0, column=3, sticky="nsew", padx=4, pady=4)

        dh_headers = ["Joint", "Alpha", "A", "D", "ThetaOff"]
        for j, lbl in enumerate(dh_headers):
            ttk.Label(col_dh, text=lbl).grid(row=0, column=j, padx=2, pady=1)

        dh_defaults = self._get_dh_defaults()
        self._refresh_profile_post_transform()

        self.dh_vars = []
        for i, (alpha, a, d, theta_off) in enumerate(dh_defaults, start=1):
            ttk.Label(col_dh, text=f"j{i}").grid(row=i, column=0, padx=2, pady=1)
            v_alpha = tk.StringVar(value=f"{alpha:.3f}")
            v_a = tk.StringVar(value=f"{a:.3f}")
            v_d = tk.StringVar(value=f"{d:.3f}")
            v_t = tk.StringVar(value=f"{theta_off:.3f}")
            ttk.Entry(col_dh, textvariable=v_alpha, width=7, justify="right").grid(
                row=i, column=1, padx=1, pady=1
            )
            ttk.Entry(col_dh, textvariable=v_a, width=7, justify="right").grid(
                row=i, column=2, padx=1, pady=1
            )
            ttk.Entry(col_dh, textvariable=v_d, width=7, justify="right").grid(
                row=i, column=3, padx=1, pady=1
            )
            ttk.Entry(col_dh, textvariable=v_t, width=7, justify="right").grid(
                row=i, column=4, padx=1, pady=1
            )
            self.dh_vars.append((v_alpha, v_a, v_d, v_t))

        # =========================================================
        # BUTTONS UNTER DEN VIER SPALTEN
        # =========================================================
        btn_frame = ttk.Frame(top)
        btn_frame.grid(row=1, column=0, columnspan=4, pady=6)

        ttk.Button(btn_frame, text="Generate", command=self._run).pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(btn_frame, text="Save DH", command=self._save_dh_to_json).pack(
            side=tk.LEFT, padx=5
        )
        ttk.Button(
            btn_frame, text="Send to CLI", command=self._send_preview_to_cli
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            btn_frame, text="Execute", command=self._execute_direct
        ).pack(side=tk.LEFT, padx=5)

        ttk.Button(
            btn_frame, text="Send to Queue", command=self._send_preview_to_queue
        ).pack(side=tk.LEFT, padx=5)

        # =========================================================
        # PREVIEW
        # =========================================================
        preview = ttk.LabelFrame(self, text="G-code preview")
        preview.grid(row=1, column=0, sticky="nsew", padx=4, pady=4)
        preview.columnconfigure(0, weight=1)
        preview.rowconfigure(0, weight=1)

        self.txt = tk.Text(preview, height=4, wrap="none")
        self.txt.grid(row=0, column=0, sticky="nsew")
        sb = ttk.Scrollbar(preview, orient="vertical", command=self.txt.yview)
        sb.grid(row=0, column=1, sticky="ns")
        self.txt.configure(yscrollcommand=sb.set)

        self.txt.tag_configure("ok", background="#e8ffe8")
        self.txt.tag_configure("bad_val", background="#ffd0d0")
        self.txt.tag_configure("plain", background="white")

    # =========================================================
    # LOW-LEVEL HELPERS
    # =========================================================

    def _refresh_profile_post_transform(self):
        post = get_post_transform()
        self._mirror_x = bool(post.get("mirror_x"))

    @staticmethod
    def _dh_axes():
        axes = get_dh_axes()
        if not axes:
            return ["A", "X", "Y", "B", "Z", "C"]
        return list(axes)

    def _get_dh_defaults(self):
        # Defaults from active DH model (deg / mm)
        dh_rows = get_dh_rows_mm_deg()
        if len(dh_rows) == 6:
            return [
                (r["alpha_deg"], r["a_mm"], r["d_mm"], r["theta_offset_deg"]) for r in dh_rows
            ]
        L1 = float(self.geom.get("L1", 240.0))
        L2 = float(self.geom.get("L2", 230.0))
        L3 = float(self.geom.get("L3", 250.0))
        L4 = float(self.geom.get("L4", 180.0))
        return [
            (90.0,  0.0, L1, 0.0),   # j1
            (0.0,   L2,  0.0, 90.0), # j2
            (-90.0, 0.0, 0.0, -90.0),# j3
            (90.0,  0.0, L3, 0.0),   # j4
            (-90.0, 0.0, 0.0, 0.0),  # j5
            (0.0,   0.0, L4, 0.0),   # j6
        ]

    def refresh_dh_table(self):
        if not hasattr(self, "dh_vars"):
            return
        try:
            if hasattr(self.exec, "GEOM_DH"):
                self.geom = self.exec.GEOM_DH
        except Exception:
            pass
        self._refresh_profile_post_transform()
        dh_defaults = self._get_dh_defaults()
        for i, (alpha, a, d, theta_off) in enumerate(dh_defaults):
            if i >= len(self.dh_vars):
                break
            v_alpha, v_a, v_d, v_t = self.dh_vars[i]
            v_alpha.set(f"{alpha:.3f}")
            v_a.set(f"{a:.3f}")
            v_d.set(f"{d:.3f}")
            v_t.set(f"{theta_off:.3f}")

    def set_mask_from_current_pose(self):
        """
        Gamepad/GUI: copies current TCP pose from ExecuteApp.tcp_panel
        in die Eingabemaske.
        """
        try:
            tcp = self.exec.get_current_tcp_mm()
        except Exception:
            tcp = None

        if not tcp:
            return

        def fmt(v):
            try:
                return f"{float(v):.4f}"
            except Exception:
                return "0.0000"

        self.tcp_x.set(fmt(tcp.get("X_mm", 0.0)))
        self.tcp_y.set(fmt(tcp.get("Y_mm", 0.0)))
        self.tcp_z.set(fmt(tcp.get("Z_mm", 0.0)))

        roll_deg = tcp.get("Roll_deg", 0.0)
        pitch_deg = tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0))
        yaw_deg = tcp.get("Yaw_deg", 0.0)

        self.tcp_roll.set(fmt(roll_deg))
        self.tcp_pitch.set(fmt(pitch_deg))
        self.tcp_yaw.set(fmt(yaw_deg))


    @staticmethod
    def _norm(v):
        x, y, z = v
        n = math.sqrt(x * x + y * y + z * z)
        return (x / n, y / n, z / n) if n > 1e-9 else (0.0, 0.0, 1.0)

    def _get_tool_axis_world(self, roll_deg, pitch_deg, yaw_deg):
        """
        Computes tool direction from Euler angles (Roll, Pitch, Yaw) in deg.
        We assume ZYX convention:
           R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Tool axis = +Z axis of the tool frame (3. Spalte).
        """
        yaw_r = math.radians(yaw_deg)
        pitch_r = math.radians(pitch_deg)
        roll_r = math.radians(roll_deg)

        cy, sy = math.cos(yaw_r), math.sin(yaw_r)
        cp, sp = math.cos(pitch_r), math.sin(pitch_r)
        cr, sr = math.cos(roll_r), math.sin(roll_r)

        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        R = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]

        # Tool axis = third column of R (Z-axis of tool)
        u = (R[0][2], R[1][2], R[2][2])
        if getattr(self, "_mirror_x", False):
            u = (-u[0], u[1], u[2])
        return self._norm(u)

    def _read_dh_table(self):
        """Read DH table (alpha, a, d, theta_offset) from UI; values are degrees/mm."""
        if not hasattr(self, "dh_vars"):
            raise ValueError("DH UI not initialized")
        dh_axes = self._dh_axes()
        table = []
        for i, (v_alpha, v_a, v_d, v_t) in enumerate(self.dh_vars):
            try:
                alpha = float(v_alpha.get().replace(",", "."))
            except Exception:
                alpha = 0.0
            try:
                a = float(v_a.get().replace(",", "."))
            except Exception:
                a = 0.0
            try:
                d = float(v_d.get().replace(",", "."))
            except Exception:
                d = 0.0
            try:
                theta_offset = float(v_t.get().replace(",", "."))
            except Exception:
                theta_offset = 0.0
            axis = dh_axes[i] if i < len(dh_axes) else f"j{i+1}"
            table.append(
                {"axis": axis, "alpha": alpha, "a": a, "d": d, "theta_offset": theta_offset}
            )
        return table

    @staticmethod
    def _dh_transform(theta_deg, alpha_deg, a, d):
        th = math.radians(theta_deg)
        al = math.radians(alpha_deg)
        ct, st = math.cos(th), math.sin(th)
        ca, sa = math.cos(al), math.sin(al)
        return [
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @staticmethod
    def _mat_mul(a, b):
        out = [[0.0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
        return out

    @staticmethod
    def _transpose(mat):
        return [list(row) for row in zip(*mat)]

    @staticmethod
    def _mat_mul_generic(a, b):
        rows = len(a)
        cols = len(b[0])
        mid = len(b)
        out = [[0.0] * cols for _ in range(rows)]
        for i in range(rows):
            for j in range(cols):
                out[i][j] = sum(a[i][k] * b[k][j] for k in range(mid))
        return out

    @staticmethod
    def _mat_add_diag(mat, val):
        out = [row[:] for row in mat]
        for i in range(min(len(out), len(out[0]))):
            out[i][i] += val
        return out

    @staticmethod
    def _rpy_to_R(roll_deg, pitch_deg, yaw_deg):
        # ZYX rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        yaw_r = math.radians(yaw_deg)
        pitch_r = math.radians(pitch_deg)
        roll_r = math.radians(roll_deg)

        cy, sy = math.cos(yaw_r), math.sin(yaw_r)
        cp, sp = math.cos(pitch_r), math.sin(pitch_r)
        cr, sr = math.cos(roll_r), math.sin(roll_r)

        return [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]

    @staticmethod
    def _mat3_mul(a, b):
        return [
            [
                a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
                a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
                a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
            ],
            [
                a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
                a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
                a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
            ],
            [
                a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
                a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
                a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
            ],
        ]

    @staticmethod
    def _mat3_transpose(a):
        return [
            [a[0][0], a[1][0], a[2][0]],
            [a[0][1], a[1][1], a[2][1]],
            [a[0][2], a[1][2], a[2][2]],
        ]

    @staticmethod
    def _so3_log(R):
        # Return rotation vector (rad) from a rotation matrix
        tr = R[0][0] + R[1][1] + R[2][2]
        cos_theta = max(-1.0, min(1.0, (tr - 1.0) * 0.5))
        theta = math.acos(cos_theta)
        if theta < 1e-8:
            return [0.0, 0.0, 0.0]

        if abs(math.pi - theta) < 1e-4:
            x = math.sqrt(max(0.0, (R[0][0] + 1.0) * 0.5))
            y = math.sqrt(max(0.0, (R[1][1] + 1.0) * 0.5))
            z = math.sqrt(max(0.0, (R[2][2] + 1.0) * 0.5))
            x = math.copysign(x, R[2][1] - R[1][2])
            y = math.copysign(y, R[0][2] - R[2][0])
            z = math.copysign(z, R[1][0] - R[0][1])
            return [theta * x, theta * y, theta * z]

        s = 2.0 * math.sin(theta)
        return [
            theta * (R[2][1] - R[1][2]) / s,
            theta * (R[0][2] - R[2][0]) / s,
            theta * (R[1][0] - R[0][1]) / s,
        ]

    def _task_from_joints(self, joints):
        # Forward kinematics from DH table. Returns (pos, R).
        dh = self._read_dh_table()
        dh_axes = self._dh_axes()
        T = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        for i, ax in enumerate(dh_axes):
            row = dh[i]
            model_joint = apply_joint_angle_post_transform(ax, float(joints[i]))
            th = model_joint + float(row.get("theta_offset", 0.0))
            Ti = self._dh_transform(th, row["alpha"], row["a"], row["d"])
            T = self._mat_mul(T, Ti)
        x, y, z = T[0][3], T[1][3], T[2][3]
        if getattr(self, "_mirror_x", False):
            x = -x
        R = [
            [T[0][0], T[0][1], T[0][2]],
            [T[1][0], T[1][1], T[1][2]],
            [T[2][0], T[2][1], T[2][2]],
        ]
        return [x, y, z], R

    def _pose_error(self, target_pos, target_R, base_pos, base_R):
        dx = target_pos[0] - base_pos[0]
        dy = target_pos[1] - base_pos[1]
        dz = target_pos[2] - base_pos[2]
        R_err = self._mat3_mul(target_R, self._mat3_transpose(base_R))
        rot_err = self._so3_log(R_err)
        return [
            dx,
            dy,
            dz,
            self.rot_weight_mm * rot_err[0],
            self.rot_weight_mm * rot_err[1],
            self.rot_weight_mm * rot_err[2],
        ]

    @staticmethod
    def _invert_matrix(mat):
        """Gauss-Jordan inversion for small square matrices."""
        n = len(mat)
        aug = []
        for i in range(n):
            row = [float(x) for x in mat[i]]
            row += [1.0 if i == j else 0.0 for j in range(n)]
            aug.append(row)

        for col in range(n):
            pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
            if abs(aug[pivot][col]) < 1e-9:
                raise ValueError("Jacobian is singular or ill-conditioned")
            if pivot != col:
                aug[col], aug[pivot] = aug[pivot], aug[col]

            piv = aug[col][col]
            for j in range(2 * n):
                aug[col][j] /= piv

            for r in range(n):
                if r == col:
                    continue
                factor = aug[r][col]
                if abs(factor) < 1e-12:
                    continue
                for j in range(2 * n):
                    aug[r][j] -= factor * aug[col][j]

        inv = [row[n:] for row in aug]
        return inv

    @staticmethod
    def _mat_vec_mul(mat, vec):
        out = []
        for row in mat:
            out.append(sum(row[i] * vec[i] for i in range(len(vec))))
        return out

    def _get_current_joint_list(self):
        """Current joint list in DH_AXES order using ExecuteApp axis_positions."""
        src = getattr(self.exec, "axis_positions", {}) or {}
        dh_axes = self._dh_axes()
        joints = []
        for ax in dh_axes:
            try:
                joints.append(float(src.get(ax, 0.0)))
            except Exception:
                joints.append(0.0)
        return joints

    def _save_dh_to_json(self):
        """Persist DH parameters (deg/mm) into the active profile."""
        path = os.path.join(os.path.dirname(__file__), "model", "dh.json")
        try:
            if os.path.isfile(path):
                with open(path, "r", encoding="utf-8") as f:
                    model = json.load(f)
            else:
                model = {}
        except Exception:
            model = {}

        model["convention"] = "DH"
        model["units"] = {"length": "m", "angle": "rad"}
        model["joint_order"] = list(self._dh_axes())
        raw_post = model.get("post_transform", {})
        post_transform = dict(raw_post) if isinstance(raw_post, dict) else {}
        post_transform["mirror_x"] = bool(self._mirror_x)
        model["post_transform"] = post_transform

        existing = {}
        for j in model.get("joints", []):
            axis = (j.get("axis") or "").strip()
            if axis:
                existing[axis] = dict(j)

        dh = self._read_dh_table()
        joints = []
        for i, row in enumerate(dh):
            axis = row["axis"]
            base = existing.get(axis, {})
            base["name"] = base.get("name") or f"joint{i+1}_{axis}"
            base["axis"] = axis
            base["a"] = float(row["a"]) / 1000.0
            base["alpha"] = math.radians(float(row["alpha"]))
            base["d"] = float(row["d"]) / 1000.0
            base["theta_offset"] = math.radians(float(row["theta_offset"]))
            base.setdefault("q_min", -math.pi)
            base.setdefault("q_max", math.pi)
            if not base.get("source_note"):
                base["source_note"] = "Updated via kinematics tab UI."
            joints.append(base)

        model["joints"] = joints

        if hasattr(self.exec, "save_dh_model_to_profile"):
            ok, msg = self.exec.save_dh_model_to_profile(model)
            if hasattr(self.exec, "log"):
                self.exec.log(msg)
        else:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(model, f, indent=2, ensure_ascii=True)
            reload_dh_model()
            if hasattr(self.exec, "log"):
                self.exec.log("DH parameters saved to model/dh.json and reloaded.")

    def _compute_jacobian(self, joints):
        # Numerical Jacobian dX/dQ from DH FK
        base_pos, base_R = self._task_from_joints(joints)
        delta = 0.1  # deg
        dof = len(joints)
        J = [[0.0] * dof for _ in range(6)]
        for j in range(dof):
            joints2 = list(joints)
            joints2[j] += delta
            pos2, R2 = self._task_from_joints(joints2)
            dp = [pos2[0] - base_pos[0], pos2[1] - base_pos[1], pos2[2] - base_pos[2]]
            R_err = self._mat3_mul(R2, self._mat3_transpose(base_R))
            rot_err = self._so3_log(R_err)
            d = [
                dp[0],
                dp[1],
                dp[2],
                self.rot_weight_mm * rot_err[0],
                self.rot_weight_mm * rot_err[1],
                self.rot_weight_mm * rot_err[2],
            ]
            for i in range(6):
                J[i][j] = d[i] / delta
        return J, base_pos, base_R

    def _damped_pinv(self, J, lam):
        """Damped least squares pseudo-inverse: (J^T J + lam^2 I)^-1 J^T."""
        JT = self._transpose(J)
        JTJ = self._mat_mul_generic(JT, J)
        JTJ = self._mat_add_diag(JTJ, lam * lam)
        inv = self._invert_matrix(JTJ)
        return self._mat_mul_generic(inv, JT)

    @staticmethod
    def _wrap180_deg(a):
        return ((a + 180.0) % 360.0) - 180.0

    def move_tcp_pose(self, x, y, z, roll_deg, pitch_deg, yaw_deg, feed=None, allow_out_of_limits=False):
        """
        Solve a TCP target pose with iterative DLS and send a single absolute G1 move.
        """
        try:
            # Refresh joint positions from last status if available
            try:
                st = getattr(self.client, "last_status", None) or {}
                mpos = st.get("MPos") or {}
                if mpos:
                    for ax, val in mpos.items():
                        if ax in AXES:
                            self.exec.axis_positions[ax] = float(val)
                    self._last_tcp_joints = self._get_current_joint_list()
            except Exception:
                pass

            lims = self._limits()
            if getattr(self, "_last_tcp_joints", None) is not None and not (st.get("MPos") or st.get("WPos")):
                joints = list(self._last_tcp_joints)
            else:
                joints = self._get_current_joint_list()
            target_R = self._rpy_to_R(roll_deg, pitch_deg, yaw_deg)
            target_pos = [x, y, z]

            def _norm3(v):
                return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

            max_iters = 15
            pos_tol = 0.05
            rot_tol = math.radians(0.5)
            max_step = 5.0

            for _ in range(max_iters):
                J, base_pos, base_R = self._compute_jacobian(joints)
                d_task = self._pose_error(target_pos, target_R, base_pos, base_R)
                pos_err = _norm3(d_task[:3])
                if self.rot_weight_mm > 1e-9:
                    rot_err = _norm3(
                        [d_task[3] / self.rot_weight_mm,
                         d_task[4] / self.rot_weight_mm,
                         d_task[5] / self.rot_weight_mm]
                    )
                else:
                    rot_err = _norm3(d_task[3:])
                if pos_err <= pos_tol and rot_err <= rot_tol:
                    break
                J_inv = self._damped_pinv(J, lam=0.1)
                d_joint = self._mat_vec_mul(J_inv, d_task)
                max_abs = max(abs(v) for v in d_joint) if d_joint else 0.0
                scale = 1.0
                if max_abs > max_step and max_abs > 1e-9:
                    scale = max_step / max_abs
                for i in range(len(joints)):
                    joints[i] += d_joint[i] * scale

            dh_axes = self._dh_axes()
            joints_map = {}
            for i, ax in enumerate(dh_axes):
                joints_map[ax] = joints[i]
            if not self._check_limits(joints_map, lims):
                if not allow_out_of_limits:
                    if hasattr(self.exec, "log"):
                        self.exec.log("TCP Move blocked: joint limits exceeded.")
                    return False
                if hasattr(self.exec, "log"):
                    self.exec.log("TCP Move warning: limits exceeded (override).")

            F = float(feed) if feed is not None else float(self.feed.get())
            g = "G90 G1 " + " ".join(f"{ax}{joints_map[ax]:.3f}" for ax in dh_axes) + f" F{F:.0f}"
            try:
                self.client.send_line(g)
                if hasattr(self.exec, "log"):
                    self.exec.log(f"TCP Move: {g}")
            except Exception as e:
                if hasattr(self.exec, "log"):
                    self.exec.log(f"TCP Move send failed: {e}")
                return False
            for ax in AXES:
                if ax in joints_map:
                    self.exec.axis_positions[ax] = float(joints_map[ax])
            self._last_tcp_joints = [joints_map[ax] for ax in dh_axes]
            if hasattr(self.exec, "_append_vis_path_kin") and not getattr(self.exec, "_vis_skip_kin", False):
                try:
                    self.exec._append_vis_path_kin((x, y, z))
                except Exception:
                    pass
            return True
        except Exception as e:
            if hasattr(self.exec, "log"):
                self.exec.log(f"TCP Move error: {e}")
            return False

    def preview_tcp_gcode(self, x, y, z, roll_deg, pitch_deg, yaw_deg, feed=None):
        """
        Solve a TCP target pose with iterative DLS and return the joint-space G1 for preview.
        """
        try:
            st = {}
            try:
                st = getattr(self.client, "last_status", None) or {}
                mpos = st.get("MPos") or {}
                if mpos:
                    for ax, val in mpos.items():
                        if ax in AXES:
                            self.exec.axis_positions[ax] = float(val)
            except Exception:
                pass

            lims = self._limits()
            if getattr(self, "_last_tcp_joints", None) is not None and not (st.get("MPos") or st.get("WPos")):
                joints = list(self._last_tcp_joints)
            else:
                joints = self._get_current_joint_list()
            target_R = self._rpy_to_R(roll_deg, pitch_deg, yaw_deg)
            target_pos = [x, y, z]

            def _norm3(v):
                return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

            max_iters = 15
            pos_tol = 0.05
            rot_tol = math.radians(0.5)
            max_step = 5.0

            for _ in range(max_iters):
                J, base_pos, base_R = self._compute_jacobian(joints)
                d_task = self._pose_error(target_pos, target_R, base_pos, base_R)
                pos_err = _norm3(d_task[:3])
                if self.rot_weight_mm > 1e-9:
                    rot_err = _norm3(
                        [d_task[3] / self.rot_weight_mm,
                         d_task[4] / self.rot_weight_mm,
                         d_task[5] / self.rot_weight_mm]
                    )
                else:
                    rot_err = _norm3(d_task[3:])
                if pos_err <= pos_tol and rot_err <= rot_tol:
                    break
                J_inv = self._damped_pinv(J, lam=0.1)
                d_joint = self._mat_vec_mul(J_inv, d_task)
                max_abs = max(abs(v) for v in d_joint) if d_joint else 0.0
                scale = 1.0
                if max_abs > max_step and max_abs > 1e-9:
                    scale = max_step / max_abs
                for i in range(len(joints)):
                    joints[i] += d_joint[i] * scale

            dh_axes = self._dh_axes()
            joints_map = {}
            for i, ax in enumerate(dh_axes):
                joints_map[ax] = joints[i]
            ok = self._check_limits(joints_map, lims)

            F = float(feed) if feed is not None else float(self.feed.get())
            g = "G90 G1 " + " ".join(f"{ax}{joints_map[ax]:.3f}" for ax in dh_axes) + f" F{F:.0f}"
            return {"gcode": g, "limits": lims, "ok": ok, "joints": joints_map}
        except Exception as e:
            if hasattr(self.exec, "log"):
                self.exec.log(f"TCP Preview error: {e}")
            return None

    # =========================================================
    # LIMITS
    # =========================================================
    def _limits(self):
        lims = {}
        exec_limits = getattr(self.exec, "axis_limits", {}) or {}
        exec_eff = getattr(self.exec, "_effective_axis_limits", None)
        use_exec = bool(getattr(self.exec, "endstop_limits_enabled", False) and exec_limits)

        if use_exec:
            if callable(exec_eff):
                for ax in AXES:
                    try:
                        lims[ax] = exec_eff(ax)
                    except Exception:
                        pass
            else:
                for ax in AXES:
                    if ax in exec_limits:
                        try:
                            lo, hi = exec_limits[ax]
                            lims[ax] = (float(lo), float(hi))
                        except Exception:
                            pass
        else:
            model = get_dh_model()
            if model:
                for j in model.get("joints", []):
                    axis = (j.get("axis") or "").strip()
                    if not axis:
                        continue
                    q_min = j.get("q_min", None)
                    q_max = j.get("q_max", None)
                    if q_min is None or q_max is None:
                        continue
                    try:
                        lims[axis] = (
                            math.degrees(float(q_min)),
                            math.degrees(float(q_max)),
                        )
                    except Exception:
                        pass

        hw = getattr(self.exec, "hw_limits", {})
        for ax in AXES:
            if ax in hw:
                hw_lo, hw_hi = hw[ax]
                skip_hw = False
                if use_exec and ax in exec_limits:
                    try:
                        end_lo, end_hi = exec_limits[ax]
                        if float(end_lo) < 0 and hw_lo >= 0:
                            skip_hw = True
                    except Exception:
                        pass
                if not skip_hw:
                    if ax in lims:
                        lo, hi = lims[ax]
                        lims[ax] = (max(lo, hw_lo), min(hi, hw_hi))
                    else:
                        lims[ax] = tuple(hw[ax])
            if ax not in lims:
                lims[ax] = exec_limits.get(ax, (-180.0, 180.0))
        return lims

    def _limits_for_preview(self):
        lims = self._limits()
        exec_limits = getattr(self.exec, "axis_limits", {}) or {}
        if getattr(self.exec, "endstop_limits_enabled", False) and exec_limits:
            for ax in AXES:
                if ax in exec_limits:
                    try:
                        lo, hi = exec_limits[ax]
                        lims[ax] = (float(lo), float(hi))
                    except Exception:
                        pass
        return lims

    def _check_limits(self, joints, lims):
        for ax in AXES:
            lo, hi = lims.get(ax, (-999, 999))
            if not (lo <= joints[ax] <= hi):
                return False
        return True

    # =========================================================
    # PREVIEW EMITTER
    # =========================================================
    def _emit(self, lines, bad_mask):
        self.txt.configure(state="normal")
        self.txt.delete("1.0", tk.END)

        lims = self._limits_for_preview()
        has_bad_vals = False

        for i, ln in enumerate(lines):
            if not ln.strip():
                self.txt.insert(tk.END, "\n", "plain")
                continue

            parts = ln.split(" ")
            for part in parts:
                tag = "plain"
                for ax in AXES:
                    if part.startswith(ax):
                        try:
                            val = float(part[len(ax):])
                            lo, hi = lims.get(ax, (-9999, 9999))
                            tag = "ok" if lo <= val <= hi else "bad_val"
                            if tag == "bad_val":
                                has_bad_vals = True
                        except Exception:
                            tag = "plain"
                self.txt.insert(tk.END, part + " ", tag)
            self.txt.insert(tk.END, "\n", "plain")

        self.txt.see("end")
        self.txt.configure(state="disabled")
        self._preview_has_limit_violations = bool(has_bad_vals)

    # =========================================================
    # MAIN SEQUENCE GENERATOR (RET  TAR  RET)
    # =========================================================
    def _run(self):
        try:
            # 1) Read pose from mask
            def f(var: tk.StringVar):
                try:
                    return float(var.get().replace(",", "."))
                except Exception:
                    return 0.0

            X = f(self.tcp_x)
            Y = f(self.tcp_y)
            Z = f(self.tcp_z)
            roll_deg_pose = f(self.tcp_roll)
            pitch_deg_pose = f(self.tcp_pitch)
            yaw_deg_pose = f(self.tcp_yaw)

            # 2) Apply overrides (0 = "no override")
            pitch_pose = (
                float(self.pitch_override.get())
                if abs(self.pitch_override.get()) > 1e-9
                else pitch_deg_pose
            )
            roll_pose = (
                float(self.roll_override.get())
                if abs(self.roll_override.get()) > 1e-9
                else roll_deg_pose
            )
            yaw = yaw_deg_pose  # currently no override

            # 3) Derive tool axis from Euler angles (pitch stays in UI convention)
            u = self._get_tool_axis_world(roll_pose, pitch_pose, yaw)
            if self.invert_u.get():
                u = (-u[0], -u[1], -u[2])

            Dret = float(self.D_ret.get())
            Dwork = float(self.D_work.get())
            anchor = (self.anchor_mode.get() or "zero").lower()

            # 4) Anchor-Logik
            #    P_zero = midpoint between retreat and target
            if anchor == "target":
                # Maske = Target
                P_tar = (X, Y, Z)
                P_zero = (
                    X - Dwork * u[0],
                    Y - Dwork * u[1],
                    Z - Dwork * u[2],
                )
                P_ret = (
                    P_zero[0] - Dret * u[0],
                    P_zero[1] - Dret * u[1],
                    P_zero[2] - Dret * u[2],
                )
            elif anchor == "retreat":
                # Maske = Retreat
                P_ret = (X, Y, Z)
                P_zero = (
                    X + Dret * u[0],
                    Y + Dret * u[1],
                    Z + Dret * u[2],
                )
                P_tar = (
                    P_zero[0] + Dwork * u[0],
                    P_zero[1] + Dwork * u[1],
                    P_zero[2] + Dwork * u[2],
                )
            else:
                # Default / "zero": Maske = Mitte
                P_zero = (X, Y, Z)
                P_ret = (
                    X - Dret * u[0],
                    Y - Dret * u[1],
                    Z - Dret * u[2],
                )
                P_tar = (
                    X + Dwork * u[0],
                    Y + Dwork * u[1],
                    Z + Dwork * u[2],
                )

            lims = self._limits()
            F = float(self.feed.get())

            lines = [
                "; ---- TCP Sequence ----",
                "G21",
                "G90",
                "G94",
                f"; Anchor={anchor}  Zero=({P_zero[0]:.3f},{P_zero[1]:.3f},{P_zero[2]:.3f})",
            ]
            bad = []

            # 5) Pre-GCode (optional)
            if self.use_prepost.get():
                pre_txt = self.txt_pre.get("1.0", "end").splitlines()
                for l in pre_txt:
                    s = l.strip()
                    if not s:
                        continue
                    lines.append(s)
                    bad.append(False)

            # Jacobian setup (dX = J * dQ)
            base_joints = self._get_current_joint_list()
            J, base_pos, base_R = self._compute_jacobian(base_joints)
            J_inv = self._damped_pinv(J, lam=0.1)
            target_R = self._rpy_to_R(roll_pose, pitch_pose, yaw)

            def to_line(P, rapid):
                d_task = self._pose_error([P[0], P[1], P[2]], target_R, base_pos, base_R)
                d_joint = self._mat_vec_mul(J_inv, d_task)
                j = {}
                for i, ax in enumerate(self._dh_axes()):
                    j[ax] = base_joints[i] + d_joint[i]
                ok = self._check_limits(j, lims)
                # Use controlled feed moves only; rapid with rotary joints can trigger controller alarms.
                g = "G1 " + " ".join(
                    f"{ax}{j[ax]:.3f}" for ax in self._dh_axes()
                )
                g += f" F{F:.0f}"
                return g, ok, j

            # RET
            if anchor == "retreat":
                j_ret = {ax: base_joints[i] for i, ax in enumerate(self._dh_axes())}
                ok = self._check_limits(j_ret, lims)
                g = "G1 " + " ".join(f"{ax}{j_ret[ax]:.3f}" for ax in self._dh_axes()) + f" F{F:.0f}"
            else:
                g, ok, j_ret = to_line(P_ret, True)
            lines.append(g)
            bad.append(not ok)

            # DH plausibility hint:
            # For anchor=retreat, first move should be ~current joint state.
            if anchor == "retreat":
                cur = {ax: base_joints[i] for i, ax in enumerate(self._dh_axes())}
                dmax = 0.0
                for ax in self._dh_axes():
                    d = abs(float(j_ret.get(ax, 0.0)) - float(cur.get(ax, 0.0)))
                    if d > dmax:
                        dmax = d
                if hasattr(self.exec, "log"):
                    self.exec.log(f"Kinematics check (retreat): joint_max={dmax:.3f}")
                    if dmax > 0.5:
                        self.exec.log(" Retreat start differs from current pose -> DH/offset mismatch likely.")
                    if dmax > 5.0:
                        raise RuntimeError(
                            f"Retreat start mismatch too large (joint_max={dmax:.2f}). "
                            "Sequence aborted for safety."
                        )

            # TAR
            g, ok, _j_tar = to_line(P_tar, False)
            lines.append(g)
            bad.append(not ok)

            # GRIPPER -> optional with pauses
            if self.use_gripper.get():
                before_ms = max(0, int(self.grip_pause_before_ms.get()))
                after_ms = max(0, int(self.grip_pause_after_ms.get()))
                sval = int(self.grip_s.get())

                if before_ms > 0:
                    lines.append(f"G4 P{before_ms/1000.0:.3f}")
                    bad.append(False)

                lines.append(f"M3 S{sval}")
                bad.append(False)

                if after_ms > 0:
                    lines.append(f"G4 P{after_ms/1000.0:.3f}")
                    bad.append(False)

            # RET BACK (safety non-rapid)
            g, ok, _j_ret2 = to_line(P_ret, False)
            lines.append(g)
            bad.append(not ok)

            # Post-GCode (optional)
            if self.use_prepost.get():
                post_txt = self.txt_post.get("1.0", "end").splitlines()
                for l in post_txt:
                    s = l.strip()
                    if not s:
                        continue
                    lines.append(s)
                    bad.append(False)

            lines.append("; ---- END ----")
            bad.append(False)

            self._emit(lines, bad)

        except Exception as e:
            try:
                if hasattr(self, "txt"):
                    self.txt.configure(state="normal")
                    self.txt.delete("1.0", tk.END)
                    self.txt.insert(tk.END, f"TCP Sequence Error: {e}")
                    self.txt.configure(state="disabled")
            except Exception:
                pass
            try:
                if hasattr(self.exec, "log"):
                    self.exec.log(f"TCP Sequence Error: {e}")
            except Exception:
                pass

    # =========================================================
    # CLI / QUEUE INTEGRATION
    # =========================================================
    def _collect_preview_lines(self):
        text = self.txt.get("1.0", tk.END).splitlines()
        return [ln for ln in text if ln.strip()]

    def _send_preview_to_cli(self):
        lines = self._collect_preview_lines()
        if not hasattr(self.exec, "entry_cmd"):
            return
        self.exec.entry_cmd.delete(0, tk.END)
        # Insert multi-line content into CLI
        self.exec.entry_cmd.insert(0, "\n".join(lines))
        if hasattr(self.exec, "log"):
            self.exec.log("Preview copied to CLI.")

    def _execute_direct(self):
        """
        Sends preview to CLI and starts cli_send_now() directly
        (i.e., send to CLI + immediate execution).
        """
        if bool(getattr(self, "_preview_has_limit_violations", False)):
            if hasattr(self.exec, "log"):
                self.exec.log("TCP Sequence blocked: preview contains joint limit violations.")
            return
        self._send_preview_to_cli()
        # ExecuteApp hat cli_send_now()  nutzen, falls vorhanden
        try:
            if hasattr(self.exec, "cli_send_now"):
                self.exec.cli_send_now()
            elif hasattr(self.exec, "_on_cli_return"):
                # Fallback: Enter-Handler direkt aufrufen
                self.exec._on_cli_return()
        except Exception as e:
            try:
                if hasattr(self.exec, "log"):
                    self.exec.log(f"Execute error: {e}")
            except Exception:
                pass

    def _send_to_queue(self, line):
        if hasattr(self.exec, "enqueue"):
            self.exec.enqueue(line)
            if hasattr(self.exec, "log"):
                self.exec.log(f"Queued: {line}")

    def _send_preview_to_queue(self):
        lines = self._collect_preview_lines()
        sent = 0
        for ln in lines:
            s = ln.strip()
            if not s or s.startswith(";"):
                continue
            if hasattr(self.exec, "enqueue"):
                self.exec.enqueue(s)
                sent += 1
        if hasattr(self.exec, "log"):
            self.exec.log(f"Preview enqueued: {sent} lines")


# =========================================================
# TCP World Kinematics  einfacher Container
# =========================================================

class TcpWorldKinematicsTabs(ttk.Frame):
    """
    Container without its own notebook:
    The kinematics tab content is directly the TcpKinematicsFrame.
    """

    def __init__(self, master, execute_app, client):
        super().__init__(master)

        self.execute_app = execute_app
        self.client = client

        # Direkt den Kinematics-Frame einbetten
        self.world = TcpKinematicsFrame(self, execute_app, client)
        self.world.pack(fill=tk.BOTH, expand=True)

        # -------------------------------------------------
        # Gamepad-compatible proxy methods for preview, etc.
        # -------------------------------------------------
    def preview(self):
        """Generiert Preview wie _run()."""
        if hasattr(self.world, "_run"):
            return self.world._run()

    def preview_sequence(self):
        return self.preview()

    def gamepad_preview_sequence(self):
        return self.preview()

    def seq_preview(self):
        return self.preview()

    def execute_sequence(self):
        """Execute immediately."""
        if hasattr(self.world, "_execute_direct"):
            return self.world._execute_direct()

    def queue_sequence(self):
        """Preview in Queue schieben."""
        if hasattr(self.world, "_send_preview_to_queue"):
            return self.world._send_preview_to_queue()

    # -------------------------------------------------
    # Gamepad / ExecuteApp-kompatible Methoden
    # -------------------------------------------------

    def set_tcp_as_reference(self):
        """
        Gamepad: transfer only TCP -> mask (set world coordinates).
        Trigger: tcp_set_tcp_as_reference
        """
        if hasattr(self.world, "pose_to_mask_only"):
            return self.world.pose_to_mask_only()
        # fallback: if not set for any reason
        if hasattr(self.world, "set_mask_from_current_pose"):
            return self.world.set_mask_from_current_pose()
        return None

    def use_current_tcp_as_ref(self):
        # Alias (ExecuteApp sucht teils danach)
        return self.set_tcp_as_reference()

    def solve_and_execute_sequence(self):
        """
        Gamepad: TCP -> mask, solve sequence, and execute.
        Trigger: tcp_solve_seq_and_execute
        """
        if hasattr(self.world, "pose_to_mask_and_solve_execute"):
            return self.world.pose_to_mask_and_solve_execute()

        # fallback: if callable is unavailable (safer path)
        self.set_tcp_as_reference()
        if hasattr(self.world, "_run"):
            self.world._run()
        if hasattr(self.world, "_execute_direct"):
            return self.world._execute_direct()
        return None

    def pose_to_mask_and_generate(self):
        """
        Gamepad: TCP -> mask, solve sequence (Generate) without execution.
        Trigger: tcp_pose_to_mask_and_generate
        """
        if hasattr(self.world, "pose_to_mask_and_generate"):
            return self.world.pose_to_mask_and_generate()
        self.set_tcp_as_reference()
        if hasattr(self.world, "_run"):
            return self.world._run()
        return None

    def solve_and_run_tcp_sequence(self):
        # Alias (ExecuteApp sucht teils danach)
        return self.solve_and_execute_sequence()

    def move_tcp_pose(self, x, y, z, roll_deg, pitch_deg, yaw_deg, feed=None, allow_out_of_limits=False):
        if hasattr(self.world, "move_tcp_pose"):
            return self.world.move_tcp_pose(
                x,
                y,
                z,
                roll_deg,
                pitch_deg,
                yaw_deg,
                feed,
                allow_out_of_limits=allow_out_of_limits,
            )
        return None

    def refresh_dh_table(self):
        if hasattr(self.world, "refresh_dh_table"):
            return self.world.refresh_dh_table()
        return None

    def preview_tcp_gcode(self, x, y, z, roll_deg, pitch_deg, yaw_deg, feed=None):
        if hasattr(self.world, "preview_tcp_gcode"):
            return self.world.preview_tcp_gcode(
                x,
                y,
                z,
                roll_deg,
                pitch_deg,
                yaw_deg,
                feed,
            )
        return None




__all__ = [
    "TcpKinematicsFrame",
    "TcpWorldKinematicsTabs",
]
