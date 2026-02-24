# tcp_pose_module.py
# =============================================================
#  TCP Pose Monitor (plug-in panel)  for RoboControl FluidNC
# - 6DOF-DH-Forward-Kinematik (A X Y B Z C)
# - Eigenes Panel + zentrale FK-Funktion
# - API:
#     * panel = TcpPosePanel(parent, client, geom_dh=GEOM_DH)
#     * panel.start() / panel.stop()
#     * panel.get_current_tcp_mm() -> dict (X_mm/Y_mm/Z_mm/Roll_deg/Pitch_deg/Yaw_deg)
#     * panel.on_pose_changed(callback_dict -> None)
# =============================================================

from __future__ import annotations
import tkinter as tk
from tkinter import ttk
import math
import json
import os
from typing import Callable, Dict, Optional

AXES = ["X", "Y", "Z", "A", "B", "C"]
MOTION_EPS = 0.01
_DEFAULT_DH_ORDER = ["A", "X", "Y", "B", "Z", "C"]

# Standard-Moveo-Geometrie (mm)
_DEFAULT_GEOM_DH = {
    "L1": 240.0,
    "L2": 230.0,
    "L3": 250.0,
    "L4": 180.0,
}

_DH_MODEL_PATH = os.path.join(os.path.dirname(__file__), "model", "dh.json")

def _load_dh_model():
    if not os.path.isfile(_DH_MODEL_PATH):
        return None
    try:
        with open(_DH_MODEL_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print("[TcpPosePanel] dh model load failed:", e)
        return None

def _ordered_axes_from_model(model, available_axes=None):
    if not isinstance(model, dict):
        return list(_DEFAULT_DH_ORDER)

    available = set(available_axes or [])
    ordered = []
    seen = set()

    joint_order = model.get("joint_order", [])
    if isinstance(joint_order, list):
        for axis in joint_order:
            ax = str(axis).strip().upper()
            if not ax or ax in seen:
                continue
            if available and ax not in available:
                continue
            ordered.append(ax)
            seen.add(ax)

    for axis in _DEFAULT_DH_ORDER:
        if axis in seen:
            continue
        if available and axis not in available:
            continue
        ordered.append(axis)
        seen.add(axis)

    if available:
        for axis in sorted(available):
            if axis in seen:
                continue
            ordered.append(axis)
            seen.add(axis)
    return ordered


def _build_dh_rows_mm_deg(model):
    if not isinstance(model, dict):
        return []

    joints_by_axis = {}
    for j in model.get("joints", []):
        axis = (j.get("axis") or "").strip().upper()
        if not axis:
            continue
        joints_by_axis[axis] = j

    order = _ordered_axes_from_model(model, available_axes=joints_by_axis.keys())
    rows = []
    for axis in order:
        j = joints_by_axis.get(axis)
        if not j:
            continue
        rows.append(
            {
                "axis": axis,
                "alpha_deg": math.degrees(float(j.get("alpha", 0.0))),
                "a_mm": float(j.get("a", 0.0)) * 1000.0,
                "d_mm": float(j.get("d", 0.0)) * 1000.0,
                "theta_offset_deg": math.degrees(float(j.get("theta_offset", 0.0))),
            }
        )
    return rows

def _geom_from_model(model):
    rows = _build_dh_rows_mm_deg(model)
    by_axis = {r["axis"]: r for r in rows}
    order = _ordered_axes_from_model(model, available_axes=by_axis.keys())

    link2_axis = None
    if "Y" in order:
        y_idx = order.index("Y")
        tail = [ax for ax in order[y_idx + 1:] if ax != "C"]
    else:
        tail = [ax for ax in order if ax not in ("A", "X", "Y", "C")]
    for ax in tail:
        d_val = abs(float(by_axis.get(ax, {}).get("d_mm", 0.0)))
        if d_val > 1e-9:
            link2_axis = ax
            break
    if link2_axis is None:
        link2_axis = "B" if "B" in by_axis else "Z"

    return {
        "L1": abs(float(by_axis.get("A", {}).get("d_mm", _DEFAULT_GEOM_DH["L1"]))),
        "L2": abs(float(by_axis.get("X", {}).get("a_mm", _DEFAULT_GEOM_DH["L2"]))),
        "L3": abs(float(by_axis.get(link2_axis, {}).get("d_mm", _DEFAULT_GEOM_DH["L3"]))),
        "L4": abs(float(by_axis.get("C", {}).get("d_mm", _DEFAULT_GEOM_DH["L4"]))),
    }

_DH_MODEL = _load_dh_model()
_DH_ROWS = _build_dh_rows_mm_deg(_DH_MODEL) if _DH_MODEL else []
_POST_TRANSFORM = _DH_MODEL.get("post_transform", {}) if _DH_MODEL else {}

def _fallback_dh_rows_from_geom(geom):
    L1 = float(geom.get("L1", _DEFAULT_GEOM_DH["L1"]))
    L2 = float(geom.get("L2", _DEFAULT_GEOM_DH["L2"]))
    L3 = float(geom.get("L3", _DEFAULT_GEOM_DH["L3"]))
    L4 = float(geom.get("L4", _DEFAULT_GEOM_DH["L4"]))
    return [
        {"axis": "A", "alpha_deg": 90.0, "a_mm": 0.0, "d_mm": L1, "theta_offset_deg": 0.0},
        {"axis": "X", "alpha_deg": 0.0, "a_mm": L2, "d_mm": 0.0, "theta_offset_deg": 90.0},
        {"axis": "Y", "alpha_deg": -90.0, "a_mm": 0.0, "d_mm": 0.0, "theta_offset_deg": -90.0},
        {"axis": "B", "alpha_deg": 90.0, "a_mm": 0.0, "d_mm": L3, "theta_offset_deg": 0.0},
        {"axis": "Z", "alpha_deg": -90.0, "a_mm": 0.0, "d_mm": 0.0, "theta_offset_deg": 0.0},
        {"axis": "C", "alpha_deg": 0.0, "a_mm": 0.0, "d_mm": L4, "theta_offset_deg": 0.0},
    ]

def get_dh_model():
    return _DH_MODEL

def get_dh_rows_mm_deg():
    return list(_DH_ROWS)

def get_dh_axes():
    if _DH_ROWS:
        return [r["axis"] for r in _DH_ROWS]
    return list(_DEFAULT_DH_ORDER)

def get_post_transform():
    return dict(_POST_TRANSFORM)


def _clean_joint_map(raw_map):
    out = {}
    if not isinstance(raw_map, dict):
        return out
    for raw_axis, raw_val in raw_map.items():
        axis = str(raw_axis).strip().upper()
        if not axis:
            continue
        try:
            out[axis] = float(raw_val)
        except Exception:
            continue
    return out


def get_joint_angle_post_maps():
    post = get_post_transform()
    offsets = _clean_joint_map(post.get("sim_theta_offset_deg", {}))
    scales = _clean_joint_map(post.get("sim_theta_scale", {}))
    return offsets, scales


def apply_joint_angle_post_transform(axis: str, machine_angle_deg: float) -> float:
    ax = str(axis or "").strip().upper()
    try:
        v = float(machine_angle_deg)
    except Exception:
        v = 0.0
    offsets, scales = get_joint_angle_post_maps()
    scale = float(scales.get(ax, 1.0))
    offset = float(offsets.get(ax, 0.0))
    return v * scale + offset


def derive_visualizer_geometry_mm(model=None):
    src = model if isinstance(model, dict) else _DH_MODEL
    geom = _geom_from_model(src if isinstance(src, dict) else {})
    order = _ordered_axes_from_model(src if isinstance(src, dict) else {}, available_axes=get_dh_axes())
    return {
        "base_height_mm": float(geom["L1"]),
        "L1_mm": float(geom["L2"]),
        "L2_mm": float(geom["L3"]),
        "L_tool_mm": float(geom["L4"]),
        "joint_order": list(order),
    }

GEOM_DH = _geom_from_model(_DH_MODEL) if _DH_MODEL else _DEFAULT_GEOM_DH.copy()

def reload_dh_model():
    global _DH_MODEL, _DH_ROWS, _POST_TRANSFORM, GEOM_DH
    _DH_MODEL = _load_dh_model()
    _DH_ROWS = _build_dh_rows_mm_deg(_DH_MODEL) if _DH_MODEL else []
    _POST_TRANSFORM = _DH_MODEL.get("post_transform", {}) if _DH_MODEL else {}
    GEOM_DH = _geom_from_model(_DH_MODEL) if _DH_MODEL else _DEFAULT_GEOM_DH.copy()
    return _DH_MODEL


def set_dh_model_from_dict(model):
    """Overrides the DH model from an in-memory dict (profile-based)."""
    global _DH_MODEL, _DH_ROWS, _POST_TRANSFORM, GEOM_DH
    _DH_MODEL = dict(model) if isinstance(model, dict) else None
    _DH_ROWS = _build_dh_rows_mm_deg(_DH_MODEL) if _DH_MODEL else []
    _POST_TRANSFORM = _DH_MODEL.get("post_transform", {}) if _DH_MODEL else {}
    GEOM_DH = _geom_from_model(_DH_MODEL) if _DH_MODEL else _DEFAULT_GEOM_DH.copy()
    return _DH_MODEL


class TcpPosePanel(ttk.LabelFrame):
    """
    Small GUI panel with its own compute engine:
      - attaches to client.listeners (raw lines)
      - erwartet client.parse_status_line(line) -> (state, positions_dict)
      - rechnet alle poll_interval_ms die TCP-Pose

    Orientierungskonvention:
       Roll  = Rotation um Tool-X
       Pitch = Rotation um Tool-Y
       Yaw   = rotation around tool Z (ZYX Euler from world view)
    """

    def __init__(
        self,
        master,
        client,
        geom_dh: Dict[str, float],
        title: str = "TCP Pose",
        poll_interval_ms: int = 200,
    ):
        super().__init__(master, text=title)
        self.client = client
        self.geom_dh = dict(geom_dh)  # erwartet Keys: L1,L2,L3,L4
        self.poll_interval = int(poll_interval_ms)
        self._running = False
        self._listener_hooked = False

        # Axis position buffer (actual)
        self.axis_positions = {ax: 0.0 for ax in AXES}

        # UI-Text (RPY in richtiger Reihenfolge)
        self._tcp_var = tk.StringVar(
            value="TCP \nX=0.00 mm\nY=0.00 mm\nZ=0.00 mm\nRoll=0.00\nPitch=0.00\nYaw=0.00"
        )
        ttk.Label(
            self,
            textvariable=self._tcp_var,
            font=("Consolas", 9, "bold"),
            justify="left",
        ).pack(anchor="w", padx=8, pady=6)

        # Optional callback (called on pose change)
        self._pose_changed_cb: Optional[Callable[[Dict[str, float]], None]] = None

        # Interner Cache der letzten TCP-Pose (in mm/deg)
        self.current_tcp_pose = {
            "X_mm": 0.0,
            "Y_mm": 0.0,
            "Z_mm": 0.0,
            "Roll_deg": 0.0,
            "Pitch_deg": 0.0,
            "Yaw_deg": 0.0,
            # Legacy-Aliases:
            "Tilt_deg": 0.0,   # = Pitch_deg
            "B_deg": 0.0,      # historisch "Tilt"/B
        }

        # Register listener (if available)
        self._attach_listener()

    # ---------------- Public API ----------------

    def start(self):
        """Startet den periodischen TCP-Update-Loop."""
        if not self._running:
            self._running = True
            self.after(self.poll_interval, self._tick_tcp_update)

    def stop(self):
        """Beendet den Update-Loop (Listener bleibt bis destroy())."""
        self._running = False

    def on_pose_changed(self, cb: Callable[[Dict[str, float]], None] | None):
        """Register optional callback; receives dict with X_mm/Y_mm/Z_mm/Roll_deg/Pitch_deg/Yaw_deg/..."""
        self._pose_changed_cb = cb

    def get_current_tcp_mm(self) -> Dict[str, float]:
        """Liefert die aktuell angezeigte TCP-Pose (mm / deg, konsistent, RPY)."""
        return dict(self.current_tcp_pose)

    def set_geom_dh(self, geom_dh: Dict[str, float]):
        """Updates the internal DH geometry (L1..L4) for FK updates."""
        self.geom_dh = dict(geom_dh)

    # --------------- Integration ---------------

    def _attach_listener(self):
        if self._listener_hooked:
            return
        try:
            if not hasattr(self.client, "listeners"):
                self.client.listeners = []
            self.client.listeners.append(self._on_serial_line)
            self._listener_hooked = True
        except Exception as e:
            print("[TcpPosePanel] listener attach failed:", e)

    def destroy(self):
        # Versuche den Listener wieder zu entfernen
        try:
            if self._listener_hooked and hasattr(self.client, "listeners"):
                if self._on_serial_line in self.client.listeners:
                    self.client.listeners.remove(self._on_serial_line)
        except Exception:
            pass
        self._running = False
        super().destroy()

    # --------------- Serial handling ---------------

    def _on_serial_line(self, line: str):
        """
        Erwartet, dass client.parse_status_line(line) -> (state, positions_dict)
        is available.
        """
        try:
            if not hasattr(self.client, "parse_status_line"):
                return
            state, pos = self.client.parse_status_line(line)
            if not pos:
                return

            for ax, v in pos.items():
                old = self.axis_positions.get(ax, 0.0)
                if abs(old - v) > MOTION_EPS:
                    self.axis_positions[ax] = float(v)

        except Exception as e:
            print("[TcpPosePanel] parse line error:", e)

    # --------------- Pose Update Loop ---------------

    def _tick_tcp_update(self):
        if not self._running:
            return
        try:
            # Joint-Map in der erwarteten Reihenfolge
            jpose = {
                "A": float(self.axis_positions.get("A", 0.0)),
                "X": float(self.axis_positions.get("X", 0.0)),
                "Y": float(self.axis_positions.get("Y", 0.0)),
                "B": float(self.axis_positions.get("B", 0.0)),
                "Z": float(self.axis_positions.get("Z", 0.0)),
                "C": float(self.axis_positions.get("C", 0.0)),
            }

            # --- Forward-Kinematics ---
            X, Y, Z, Roll, Pitch, Yaw, Tilt = fk6_forward_mm(self.geom_dh, jpose)

            # Text for label (RPY)
            lines = [
                "TCP ",
                f"X={X:.2f} mm",
                f"Y={Y:.2f} mm",
                f"Z={Z:.2f} mm",
                f"Roll={Roll:.2f}",
                f"Pitch={Pitch:.2f}",
                f"Yaw={Yaw:.2f}",
            ]
            txt = "\n".join(lines)
            self._tcp_var.set(txt)

            # Consistent output in mm/deg for other modules
            pose = {
                "X_mm": X,
                "Y_mm": Y,
                "Z_mm": Z,
                "Roll_deg": Roll,
                "Pitch_deg": Pitch,
                "Yaw_deg": Yaw,
                # Legacy aliases (for old code still expecting Tilt):
                "Tilt_deg": Tilt,   # = -Pitch in unserer FK-Anpassung
                "B_deg": Tilt,
            }

            # Callback only on real change
            if _pose_diff(self.current_tcp_pose, pose) > 1e-6:
                self.current_tcp_pose = pose
                if self._pose_changed_cb:
                    try:
                        self._pose_changed_cb(dict(pose))
                    except Exception as e:
                        print("[TcpPosePanel] pose callback error:", e)

        except Exception as e:
            print("[TcpPosePanel] TCP update error:", e)

        # Next tick
        self.after(self.poll_interval, self._tick_tcp_update)


# ---------------- kleine Helfer ----------------
def _pose_diff(a: Dict[str, float], b: Dict[str, float]) -> float:
    """Small delta score used to suppress callback noise."""
    keys = ("X_mm", "Y_mm", "Z_mm", "Roll_deg", "Pitch_deg", "Yaw_deg")
    s = 0.0
    for k in keys:
        s += (float(a.get(k, 0.0)) - float(b.get(k, 0.0))) ** 2
    return s


# ===== FK6 (DH) as pure module function - returns values in mm =====
def fk6_forward_mm(geom: Dict[str, float], j: Dict[str, float]):
    """
    KORREKTE MOVE0 6DOF FORWARD KINEMATICS
    (identisch zu world_kinematics_frame_v7 + Visualizer + Gamepad)

    Return:
      X_mm, Y_mm, Z_mm, Roll_deg, Pitch_deg, Yaw_deg, Tilt_deg

    Hinweis:
       Tilt_deg is a historical alias (=-Pitch) from older world kinematics.
       For new modules, use only Roll/Pitch/Yaw.
    """
    import numpy as np

    # Use SSOT DH model if available; fallback to legacy constants.
    dh_rows = get_dh_rows_mm_deg()
    if not dh_rows:
        dh_rows = _fallback_dh_rows_from_geom(geom)

    def T_dh(th_rad, al_rad, d_mm, a_mm):
        ct = math.cos(th_rad)
        st = math.sin(th_rad)
        ca = math.cos(al_rad)
        sa = math.sin(al_rad)
        return np.array(
            [
                [ct, -st * ca, st * sa, a_mm * ct],
                [st, ct * ca, -ct * sa, a_mm * st],
                [0, sa, ca, d_mm],
                [0, 0, 0, 1],
            ]
        )

    T = np.eye(4)
    for row in dh_rows:
        machine_deg = float(j.get(row["axis"], 0.0))
        model_deg = apply_joint_angle_post_transform(row["axis"], machine_deg)
        theta_deg = model_deg + row["theta_offset_deg"]
        th = math.radians(theta_deg)
        al = math.radians(row["alpha_deg"])
        d = float(row["d_mm"])
        a = float(row["a_mm"])
        T = T @ T_dh(th, al, d, a)

    # -----------------------------
    # Position
    # -----------------------------
    x = float(T[0, 3])
    y = float(T[1, 3])
    z = float(T[2, 3])

    # -----------------------------
    # Orientierung (ZYX Euler)
    # -----------------------------
    r11, r12, r13 = T[0, 0], T[0, 1], T[0, 2]
    r21, r22, r23 = T[1, 0], T[1, 1], T[1, 2]
    r31, r32, r33 = T[2, 0], T[2, 1], T[2, 2]

    pitch = math.degrees(math.atan2(-r31, math.sqrt(r11 * r11 + r21 * r21)))
    roll = math.degrees(math.atan2(r32, r33))
    yaw = math.degrees(math.atan2(r21, r11))

    # -----------------------------
    # Moveo-/Visualizer-Anpassungen
    # -----------------------------
    # Invert X axis (compatible with your world kinematics)
    if get_post_transform().get("mirror_x"):
        x = -x

    # "Tilt" was the historical name for -Pitch
    tilt = -pitch


    return x, y, z, roll, pitch, yaw, tilt
