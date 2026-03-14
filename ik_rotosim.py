# ik_rotosim.py
# ------------------------------------------------------------
# 5-DOF IK for RoboSim v8.1 (debug trace edition)
# - numeric DLS IK (Damped Least Squares) with optional locks
# - auto-unlock A if target yaw is inconsistent
# - Very verbose trace (logger=callable)
# - ASCII-only
# ------------------------------------------------------------

from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from typing import Dict, Optional, Callable, Tuple

# ------------------ Datenklassen ------------------

@dataclass
class IKLimits:
    A: tuple = (-180.0, 180.0)   # Base yaw
    X: tuple = (-120.0, 120.0)   # Shoulder pitch
    Y: tuple = (-135.0, 135.0)   # Elbow pitch
    Z: tuple = (-180.0, 180.0)   # Tool pitch
    B: tuple = (-180.0, 180.0)   # Tool roll

# ------------------ Hilfsfunktionen ------------------

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def _wrap_pm180(a_deg: float) -> float:
    a = (a_deg + 180.0) % 360.0 - 180.0
    return 180.0 if abs(a + 180.0) < 1e-9 else a

def _rot_z(p: np.ndarray, yaw_deg: float) -> np.ndarray:
    a = math.radians(yaw_deg)
    c, s = math.cos(a), math.sin(a)
    x, y, z = p
    return np.array([c*x - s*y, s*x + c*y, z], dtype=float)

def _fmt(v, p=6):
    if isinstance(v, (list, tuple, np.ndarray)):
        arr = np.array(v, dtype=float)
        return np.array2string(arr, precision=p, floatmode="fixed", suppress_small=False)
    try:
        return f"{float(v):.{p}f}"
    except Exception:
        return str(v)

# ------------------ RotoSimIK ------------------

class RotoSimIK:
    """
    Numerische IK, abgestimmt auf deine FK:
      - Pose-Felder: A (Yaw), X, Y, Z (Tool-Pitch), B (Roll)
      - Geometrie-Felder: base_height, L1, L2, L_tool
    """
    def __init__(self, geom, limits: IKLimits, prefer: str = "down"):
        self.geom = geom
        self.limits = limits
        self.prefer = prefer
        self._last_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # [A,X,Y,Z,B] in Grad

    # -------- Forward Kinematics (identisch zur App) --------

    def _fk_points_and_dir(self, qA, qX, qY, qZ, qB):
        th_x = math.radians(qX)
        th_y = math.radians(qY)
        th_z = math.radians(qZ)
        yaw  = math.radians(qA)
        roll = math.radians(qB)

        P0 = np.array([0.0, 0.0, 0.0])
        P1 = np.array([0.0, 0.0, self.geom.base_height])

        P2 = P1 + np.array([0.0,
                            self.geom.L1 * math.sin(th_x),
                            self.geom.L1 * math.cos(th_x)])
        a_L2 = th_x + th_y
        P3 = P2 + np.array([0.0,
                            self.geom.L2 * math.sin(a_L2),
                            self.geom.L2 * math.cos(a_L2)])

        axis = np.array([0.0, math.sin(a_L2), math.cos(a_L2)], dtype=float)
        axis /= np.linalg.norm(axis)

        n1 = np.cross(axis, np.array([1.0, 0.0, 0.0]))
        if np.linalg.norm(n1) < 1e-8:
            n1 = np.cross(axis, np.array([0.0, 1.0, 0.0]))
        n1 /= np.linalg.norm(n1)
        n2 = np.cross(axis, n1)

        cr, sr = math.cos(roll), math.sin(roll)
        n1r = cr * n1 + sr * n2

        dir_tool = math.cos(th_z) * axis + math.sin(th_z) * n1r
        dir_tool /= np.linalg.norm(dir_tool)

        P4 = P3 + self.geom.L_tool * dir_tool

        cy, sy = math.cos(yaw), math.sin(yaw)
        def rot(p):
            x, y, z = p
            return np.array([cy * x - sy * y, sy * x + cy * y, z], dtype=float)

        pts = [rot(P0), rot(P1), rot(P2), rot(P3), rot(P4)]
        dir_world = rot(dir_tool)
        return pts, dir_world

    def _tcp_pose5(self, q: np.ndarray):
        pts, dir_tool = self._fk_points_and_dir(q[0], q[1], q[2], q[3], q[4])
        X, Y, Z = pts[-1]
        # 4th feature is now joint angle A directly:
        yawA = q[0]
        dirn = dir_tool / (np.linalg.norm(dir_tool) + 1e-12)
        tilt = math.degrees(math.acos(np.clip(np.dot(dirn, np.array([0.0, 0.0, 1.0])), -1.0, 1.0)))
        return np.array([X, Y, Z, yawA, tilt], dtype=float)

    # ------------------ IK Kern ------------------

    def _limit_q(self, q: np.ndarray) -> np.ndarray:
        A = _clamp(q[0], *self.limits.A)
        X = _clamp(q[1], *self.limits.X)
        Y = _clamp(q[2], *self.limits.Y)
        Z = _clamp(q[3], *self.limits.Z)
        B = _clamp(q[4], *self.limits.B)
        return np.array([A, X, Y, Z, B], dtype=float)

    def _numeric_jacobian(self, q: np.ndarray, active_mask: np.ndarray, eps_deg: float = 0.75):

        """
        Numeric Jacobian for f(q) = [x,y,z,yaw,tilt].
        Only columns of active joints are varied.
        """
        f0 = self._tcp_pose5(q)
        J = np.zeros((5, 5), dtype=float)
        for j in range(5):
            if not active_mask[j]:
                continue
            dq = np.zeros(5, dtype=float); dq[j] = eps_deg
            f1 = self._tcp_pose5(self._limit_q(q + dq))
            df = f1 - f0
            J[:, j] = df / eps_deg
        return J

    def _pick_seed(self, Xd, Yd, Zd, Yawd, Tiltd, lock_A_deg, lock_B_deg) -> np.ndarray:
        """
        Planarer Startwert:
          - A = Lock falls gegeben, sonst Ziel-Yaw
          - B = Lock falls gegeben, sonst 0
          - X,Y approximately from 2R chain (y-z plane)
          - Z  90 - Tilt
        """
        A0 = lock_A_deg if lock_A_deg is not None else Yawd
        B0 = lock_B_deg if lock_B_deg is not None else 0.0

        p_local = _rot_z(np.array([Xd, Yd, Zd], dtype=float), -A0)
        y = float(p_local[1])
        z = float(p_local[2] - self.geom.base_height)
        L1, L2 = float(self.geom.L1), float(self.geom.L2)

        r = math.hypot(y, z)
        r = max(min(r, L1 + L2 - 1e-3), abs(L1 - L2) + 1e-3)

        cos_elbow = (L1*L1 + L2*L2 - r*r) / max(1e-9, (2.0*L1*L2))
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        phi_elbow = math.acos(cos_elbow)
        if self.prefer.lower().startswith("down"):
            phi_elbow = -phi_elbow

        gamma = math.atan2(y, z)
        beta = math.acos(max(-1.0, min(1.0, (L1*L1 + r*r - L2*L2) / max(1e-9, 2.0*L1*r))))
        th_x = gamma + (beta if self.prefer.lower().startswith("down") else -beta)

        X0 = math.degrees(th_x)
        Y0 = math.degrees(phi_elbow)
        Z0 = 90.0 - Tiltd

        q0 = np.array([A0, X0, Y0, Z0, B0], dtype=float)
        return self._limit_q(q0)

    # ------------------ Public API (with debug trace) ------------------
    def solve_from_pose_locked(
        self,
        Xd: float, Yd: float, Zd: float,
        Yawd: float, Tiltd: float,
        lock_A_deg: Optional[float] = None,
        lock_B_deg: Optional[float] = None,
        seed: Optional[Dict[str, float]] = None,
        max_iters: int = 150,
        pos_tol_mm: float = 0.3,
        ang_tol_deg: float = 0.6,
        lambd: float = 3.0,
        debug: bool = False,
        logger=None,
        tilt_weight: float = 1.0,          # <-- NEW
    ) -> Optional[Dict[str, float]]:
        """
        Finde q=[A,X,Y,Z,B] (in Grad), so dass TCP  [Xd,Yd,Zd,Yaw,Tilt].
        - lock_A_deg / lock_B_deg: lock joints (if consistent).
        - seed: optionaler Startwert {"A":..,"X":..,"Y":..,"Z":..,"B":..}
        - tilt_weight < 1.0 erlaubt kleine Tilt-Abweichungen, stabiler bei B0.
        """

        def log(msg: str):
            if logger:
                logger(msg)

        # ---------- Startwert ----------
        if seed is not None:
            q = np.array([seed.get("A", 0.0),
                          seed.get("X", 0.0),
                          seed.get("Y", 0.0),
                          seed.get("Z", 0.0),
                          seed.get("B", 0.0)], dtype=float)
            q = self._limit_q(q)
        else:
            A0 = lock_A_deg if lock_A_deg is not None else Yawd
            q = np.array([A0, 0.0, 0.0, 0.0, 0.0], dtype=float)
            q = self._limit_q(q)

        q_seed2 = self._pick_seed(Xd, Yd, Zd, Yawd, Tiltd, lock_A_deg, lock_B_deg)

        # ---------- Locks + Konsistenz ----------
        desired_yaw = Yawd
        active = np.array([True, True, True, True, True], dtype=bool)
        if lock_A_deg is not None:
            yaw_mismatch = abs(_wrap_pm180(lock_A_deg - desired_yaw))
            if yaw_mismatch <= 1.0:
                q[0] = float(lock_A_deg)
                active[0] = False
        if lock_B_deg is not None:
            q[4] = float(lock_B_deg)
            active[4] = False

        # ---------- Error weights ----------
        arm_scale = max(1.0, (float(self.geom.L1) + float(self.geom.L2) + float(self.geom.L_tool)) / 3.0)
        w = np.array([
            1.0,                 # X
            1.0,                 # Y
            1.0,                 # Z
            arm_scale * 0.10,    # Yaw
            arm_scale * 0.15,    # Tilt
        ], dtype=float)
        # NEW: relax tilt if requested
        w[4] *= max(0.0, float(tilt_weight))


        # Error weights
        arm_scale = max(1.0, (float(self.geom.L1) + float(self.geom.L2) + float(self.geom.L_tool)) / 3.0)
        w = np.array([1.0, 1.0, 1.0, arm_scale * 0.10, arm_scale * 0.15], dtype=float)

        target = np.array([Xd, Yd, Zd, Yawd, Tiltd], dtype=float)

        log("=== IK TRACE BEGIN ===")
        log(f"Target     : [{_fmt(target)}]")
        log(f"Locks      : A={_fmt(lock_A_deg)}  B={_fmt(lock_B_deg)}  ActiveMask={active.tolist()}")
        log(f"Seed#1     : q={_fmt(q)}")
        log(f"Seed#2     : q={_fmt(q_seed2)}")
        log(f"Limits     : A{self.limits.A} X{self.limits.X} Y{self.limits.Y} Z{self.limits.Z} B{self.limits.B}")
        log(f"Params     : iters={max_iters}  pos_tol={pos_tol_mm}mm  ang_tol={ang_tol_deg}deg  lambda={lambd:.3f}")
        log("================================")

        for attempt in (1, 2):
            if attempt == 2:
                q = q_seed2.copy()
                log("---- RETRY with Seed#2 ----")

            lam = float(lambd)
            for it in range(1, max_iters+1):
                feat = self._tcp_pose5(q)
                err = target - feat


                pos_err = float(np.linalg.norm(err[:3]))
                ang_err = float(max(abs(err[3]), abs(err[4])))

                log(f"[it {attempt}.{it:03d}] q={_fmt(q)}")
                log(f"            f(q)=[{_fmt(feat)}]")
                log(f"            err =[{_fmt(err)}]  pos_err={pos_err:.6f}mm  ang_err={ang_err:.6f}deg")

                if pos_err <= pos_tol_mm and ang_err <= ang_tol_deg:
                    q = self._limit_q(q)
                    self._last_q = q.copy()
                    log(f" CONVERGED at it {attempt}.{it:03d}  pos_err={pos_err:.6f}  ang_err={ang_err:.6f}")
                    log(f"Result q   : {_fmt(q)}")
                    log("=== IK TRACE END (OK) ===")
                    return {"A": q[0], "X": q[1], "Y": q[2], "Z": q[3], "B": q[4]}

                # Jacobian
                J = self._numeric_jacobian(q, active_mask=active, eps_deg=0.5)
                # Metriken
                JT = J.T
                JJt = J @ JT
                # simple condition-number estimate (via SVD)
                try:
                    s = np.linalg.svd(J, compute_uv=False)
                    cond = float((s.max() / max(1e-12, s.min()))) if s.size else float('inf')
                except Exception:
                    cond = float('inf')

                # DLS Schritt
                # we = w * err (simple weighting im Messraum)
                we = w * err
                for i in range(JJt.shape[0]):
                    JJt[i, i] += (lam * lam)
                try:
                    step = JT @ np.linalg.solve(JJt, we)
                    # active DOFs only
                    step = step * active.astype(float)
                except np.linalg.LinAlgError:
                    lam *= 1.5
                    log(f"            (JJt ill-conditioned) -> increase lambda -> {lam:.6f}")
                    continue

                # Step limiting (coarser steps allowed to cross barriers)
                max_step = 15.0
                step_clamped = np.clip(step, -max_step, +max_step)

                # Protokoll
                log(f"            J-norms: ||J||F={np.linalg.norm(J):.6f}  cond~={cond:.3f}  lambda={lam:.6f}")
                log(f"            step   : {_fmt(step)} (clamped {_fmt(step_clamped)})  active={active.tolist()}")

                # Update
                q_new = self._limit_q(q + step_clamped)
                feat_new = self._tcp_pose5(q_new)
                err_new = target - feat_new
                err_new[3] = _wrap_pm180(err_new[3])
                pos_err_new = float(np.linalg.norm(err_new[:3]))
                ang_err_new = float(max(abs(err_new[3]), abs(err_new[4])))

                # Adaptives lambda (sehr simpel)
                if pos_err_new < pos_err or ang_err_new < ang_err:
                    lam = max(0.5, lam * 0.9)
                else:
                    lam = min(50.0, lam * 1.25)

                log(f"            q_new  : {_fmt(q_new)}")
                log(f"            f(qnew): [{_fmt(feat_new)}]")
                log(f"            errnew : [{_fmt(err_new)}]  pos_err_new={pos_err_new:.6f}  ang_err_new={ang_err_new:.6f}")
                log(f"            lambda' -> {lam:.6f}")
                q = q_new

            log(f" attempt {attempt}: no convergence within {max_iters} iterations.")

        log(" IK TRACE END (NO SOLUTION)")
        return None
