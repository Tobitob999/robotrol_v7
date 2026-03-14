"""
autocalib_v1.py  –  Automatic eye-to-hand hand-eye calibration
==============================================================
Solves  base_T_cam  (4×4 homogeneous) by collecting paired observations:

  base_T_tcp_i   – TCP pose in robot base frame  (from FK, known)
  cam_T_marker_i – ArUco marker pose in camera frame (measured)

With a marker rigidly mounted at the TCP flange, the calibration
constraint is (across all pose pairs):

    base_T_cam  *  cam_T_marker_i  ≈  base_T_tcp_i  *  tcp_T_marker

OpenCV's calibrateHandEye (TSAI method) is used with the eye-to-hand
formulation:  pass  tcp_T_base  (inverse FK)  as  "gripper→base"
→ result  (R_cam2base, t_cam2base)  =  base_T_cam.

Minimum recommended samples: 8–12, with varied TCP orientations.
Each capture should differ by at least ±10° in at least one rotation axis.
"""

import math
import numpy as np
import cv2

# ── RPY convention (ZYX extrinsic, same as robotrol _rpy_to_R) ─────────────────

def rpy_deg_to_R(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Build 3×3 rotation matrix from roll / pitch / yaw in degrees.
    Convention: R = Rz(yaw) · Ry(pitch) · Rx(roll)  (ZYX extrinsic).
    Matches robotrol's internal _rpy_to_R method.
    """
    yr = math.radians(yaw_deg)
    pr = math.radians(pitch_deg)
    rr = math.radians(roll_deg)
    cy, sy = math.cos(yr), math.sin(yr)
    cp, sp = math.cos(pr), math.sin(pr)
    cr, sr = math.cos(rr), math.sin(rr)
    return np.array([
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
        [-sp,      cp * sr,                 cp * cr               ],
    ], dtype=np.float64)


def build_tcp_matrix(x_mm: float, y_mm: float, z_mm: float,
                     roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Return 4×4 base_T_tcp homogeneous matrix from FK output (mm, deg)."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rpy_deg_to_R(roll_deg, pitch_deg, yaw_deg)
    T[:3, 3] = [x_mm, y_mm, z_mm]
    return T


def invert_T(T: np.ndarray) -> np.ndarray:
    """Fast 4×4 rigid-body inverse (no np.linalg.inv needed)."""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -(R.T @ t)
    return Ti


# ── ArUco helpers ──────────────────────────────────────────────────────────────

ARUCO_DICT_NAMES = [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_7X7_50",
]

_ARUCO_DICT_IDS = {
    "DICT_4X4_50":  cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_5X5_50":  cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50":  cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_7X7_50":  cv2.aruco.DICT_7X7_50,
}


def get_aruco_dict(dict_name: str):
    """Return a cv2 ArUco dictionary object by name string."""
    d_id = _ARUCO_DICT_IDS.get(dict_name, cv2.aruco.DICT_4X4_50)
    return cv2.aruco.getPredefinedDictionary(d_id)


def detect_marker_pose(
    frame_bgr: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    aruco_dict,
    marker_id: int,
    marker_size_mm: float,
):
    """
    Detect a specific ArUco marker in frame_bgr and return its pose.

    Parameters
    ----------
    frame_bgr      : BGR image (uint8)
    camera_matrix  : 3×3 intrinsic matrix
    dist_coeffs    : distortion coefficients
    aruco_dict     : cv2.aruco dictionary object
    marker_id      : integer ID of the target marker
    marker_size_mm : physical side length of the printed marker (mm)

    Returns
    -------
    (T_cam_marker, annotated_bgr)
        T_cam_marker : 4×4 np.ndarray (marker pose in camera frame), or None
        annotated_bgr: copy of frame with detected markers drawn
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    annotated = frame_bgr.copy()

    # Support both OpenCV 4.7+ (ArucoDetector) and older (legacy API)
    try:
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(gray)
    except AttributeError:
        # Fallback: legacy OpenCV ≤ 4.6
        params = cv2.aruco.DetectorParameters_create()  # type: ignore[attr-defined]
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)  # type: ignore[attr-defined]

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

    if ids is None:
        return None, annotated

    ids_flat = ids.flatten().tolist()
    if marker_id not in ids_flat:
        return None, annotated

    idx = ids_flat.index(marker_id)

    # Object points: marker corners in marker frame (Z=0 plane)
    h = marker_size_mm / 2.0
    obj_pts = np.array([
        [-h,  h, 0.0],
        [ h,  h, 0.0],
        [ h, -h, 0.0],
        [-h, -h, 0.0],
    ], dtype=np.float32)

    corner = corners[idx].reshape(4, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, corner, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        return None, annotated

    cv2.drawFrameAxes(
        annotated, camera_matrix, dist_coeffs,
        rvec, tvec, marker_size_mm * 0.5,
    )

    R_mat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_mat
    T[:3, 3] = tvec.flatten()
    return T, annotated


# ── Calibration session ────────────────────────────────────────────────────────

class AutoCalibSession:
    """
    Collects (base_T_tcp, cam_T_marker) observation pairs and solves
    for base_T_cam via OpenCV calibrateHandEye (eye-to-hand variant).

    Usage
    -----
    sess = AutoCalibSession(K, D, aruco_dict, marker_id=0, marker_size_mm=50)
    while not done:
        found, vis = sess.add_capture(frame_bgr, x, y, z, roll, pitch, yaw)
    if sess.can_solve():
        base_T_cam = sess.solve()
        rms = sess.reprojection_error_mm(base_T_cam)
    """

    MIN_SAMPLES = 5

    def __init__(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        aruco_dict,
        marker_id: int,
        marker_size_mm: float,
    ):
        self.camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.asarray(dist_coeffs, dtype=np.float64).flatten()
        self.aruco_dict = aruco_dict
        self.marker_id = int(marker_id)
        self.marker_size_mm = float(marker_size_mm)

        # Stored observations
        self._R_tcp: list[np.ndarray] = []   # TCP rotation in base (from FK)
        self._t_tcp: list[np.ndarray] = []   # TCP position in base [mm]
        self._R_mc:  list[np.ndarray] = []   # marker rotation in camera
        self._t_mc:  list[np.ndarray] = []   # marker position in camera [mm]
        self._vis_frames: list[np.ndarray] = []  # annotated RGB frames

    # ── public properties ──────────────────────────────────────────────────────

    def n_samples(self) -> int:
        return len(self._R_tcp)

    def can_solve(self) -> bool:
        return self.n_samples() >= self.MIN_SAMPLES

    def last_frame_rgb(self):
        """Return most recent annotated frame (RGB uint8) or None."""
        return self._vis_frames[-1] if self._vis_frames else None

    # ── capture ───────────────────────────────────────────────────────────────

    def add_capture(
        self,
        frame_bgr: np.ndarray,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
    ):
        """
        Try to detect the ArUco marker and store the pose pair.

        Parameters
        ----------
        frame_bgr : BGR camera frame (numpy array)
        x_mm … yaw_deg : current robot TCP pose from FK

        Returns
        -------
        (found: bool, annotated_bgr: np.ndarray)
        """
        T_cm, annotated_bgr = detect_marker_pose(
            frame_bgr,
            self.camera_matrix, self.dist_coeffs,
            self.aruco_dict, self.marker_id, self.marker_size_mm,
        )
        if T_cm is None:
            return False, annotated_bgr

        R_tcp = rpy_deg_to_R(roll_deg, pitch_deg, yaw_deg)
        t_tcp = np.array([x_mm, y_mm, z_mm], dtype=np.float64)

        self._R_tcp.append(R_tcp)
        self._t_tcp.append(t_tcp.reshape(3, 1))
        self._R_mc.append(T_cm[:3, :3].copy())
        self._t_mc.append(T_cm[:3, 3].reshape(3, 1).copy())
        self._vis_frames.append(cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB))
        return True, annotated_bgr

    # ── solve ─────────────────────────────────────────────────────────────────

    def solve(self, method=cv2.CALIB_HAND_EYE_TSAI) -> np.ndarray:
        """
        Solve for base_T_cam.

        Eye-to-hand formulation:
            pass  tcp_T_base  (inverse FK)  as "gripper2base"
            → calibrateHandEye returns  (R_cam2base, t_cam2base)
            → assemble 4×4 base_T_cam.

        Returns
        -------
        base_T_cam : 4×4 np.ndarray (homogeneous, translation in mm)
        """
        if not self.can_solve():
            raise RuntimeError(
                f"Need ≥ {self.MIN_SAMPLES} samples, have {self.n_samples()}."
            )

        # Build tcp_T_base (inverse of base_T_tcp) for each pose
        R_inv_list, t_inv_list = [], []
        for R, t in zip(self._R_tcp, self._t_tcp):
            R_inv = R.T
            t_inv = -(R_inv @ t)
            R_inv_list.append(R_inv)
            t_inv_list.append(t_inv)

        R_c2b, t_c2b = cv2.calibrateHandEye(
            R_inv_list, t_inv_list,
            self._R_mc, self._t_mc,
            method=method,
        )

        base_T_cam = np.eye(4, dtype=np.float64)
        base_T_cam[:3, :3] = R_c2b
        base_T_cam[:3, 3] = t_c2b.flatten()
        return base_T_cam

    # ── quality metric ────────────────────────────────────────────────────────

    def reprojection_error_mm(self, base_T_cam: np.ndarray) -> float:
        """
        Consistency check: for each pose pair re-derive the marker origin
        in base frame via FK and via camera, compare the two.

        base_T_cam * cam_T_marker  should match  base_T_tcp * tcp_T_marker.
        Since tcp_T_marker is approximately identity (marker at TCP flange),
        compare the translation part.

        Returns mean positional error in mm.
        """
        errors = []
        for R_tcp, t_tcp, R_mc, t_mc in zip(
            self._R_tcp, self._t_tcp, self._R_mc, self._t_mc
        ):
            T_base_tcp = np.eye(4)
            T_base_tcp[:3, :3] = R_tcp
            T_base_tcp[:3, 3] = t_tcp.flatten()

            T_cam_marker = np.eye(4)
            T_cam_marker[:3, :3] = R_mc
            T_cam_marker[:3, 3] = t_mc.flatten()

            # Reconstructed marker origin in base frame via camera path
            T_base_marker_cam = base_T_cam @ T_cam_marker
            # Expected marker origin in base frame via FK (marker ≈ TCP)
            t_expected = T_base_tcp[:3, 3]
            t_got = T_base_marker_cam[:3, 3]
            errors.append(float(np.linalg.norm(t_got - t_expected)))

        return float(np.mean(errors)) if errors else 0.0

    # ── misc ──────────────────────────────────────────────────────────────────

    def clear(self):
        """Discard all collected samples."""
        self._R_tcp.clear()
        self._t_tcp.clear()
        self._R_mc.clear()
        self._t_mc.clear()
        self._vis_frames.clear()

    def diversity_score(self) -> float:
        """
        Return a rough angular diversity score (sum of pairwise angle
        differences between consecutive TCP orientations, in degrees).
        A score above ~90° is considered sufficient for a good calibration.
        """
        if len(self._R_tcp) < 2:
            return 0.0
        total = 0.0
        for i in range(len(self._R_tcp) - 1):
            R_rel = self._R_tcp[i].T @ self._R_tcp[i + 1]
            cos_a = (np.trace(R_rel) - 1.0) / 2.0
            cos_a = float(np.clip(cos_a, -1.0, 1.0))
            total += math.degrees(math.acos(cos_a))
        return total
