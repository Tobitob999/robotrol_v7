"""
Shared math utilities for calibration and verification tools.

Single source of truth for vector/matrix operations used across
auto_correct_frame.py and verify_calib_from_poses.py.
"""
import json
import math
import os

BASE = os.path.dirname(os.path.dirname(__file__))
CAP_PATH = os.path.join(BASE, "data", "capture_poses.json")
CALIB_PATH = os.path.join(BASE, "configs", "calibration.json")


# --- Vector operations ---

def norm(v):
    return math.sqrt(sum(x * x for x in v))


def normalize(v):
    n = norm(v)
    if n < 1e-9:
        return None
    return [x / n for x in v]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


# --- Matrix operations ---

def mat_from_axes(x_axis, y_axis, z_axis, t):
    return [
        [x_axis[0], y_axis[0], z_axis[0], t[0]],
        [x_axis[1], y_axis[1], z_axis[1], t[1]],
        [x_axis[2], y_axis[2], z_axis[2], t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def rot_error_deg(R_a, R_b):
    t = 0.0
    for i in range(3):
        for j in range(3):
            t += R_a[i][j] * R_b[i][j]
    cos_theta = max(-1.0, min(1.0, (t - 1.0) * 0.5))
    return math.degrees(math.acos(cos_theta))


def compute_from_p0p1p2(p0, p1, p2, cols=7, rows=7, square=30.0):
    vx = [p1[i] - p0[i] for i in range(3)]
    vy = [p2[i] - p0[i] for i in range(3)]
    x_axis = normalize(vx)
    y_axis = normalize(vy)
    if x_axis is None or y_axis is None:
        raise RuntimeError("Invalid points for axis computation")
    z_axis = normalize(cross(x_axis, y_axis))
    if z_axis is None:
        raise RuntimeError("Points collinear")
    y_axis = normalize(cross(z_axis, x_axis))
    return mat_from_axes(x_axis, y_axis, z_axis, p0)


# --- Data loading ---

def load_captures(cap_path=None, verbose=True):
    path = cap_path or CAP_PATH
    if not os.path.isfile(path):
        if verbose:
            print("No capture file:", path)
        return []
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def find_latest_for_key(captures, key):
    for e in reversed(captures):
        if e.get("type") == "board_point" and e.get("key") == key:
            return e
    return None


def load_config_base_T_board(calib_path=None):
    path = calib_path or CALIB_PATH
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    return cfg.get("base_T_board")
