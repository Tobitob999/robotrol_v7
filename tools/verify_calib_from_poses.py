"""
Simple verification tool for base_T_board from captured poses.
Usage: python -m tools.verify_calib_from_poses or run directly.

- Looks for data/capture_poses.json
- Attempts to compute base_T_board from P0, P1, P2 captures (keys 'p0','p1','p2')
- Compares result to configs/calibration.json base_T_board if present
- Optionally runs a small Monte-Carlo stability test
"""
import json
import math
import os
import sys
from statistics import mean

BASE = os.path.dirname(os.path.dirname(__file__))
DATA_PATH = os.path.join(BASE, "data", "capture_poses.json")
CALIB_PATH = os.path.join(BASE, "configs", "calibration.json")


def load_captures():
    if not os.path.isfile(DATA_PATH):
        print("No capture file:", DATA_PATH)
        return []
    with open(DATA_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def find_latest_for_key(captures, key):
    for e in reversed(captures):
        if e.get("type") == "board_point" and e.get("key") == key:
            return e
    return None


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def normalize(v):
    n = norm(v)
    if n < 1e-9:
        return None
    return [x / n for x in v]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def mat_from_axes(x_axis, y_axis, z_axis, t):
    R = [
        [x_axis[0], y_axis[0], z_axis[0], t[0]],
        [x_axis[1], y_axis[1], z_axis[1], t[1]],
        [x_axis[2], y_axis[2], z_axis[2], t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return R


def rot_error_deg(Ra, Rb):
    t = 0.0
    for i in range(3):
        for j in range(3):
            t += Ra[i][j] * Rb[i][j]
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
    R = mat_from_axes(x_axis, y_axis, z_axis, p0)
    return R


def load_config_base_T_board():
    if not os.path.isfile(CALIB_PATH):
        return None
    with open(CALIB_PATH, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    return cfg.get("base_T_board")


def compare(mat_a, mat_b):
    if not mat_a or not mat_b:
        print("Missing matrix to compare")
        return
    ta = [mat_a[0][3], mat_a[1][3], mat_a[2][3]]
    tb = [mat_b[0][3], mat_b[1][3], mat_b[2][3]]
    pos_err = math.sqrt(sum((ta[i]-tb[i])**2 for i in range(3)))
    Ra = [row[:3] for row in mat_a[:3]]
    Rb = [row[:3] for row in mat_b[:3]]
    rot_err = rot_error_deg(Ra, Rb)
    print(f"Position error: {pos_err:.3f} mm, Rotation error: {rot_err:.3f} deg")


def monte_carlo_stability(p0, p1, p2, runs=100, sigma=1.0):
    import random
    errs = []
    for _ in range(runs):
        def noisy(p):
            return [p[i] + random.gauss(0, sigma) for i in range(3)]
        try:
            Rn = compute_from_p0p1p2(noisy(p0), noisy(p1), noisy(p2))
            errs.append(Rn)
        except Exception:
            continue
    # compute mean translation spread
    if not errs:
        print("Monte-Carlo produced no valid trials")
        return
    ts = [ [m[0][3], m[1][3], m[2][3]] for m in errs ]
    diffs = [ math.sqrt(sum((t[i]-ts[0][i])**2 for i in range(3))) for t in ts ]
    print(f"Monte-Carlo {len(errs)} trials, translation std-approx p95 ~ {sorted(diffs)[int(0.95*len(diffs))-1]:.3f} mm")


def main():
    caps = load_captures()
    if not caps:
        return
    p0e = find_latest_for_key(caps, "p0")
    p1e = find_latest_for_key(caps, "p1")
    p2e = find_latest_for_key(caps, "p2")
    if not (p0e and p1e and p2e):
        print("Need captures for p0, p1, p2. Found:")
        print("p0:", bool(p0e), "p1:", bool(p1e), "p2:", bool(p2e))
        return
    p0 = p0e.get("tcp_mm")
    p1 = p1e.get("tcp_mm")
    p2 = p2e.get("tcp_mm")
    print("P0", p0)
    print("P1", p1)
    print("P2", p2)
    mat = compute_from_p0p1p2(p0, p1, p2)
    print("Computed base_T_board:")
    for r in mat:
        print(" ", [f"{x:.4f}" for x in r])
    cfg_mat = load_config_base_T_board()
    if cfg_mat:
        print("Comparing to configs/calibration.json base_T_board:")
        compare(mat, cfg_mat)
    print("Running small Monte-Carlo stability test...")
    monte_carlo_stability(p0, p1, p2, runs=200, sigma=0.35)

if __name__ == "__main__":
    main()
