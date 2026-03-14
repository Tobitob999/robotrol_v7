"""
Simple verification tool for base_T_board from captured poses.
Usage: python -m tools.verify_calib_from_poses or run directly.

- Looks for data/capture_poses.json
- Attempts to compute base_T_board from P0, P1, P2 captures (keys 'p0','p1','p2')
- Compares result to configs/calibration.json base_T_board if present
- Optionally runs a small Monte-Carlo stability test
"""
import math

from tools.math_utils import (
    load_captures, find_latest_for_key, compute_from_p0p1p2,
    rot_error_deg, load_config_base_T_board, norm,
)


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
