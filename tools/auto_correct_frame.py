"""
Auto-correct calibration frame by testing axis permutations, sign flips and 90° Z-rotations.
Loads computed base_T_board from captures (P0/P1/P2) and compares to configs/calibration.json.
Prints top candidates with position and rotation error.
"""
import os
import json
import math
from itertools import permutations, product

BASE = os.path.dirname(os.path.dirname(__file__))
CAP_PATH = os.path.join(BASE, "data", "capture_poses.json")
CALIB_PATH = os.path.join(BASE, "configs", "calibration.json")

from statistics import mean


# ---- Inline helpers (copied from verify_calib_from_poses.py) -----------------
def load_captures():
    if not os.path.isfile(CAP_PATH):
        return []
    with open(CAP_PATH, "r", encoding="utf-8") as f:
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
    R = [
        [x_axis[0], y_axis[0], z_axis[0], p0[0]],
        [x_axis[1], y_axis[1], z_axis[1], p0[1]],
        [x_axis[2], y_axis[2], z_axis[2], p0[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
    return R


def rot_error_deg(R_a, R_b):
    t = (
        R_a[0][0] * R_b[0][0] + R_a[1][0] * R_b[1][0] + R_a[2][0] * R_b[2][0]
        + R_a[0][1] * R_b[0][1] + R_a[1][1] * R_b[1][1] + R_a[2][1] * R_b[2][1]
        + R_a[0][2] * R_b[0][2] + R_a[1][2] * R_b[1][2] + R_a[2][2] * R_b[2][2]
    )
    cos_theta = max(-1.0, min(1.0, (t - 1.0) * 0.5))
    return math.degrees(math.acos(cos_theta))

# ---- end inline helpers -----------------------------------------------------


def mat3_mul(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def mat3_vec_mul(A, v):
    return [sum(A[i][k]*v[k] for k in range(3)) for i in range(3)]


def try_candidates(Rc, tc, Rref, tref):
    axes = [0,1,2]
    best = []
    perms = list(permutations(axes))
    signs = list(product([1,-1], repeat=3))
    zrots = [0, 90, 180, 270]
    for perm in perms:
        for sign in signs:
            # build permutation+sign matrix P such that new_axis_i = sign[i] * e_perm[i]
            P = [[0]*3 for _ in range(3)]
            for i in range(3):
                P[i][perm[i]] = sign[i]
            for zdeg in zrots:
                zr = math.radians(zdeg)
                cz, sz = math.cos(zr), math.sin(zr)
                Rz = [[cz, -sz, 0],[sz, cz, 0],[0,0,1]]
                # Candidate transform M = Rz * P
                M = mat3_mul(Rz, P)
                # Apply: R_new = M * Rc; t_new = M * tc
                Rnew = mat3_mul(M, Rc)
                tnew = mat3_vec_mul(M, tc)
                # compare to ref
                pos_err = math.sqrt(sum((tnew[i]-tref[i])**2 for i in range(3)))
                rot_err = rot_error_deg([list(row) for row in Rnew], [list(row) for row in Rref])
                best.append((pos_err, rot_err, perm, sign, zdeg, M, Rnew, tnew))
    best.sort(key=lambda x: (x[0], x[1]))
    return best


def load_ref():
    if not os.path.isfile(CALIB_PATH):
        return None
    with open(CALIB_PATH, 'r', encoding='utf-8') as f:
        cfg = json.load(f)
    return cfg.get('base_T_board')


def main():
    caps = load_captures()
    if not caps:
        print('No captures found')
        return
    p0 = find_latest_for_key(caps, 'p0')
    p1 = find_latest_for_key(caps, 'p1')
    p2 = find_latest_for_key(caps, 'p2')
    if not (p0 and p1 and p2):
        print('Need p0/p1/p2 captures')
        return
    pc0 = p0.get('tcp_mm')
    pc1 = p1.get('tcp_mm')
    pc2 = p2.get('tcp_mm')
    Rc = compute_from_p0p1p2(pc0, pc1, pc2)
    tc = [Rc[0][3], Rc[1][3], Rc[2][3]]
    Rref_mat = load_ref()
    if not Rref_mat:
        print('No reference base_T_board in configs')
    else:
        Rref = [row[:3] for row in Rref_mat[:3]]
        tref = [Rref_mat[0][3], Rref_mat[1][3], Rref_mat[2][3]]
        print('Testing candidates...')
        best = try_candidates(Rc[:3], tc, Rref, tref)
        for i, (p_err, r_err, perm, sign, zdeg, M, Rnew, tnew) in enumerate(best[:8]):
            print(f'# {i+1}: pos_err={p_err:.3f} mm rot_err={r_err:.3f} deg zrot={zdeg} perm={perm} sign={sign}')
            print(' tnew=', [f'{x:.3f}' for x in tnew])
        # show best full matrix
        p_err, r_err, perm, sign, zdeg, M, Rnew, tnew = best[0]
        print('\nBest candidate:')
        print(f'pos_err={p_err:.3f} mm rot_err={r_err:.3f} deg  zrot={zdeg} perm={perm} sign={sign}')
        mat = [
            [Rnew[0][0], Rnew[0][1], Rnew[0][2], tnew[0]],
            [Rnew[1][0], Rnew[1][1], Rnew[1][2], tnew[1]],
            [Rnew[2][0], Rnew[2][1], Rnew[2][2], tnew[2]],
            [0.0,0.0,0.0,1.0]
        ]
        print('\nSuggested corrected base_T_board:')
        for r in mat:
            print(' ', [f'{x:.6f}' for x in r])

if __name__ == '__main__':
    main()
