"""
Auto-correct calibration frame by testing axis permutations, sign flips and 90deg Z-rotations.
Loads computed base_T_board from captures (P0/P1/P2) and compares to configs/calibration.json.
Prints top candidates with position and rotation error.
"""
import math
from itertools import permutations, product

from tools.math_utils import (
    load_captures, find_latest_for_key, compute_from_p0p1p2,
    rot_error_deg, load_config_base_T_board,
)


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
            P = [[0]*3 for _ in range(3)]
            for i in range(3):
                P[i][perm[i]] = sign[i]
            for zdeg in zrots:
                zr = math.radians(zdeg)
                cz, sz = math.cos(zr), math.sin(zr)
                Rz = [[cz, -sz, 0],[sz, cz, 0],[0,0,1]]
                M = mat3_mul(Rz, P)
                Rnew = mat3_mul(M, Rc)
                tnew = mat3_vec_mul(M, tc)
                pos_err = math.sqrt(sum((tnew[i]-tref[i])**2 for i in range(3)))
                rot_err = rot_error_deg([list(row) for row in Rnew], [list(row) for row in Rref])
                best.append((pos_err, rot_err, perm, sign, zdeg, M, Rnew, tnew))
    best.sort(key=lambda x: (x[0], x[1]))
    return best


def main():
    caps = load_captures(verbose=False)
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
    Rref_mat = load_config_base_T_board()
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
