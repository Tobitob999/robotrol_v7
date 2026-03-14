# DH Table (MOVEo 6DOF)

Conventions and joint order are defined in `model/frames_convention.md`.
All values here are in meters and radians.

| i | axis | a_i (m) | alpha_i (rad) | d_i (m) | theta_offset_i (rad) | source_note |
|---|------|---------|---------------|---------|----------------------|-------------|
| 1 | A | 0.000 | +1.57079632679 | 0.240 | 0.00000000000 | Set per provided DH table: theta=0, alpha=+90deg, d=L1, a=0. |
| 2 | X | 0.230 | 0.00000000000 | 0.000 | +1.57079632679 | Set per provided DH table: theta=+90deg, alpha=0, d=0, a=L2. |
| 3 | Y | 0.000 | -1.57079632679 | 0.000 | -1.57079632679 | Set per provided DH table: theta=-90deg, alpha=-90deg, d=0, a=0. |
| 4 | B | 0.000 | +1.57079632679 | 0.250 | 0.00000000000 | Set per provided DH table: theta=0, alpha=+90deg, d=L3, a=0. |
| 5 | Z | 0.000 | -1.57079632679 | 0.000 | 0.00000000000 | Set per provided DH table: theta=0, alpha=-90deg, d=0, a=0. |
| 6 | C | 0.000 | 0.00000000000 | 0.180 | 0.00000000000 | Set per provided DH table: theta=0, alpha=0, d=L4, a=0. |
