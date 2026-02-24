# Frames and DH Conventions (MOVEo 6DOF)

- Convention: Classical DH (Craig)
  T_i = Rz(theta_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
- Units: length = m, angle = rad
- Joint order: A, X, Y, B, Z, C
- Right-handed frames; rotation sign uses right-hand rule
- Base frame {0}: DH base used by the FK implementation in `tcp_pose_module_v3.py`
- Flange frame {6}: after joint C
- TCP frame {TCP}: optional fixed transform from {6}; default is identity
- Joint zero: theta_i = q_i + theta_offset_i (see `model/dh.json`)
- Angle wrapping: canonical range [-pi, +pi], clamp to limits when enforcing

Legacy UI note:
- The GUI applies an X mirror to match historical world coordinates.
  This mirror is a compatibility transform and is not part of the DH chain.
