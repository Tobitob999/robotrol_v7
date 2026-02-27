# Robotrol V7.3 — Changelog

## Summary
Kinematics overhaul for EB300: correct MPos/WPos separation, fixed tool direction,
B-axis post-transform correction, and smoother gamepad TCP jogging.

## Changes

### 1. MPos / WPos separation (IK always uses real machine position)
- **New `_mpos` dict** in `ExecuteApp.__init__` — always stores real machine
  positions (MPos) from the status parser, independent of G92 offset.
- **Status parser** always fills `_mpos` from MPos; `axis_positions` follows
  `_use_wpos` flag for UI display (WPos after G92, MPos otherwise).
- **IK/FK reads `_mpos`**: `_get_current_joint_list()`,
  `move_tcp_pose()`, `preview_tcp_gcode()` all use `_mpos` instead of
  `axis_positions`. This prevents the IK from thinking the arm is at
  home (all-zero) after a G92 null-set.

### 2. G92 zero-setting fix
- `do_zero()` sends `G92 X0 Y0 Z0 A0 B0 C0` and sets `_use_wpos = True`.
- UI immediately shows 0.000 for all axes (WPos).
- Homing (`$H`) resets `_use_wpos = False`.
- TCP sequence wraps IK moves in `G92.1` / `G92.3` (suspend/restore WCO)
  so FluidNC interprets absolute joint angles as MPos.

### 3. Log "?" filter
- FluidNC echoes the `?` status query character back. Added `line == "?"`
  to the serial response filter to suppress it from the log output.

### 4. FK geometry fix for EB300 (L3 extraction)
- `_geom_from_model()` in `tcp_pose_module_v3.py` now checks `Y.a_mm`
  first (EB300: 318 mm forearm) before falling back to the `d_mm` scan
  after Y. Previously L3 was incorrectly 122 mm (Z.d) for EB300.

### 5. B-axis post-transform correction
- Removed `sim_theta_scale: {"B": -1.0}` from `EB300.json`.
  The -1.0 scale negated the B joint in the DH model, which caused the
  FK orientation and IK output for B to be inverted.
  Effective B scale is now 1.0 (no negation).

### 6. Tool direction in TCP sequence
- `_get_tool_axis_world()` returns `+Z` of the tool frame (third column
  of the rotation matrix). With the corrected B-axis, `+Z` now points
  in the correct approach direction. The `invert_u` checkbox remains as
  a UI override.

### 7. No-op RET move eliminated
- When `anchor = "retreat"` (default), the first RET move was a G1 to
  the current position — causing a tiny unwanted movement due to
  rounding. Now emitted as a comment (`; RET = current pos (no move)`)
  instead of an actual G1 command.

### 8. IK solver consolidation
- `move_tcp_pose()` and `preview_tcp_gcode()` now use the shared
  `_solve_ik_iterative()` instead of inline iteration loops.
- Tuned: `max_iters` 50 → 80, `max_step` 5 → 8 (for full solves).
- New `max_iters` parameter on `move_tcp_pose()` allows callers to
  request fewer iterations for small incremental moves.

### 9. Gamepad TCP mode — smoother jogging
- **Stale-command guard**: `_gp_tcp_busy` flag in
  `move_fixed_tcp_gamepad()` drops new gamepad commands while a
  previous IK solve is still running. Prevents command queue buildup.
- **Fewer IK iterations**: Gamepad calls `move_tcp_pose(max_iters=15)`
  instead of 80. Small incremental moves converge in 5–10 iterations.
- **Slower TCP poll rate**: TCP-mode gamepad polling at 60 ms (≈16 Hz)
  instead of 25 ms (40 Hz), giving the IK solver enough time per cycle.
  Regular joint-jog mode (Mode 0) stays at 40 Hz.

## Files changed
| File | What |
|------|------|
| `Robotrol_FluidNC_v7_3.py` | Main app (was v7_2): _mpos, G92, log filter, gamepad guard |
| `tcp_world_kinematics_frame.py` | IK solver, tool axis, TCP sequence, move_tcp_pose |
| `tcp_pose_module_v3.py` | FK L3 fix for EB300 |
| `gamepad_block_v3.py` | TCP-mode poll rate |
| `EB300.json` | Removed B:-1.0 scale, updated manual_zero_reference |
| `README.md` | Updated run command and repo layout |
