# Cleanup Commit Plan v7.0

## Scope
This plan is for `D:/VSCode/robotrol_v7` after migration from v6.2/v6.3 naming to v7.0.

## Done
- Main entry renamed to `Robotrol_FluidNC_v7_0.py`.
- Window/app version text switched to `V7.0`.
- CI compile target switched to `Robotrol_FluidNC_v7_0.py`.
- PyInstaller spec switched to `Robotrol_FluidNC_v7_0.spec` and new script name.
- Compatibility launcher added: `Robotrol_FluidNC_v7_0_launcher.py`.
- README run command and file references updated to v7.0.
- `pickplace_ui.py` help text updated from `v6.3` to `v7.0`.
- Legacy/IDE/cache/temp files removed from active folder.
- Full manual created: `BEDIENUNGSANLEITUNG_V7_0.md`.

## Pre-Commit Checks
1. Run syntax checks:
   `python -m py_compile Robotrol_FluidNC_v7_0.py Robotrol_FluidNC_v7_0_launcher.py board_pose_v1.py camera_capturev_v1_1.py fluidnc_updater_v2.py gamepad_block_v3.py ik_rotosim.py pickplace_ui.py robosim_visualizer_v90.py tcp_pose_module_v3.py tcp_world_kinematics_frame.py control/config.py tools/check_english_text.py tools/smoke_checks.py`
2. Optional smoke checks:
   `python tools/smoke_checks.py`
3. Start app once:
   `python Robotrol_FluidNC_v7_0.py`

## Commit Proposal
- Commit title:
  `release: finalize robotrol v7.0 folder and documentation`
- Include:
  - v7.0 rename/update files
  - CI/spec/README updates
  - manual and function index
  - cleanup of non-required artifacts

## Optional Follow-up
- If you want zero compatibility debt, remove `Robotrol_FluidNC_v7_0_launcher.py` in a later cleanup commit.
