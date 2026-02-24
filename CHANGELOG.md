# Changelog

## [Unreleased] - 2026-02-23

### Added
- `pickplace_ui.py`: Square Fit calibration workflow — capture individual chess squares with TCP, fit `base_T_board` via Kabsch algorithm (3+ points).
- `pickplace_ui.py`: Board Test Targets UI — move robot to named squares (e.g. `a1 h1 e4`) or XY coordinates with configurable Z-safe/Z-touch, includes dry-run mode.
- `pickplace_ui.py`: Workspace bounds check (`_check_workspace_bounds`) — configurable XY/Z limits in `configs/robot.json`; blocks pick/place if target is out of bounds.
- `pickplace_ui.py`: Calibration sanity check (`_check_calibration_sanity`) — validates detected object pose against board bounds and verifies `base_T_cam` is a valid rotation matrix (orthonormality + determinant).
- `pickplace_ui.py`: Detection overlay (`_update_detection_overlay`) — draws TNT contour into camera preview after each detect call.
- `pickplace_ui.py`: `_resolve_grasp_normal()` — reads `grasp_normal_axis`/`grasp_normal_sign` from `cube.yaml` instead of hardcoded `[0, 0, 1]`.
- `Robotrol_FluidNC_v7_0.py`: `supports_global_homing` capability flag for all backends (FluidNC, GRBL, custom).
- `Robotrol_FluidNC_v7_0.py`: `SerialClient.status_query_line()` — returns `"?"` for FluidNC/GRBL, `"(TM)"` for custom backend. Replaces hardcoded `"(TM)"` in status polling and connect flow.
- `configs/robot.json`: `workspace` block with `enabled`, `margin_mm`, and XYZ min/max bounds (disabled by default).
- `perception/pose_estimator.py` + `perception/tnt_detector.py`: Dynamic camera matrix scaling — automatically scales fx/fy/cx/cy when actual frame resolution differs from calibration resolution.
- `_CameraCaptureAdapter`: `color_order = "rgb"` attribute for correct color conversion downstream.

### Changed
- Camera resolution lowered from 1920×1080 to 640×480 everywhere (config default, `camera.json`, preview sizes).
- Preview sizes reduced from 420×320 to 300×225 in `CameraCapture`, `BoardPose`, and `ExecuteApp`.
- New camera intrinsics calibrated for 640×480: `fx=649.38`, `fy=802.33`, `cx=305.60`, `cy=234.66`.
- New `base_T_board` and `base_T_cam` matrices in `configs/calibration.json` and `configs/transforms.json`.
- `pickplace_ui.py`: `_attach_camera_capture()` now releases the old capture handle before switching, and restarts camera if needed.
- `pickplace_ui.py`: Pick plan now logs `obj`, `approach`, `grasp`, `lift` positions and `base_T_cam` for debugging.
- `Robotrol_FluidNC_v7_0.py`: `_apply_profile_runtime_flags()` uses `supports_global_homing` capability instead of `is_custom` attribute for global homing button state.
- `Robotrol_FluidNC_v7_0.py`: Endstop display no longer gated by `profile_has_endstops` (controlled solely by `supports_endstops`).
- `Robotrol_FluidNC_v7_0.py`: After connect, `_apply_profile_runtime_flags()` is called to update homing button states immediately.
- `board_pose_v1.py`: `start_camera()` / `stop_camera()` fixed — were incorrectly indented as local functions inside `__init__`, now proper class methods.
- `robosim_visualizer_v90.py`: Removed duplicate axis-limit and title setup block in `_draw_3d`.

### Fixed
- `configs/camera.json`: `image_size` corrected from `[432, 240]` to `[640, 480]` — the incorrect value caused false scaling of the camera matrix in `pose_estimator.py` and `tnt_detector.py`, leading to wrong 3D pose estimates.
- `Robotrol_FluidNC_v7_0.py`: Removed redundant `current_theme = {"dark": False}` inside the style `try` block (duplicate of the unconditional assignment below).
- `pickplace_ui.py` `_parse_board_targets`: Removed no-op `.replace(",", ",")` call.
- `pickplace_ui.py` `_capture_square_point`: Rank validation now checks range 1–8 (previously only checked `isdigit()`). Also normalises the stored key to the first two characters.
- `fluidnc_updater_v2.py`: Corrupted comment strings containing `"GRIP CLOSE"` cleaned up in `command()` docstring and `get_active_config()` regex.
- `gamepad_block_v3.py`: Corrupted comment `"GlGRIP CLOSEttung"` → `"Glättung"`.

