# Robotrol

Robotrol is a Python-based control center for 6-DoF robotic arms running FluidNC/GRBL. It combines serial control, a live queue, gamepad jogging, DH-based kinematics (FK/IK + TCP pose), vision tools, and optional OTA configuration.

## Features
- FluidNC/GRBL serial control with queue and live status
- Gamepad jogging with configurable profiles
- DH6 FK/IK and TCP pose monitor
- TCP sequence generator (retreat/target/retreat)
- OpenCV-based camera and board-vision tools
- UDP mirror to 3D visualizer
- Profile system (Moveo, EB15_red, EB300)

## Requirements
- OS: Windows/Linux/macOS (tested primarily on Windows)
- Python 3.10+
- Python packages: pyserial, pygame, Pillow, numpy, opencv-python

## Run
```powershell
python Robotrol_FluidNC_v7_3.py
```

## Profiles
Profiles live in `Moveo.json`, `EB15_red.json`, and `EB300.json`. Use the profile selector in the UI to switch.

## Repo layout
- `Robotrol_FluidNC_v7_3.py` main UI core (window title: V7.3)
- `Robotrol_FluidNC_v7_0_launcher.py` optional compatibility launcher entrypoint
- `config_profiles.py` profile loader
- `Moveo.json`, `EB15_red.json`, `EB300.json` profiles
- `tcp_pose_module_v3.py` FK/TCP panel
- `tcp_world_kinematics_frame.py` TCP sequence generator
- `fluidnc_updater_v2.py` OTA/serial tools
- `camera_capturev_v1_1.py`, `board_pose_v1.py`, `chess_vision_ui.py`, `pickplace_ui.py` vision/calibration UI
- `configs/`, `chess/`, `perception/`, `control/`, `planning/`, `simulation/`, `model/` supporting modules/configs

## Notes
- Build artifacts (`build/`, `dist/`) and caches are not tracked in Git.

## Calibration
- See `BEDIENUNGSANLEITUNG_V7_0.md` for full v7.0 operation, calibration and function documentation.
- Project language policy is defined in `configs/project_flags.json` (`language = en`).
- Ongoing migration tracker: `LANGUAGE_MIGRATION_EN.md`.
- Self-learning concept: `SELF_LEARNING_TNT_CONCEPT.md`.

## Self-Learning (TNT)
- In `Pick&Place -> Run`, use `Self-Learning (TNT)` controls:
  - enable learning
  - start in `shadow` mode for validation
  - switch to `active` only after stable tests
  - use `Reset Policy` to return to baseline

## Quality Checks
- Run language consistency check:
  - `python tools/check_english_text.py`
  - `python tools/check_english_text.py --strict` (non-zero exit on findings; CI-friendly)
- Run full smoke checks (language + mock simulation):
  - `python tools/smoke_checks.py`
- Hardware validation checklist:
  - `HARDWARE_VALIDATION_CHECKLIST.md`



