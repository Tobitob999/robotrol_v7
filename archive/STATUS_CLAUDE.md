# STATUS FOR CLAUDE (READ FIRST)

## Project
- Path: `D:/VSCode/robotrol_v7`
- Target version: `v7.0`
- Purpose: clean release folder migrated from old v6.2/v6.3 naming.

## Current State
- Main app file is now `Robotrol_FluidNC_v7_0.py`.
- App title/version string is `Robotrol V7.0`.
- Build spec is `Robotrol_FluidNC_v7_0.spec`.
- CI workflow compiles `Robotrol_FluidNC_v7_0.py` (old v6 targets removed).
- README run command points to `python Robotrol_FluidNC_v7_0.py`.
- Full user manual exists: `BEDIENUNGSANLEITUNG_V7_0.md`.
- Function inventory exists: `tools/function_index_v7_0.json`.

## Cleanup State
- Non-required artifacts were removed from active folder:
  - IDE folders (`.claude`, `.vscode`)
  - caches (`__pycache__`)
  - temporary TNT images
  - legacy launcher file for v6.3

## Important Files To Check First
1. `Robotrol_FluidNC_v7_0.py`
2. `README.md`
3. `.github/workflows/ci.yml`
4. `Robotrol_FluidNC_v7_0.spec`
5. `BEDIENUNGSANLEITUNG_V7_0.md`

## Known Note
- `BEDIENUNGSANLEITUNG_V7_0.md` contains section numbering like `6.2` as chapter index; this is documentation structure, not app version.

## Next Recommended Actions
1. Run `python tools/smoke_checks.py`.
2. Start app once with `python Robotrol_FluidNC_v7_0.py`.
3. If all good, commit using `CLEANUP_COMMIT_PLAN_V7_0.md`.

## Kurzuebersicht Funktionalitaet
- Serial Control: Verbindung zu FluidNC/GRBL, Live-Status, direkte G-Code-Kommandos.
- Motion/Queue: Programm-Queue mit Start/Pause/Resume/Abort und manuellen Achsfahrten.
- Kinematics: FK/IK, TCP-Pose-Anzeige, TCP-Sequenzvorschau und -Ausfuehrung.
- Gamepad: Konfigurierbares Mapping, Jogging-Modi und Geschwindigkeitsprofile.
- Vision/Calibration: Kamera-Feed, Board-/Marker-Kalibrierung, Transform-Validierung.
- Pick&Place: Zyklusausfuehrung mit Checks fuer Bounds/Workspace.
- Self-Learning (TNT): Shadow/Active-Modus mit Policy-Management und Rollback-Mechanismen.
- OTA/Config: FluidNC-Inspector, Config/Firmware-Upload und Reboot-Funktionen.
