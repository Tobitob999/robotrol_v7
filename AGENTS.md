# AGENTS.md

## Projektueberblick
- Repository: `d:\VSCode\robotrol_v7`
- Typ: Python-Steuerzentrale fuer 6-DoF-Roboterarme mit FluidNC/GRBL
- Aktueller Haupteinstieg laut README: `Robotrol_FluidNC_v7_3.py`
- Hauptbereiche: Robotiksteuerung, Kinematik (FK/IK), Vision/Kalibrierung, Pick&Place, Simulation, Self-Learning

## Ordnerinhalt (Analyse)
- `configs/`: System-, Robotik-, Kamera-, Kalibrier- und Feature-Flags (JSON/YAML)
- `control/`: Ausfuehrungspipeline, FSM, Robot/Gripper/Transform-Logik
- `perception/`: Kamera-, Marker-, Pose- und Qualitaetsmodule
- `planning/`: Grasp-/Place-Planung und Typdefinitionen
- `simulation/`: Mock-World und Simulationsloop
- `chess/`: Schachbrett-/Figurenerkennung und Zustand
- `model/`: DH- und Frame-Konventionen
- `tools/`: Smoke-Checks, Kalibrier- und Validierungsskripte
- `data/`: Testdaten, Beispiele und Hilfsdateien
- `build/`, `dist/`, `__pycache__/`: Build-/Cache-Artefakte

## Relevante Top-Level Dateien
- `Robotrol_FluidNC_v7_3.py`: Hauptanwendung (aktuellster v7.x Einstieg)
- `tcp_world_kinematics_frame.py`: TCP/IK-Sequenzen und Bewegungslogik
- `pickplace_ui.py`: Pick&Place UI + Board-/Auto-Kalibrier-Workflows
- `autocalib_v1.py`: Hand-Eye-Autokalibrierung
- `README.md`: Start, Features, Struktur
- `CHANGELOG_V7_3.md`: letzte dokumentierte Kern-Aenderungen

## Aktueller Stand (2026-02-28)
- Projekt arbeitet auf v7.3-Niveau (README + `Robotrol_FluidNC_v7_3.py`).
- Changelog v7.3 dokumentiert Kinematik-/IK-Fixes (MPos/WPos-Trennung, EB300-B-Achse, glatteres TCP-Gamepad-Jogging).
- Aeltere Statusdateien (`STATUS_CLAUDE.md`, `STATUS_PICKPLACE_V7_2.md`) enthalten teils historischen Stand (v7.0/v7.2) und sind nicht voll synchron zum v7.3-Einstieg.
- Lokale Arbeitskopie ist nicht clean (uncommitted Aenderungen vorhanden).

## Session 2026-02-28: Kinematics/Fixed-TCP Settings Persistenz

### 1. Kinematics + Fixed TCP Settings Save/Load (neu)
Neues Persistenz-System fuer alle Kinematics-Tab-Parameter und Fixed-TCP-Einstellungen.
- **Profil-Key**: `"kinematics_settings"` → `{"kinematics": {...}, "fixed_tcp": {...}}`
- **Kinematics-Tab** (`tcp_world_kinematics_frame.py`):
  - `get_settings()` / `apply_settings(data)` — sammelt/setzt alle Parameters + Gripper-Sequenz Felder
  - Gespeicherte Felder: D_ret, D_work, feed, pitch_override, roll_override, invert_u, anchor_mode, use_gripper, grip_s, grip_pause_before_ms, grip_pause_after_ms
  - "Save Settings" / "Load Settings" Buttons in der Button-Leiste (Zeile ~411-420)
  - Proxy-Methoden in `TcpWorldKinematicsTabs` (Zeile ~1752-1759)
- **Fixed TCP** (`Robotrol_FluidNC_v7_3.py`):
  - `get_fixed_tcp_settings()` (Zeile ~4450) — 19 Felder: enabled, roll, pitch, yaw, step, feed, exec_on_release, tcp_gcode_mode, mode, gamepad_mode, gp_xy_step, gp_z_step, max_dist, point_dx/dy/dz, gp_invert_x/y/z
  - `apply_fixed_tcp_settings(data)` (Zeile ~4474) — Restore aller Felder
  - `save_all_kinematics_settings()` / `load_all_kinematics_settings()` — kombiniert beide Bereiche
- **Automatisches Laden**:
  - Bei Profilwechsel via `apply_profile()` (Zeile ~4416)
  - Beim App-Start am Ende von `ExecuteApp.__init__()` (Zeile ~824)

### 2. Gamepad-Achsen-Invertierung (Fixed TCP)
3 Checkboxen "Invert Gamepad: X Y Z" im Fixed TCP Bereich.
- **Variablen**: `fixed_tcp_gp_invert_x/y/z` (BooleanVar, Zeile ~1108-1110)
- **UI**: Eigene Zeile (`row_gp_inv`) in `fixed_wrap`, zwischen Gamepad-XYZ-Zeile und Mode-Zeile (Zeile ~1364)
- **Logik**: Invertiert dx/dy/dz in `move_fixed_tcp_gamepad()` (Zeile ~5728-5732)
- **Persistenz**: In `get_fixed_tcp_settings` / `apply_fixed_tcp_settings` enthalten

### 3. Bugfix: Settings nicht beim App-Start geladen
- **Problem**: `load_all_kinematics_settings()` wurde nur bei Profilwechsel (`apply_profile`) aufgerufen, nicht beim initialen App-Start. Alle gespeicherten Werte (step, feed, invert flags etc.) wurden beim Start ignoriert.
- **Fix**: `self.load_all_kinematics_settings()` am Ende von `ExecuteApp.__init__()` nach `_build_ui()` eingefuegt (Zeile ~824-827).

### Offene Punkte / bekannte Probleme
- Die "Invert Gamepad" Checkboxen-Zeile war in der GUI nicht sichtbar, weil sie zunaechst als Grid-Row in `row_opts` platziert wurde (abgeschnitten). Dann in `gp_sens_frame` verschoben (horizontal abgeschnitten). Zuletzt als eigene `pack`-Zeile (`row_gp_inv`) in `fixed_wrap` — muss nach App-Neustart geprueft werden, ob sie jetzt sichtbar ist.

## Session 2026-02-28: Homing Safety Lock

### Problem
Roboter (EB300/EB15) durch versehentliches Homing beschaedigt. Diese Roboter haben keine Endstops — `$H` fuehrt zu unkontrollierter Bewegung.

### Loesung: Defense-in-Depth (3 Ebenen)
Nur `Robotrol_FluidNC_v7_3.py` geaendert. Kein neues JSON-Flag — nutzt bestehendes `has_endstops: false` aus EB300.json/EB15_red.json.

- **Helper**: `_is_homing_command(cmd)` (Zeile ~192) — Regex `^\$H([XYZABC])?$`, praeziser als `startswith("$H")`
- **Ebene 1 (UI)**: `_apply_profile_runtime_flags()` setzt `can_global_home=False` und `can_axis_home=False` wenn `profile_has_endstops==False` → Buttons grau
- **Ebene 2 (Command)**: Guards in `send_now()` (mit messagebox.showwarning), `_on_cli_return()`, `cli_send_now()` — blockt `$H` mit Log-Meldung
- **Ebene 3 (Queue)**: Guard in `worker()` — ueberspringt `$H` in geladenen Programmen mit Log

### Nicht gesperrt
- "Zero" (G92) — Software-Null, sicher
- "Go Home" / "Home->Q" — G1-Bewegung zu X0Y0Z0, kein Hardware-Homing
- "$X" (Unlock) — Entriegeln, kein Homing

### Betroffene Profile
| Profil | `has_endstops` | Homing |
|--------|---------------|--------|
| Moveo | true (default) | erlaubt |
| EB300 | false | gesperrt |
| EB15_red | false | gesperrt |

## Lokale uncommitted Aenderungen (git status)
- Geaendert: `EB15_red.json`, `EB300.json`, `Moveo.json`, `Robotrol_FluidNC_v7_3.py`, `configs/settings.json`, `pickplace_ui.py`, `tcp_world_kinematics_frame.py`
- Untracked: `.claude/settings.json`, `AGENTS.md`, `robotrol_v7.code-workspace`

## Session 2026-03-01: G92/WCO Fix + EB300 post_transform Recovery

### G92 Zero Fix (WCO-based)
- **Problem**: FluidNC with $10=1 only sends MPos, never WPos. Old code set UI to 0 on G92 but next status report immediately overwrote it back to MPos.
- **Fix**: Parse `WCO:` field from status reports, compute `WPos = MPos - WCO`. Pre-set `_wco = _mpos` in `do_zero()` for immediate zero display.
- **Reset triggers**: $H (homing), Ctrl+X (stop_abort), Grbl boot message, ALARM
- **Files changed**: `Robotrol_FluidNC_v7_3.py` (status parser, do_zero, stop_abort)

### CRITICAL BUG FIX: EB300 post_transform restored
- **Root cause**: Session 2026-02-28 removed `sim_theta_offset_deg` from EB300.json when cleaning up `sim_theta_scale`. The entire `post_transform` block was replaced with just `{"mirror_x": false}`.
- **Impact**: ALL kinematics broken — FK 90 degrees off, TCP sequences in wrong direction, gamepad IK diverging/jerky.
- **Fix**: Restored `sim_theta_offset_deg: {A:-90, X:-90, Z:-90}` and `sim_theta_scale: {}` to EB300.json.
- **Prevention**: Created `CLAUDE.md` with mandatory FK verification rule after any profile change.

### New: CLAUDE.md project rules
- Mandatory FK verification after profile JSON changes
- Reference FK values for EB300 and Moveo at MPos=0
- post_transform protection rules

## Kurzempfehlung fuer den naechsten Agenten
1. **LESE ZUERST CLAUDE.md** — enthalt kritische Regeln fuer Profilaenderungen.
2. Vor inhaltlichen Aenderungen zuerst Zielbasis klaeren: `v7.3` ist aktuell, v7.0/v7.2-Dokumente nur als Historie lesen.
3. **NIEMALS `post_transform` in Profil-JSONs aendern** ohne FK-Verifikation und User-Bestaetigung.
4. Nach jeder funktionalen Aenderung mindestens `python -m py_compile <file>.py` ausfuehren.
5. Bei Kalibrier-/IK-Themen immer zusammen pruefen: `pickplace_ui.py`, `autocalib_v1.py`, `tcp_world_kinematics_frame.py`.
6. **Homing Safety Lock**: Bei neuen Profilen immer `"features": {"has_endstops": false}` setzen, wenn der Roboter keine Endstops hat.
7. **FK-Referenzwerte** (MPos=0): EB300 → TCP=(179,0,860), Moveo → TCP=(-0,0,900). Bei Abweichung: STOPP.

