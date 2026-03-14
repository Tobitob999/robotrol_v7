# Pick & Place – Arbeitsstand v7.2

Stand: 2026-02-25
Dateien: `pickplace_ui.py`, `tcp_world_kinematics_frame.py`, `Robotrol_FluidNC_v7_2.py`

---

## Erledigte Änderungen in v7.2

### A) Test Board Moves – Positionsfehler behoben (`pickplace_ui.py`)

**Problem:** Roboter fuhr bei "Test Board Moves" ohne Berücksichtigung der aktuellen Position los.

**Fix:**
- **Fail-early bei fehlendem MPos:** Startet nicht wenn `last_status["MPos"]` leer ist.
- **axis_positions aus live MPos primen:** Vor dem ersten IK-Call wird `kt.world.exec.axis_positions` direkt aus dem aktuellen MPos befüllt.
- **Safe-Z Retreat als ersten Schritt:** Roboter fährt zuerst bei unverändertem X/Y auf `z_safe`, bevor das erste Feld angefahren wird.
- **Hint-Refresh pro Move:** Nach jedem `_wait_for_idle()` wird MPos neu gelesen und `kw._initial_joints_next` gesetzt.
- **Warnung in `_get_current_joint_list`** (`tcp_world_kinematics_frame.py`): Loggt wenn alle Achsen = 0.0.

### B) Plausibilitätsprüfung base_T_board (`pickplace_ui.py`)

Zwei neue Methoden, automatisch nach "Compute base_T_board" und "Fit base_T_board":

| Methode | Checks |
|---|---|
| `_check_base_T_board_point_geometry(p0,p1,p2,x_len,y_len)` | Abstände P0→P1/P2 ±20 %, Winkel 90° ±15° |
| `_check_base_T_board_plausibility(base_T_board)` | det(R)>0, Brett-Neigung <45°, Δ-Translation <200 mm, Δ-Rotation <30° vs gespeicherter Kalibrierung |

Ausgabe im Log als `[OK]` / `[WARN]` mit konkretem Messwert und Hinweis auf Ursache.

### C) Pitch-Adaptation beim Anfahren (`pickplace_ui.py`)

**UI:** Im "Board Test Targets"-Frame:
- Checkbox: "Pitch adaptation (tilt outward from board centre)"
- Feld: "Max pitch (°)" – Default 35°

**Methode `_compute_approach_rpy(sq_x, sq_y, roll0, pitch0, yaw0)`:**
- Berechnet Offset (dx, dy) des Feldes vom Brettzentrum in Board-Koordinaten
- Kippmagnitude: `tilt = max_pitch × (d / half_diagonal)` – linear
- Kippachse in Board-Frame: `(-dy, dx, 0)` normiert → in Base-Frame transformiert
- Rodrigues-Rotation → neues RPY per `_rpy_from_R`
- Wird pro Feld in `_test_board_moves` aufgerufen; RPY wird im Log gezeigt

---

## Offene Punkte

### 1. Pitch-Vorzeichen / Orientierung bei der Anfahrt prüfen und kalibrieren

**Status:** Implementiert, noch nicht am echten Brett getestet.

**Problem:** Die Kipprichtung ("outward from centre") ist mathematisch korrekt definiert —
Kippachse = `(-dy, dx, 0)` in Board-Frame. Ob Vorzeichen und Richtung in der Praxis
"intuitiv plausibel" sind, hängt vom Roboteraufbau und der TCP-Definition ab.

**Nächste Schritte:**
- Dry-run mit "Pitch adaptation" aktiviert auf 3 Feldern (a1, e4, h8) durchführen.
- Im Log die ausgegebenen `rpy=(r,p,y)` Werte pro Feld vergleichen.
- Falls Kipprichtung falsch (Roboter kippt nach innen statt außen): Vorzeichen der Kippachse
  in `_compute_approach_rpy` umkehren: `tilt_axis_board = normalize_vector([dy, -dx, 0.0])`.
- Falls Kippebene falsch (z.B. nur Roll statt Pitch bewegt): Basisorientierung des TCP prüfen —
  möglicherweise muss `R_new = R0 @ R_tilt` statt `R_tilt @ R0` verwendet werden.

**Technische Details:**
`_compute_approach_rpy` in [pickplace_ui.py](pickplace_ui.py) ca. Zeile ~760.
Vorzeichen-Flip: eine Zeile ändern.

---

### 2. Erweiterung Testbereich – mehr Konfigurierbarkeit

**Status:** Noch nicht implementiert.

**Ideen / Anforderungen:**
- **Mehrere Zielsequenzen speicherbar:** Named presets ("Ecken", "Diagonale", "Alle Felder")
  als Dropdown + Save/Load aus JSON.
- **Schleifenanzahl:** Eingabefeld "Runs (Anzahl Wiederholungen)" – für Dauertest.
- **Pause zwischen Feldern:** Eingabefeld "Pause (s)" – für langsamen Ablauf / Beobachtung.
- **Z-Profil:** Statt fest z_safe + z_touch: optionales "Hover-Z" (zwischenebene für laterale
  Bewegung) um sicherer über Hindernisse zu fahren.
- **Abbruch-Button:** Während laufendem Test-Board-Moves Thread abbrechen.
  `_stop_event` in `_test_board_moves` checken.

---

### 3. Berechnung und Darstellung der Zielkoordinaten bei Feldeingabe

**Status:** Noch nicht implementiert.

**Anforderung:** Wenn der Nutzer ein Zielfeld (z.B. "e4") eingibt, soll sofort angezeigt werden:
- Board-Frame-Koordinaten des Feldzentrums (x_board, y_board)
- Base-Frame-Koordinaten (x_base, y_base, z_base) für z_safe und z_touch
- Vorschau-RPY (mit und ohne Pitch-Adaptation)

**Implementierungsplan:**
1. Neues `ttk.LabelFrame` "Target Preview" unter dem Squares-Eingabefeld.
2. Eingabe-Trace auf `_test_squares_var` (`.trace_add("write", ...)`) oder Button "Preview".
3. Für jedes geparste Feld: `_board_square_center` → `_board_to_base` → `_compute_approach_rpy` aufrufen.
4. Ergebnis in scrollbarer Textbox oder `ttk.Treeview` anzeigen:
   ```
   Feld | Board X | Board Y | Base X  | Base Y  | Base Z  | Roll | Pitch | Yaw
   e4   | 144.0   | 128.0   | 312.4   | -88.3   | 80.0    | 0.0  | -8.3  | 175.2
   ```
5. Kein Roboter-Move – rein informativer Vorschau-Modus.

**Voraussetzung:** base_T_board muss in der UI geladen sein.

---

### 4. Überprüfung und Test Auto-Calibration (`autocalib_v1.py` / pickplace_ui.py)

**Status:** Phase 1 implementiert (siehe CODEX_V7_1_RELEASE.md), noch nicht vollständig getestet.

**Offene Aufgaben:**

#### 4a. Funktionstest der Capture-Loop
- ArUco-Marker an TCP montieren (DICT_4X4_50 ID 0, 80 mm Seite).
- ≥8 Posen mit je ±25° Neigung, ±30° Rotation, ±30 mm Translation aufnehmen.
- Überprüfen ob `detect_marker_pose()` in `autocalib_v1.py` den Marker zuverlässig findet.
- Diversity-Score im Log kontrollieren (sollte >20° sein).

#### 4b. Solve + Apply testen
- "Solve" → `cv2.calibrateHandEye(TSAI)` aufrufen.
- Ergebnis `base_T_cam` (4×4) im Log und in der UI anzeigen.
- "Apply" → Wert in `configs/transforms.json["base_T_cam"]` schreiben.
- Vergleich: neues `base_T_cam` vs altes (aus transforms.json) – Δ-Translation + Δ-Rotation loggen.

#### 4c. Plausibilitätsprüfung base_T_cam (noch nicht vorhanden)
- Analog zur base_T_board-Prüfung: det(R), Z-Achsen-Plausibilität, Vergleich mit Vorwert.
- Kamera-Position in Base-Frame sollte im erwarteten Montagebereich liegen.

#### 4d. Validation nach Calibration
- Nach Apply: "Validate base_T_cam"-Button verwenden.
- Schachbrettmuster an bekannter Stelle hinlegen, Position per Kamera messen, per FK gegenchecken.
- Pos-Error sollte < 5 mm sein.

#### 4e. Integration in Workflow
- Aktuell: Auto-Calib im letzten Sub-Tab, danach manuell "Save base_T_cam" nötig.
- Verbesserung: "Apply" soll direkt in `transforms.json` schreiben (ohne separaten Save-Schritt).
- Prüfen ob `_write_config("transforms", ...)` in `autocalib_v1.py`-Integration korrekt aufgerufen wird.

---

## Kalibrierungskonvention (zur Erinnerung)

```
Skizze Brett (Draufsicht):

  P0 o──────o P1   ← +X des Boards (Dateien a→h)
  │
  │  +Y (Ränge 1→8)
  │
  P2

P0 = linke Vorderkante (Rang 1, Datei a – innere Ecke)
P1 = rechte Vorderkante (Rang 1, Datei h – innere Ecke)  → definiert +X
P2 = linke Hinterkante  (Rang 8, Datei a – innere Ecke)  → definiert +Y

ACHTUNG: Der bisherige Helptext war missverständlich (P2 und P1 vertauscht dargestellt).
Wenn P0→P1 und P0→P2 nicht rechtwinklig sind, meldet die neue Plausibilitätsprüfung [WARN].
```

---

## Relevante Dateien

| Datei | Inhalt |
|---|---|
| `pickplace_ui.py` | Haupt-UI Pick & Place, alle Kalibrierlogik, `_test_board_moves`, Plausibilitätsprüfung, Pitch-Adaptation |
| `autocalib_v1.py` | Auto Hand-Eye-Calibration Engine (Phase 1) |
| `tcp_world_kinematics_frame.py` | IK-Solver, `move_tcp_pose`, `preview_tcp_gcode`, `_get_current_joint_list` |
| `configs/calibration.json` | `base_T_board` + Board-Pattern-Parameter |
| `configs/transforms.json` | `base_T_cam`, `gripper_T_tcp` |
| `Robotrol_FluidNC_v7_2.py` | Aktueller Haupteinstiegspunkt (v7.2) |
