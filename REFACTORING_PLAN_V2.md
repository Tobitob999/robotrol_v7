# Robotrol v2.0 — Refactoring-Plan

**Erstellt:** 2026-03-14
**Basis:** Robotrol v7.3 (Commit cd2c033)
**Ziel:** Saubere Modulstruktur, testbar, wartbar. Alle Features bleiben erhalten.

---

## Getroffene Entscheidungen

- Pick&Place-Packages (control/, perception/, planning/, simulation/, learning/) werden unter `pickplace/` gruppiert
- `gamepad_block_v3.py` bleibt als separate Library-Datei, Tab-Wrapper in `gui/tabs/`
- Profile-JSONs wandern nach `profiles/`
- Robosim bleibt Subprocess (separater Prozess)
- GCode-Tab bleibt eine Datei
- Config-Pfade relativ zum Package (`Path(__file__).parent`)

---

## Goldene Regeln

1. **Nichts ausserhalb von `gui/` darf tkinter importieren.** Einzige Ausnahme: Legacy-Module (gamepad_block_v3, fluidnc_updater_v2, board_pose_v1, camera_capturev_v1_1) die 1:1 uebernommen und nur von gui/tabs/ importiert werden.
2. **FK-Verifikation nach JEDEM Kinematics-Refactoring.** EB300 bei MPos=0: TCP=(179, 0, 860). Moveo bei MPos=0: TCP=(-0, 0, 900). Abweichung = STOPP.
3. **JSON-Profilstruktur aendert sich NICHT.** Nur Dateipfade aendern sich.
4. **Jede Phase endet mit `python -m py_compile` auf allen .py-Dateien.** 0 Fehler = weiter. Sonst fixen.
5. **Backend-Module muessen ohne tkinter importierbar sein.** Test: `python -c "from robotrol.kinematics import fk"` darf nicht `import tkinter` ausloesen.

---

## Ziel-Verzeichnisstruktur

```
robotrol_v2/
  robotrol/
    __init__.py                 # __version__ = "2.0.0", AXES = ["A","X","Y","Z","B","C"]
    __main__.py                 # Entry: python -m robotrol

    config/
      __init__.py
      constants.py              # AXES, DEFAULT_ENDSTOP_LIMITS, MAX_FEED, MOTION_EPS, UDP defaults
      app_config.py             # AppConfig dataclass — laedt settings.json, project_flags.json
      profiles.py               # ProfileManager — Profil laden/speichern/wechseln, Settings-Persistenz

    serial/
      __init__.py
      client.py                 # SerialClient (connect, disconnect, send_line, worker-thread, listeners)
      protocol.py               # GrblProtocol: Status-Parsing (<Idle|MPos:...>), $$ Parser, _is_homing_command()

    kinematics/
      __init__.py
      dh_model.py               # DH-Modell laden (dh.json + Profil), set_dh_model_from_dict(), GEOM_DH
      fk.py                     # fk6_forward_mm(), apply_joint_angle_post_transform() — pure Mathe
      ik.py                     # IK6 Klasse (aus tcp_world_kinematics_frame.py Zeile 33-80)
      ik_dls.py                 # DLS Solver: Jacobian, damped_pinv, solve_ik_iterative (aus tcp_world_kinematics_frame.py)
      transforms.py             # 4x4 Matrizen, RPY, Quaternion (1:1 aus control/transforms.py + Erweiterungen)

    queue/
      __init__.py
      gcode_queue.py            # GCodeQueue: enqueue(), worker(), start_run(), pause_run(), stop_abort()
      cli_history.py            # CLI-History Ring-Buffer (~50 Zeilen)

    visualizer/
      __init__.py
      udp_mirror.py             # UDPMirror: send_path(), send_fixed_frame(), send_robot_profile()
      robosim.py                # robosim_visualizer_v90.py (1:1 kopiert, Imports angepasst)
      ik_rotosim.py             # ik_rotosim.py (1:1 kopiert)

    vision/
      __init__.py
      camera_capture.py         # camera_capturev_v1_1.py (1:1 kopiert)
      board_pose.py             # board_pose_v1.py (1:1 kopiert)
      autocalib.py              # autocalib_v1.py (1:1 kopiert)

    chess/                      # 1:1 aus bestehendem chess/ Package
      __init__.py
      board_detector.py
      config.py
      piece_classifier.py
      piece_detector.py
      state.py
      types.py
      vision_pipeline.py

    pickplace/
      __init__.py
      control/                  # 1:1 aus bestehendem control/ Package
        __init__.py
        app.py
        config.py
        errors.py
        executor.py
        fsm.py
        gripper.py
        pipeline.py
        robot.py
        transforms.py
      perception/               # 1:1 aus bestehendem perception/ Package
        __init__.py
        camera.py
        marker_detector.py
        pose_estimator.py
        quality.py
        tnt_detector.py
        types.py
      planning/                 # 1:1 aus bestehendem planning/ Package
        __init__.py
        grasp_planner.py
        place_planner.py
        types.py
      simulation/               # 1:1 aus bestehendem simulation/ Package
        __init__.py
        mock_world.py
        simulation_loop.py
      learning/                 # 1:1 aus bestehendem learning/ Package
        __init__.py
        tnt_self_learning.py

    gui/
      __init__.py
      app.py                    # RobotrolApp — duenner Koordinator, KEIN ttk.Frame
      theme.py                  # ThemeManager: toggle_theme(), Dark/Light
      toolbar.py                # ToolbarFrame: Port, Baud, Backend, Connect, Profile, Theme, GP-Mode

      tabs/
        __init__.py
        manual_control.py       # ManualControlTab: Achsen-Buttons, Sliders, Jog, Speed
        endstops.py             # EndstopsTab: Endstop-Limit-Editor
        vision.py               # VisionTab: Camera + BoardPose Setup
        chess_vision.py         # ChessVisionTab (duenner Wrapper)
        ota_updater.py          # OTATab: Wrapper um FluidNCUpdaterFrame
        gamepad.py              # GamepadTab: Wrapper um gamepad_block_v3.attach_gamepad_tab()
        kinematics.py           # KinematicsTab: IK/FK UI, DH-Tabelle, TCP-Sequenzen
        fixed_tcp.py            # FixedTcpTab: Fixed TCP Orientation, Planes, Gamepad-Steuerung
        gcode.py                # GCodeTab: Plane-Definition, Arc-Generator, File-Import, Preview
        pickplace.py            # PickPlaceTab: Wrapper/Refactor von pickplace_ui.py
        simulation_pose.py      # SimulationTab: Visualizer-Starter, Pose-Sender
        commands.py             # CommandsTab: G-Code-Referenz (statischer Text)

      panels/
        __init__.py
        queue_panel.py          # QueuePanel: Listbox, Run/Pause/Stop, CLI, Log
        tcp_pose_panel.py       # TcpPosePanel: FK-Anzeige, Live-Update (UI-Teil von tcp_pose_module_v3)
        status_block.py         # StatusBlock: Idle/Run/Hold/Alarm Farbanzeige

    tools/                      # 1:1 uebernommen
      __init__.py
      math_utils.py
      auto_correct_frame.py
      check_english_text.py
      plane_g2g3_check.py
      smoke_checks.py
      verify_calib_from_poses.py

  profiles/                     # Roboter-Profile (aus Root verschoben)
    EB300.json
    EB15_red.json
    Moveo.json

  configs/                      # Runtime-Config (aus Root verschoben)
    settings.json
    camera.json
    chess.json
    calibration.json
    grid.json
    markers.json
    perception.json
    robot.json
    simulation.json
    system.json
    tnt.json
    tnt_learning.json
    tnt_learning_policy.json
    transforms.json
    project_flags.json
    cube.yaml

  model/                        # DH-Modell Daten
    dh.json
    dh_table.md
    frames_convention.md
    tool_frames.md

  data/                         # Runtime-Daten
    capture_poses.json
    examples/
    ...

  tests/
    __init__.py
    conftest.py                 # Fixtures: Mock-Profile, Mock-SerialClient
    test_fk.py                  # FK bei MPos=0 fuer alle 3 Profile
    test_ik.py                  # IK round-trip (FK -> IK -> FK)
    test_config.py              # AppConfig laden, Profile laden/speichern
    test_homing_lock.py         # Safety Lock fuer Profile ohne Endstops
    test_protocol.py            # Status-Line Parsing, $$ Parsing
    test_queue.py               # Queue-Logik mit Mock-Client

  gamepad_config.json           # Gamepad-Presets
  CLAUDE.md                     # Projekt-Regeln (aktualisiert fuer v2.0)
  AGENTS.md                     # Session-History
  README.md
  LICENSE
  .gitattributes
  .gitignore
```

---

## Abhaengigkeitsgraph (Import-Richtung)

```
config/         ← keine Abhaengigkeiten (Basis-Layer)
     ↑
serial/         ← config/
     ↑
kinematics/     ← config/
     ↑
queue/          ← serial/, config/
     ↑
visualizer/     ← config/
     ↑
vision/         ← standalone (optional opencv)
     ↑
chess/          ← standalone
     ↑
pickplace/      ← vision/, kinematics/, config/
     ↑
gui/            ← ALLES (einziger Layer der tkinter importiert)
  app.py        ← serial/, kinematics/, queue/, visualizer/, config/
  toolbar.py    ← serial/, config/
  tabs/*        ← serial/, kinematics/, queue/, visualizer/, vision/, pickplace/
  panels/*      ← queue/, kinematics/
```

**Verbotene Imports:**
- `serial/` darf NICHT `gui/` importieren
- `kinematics/` darf NICHT `gui/` oder `serial/` importieren
- `queue/` darf NICHT `gui/` importieren
- Kein Modul ausser `gui/` darf `import tkinter` enthalten

---

## Phase 0: Vorbereitung

**Ziel:** Verzeichnisse anlegen, Konstanten extrahieren, pytest einrichten.

### Schritt 0.1: Projekt-Skelett

1. Neues Verzeichnis `robotrol_v2/` anlegen (NEBEN dem alten Code, nicht darin)
2. Alle Verzeichnisse und `__init__.py` gemaess Zielstruktur erstellen (leere Dateien)
3. `profiles/`, `configs/`, `model/`, `data/` aus dem alten Repo kopieren

### Schritt 0.2: config/constants.py

Quelle: `Robotrol_FluidNC_v7_3.py` Zeile 35-90 + 160-197

Extrahiere:
```python
AXES = ["A", "X", "Y", "Z", "B", "C"]
AXIS_INDICES = {"A": 0, "X": 1, "Y": 2, "Z": 3, "B": 4, "C": 5}

MAX_FEED = 15000.0
MAX_HOME_FEED = 10000.0
DEFAULT_TIMEOUT = 30.0
MOTION_EPS = 0.001

UDP_MIRROR_DEFAULT = True
UDP_ADDR_DEFAULT = ("127.0.0.1", 9999)

DEFAULT_ENDSTOP_LIMITS = {
    "A": (0.0, 360.0), "X": (0.0, 360.0), "Y": (0.0, 360.0),
    "Z": (0.0, 360.0), "B": (0.0, 360.0), "C": (0.0, 360.0),
}

# Regex-Patterns fuer G-Code Parsing
import re
GC_RE = re.compile(r"([A-Z])(-?[\d.]+)")
SOFTMAX_RE = re.compile(r"\$(\d+)/MaxTravel=(\d+\.?\d*)")
RE_HOMING_CMD = re.compile(r"^\$H([XYZABC])?$")
```

### Schritt 0.3: config/app_config.py

```python
from dataclasses import dataclass, field
from pathlib import Path
import json

@dataclass
class AppConfig:
    base_dir: Path
    last_profile: str = "Moveo"
    last_port: str = ""
    last_baud: int = 115200
    udp_mirror: bool = True
    udp_addr: tuple = ("127.0.0.1", 9999)

    @classmethod
    def load(cls, base_dir: Path = None) -> "AppConfig":
        if base_dir is None:
            base_dir = Path(__file__).parent.parent.parent  # robotrol_v2/
        settings_path = base_dir / "configs" / "settings.json"
        data = {}
        if settings_path.exists():
            with open(settings_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        return cls(
            base_dir=base_dir,
            last_profile=data.get("last_profile", "Moveo"),
            last_port=data.get("last_port", ""),
            last_baud=data.get("last_baud", 115200),
        )

    def save(self):
        settings_path = self.base_dir / "configs" / "settings.json"
        data = {
            "last_profile": self.last_profile,
            "last_port": self.last_port,
            "last_baud": self.last_baud,
        }
        with open(settings_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    @property
    def profiles_dir(self) -> Path:
        return self.base_dir / "profiles"

    @property
    def configs_dir(self) -> Path:
        return self.base_dir / "configs"

    @property
    def model_dir(self) -> Path:
        return self.base_dir / "model"

    @property
    def data_dir(self) -> Path:
        return self.base_dir / "data"
```

### Schritt 0.4: config/profiles.py

Quelle: `config_profiles.py` (1:1 kopieren) + erweitern um:

```python
class ProfileManager:
    def __init__(self, config: AppConfig):
        self.config = config
        self._active = None
        self._data = {}
        self._on_change_callbacks = []

    @property
    def active(self) -> str:
        return self._active

    def load(self, name: str) -> dict:
        path = self.config.profiles_dir / f"{name}.json"
        with open(path, "r", encoding="utf-8") as f:
            self._data = json.load(f)
        self._active = name
        for cb in self._on_change_callbacks:
            cb(name, self._data)
        return self._data

    def get_section(self, key: str, default=None):
        return self._data.get(key, default)

    def set_section(self, key: str, value):
        self._data[key] = value

    def save(self):
        if not self._active:
            return
        path = self.config.profiles_dir / f"{self._active}.json"
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self._data, f, indent=2)

    def on_change(self, callback):
        self._on_change_callbacks.append(callback)

    def has_endstops(self) -> bool:
        features = self._data.get("features", {})
        return bool(features.get("has_endstops", True))

    def available_profiles(self) -> list:
        return [p.stem for p in self.config.profiles_dir.glob("*.json")]
```

### Schritt 0.5: pytest einrichten

```
# tests/conftest.py
import pytest
from pathlib import Path

@pytest.fixture
def base_dir():
    return Path(__file__).parent.parent

@pytest.fixture
def eb300_profile(base_dir):
    import json
    with open(base_dir / "profiles" / "EB300.json") as f:
        return json.load(f)

@pytest.fixture
def moveo_profile(base_dir):
    import json
    with open(base_dir / "profiles" / "Moveo.json") as f:
        return json.load(f)
```

### Schritt 0.6: Verifizierung Phase 0

- [ ] `python -c "from robotrol.config.constants import AXES; print(AXES)"`
- [ ] `python -c "from robotrol.config.app_config import AppConfig; c = AppConfig.load(); print(c)"`
- [ ] `pytest tests/ -v` (noch keine Tests, aber pytest laeuft)

---

## Phase 1: Backend-Extraktion (KEIN tkinter)

**Ziel:** Alle Logik-Module ohne GUI-Abhaengigkeit. Nach dieser Phase kann jedes Backend-Modul isoliert getestet werden.

### Schritt 1.1: serial/client.py

**Quelle:** `Robotrol_FluidNC_v7_3.py` Zeile 208-598 (Klasse `SerialClient`)

Die Klasse ist bereits relativ sauber. Extrahiere 1:1 mit diesen Anpassungen:
- Import `AXES` aus `config.constants` statt Global
- `_is_homing_command()` (Zeile 192) nach `serial/protocol.py`
- `on_serial_line()` Callback-System beibehalten (listeners-Liste)

Die Klasse hat diese Methoden:
- `__init__()` — Serial-Setup, Queue, Thread
- `connect(port, baud)` — Verbindung oeffnen
- `disconnect()` — Verbindung schliessen
- `send_now(line)` — Direkt senden (mit Homing-Guard)
- `enqueue(line)` — In Queue einreihen
- `worker()` — Thread: Queue abarbeiten, Status empfangen
- `_parse_status(line)` — `<Idle|MPos:...>` parsen → nach protocol.py
- `start_run()`, `pause_run()`, `stop_abort()` — Queue-Steuerung
- `do_zero()`, `goto_home()` — Convenience
- `_mirror_udp(joints)` — UDP-Mirror → nach visualizer/udp_mirror.py

### Schritt 1.2: serial/protocol.py

**Quelle:** Verstreut in Robotrol_FluidNC_v7_3.py

```python
import re
from robotrol.config.constants import RE_HOMING_CMD, GC_RE, SOFTMAX_RE

def is_homing_command(cmd: str) -> bool:
    """Check if command is a homing command ($H, $HX, etc.)"""
    return bool(RE_HOMING_CMD.match(cmd.strip()))

def parse_status_line(line: str) -> dict | None:
    """Parse FluidNC/GRBL status: <Idle|MPos:0.000,0.000,...|WCO:...>
    Returns dict with keys: state, mpos, wco, or None if not a status line."""
    ...

def parse_settings_line(line: str) -> tuple | None:
    """Parse $N=value settings responses."""
    ...

def parse_softmax(line: str) -> tuple | None:
    """Parse $N/MaxTravel=value."""
    ...
```

### Schritt 1.3: kinematics/dh_model.py

**Quelle:** `tcp_pose_module_v3.py` Zeile 1-120 (Modul-Level Code + `_load_dh_model`, `set_dh_model_from_dict`)

Extrahiere:
- `_load_dh_model(path)` — JSON laden
- `set_dh_model_from_dict(data)` — DH-Rows + Post-Transform setzen
- `get_dh_model()` — Aktuelles Modell zurueckgeben
- `GEOM_DH` Berechnung
- `apply_joint_angle_post_transform()` — sim_theta_offset + sim_theta_scale

**ACHTUNG:** Diese Funktionen nutzen aktuell globale Variablen (`_DH_ROWS`, `_POST_TRANSFORM`, `GEOM_DH`). In v2.0 als Klasse kapseln:

```python
class DHModel:
    def __init__(self):
        self.dh_rows = []
        self.post_transform = {}
        self.geom = None

    @classmethod
    def from_profile(cls, profile_data: dict) -> "DHModel":
        model = cls()
        model.set_from_dict(profile_data.get("dh_model", {}))
        return model

    @classmethod
    def from_json(cls, path: Path) -> "DHModel":
        ...

    def set_from_dict(self, data: dict):
        ...

    def apply_post_transform(self, joints: dict) -> list:
        ...
```

### Schritt 1.4: kinematics/fk.py

**Quelle:** `tcp_pose_module_v3.py` Zeile 420-518 (Funktionen)

Extrahiere:
- `fk6_forward_mm(geom, joints)` — Forward Kinematics (pure Mathe)
- `_dh_transform(theta, d, a, alpha)` — Einzelne DH-Transformation
- `_matmul4(A, B)` — 4x4 Matrix-Multiplikation

Diese Funktionen sind bereits stateless — direkt uebernehmbar.

### Schritt 1.5: kinematics/ik.py

**Quelle:** `tcp_world_kinematics_frame.py` Zeile 33-80 (Klasse `IK6`)

Die IK6 Klasse ist ein einfacher analytischer IK-Solver. 1:1 uebernehmbar.

### Schritt 1.6: kinematics/ik_dls.py

**Quelle:** `tcp_world_kinematics_frame.py` Zeile 949-1140

Die DLS (Damped Least Squares) Methoden sind in der UI-Klasse `TcpKinematicsFrame` versteckt. Extrahiere:
- `compute_jacobian(geom, joints)` — Numerischer Jacobian
- `damped_pinv(J, damping)` — Gedaempfte Pseudo-Inverse
- `solve_ik_iterative(geom, target_pose, initial_joints, ...)` — Iterativer IK-Solver
- `move_tcp_pose(geom, current_joints, dx, dy, dz, ...)` — Inkrementelle TCP-Bewegung

**Wichtig:** Diese Methoden referenzieren `self.exec` (die App) fuer Axis-Limits. Stattdessen Limits als Parameter uebergeben.

### Schritt 1.7: queue/gcode_queue.py

**Quelle:** `Robotrol_FluidNC_v7_3.py` verstreut (worker, enqueue, start_run, pause_run, stop_abort)

```python
class GCodeQueue:
    def __init__(self, serial_client, on_log=None):
        self.client = serial_client
        self.queue = []
        self.running = False
        self.paused = False
        self._on_log = on_log

    def enqueue(self, line: str):
        ...

    def start_run(self):
        ...

    def pause_run(self):
        ...

    def stop_abort(self):
        ...

    def clear(self):
        ...
```

### Schritt 1.8: visualizer/udp_mirror.py

**Quelle:** `Robotrol_FluidNC_v7_3.py` Methoden `_send_vis_path`, `_send_vis_fixed_frame`, `_send_vis_robot_profile`, `_mirror_udp`

```python
class UDPMirror:
    def __init__(self, addr=("127.0.0.1", 9999), enabled=True):
        self.addr = addr
        self.enabled = enabled
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_joints(self, joints: dict):
        ...

    def send_path(self, points: list):
        ...

    def send_fixed_frame(self, origin, u, v, n):
        ...

    def send_robot_profile(self, profile_data: dict):
        ...
```

### Schritt 1.9: Tests fuer Phase 1

```python
# tests/test_fk.py
def test_eb300_fk_at_zero(eb300_profile):
    """EB300 FK bei MPos=0 muss TCP=(179, 0, 860) ergeben."""
    from robotrol.kinematics.dh_model import DHModel
    from robotrol.kinematics.fk import fk6_forward_mm

    dh = DHModel.from_profile(eb300_profile)
    joints = {"A": 0, "X": 0, "Y": 0, "Z": 0, "B": 0, "C": 0}
    transformed = dh.apply_post_transform(joints)
    x, y, z, r, p, yaw, T = fk6_forward_mm(dh.geom, transformed)
    assert abs(x - 179) < 1.0
    assert abs(y - 0) < 1.0
    assert abs(z - 860) < 1.0

def test_moveo_fk_at_zero(moveo_profile):
    """Moveo FK bei MPos=0 muss TCP=(-0, 0, 900) ergeben."""
    ...

# tests/test_protocol.py
def test_parse_status_idle():
    from robotrol.serial.protocol import parse_status_line
    result = parse_status_line("<Idle|MPos:0.000,0.000,0.000,0.000,0.000,0.000>")
    assert result["state"] == "Idle"
    assert result["mpos"]["A"] == 0.0

def test_homing_command_detection():
    from robotrol.serial.protocol import is_homing_command
    assert is_homing_command("$H") == True
    assert is_homing_command("$HX") == True
    assert is_homing_command("$X") == False
    assert is_homing_command("G1 X10") == False

# tests/test_homing_lock.py
def test_eb300_homing_blocked(eb300_profile):
    """EB300 hat has_endstops=false, Homing muss blockiert sein."""
    from robotrol.config.profiles import ProfileManager
    # ... ProfileManager mit EB300 laden, has_endstops() == False pruefen
```

### Verifizierung Phase 1

- [ ] `python -c "from robotrol.serial.client import SerialClient"` — kein tkinter
- [ ] `python -c "from robotrol.kinematics.fk import fk6_forward_mm"` — kein tkinter
- [ ] `python -c "from robotrol.queue.gcode_queue import GCodeQueue"` — kein tkinter
- [ ] `pytest tests/test_fk.py tests/test_protocol.py tests/test_homing_lock.py -v`
- [ ] Alle FK-Referenzwerte stimmen (EB300: 179/0/860, Moveo: 0/0/900)

---

## Phase 2: GUI-Framework

**Ziel:** App-Koordinator, Toolbar und Panels stehen. Noch keine Tabs.

### Schritt 2.1: gui/theme.py

**Quelle:** `Robotrol_FluidNC_v7_3.py` Zeile 602-647

```python
class ThemeManager:
    def __init__(self, root):
        self.root = root
        self.current = "dark"

    def toggle(self):
        ...

    def apply(self, theme: str):
        ...
```

### Schritt 2.2: gui/toolbar.py

**Quelle:** `Robotrol_FluidNC_v7_3.py` Zeile 738-785

ToolbarFrame enthaelt:
- Port-Dropdown (COM-Ports)
- Baud-Dropdown
- Backend-Dropdown (FluidNC/GRBL)
- Connect/Disconnect Button
- Profil-Dropdown
- Theme-Toggle
- Gamepad-Mode-Anzeige

### Schritt 2.3: gui/panels/queue_panel.py

**Quelle:** `Robotrol_FluidNC_v7_3.py` Zeile 4216-4268 + CLI-Bereich

QueuePanel enthaelt:
- Queue-Listbox (rechte Seite)
- Run/Pause/Stop/Abort Buttons
- CLI-Eingabezeile mit History
- Log-Textfeld

### Schritt 2.4: gui/panels/tcp_pose_panel.py

**Quelle:** `tcp_pose_module_v3.py` Zeile 122-420 (Klasse `TcpPosePanel`)

UI-Teil der FK-Anzeige. Zeigt live: X/Y/Z mm, Roll/Pitch/Yaw deg.
Die Klasse hat schon eine saubere Trennung (Panel + Callbacks).

### Schritt 2.5: gui/panels/status_block.py

**Quelle:** `Robotrol_FluidNC_v7_3.py` Methode `_update_status_block()`

StatusBlock zeigt den aktuellen Zustand: Idle (gruen), Run (blau), Hold (gelb), Alarm (rot).

### Schritt 2.6: gui/app.py

```python
class RobotrolApp:
    """Hauptfenster-Koordinator. Verbindet Backend mit GUI."""

    def __init__(self, root):
        self.root = root
        self.root.title("Robotrol V2.0")

        # Backend
        self.config = AppConfig.load()
        self.profile_mgr = ProfileManager(self.config)
        self.serial = SerialClient()
        self.protocol = GrblProtocol()
        self.dh = None  # Wird bei Profilwechsel gesetzt
        self.queue = GCodeQueue(self.serial, on_log=self.log)
        self.udp = UDPMirror()

        # State
        self.axis_positions = {ax: 0.0 for ax in AXES}
        self.mpos = {ax: 0.0 for ax in AXES}
        self.wco = {ax: 0.0 for ax in AXES}
        self.hw_limits = {}
        self.can_global_home = True
        self.can_axis_home = True

        # GUI
        self.theme = ThemeManager(root)
        self.toolbar = ToolbarFrame(root, self)
        self.notebook = ttk.Notebook(root)
        self.queue_panel = QueuePanel(root, self)
        self.status_block = StatusBlock(root, self)

        # Tabs werden in _build_tabs() hinzugefuegt
        self._tabs = {}
        self._build_tabs()

        # Serial-Listener
        self.serial.on_line(self._on_serial_line)

        # Profil laden
        self._load_initial_profile()

    def _build_tabs(self):
        # Jeder Tab wird einzeln instanziiert
        ...

    def _on_serial_line(self, line: str):
        # Status-Parsing, Axis-Update, UI-Update
        ...

    def log(self, msg: str):
        self.queue_panel.log(msg)

    def send_now(self, cmd: str):
        self.serial.send_now(cmd)

    def enqueue(self, cmd: str):
        self.queue.enqueue(cmd)

    def get_current_tcp_mm(self) -> dict:
        ...

    def shutdown(self):
        # Gamepad stoppen, Serial schliessen, Config speichern
        ...
```

### Verifizierung Phase 2

- [ ] App startet mit leerem Notebook (keine Tabs)
- [ ] Toolbar zeigt Ports, Connect-Button funktioniert
- [ ] Theme-Toggle funktioniert
- [ ] Profilwechsel funktioniert (Log-Meldung)

---

## Phase 3: Einfache Tabs

**Ziel:** 8 einfache Tabs funktionieren.

### Reihenfolge (einfachste zuerst):

1. **`tabs/commands.py`** — Statischer G-Code-Referenztext. Quelle: Zeile 3420-3494. Trivial.

2. **`tabs/endstops.py`** — Endstop-Limit-Editor. Quelle: Zeile 873-964. ~90 Zeilen.

3. **`tabs/chess_vision.py`** — Duenner Wrapper. Quelle: Zeile 1018-1024. 6 Zeilen.

4. **`tabs/ota_updater.py`** — Wrapper um `FluidNCUpdaterFrame`. Quelle: Zeile 1027-1046. Import von `fluidnc_updater_v2.py` (1:1).

5. **`tabs/gamepad.py`** — Wrapper um `gamepad_block_v3.attach_gamepad_tab()`. Quelle: Zeile 1048-1054.

6. **`tabs/vision.py`** — Camera + BoardPose Setup. Quelle: Zeile 966-1017. ~50 Zeilen.

7. **`tabs/simulation_pose.py`** — Visualizer-Starter, UDP-Pose-Sender. Quelle: Zeile 3314-3419. ~100 Zeilen.

8. **`tabs/kinematics.py`** — UI-Teil von `TcpWorldKinematicsTabs`. Quelle: `tcp_world_kinematics_frame.py`. Die 3 Klassen (`IK6`, `TcpKinematicsFrame`, `TcpWorldKinematicsTabs`) werden aufgeteilt:
   - `IK6` → bereits in `kinematics/ik.py` (Phase 1)
   - `TcpKinematicsFrame` Solver-Logik → bereits in `kinematics/ik_dls.py` (Phase 1)
   - `TcpKinematicsFrame` UI-Teile → `tabs/kinematics.py`
   - `TcpWorldKinematicsTabs` → `tabs/kinematics.py`

### Template fuer jeden Tab:

```python
# gui/tabs/example.py
import tkinter as tk
from tkinter import ttk

class ExampleTab(ttk.Frame):
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self._build_ui()

    def _build_ui(self):
        # Tab-spezifische UI hier
        ...
```

### Verifizierung Phase 3

- [ ] Alle 8 Tabs laden ohne Fehler
- [ ] Endstop-Editor zeigt/speichert Werte
- [ ] Gamepad-Tab zeigt Konfiguration
- [ ] Kinematics-Tab zeigt DH-Tabelle und IK-Controls

---

## Phase 4: Komplexe Tabs

**Ziel:** Die 4 grossen Tabs funktionieren.

### Schritt 4.1: tabs/manual_control.py (~700 Zeilen)

**Quelle:** `Robotrol_FluidNC_v7_3.py` Zeile 3500-4200

Das Herzstuck: Achsen-Buttons (+/-), Step-Size-Slider, Speed-Factor, Jog-Buttons, Zero-Buttons, Home-Buttons.

Wichtigste Abhaengigkeiten:
- `app.send_now()` fuer direkte G-Code-Befehle
- `app.axis_positions` fuer aktuelle Positionen
- `app.hw_limits` fuer Clamp
- `app.can_global_home` / `app.can_axis_home` fuer Homing Safety Lock

Die inneren Funktionen (`clamp_with_limits`, `_get`, `axis_to_queue`, `axis_direct_send`) werden Methoden der Tab-Klasse.

### Schritt 4.2: tabs/fixed_tcp.py (~1060 Zeilen)

**Quelle:** `Robotrol_FluidNC_v7_3.py` Zeile 1079-2140

Fixed TCP Orientation Control:
- Roll/Pitch/Yaw Einstellung
- XY/XZ/YZ Plane-Auswahl
- Gamepad TCP-Jogging (move_fixed_tcp_gamepad)
- Gamepad Roll-Jogging (rotate_fixed_tcp_roll)
- Settings-Persistenz (get/apply_fixed_tcp_settings)

Abhaengigkeiten:
- `app.serial` fuer G-Code senden
- `app.dh` fuer FK-Berechnung
- `app.udp` fuer Visualizer-Updates
- `app.profile_mgr` fuer Settings laden/speichern

### Schritt 4.3: tabs/gcode.py (~1180 Zeilen)

**Quelle:** `Robotrol_FluidNC_v7_3.py` Zeile 2140-3320

G-Code Tab mit Sub-Bereichen:
- Plane-Definition (3-Punkt-Ebene)
- Arc-Generator (G2/G3)
- File-Import (.gcode, .nc)
- Preview-Canvas
- Plane-Marker Visualisierung

### Schritt 4.4: tabs/pickplace.py

**Quelle:** `pickplace_ui.py` (2770 Zeilen)

Dieses File ist bereits ein eigenstaendiger Tab (`PickPlaceTab`). Hauptarbeit:
- Imports anpassen (statt `from Robotrol_FluidNC_v7_3 import ...` → `from robotrol.xxx import ...`)
- Referenz auf `execute_app` → `self.app`
- Vision/Camera-Imports → `from robotrol.vision import ...`
- Pickplace-Pipeline → `from robotrol.pickplace import ...`

Das File wird NICHT komplett neu geschrieben, sondern refactored (Imports + App-Referenz).

### Verifizierung Phase 4

- [ ] Manual Control: Achsen bewegen sich (mit Serial-Verbindung)
- [ ] Fixed TCP: Gamepad-Jogging funktioniert
- [ ] GCode: Datei laden und Queue fuellen
- [ ] PickPlace: Tab laedt, Kalibrierung startet

---

## Phase 5: Integration + Cleanup

### Schritt 5.1: __main__.py

```python
"""Entry point: python -m robotrol"""
import tkinter as tk
from robotrol.gui.app import RobotrolApp

def main():
    root = tk.Tk()
    app = RobotrolApp(root)
    root.protocol("WM_DELETE_WINDOW", app.shutdown)
    root.mainloop()

if __name__ == "__main__":
    main()
```

### Schritt 5.2: on_close / shutdown

Sammle alle Cleanup-Logik aus `Robotrol_FluidNC_v7_3.py` Zeile 6063-6102:
- TCP-Panel stoppen
- Gamepad stoppen
- Serial disconnecten
- Config speichern
- Root zerstoeren

### Schritt 5.3: 1:1-Dateien verschieben

| Alt | Neu |
|-----|-----|
| `robosim_visualizer_v90.py` | `robotrol/visualizer/robosim.py` |
| `ik_rotosim.py` | `robotrol/visualizer/ik_rotosim.py` |
| `camera_capturev_v1_1.py` | `robotrol/vision/camera_capture.py` |
| `board_pose_v1.py` | `robotrol/vision/board_pose.py` |
| `autocalib_v1.py` | `robotrol/vision/autocalib.py` |
| `chess/` | `robotrol/chess/` |
| `control/` | `robotrol/pickplace/control/` |
| `perception/` | `robotrol/pickplace/perception/` |
| `planning/` | `robotrol/pickplace/planning/` |
| `simulation/` | `robotrol/pickplace/simulation/` |
| `learning/` | `robotrol/pickplace/learning/` |
| `tools/` | `robotrol/tools/` |

Bei jeder verschobenen Datei: Interne Imports anpassen.

### Schritt 5.4: Alte Dateien loeschen

ERST wenn alle Tests gruen sind und die App vollstaendig funktioniert:
- `Robotrol_FluidNC_v7_3.py` (ersetzt durch gui/app.py + tabs/ + panels/)
- `tcp_pose_module_v3.py` (ersetzt durch kinematics/ + panels/tcp_pose_panel.py)
- `tcp_world_kinematics_frame.py` (ersetzt durch kinematics/ + tabs/kinematics.py)
- `config_profiles.py` (ersetzt durch config/profiles.py)
- `chess_vision_ui.py` (ersetzt durch tabs/chess_vision.py)

### Schritt 5.5: CLAUDE.md aktualisieren

```markdown
# CLAUDE.md — Robotrol v2.0 Project Rules

## Project Structure
- Entry point: `python -m robotrol`
- Backend (kein tkinter): robotrol/config/, serial/, kinematics/, queue/, visualizer/
- GUI (tkinter): robotrol/gui/
- Tests: tests/

## Critical: Robot Profile Protection
[... bestehende Regeln beibehalten ...]

## Development Rules
1. Syntax check: python -m py_compile <file>.py
2. Start: python -m robotrol
3. All UI text in English
4. Backend darf NICHT tkinter importieren
5. FK-Verifikation nach Kinematics-Aenderungen (siehe tests/test_fk.py)
6. Tests: pytest tests/ -v
```

### Verifizierung Phase 5

- [ ] `python -m robotrol` startet die App
- [ ] Alle Tabs funktionieren
- [ ] Profilwechsel funktioniert
- [ ] Serial-Verbindung funktioniert
- [ ] Gamepad funktioniert
- [ ] `pytest tests/ -v` — alle Tests gruen
- [ ] Kein `import tkinter` ausserhalb von `gui/` und Legacy-Modulen
- [ ] Keine Referenz auf `Robotrol_FluidNC_v7_3` mehr im Code

---

## Zeilen-Referenz fuer die Extraktion

Diese Tabelle zeigt wo der Code in `Robotrol_FluidNC_v7_3.py` liegt:

| Zeilen | Inhalt | Ziel in v2.0 |
|--------|--------|-------------|
| 1-34 | Header, Imports | gui/app.py Imports |
| 35-90 | Konstanten | config/constants.py |
| 91-197 | Helper-Funktionen (_is_homing_command, load_project_flags) | serial/protocol.py, config/app_config.py |
| 208-598 | SerialClient Klasse | serial/client.py |
| 600-647 | Theme-Toggle | gui/theme.py |
| 648-735 | on_close, Profil-Helper | gui/app.py shutdown() |
| 738-785 | Toolbar (Port, Baud, Profile) | gui/toolbar.py |
| 787-838 | ExecuteApp.__init__ | gui/app.py RobotrolApp.__init__ |
| 849-4268 | _build_ui() — DER GROSSE BLOCK | gui/tabs/* (12 Dateien) |
| 849-872 | Notebook Setup | gui/app.py _build_tabs() |
| 873-964 | Endstop Tab | tabs/endstops.py |
| 966-1017 | Vision Tab | tabs/vision.py |
| 1018-1024 | Chess Vision Tab | tabs/chess_vision.py |
| 1027-1046 | OTA/Config Tab | tabs/ota_updater.py |
| 1048-1054 | Gamepad Tab | tabs/gamepad.py |
| 1057-1076 | Kinematics Tab | tabs/kinematics.py |
| 1079-2140 | Fixed TCP | tabs/fixed_tcp.py |
| 2140-3320 | GCode/Plane/Arc | tabs/gcode.py |
| 3314-3419 | Simulation/Pose | tabs/simulation_pose.py |
| 3420-3494 | Commands Tab | tabs/commands.py |
| 3500-4200 | Manual Control Tab | tabs/manual_control.py |
| 4200-4268 | Queue/CLI/Log | panels/queue_panel.py |
| 4269-4380 | Layout-Metrics, Vision-Tick | gui/app.py |
| 4381-4440 | get_current_tcp_mm, apply_profile | gui/app.py + config/profiles.py |
| 4441-4510 | Settings Persistenz | config/profiles.py |
| 4511-4600 | Homing Safety Lock | serial/protocol.py + gui/toolbar.py |
| 4601-4680 | UDP Visualizer | visualizer/udp_mirror.py |
| 4681-5130 | on_serial_line (Status-Parser) | serial/protocol.py + gui/app.py |
| 5131-5700 | Fixed TCP Gamepad-Methoden | tabs/fixed_tcp.py |
| 5701-5960 | Gamepad Mode/Fix/Roll | tabs/fixed_tcp.py + gui/toolbar.py |
| 5961-6063 | Auto-Start, Mainloop | __main__.py |
| 6063-6102 | on_close Cleanup | gui/app.py shutdown() |

---

## Zeitschaetzung

| Phase | Sessions | Beschreibung |
|-------|----------|-------------|
| 0 | 1 | Skelett, Config, pytest |
| 1 | 2 | Backend (Serial, Kinematics, Queue, UDP) + Tests |
| 2 | 1 | GUI-Framework (App, Toolbar, Panels) |
| 3 | 2 | 8 einfache Tabs |
| 4 | 2-3 | 4 komplexe Tabs |
| 5 | 1 | Integration, Cleanup, Final-Tests |
| **Total** | **9-10** | |

Jede "Session" = 1 Claude-Code-Sitzung mit ~30-60 Minuten Kontext.
