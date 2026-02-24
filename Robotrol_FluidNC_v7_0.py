# ============================================================================================
# 34 Robotrol V7.1
# Full G-code control center with queue, gamepad, kinematics, vision, and OTA
# ============================================================================================
# Compatible with:
#    FluidNC v3.73.9 (ESP32)
#    GRBL 1.1 / 1.2 (via serial)
#
# Features:
#    Real-time control via FluidNC/GRBL serial link
#    Command line + live queue (buffer handling, priorities, safe mixing)
#    Gamepad control with 3 speed profiles (Slow / Mid / Fast)
#    DH6-Kinematik (FK/IK) + DLS-Inverse-Kinematik
#    Live-TCP-Pose-Anzeige (Roll / Pitch / Yaw) per DH-FK
#    Externer 3D-Visualizer (UDP Mirror  Port 9999)
#    Vision-System:
#         - OpenCV camera module
#         - Chessboard detection
#         - Board pose estimation
#    OTA / HTTP config tool for FluidNC
#    Dark-/Light-Theme umschaltbar
#
# Enthaltene Module / Dateien:
#   - tcp_world_kinematics_frame.py  (TM) DLS IK + world-coordinate control
#   - tcp_pose_module_v3.py          (TM) FK6 (DH model), Roll/Pitch/Yaw, mm output
#   - tcp_world_kinematics_frame.py  (TM) TCP sequences (RET / TARGET / RET + gripper)
#   - gamepad_block_v3.py            (TM) Gamepad-Tabs & Trigger-Integration
#   - robosim_visualizer_v90.py/exe  (TM) 3D-Visualizer (UDP 127.0.0.1:9999)
#   - camera_capturev_v1_1.py         (TM) Live camera / OpenCV
#   - board_pose_v1.py               (TM) Chessboard detection & pose solve
#   - fluidnc_updater_v2.py          (TM) OTA-Updater, $$-Inspector, Config-Manager



import os
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog
import math
import json
import config_profiles
from PIL import Image, ImageTk



# Serielle/IO/Gamepad/Netzwerk
import serial, serial.tools.list_ports
import threading, time, re, socket, pygame, subprocess, sys

# App-Module
from fluidnc_updater_v2 import FluidNCUpdaterFrame
from camera_capturev_v1_1 import CameraCapture
from board_pose_v1 import BoardPose
import tcp_pose_module_v3 as tcp_pose_module
from tcp_world_kinematics_frame import TcpWorldKinematicsTabs
from pickplace_ui import PickPlaceTab
from chess_vision_ui import ChessVisionTab
# === Deaktiviert alle Popups ===
def _no_popup(*args, **kwargs):
    
    
    
    return None
messagebox.showinfo = _no_popup
messagebox.showwarning = _no_popup
messagebox.showerror = _no_popup

# =========================
# Configuration
# =========================
UDP_MIRROR = True
UDP_ADDR = ("127.0.0.1", 9999)
GC_RE = re.compile(r"\[GC:(.+)\]")
AXES = ["X", "Y", "Z", "A", "B", "C"]
AXIS_LIMITS_DEFAULT = {ax: (-180.0, 180.0) for ax in AXES}  # Fallback 180
DEFAULT_ENDSTOP_LIMITS = {
    "X": (-100.0, 100.0),
    "Y": (-100.0, 100.0),
    "Z": (-90.0, 90.0),
    "A": (-90.0, 90.0),
    "B": (-180.0, 180.0),
    "C": (-180.0, 180.0),
}
MAX_FEED = 15000.0                     # mm/min (interpretiert als Grad/min)
MAX_HOME_FEED = 5000   # example value, freely adjustable
DEFAULT_TIMEOUT = 30.0                # seconds per command
MOTION_EPS = 0.01                     # min. position change for live move
# $$ Softlimits (Travel)  Grbl/FluidNC: $130..$134  X..B max travel
SOFTMAX_RE = {
    "X": re.compile(r"^\$130=([0-9\.]+)"),
    "Y": re.compile(r"^\$131=([0-9\.]+)"),
    "Z": re.compile(r"^\$132=([0-9\.]+)"),
    "A": re.compile(r"^\$133=([0-9\.]+)"),
    "B": re.compile(r"^\$134=([0-9\.]+)"),
    "C": re.compile(r"^\$135=([0-9\.]+)"),
}

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_FLAGS_PATH = os.path.join(BASE_DIR, "configs", "project_flags.json")
CAMERA_CONFIG_PATH = os.path.join(BASE_DIR, "configs", "camera.json")


def _load_project_flags():
    default = {
        "language": "en",
        "enforce_english_text": True,
        "language_migration_policy": "low_priority_incremental",
    }
    try:
        with open(PROJECT_FLAGS_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            out = dict(default)
            out.update(data)
            return out
    except Exception:
        pass
    return default


PROJECT_FLAGS = _load_project_flags()


def _load_camera_config():
    default = {
        "device_index": 0,
        "width": 640,
        "height": 480,
        "fps": 30,
    }
    try:
        with open(CAMERA_CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data, dict):
            out = dict(default)
            out.update(data)
            return out
    except Exception:
        pass
    return default
DEFAULT_PROFILE = config_profiles.DEFAULT_PROFILE
ACTIVE_PROFILE = {
    "name": DEFAULT_PROFILE,
    "data": config_profiles.load_profile(DEFAULT_PROFILE, base_dir=BASE_DIR) or {"name": DEFAULT_PROFILE, "version": 1},
}
if isinstance(ACTIVE_PROFILE["data"], dict):
    dh_model = ACTIVE_PROFILE["data"].get("dh_model")
    if dh_model:
        tcp_pose_module.set_dh_model_from_dict(dh_model)

NO_ENDSTOP_PROFILES = {"eb15_red", "eb300"}


def _profile_has_endstops(profile_name, profile_data):
    if isinstance(profile_data, dict):
        features = profile_data.get("features")
        if isinstance(features, dict) and "has_endstops" in features:
            try:
                return bool(features.get("has_endstops"))
            except Exception:
                pass
    norm = (profile_name or "").strip().lower()
    if norm in NO_ENDSTOP_PROFILES:
        return False
    return True

def map_speed(val_0_1000: int) -> int:
    """Linear: 01000  0MAX_FEED."""
    val = max(0, min(1000, int(val_0_1000)))
    return int((val / 1000.0) * MAX_FEED)


# =========================
# Serial + UDP
# =========================
class SerialClient:
    """
    Simple abstraction over serial connection + UDP mirror.
    Supports backends:
      - fluidnc
      - grbl
      - custom
    """

    BACKEND_CAPS = {
        "fluidnc": {
            "name": "FluidNC",
            "supports_endstops": True,
            "supports_ota": True,
            "supports_axis_homing": True,   # $HX/$HY/...
            "supports_global_homing": True, # $H
            "supports_softlimits": True,
        },
        "grbl": {
            "name": "GRBL",
            "supports_endstops": False,     # no Pn:
            "supports_ota": False,
            "supports_axis_homing": False,  # only $H
            "supports_global_homing": True, # $H
            "supports_softlimits": True,
        },
        "custom": {
            "name": "Custom",
            "supports_endstops": False,
            "supports_ota": False,
            "supports_axis_homing": False,
            "supports_global_homing": True,
            "supports_softlimits": False,
        },
    }

    DEBUG_SERIAL = True  # set to False if needed to reduce console noise

    def __init__(self):
        self.ser = None

        # No GUI references here!
        self.rx_thread = None
        self.rx_running = False
        self.listeners = []       # Must exist for TCP panel
        self.lock = threading.Lock()

        # UDP Mirror
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if UDP_MIRROR else None

        # Axis Limits
        self.axis_limits = {ax: list(DEFAULT_ENDSTOP_LIMITS[ax]) for ax in AXES}

        # Backend
        self.backend_type = "fluidnc"
        self.backend_caps = self.BACKEND_CAPS[self.backend_type]

        # Status
        self._warned_not_connected = False
        self.last_status = None

    # ------------------------------------------------------------
    #  Select backend (FluidNC / GRBL / custom)
    # ------------------------------------------------------------
    def set_backend(self, backend_name: str):
        name = (backend_name or "").strip().lower()
        if name not in self.BACKEND_CAPS:
            name = "fluidnc"
        self.backend_type = name
        self.backend_caps = self.BACKEND_CAPS[name]
        if self.DEBUG_SERIAL:
            print(f"[Backend] -> {self.backend_caps['name']} ({self.backend_type})")

    @property
    def is_fluidnc(self) -> bool:
        return self.backend_type == "fluidnc"

    @property
    def is_grbl(self) -> bool:
        return self.backend_type == "grbl"

    @property
    def is_custom(self) -> bool:
        return self.backend_type == "custom"

    @property
    def supports_endstops(self) -> bool:
        return self.backend_caps.get("supports_endstops", False)

    @property
    def supports_ota(self) -> bool:
        return self.backend_caps.get("supports_ota", False)

    @property
    def supports_axis_homing(self) -> bool:
        return self.backend_caps.get("supports_axis_homing", False)

    @property
    def supports_global_homing(self) -> bool:
        return self.backend_caps.get("supports_global_homing", True)

    @property
    def supports_softlimits(self) -> bool:
        return self.backend_caps.get("supports_softlimits", False)

    def status_query_line(self) -> str:
        """Return the status query for the active backend."""
        if self.backend_type == "custom":
            return "(TM)"
        return "?"

    # ------------------------------------------------------------
    #  Central status parsing function (for TCP panel, etc.)
    # ------------------------------------------------------------
    def parse_status_line(self, line: str):
        """
        Parse a typical status line:
          <Idle|MPos:...|WPos:...|FS:...|Pn:...>
        and return (state, positions_dict).
        positions_dict uses MPos if available, otherwise WPos.
        """
        state = None
        positions = {}

        if not (line.startswith("<") and line.endswith(">")):
            return state, positions

        payload = line[1:-1]
        parts = payload.split("|")
        if not parts:
            return state, positions

        state = parts[0]
        mpos = {}
        wpos = {}

        for part in parts[1:]:
            if part.startswith("MPos:"):
                nums = part[5:].split(",")
                for idx, ax in enumerate(AXES):
                    if idx < len(nums):
                        try:
                            mpos[ax] = float(nums[idx])
                        except ValueError:
                            pass
            elif part.startswith("WPos:"):
                nums = part[5:].split(",")
                for idx, ax in enumerate(AXES):
                    if idx < len(nums):
                        try:
                            wpos[ax] = float(nums[idx])
                        except ValueError:
                            pass

        pose_src = mpos or wpos
        for ax, val in pose_src.items():
            positions[ax] = val

        # store for other parts
        self.last_status = {"state": state, "MPos": mpos, "WPos": wpos}
        return state, positions


    # ------------------------------------------------------------
    #  COM-Port-Management
    # ------------------------------------------------------------
    def connect(self, port, baud=115200):
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except:
                    pass

            # --------------------------
            #  EXPERTE: ARDUINO MEGA
            # No DTR/RTS, no reset
            # --------------------------
            self.ser = serial.Serial(
                port,
                baudrate=baud,
                timeout=0.05,
                write_timeout=0.2,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
                # Mega-specific: do NOT touch DTR and RTS!
            )

            time.sleep(0.2)

            #  Buffer leeren
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            #  Send one \r\n once (wakes up Mega)
            with self.lock:
                self.ser.write(b"\r\n")

            time.sleep(0.1)

            # Start RX thread
            self.rx_running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()

            if self.DEBUG_SERIAL:
                print(f"[OK] Connected to {port} @ {baud} (Mega/GRBL)")

            #  Jetzt die erste echte Anfrage
            self.send_line("$$")
            self.send_line(self.status_query_line())

        except Exception as e:
            self.ser = None
            print(f"[Error] Connection failed: {e}")
            raise






    def disconnect(self):
        """Cleanly close the serial connection."""
        try:
            self.rx_running = False
            if self.ser:
                if self.DEBUG_SERIAL:
                    print("[INFO] Serial disconnect()")
                self.ser.close()
            self.ser = None
        except Exception as e:
            print("[Error disconnect()]", e)

    # ------------------------------------------------------------
    #  COM-Port Liste abrufen
    # ------------------------------------------------------------
    def list_ports(self):
        """Return all available COM ports."""
        return [p.device for p in serial.tools.list_ports.comports()]

    # ------------------------------------------------------------
    #  UDP Mirror (Visualizer / Status Broadcast)
    # ------------------------------------------------------------
    def _mirror_udp(self, line: str):
        if not self.udp_sock:
            return
        try:
            self.udp_sock.sendto((line + "\n").encode("utf-8"), UDP_ADDR)

            # einfaches M3 S-Parsing
            if line.startswith("M3") and "S" in line:
                try:
                    s_val = float(line.split("S", 1)[1].strip())
                    import json, time as _time
                    msg = json.dumps({
                        "type": "abs",
                        "S": s_val,
                        "timestamp": _time.time()
                    })
                    self.udp_sock.sendto(msg.encode("utf-8"), UDP_ADDR)
                except Exception:
                    pass
        except Exception as e:
            if self.DEBUG_SERIAL:
                print("[UDP Mirror error]", e)

    # ------------------------------------------------------------
    #   Command sending
    # ------------------------------------------------------------
    def send_line(self, line: str):
        """Send one line to serial (and optionally mirror via UDP)."""
        if not self.ser:
            if not self._warned_not_connected:
                print("[Warn] Not connected (send_line ignored)")
                self._warned_not_connected = True
            return
        self._warned_not_connected = False

        try:
            text = line.strip()
            data = (text + "\n").encode("utf-8")
            with self.lock:
                self.ser.write(data)
            # DEBUG only if this is NOT the status query
            if self.DEBUG_SERIAL and text != "(TM)":
                print("TX>", text)
            if self.udp_sock:
                self._mirror_udp(text)
        except Exception as e:
            print(f"[Warn] Send error: {e}")

    def send_ctrl_x(self):
        """Send Ctrl+X (soft reset), without log spam when disconnected."""
        if not self.ser:
            if not self._warned_not_connected:
                print("[Warn] Not connected (Ctrl+X ignored)")
                self._warned_not_connected = True
            return
        self._warned_not_connected = False
        try:
            with self.lock:
                self.ser.write(b"\x18")
            if self.DEBUG_SERIAL:
                print("TX> <Ctrl+X>")
        except Exception as e:
            print(f"[Warn] send_ctrl_x failed: {e}")

    def _rx_loop(self):
        """Continuously read serial data and dispatch to listeners."""
        buf = b""
        if self.DEBUG_SERIAL:
            print("[RX] Thread started")

        while self.rx_running and self.ser:
            try:
                data = self.ser.read(1024)
                if data:
                    buf += data
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        txt = raw.decode("utf-8", errors="replace").strip()
                        if not txt:
                            continue
                        if self.DEBUG_SERIAL:
                            if not (
                                txt.startswith("<") or   # Statuszeilen <Idle|...>
                                txt == "ok" or
                                txt.startswith("[MSG:") or
                                txt.startswith("[GC:")
                            ):
                                print("RX<", txt)
                        # ---------------------------------

                        for cb in list(self.listeners):
                            try:
                                cb(txt)
                            except Exception as e:
                                print("[Warn Listener]", e)
                else:
                    time.sleep(0.02)

            except Exception as e:
                if self.DEBUG_SERIAL:
                    print("[RX-loop error]", e)
                time.sleep(0.1)

        if self.DEBUG_SERIAL:
            print("[RX] Thread ended")

# =========================
# GUI Rahmen
# =========================
root = tk.Tk()
root.title("Robotrol V7.1")
root.geometry("1150x1000")

style = ttk.Style()
try:
    style.theme_use("clam")

    # ===    Standard: Light Mode aktiv ===
    light_bg = "#f0f0f0"
    light_fg = "black"

    style.configure(".", background=light_bg, foreground=light_fg)
    style.configure("TLabel", background=light_bg, foreground=light_fg)
    style.configure("TFrame", background=light_bg)
    style.configure("TButton", background="#e6e6e6", foreground="black")
    style.map("TButton",
        background=[("active", "#d0d0d0"), ("pressed", "#c0c0c0")],
        foreground=[("active", "black"), ("pressed", "black")]
    )
    style.configure("TNotebook", background=light_bg, foreground=light_fg)
    style.configure("TNotebook.Tab", background="#e6e6e6", foreground=light_fg)
    style.map("TNotebook.Tab",
        background=[("selected", "#ffffff")],
        foreground=[("selected", "black")]
    )
    style.configure("TEntry", fieldbackground="white", foreground="black")
    style.configure("Horizontal.TScale", background=light_bg)
    root.configure(bg=light_bg)

except Exception as e:
    print("[Warn]", e)

current_theme = {"dark": False}  # keep state (mutable for closure)

def toggle_theme():
    if current_theme["dark"]:
        # ---- LIGHT MODE aktivieren ----
        bg = "#f0f0f0"
        fg = "black"
        style.configure(".", background=bg, foreground=fg)
        style.configure("TLabel", background=bg, foreground=fg)
        style.configure("TFrame", background=bg)
        style.configure("TButton", background="#e6e6e6", foreground="black")
        style.map("TButton",
            background=[("active", "#d0d0d0"), ("pressed", "#c0c0c0")],
            foreground=[("active", "black"), ("pressed", "black")]
        )
        style.configure("TNotebook", background=bg, foreground=fg)
        style.configure("TNotebook.Tab", background="#e6e6e6", foreground=fg)
        style.map("TNotebook.Tab",
            background=[("selected", "#ffffff")],
            foreground=[("selected", "black")]
        )
        style.configure("TEntry", fieldbackground="white", foreground="black")
        style.configure("Horizontal.TScale", background=bg)
        root.configure(bg=bg)
        current_theme["dark"] = False

    else:
        # ---- DARK MODE aktivieren ----
        bg = "#2b2b2b"
        fg = "#e0e0e0"
        style.configure(".", background=bg, foreground=fg)
        style.configure("TLabel", background=bg, foreground=fg)
        style.configure("TFrame", background=bg)
        style.configure("TButton", background="#3c3f41", foreground="#ffffff")
        style.map("TButton",
            background=[("active", "#505354"), ("pressed", "#606364")],
            foreground=[("active", "#ffffff"), ("pressed", "#ffffff")]
        )
        style.configure("TNotebook", background=bg, foreground=fg)
        style.configure("TNotebook.Tab", background="#3a3a3a", foreground=fg)
        style.map("TNotebook.Tab",
            background=[("selected", "#4a4a4a")],
            foreground=[("selected", "#ffffff")]
        )
        style.configure("TEntry", fieldbackground="#3c3f41", foreground="#ffffff")
        style.configure("Horizontal.TScale", background=bg)
        root.configure(bg=bg)
        current_theme["dark"] = True

client = SerialClient()
# Header

top = ttk.Frame(root)
top.pack(fill=tk.X, pady=2)

profile_var = tk.StringVar(value=ACTIVE_PROFILE["name"])
profile_last = {"name": ACTIVE_PROFILE["name"]}

def _on_profile_select(event=None):
    name = profile_var.get().strip()
    if not name:
        return
    if apply_profile(name):
        profile_last["name"] = name
    else:
        profile_var.set(profile_last["name"])

def refresh_ports():
    ports = client.list_ports()
    combo_ports["values"] = ports
    if ports:
        combo_ports.set(ports[0])
    print("[INFO] Ports refreshed:", ports)

def connect():
    port = combo_ports.get().strip()
    if not port:
        print("[WARN] No port selected")
        return

    baud = int(combo_baud.get())

    backend_map = {
        "FluidNC": "fluidnc",
        "GRBL": "grbl",
        "Custom": "custom"
    }
    client.set_backend(backend_map.get(combo_backend.get(), "fluidnc"))

    try:
        client.connect(port, baud)
        conn_lbl.configure(text="Connected", foreground="#2e7d32")
        print(f"[INFO] Connected to {port} @ {baud} Backend={client.backend_type}")
        if "execute_app" in globals():
            try:
                execute_app._apply_profile_runtime_flags(log_note=False)
            except Exception:
                pass
    except Exception as e:
        conn_lbl.configure(text="Disconnected", foreground="#b71c1c")
        print("[ERROR] Connect:", e)

def disconnect():
    try:
        client.disconnect()
        conn_lbl.configure(text="Disconnected", foreground="#b71c1c")
        print("[INFO] Disconnected")
    except Exception as e:
        print("[ERROR] Disconnect:", e)


def _log_profile_msg(msg):
    if "execute_app" in globals():
        try:
            execute_app.log(msg)
            return
        except Exception:
            pass
    print(msg)


def apply_profile(name):
    data = config_profiles.load_profile(name, base_dir=BASE_DIR, create_from_legacy=(name == DEFAULT_PROFILE))
    if not isinstance(data, dict) or data.get("tbd"):
        _log_profile_msg(f"[Profile] {name} is tbd.")
        return False
    ACTIVE_PROFILE["name"] = name
    ACTIVE_PROFILE["data"] = data
    dh_model = data.get("dh_model")
    if dh_model:
        tcp_pose_module.set_dh_model_from_dict(dh_model)
    if "execute_app" in globals():
        execute_app.apply_profile(name, data)
    return True



# --- Port Auswahl ---
ttk.Label(top, text="Port:").pack(side=tk.LEFT, padx=(4, 2))
combo_ports = ttk.Combobox(top, width=18, state="readonly")
combo_ports.pack(side=tk.LEFT, padx=(0, 10))

# --- Baudrate ---
ttk.Label(top, text="Baud:").pack(side=tk.LEFT, padx=(0, 2))
combo_baud = ttk.Combobox(top, width=10, state="readonly",
                          values=[115200, 230400, 460800, 921600])
combo_baud.set(115200)
combo_baud.pack(side=tk.LEFT, padx=(0, 10))

# --- Backend Auswahl ---
ttk.Label(top, text="Backend:").pack(side=tk.LEFT, padx=(0, 2))
combo_backend = ttk.Combobox(top, width=10, state="readonly",
                              values=["FluidNC", "GRBL", "Custom"])
combo_backend.set("FluidNC")
combo_backend.pack(side=tk.LEFT, padx=(0, 10))

# --- Buttons ---
ttk.Button(top, text="Refresh", width=8, command=refresh_ports).pack(side=tk.LEFT, padx=2)
ttk.Button(top, text="Connect", command=connect).pack(side=tk.LEFT, padx=2)
ttk.Button(top, text="Disconnect", command=disconnect).pack(side=tk.LEFT, padx=2)
ttk.Button(top, text="Theme", width=12, command=toggle_theme).pack(side=tk.LEFT, padx=4)
ttk.Label(top, text="Config:").pack(side=tk.LEFT, padx=(4, 2))
combo_profile = ttk.Combobox(
    top,
    width=10,
    state="readonly",
    textvariable=profile_var,
    values=list(config_profiles.PROFILE_FILES.keys()),
)
combo_profile.pack(side=tk.LEFT, padx=(0, 6))
combo_profile.bind("<<ComboboxSelected>>", _on_profile_select)

# --- Statusanzeige ---
conn_lbl = ttk.Label(top, text="Disconnected",
                     foreground="#b71c1c", width=14, anchor="e")
conn_lbl.pack(side=tk.RIGHT, padx=8)

# --- Gamepad-Modus-Anzeige (wird von ExecuteApp.set_gamepad_mode_cycle aktualisiert) ---
_gp_mode_var = tk.StringVar(value="GP: Normal")
_gp_mode_lbl = tk.Label(top, textvariable=_gp_mode_var,
                         bg="#455a64", fg="white", padx=7, pady=2,
                         font=("", 8, "bold"), relief="flat", cursor="hand2")
_gp_mode_lbl.pack(side=tk.RIGHT, padx=(2, 4))
# --- Run initial port scan ---
refresh_ports()
# Execute App
class ExecuteApp(ttk.Frame):
    def __init__(self, master, client, profile_name, profile_data):
        super().__init__(master)
        self.GEOM_DH = tcp_pose_module.GEOM_DH.copy()
        self.pack(fill=tk.BOTH, expand=True)
        self.client = client
        if not hasattr(self.client, "listeners"):
            self.client.listeners = []
        self.client.listeners.append(self.on_serial_line)
        self.paused = False
        self.repeat = tk.BooleanVar(value=False)
        self.repeat_count = tk.IntVar(value=0)
        self.repeat_times = tk.IntVar(value=1)   # desired repeat count
        # State
        self.axis_positions = {ax: 0.0 for ax in AXES}
        self._user_editing = {ax: False for ax in AXES}
        self.mode_absolute = tk.BooleanVar(value=True)
        self.live_move = tk.BooleanVar(value=True)
        self.poll_positions = tk.BooleanVar(value=True)
        self.speed_val = tk.IntVar(value=500)
        self.accel_val = tk.IntVar(value=500)
        self.vision_right_enabled = tk.BooleanVar(value=False)
        self.hw_limits = {}  # soft max (0..max) per axis from $$
        self._vis_skip_kin = False
        # Gamepad multi-mode (0=Normal, 1=Fixed/Tool, 2=Fixed/Point); cycled via RT
        self.gamepad_mode_cycle = tk.IntVar(value=0)
        self._gp_tcp_prev_rpy = None  # RPY continuity cache for Point mode
        # Max XYZ offset distance for fixed TCP gamepad (editable)
        self.fixed_tcp_max_dist = tk.DoubleVar(value=300.0)
        self.profile_name = profile_name
        self.profile_data = profile_data if isinstance(profile_data, dict) else {"name": profile_name, "version": 1}
        self.profile_has_endstops = _profile_has_endstops(self.profile_name, self.profile_data)
        self.profile_path = config_profiles.profile_path(profile_name, base_dir=BASE_DIR)
        self.gamepad_config_ui = None
        self.axis_limits = self._load_endstop_limits()
        self.manual_limits = self._load_manual_limits()
        self.endstop_limits_enabled = True
        # Program/Queue
        self.program = []
        self.run_event = threading.Event()
        self.abort_event = threading.Event()
        self._awaiting_ack = False

        self._build_ui()
        self._apply_profile_runtime_flags(log_note=False)
        self._send_vis_robot_profile()

        # Worker-Thread
        self.worker_thread = threading.Thread(target=self.worker, daemon=True)
        self.worker_thread.start()

        # Status-Poll
        self.after(250, self._tick_status_poll)

    # ---------- UI ----------
    def _build_ui(self):
        outer = ttk.Frame(self); outer.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.axis_entry_order = []   # tab order

        # Positions + Speed/Accel
        wrap = ttk.Frame(outer)
        wrap.pack(fill=tk.BOTH, expand=True, pady=8)

        #  Tabs for axis control
        pos_tabs = ttk.Notebook(wrap, width=900)
        pos_tabs.pack(side=tk.LEFT, fill=tk.Y, expand=False, padx=(0, 2))

        # --- Tab 1: Manuelle Achssteuerung (bisheriger posf-Inhalt)
        tab_manual = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_manual, text="Manual")

        # ============================================================
        #   Vision - camera + chessboard detection
        # ============================================================
        tab_endstops = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_endstops, text="Endstops")

        endstop_wrap = ttk.LabelFrame(tab_endstops, text="Endstop Deltas (relative to zero)")
        endstop_wrap.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        ttk.Label(
            endstop_wrap,
            text="Edit min/max deltas relative to zero (deg).",
        ).pack(anchor="w", padx=6, pady=(4, 8))

        endstop_grid = ttk.Frame(endstop_wrap)
        endstop_grid.pack(anchor="nw", padx=6, pady=(0, 8))

        ttk.Label(endstop_grid, text="Axis").grid(row=0, column=0, padx=4, pady=2, sticky="w")
        ttk.Label(endstop_grid, text="Min").grid(row=0, column=1, padx=4, pady=2, sticky="w")
        ttk.Label(endstop_grid, text="Max").grid(row=0, column=2, padx=4, pady=2, sticky="w")
        ttk.Label(endstop_grid, text="From current").grid(row=0, column=3, columnspan=2, padx=4, pady=2, sticky="w")

        self.endstop_vars = {}

        def _endstop_set_now(ax, which):
            try:
                val = float(self.axis_positions.get(ax, 0.0))
            except Exception:
                val = 0.0
            vmin, vmax = self.endstop_vars.get(ax, (None, None))
            if which == "min" and vmin is not None:
                vmin.set(f"{val:.3f}")
            elif which == "max" and vmax is not None:
                vmax.set(f"{val:.3f}")
            self._apply_endstop_limits_from_vars()

        def _endstop_apply_event(_evt=None):
            self._apply_endstop_limits_from_vars()

        for row_idx, ax in enumerate(AXES, start=1):
            lo, hi = self._effective_axis_limits(ax)
            vmin = tk.StringVar(value=f"{lo:.3f}")
            vmax = tk.StringVar(value=f"{hi:.3f}")
            self.endstop_vars[ax] = (vmin, vmax)

            ttk.Label(endstop_grid, text=ax, width=3).grid(row=row_idx, column=0, padx=4, pady=2, sticky="w")
            ent_min = ttk.Entry(endstop_grid, textvariable=vmin, width=10, justify="right")
            ent_min.grid(row=row_idx, column=1, padx=4, pady=2)
            ent_min.bind("<Return>", _endstop_apply_event)
            ent_max = ttk.Entry(endstop_grid, textvariable=vmax, width=10, justify="right")
            ent_max.grid(row=row_idx, column=2, padx=4, pady=2)
            ent_max.bind("<Return>", _endstop_apply_event)
            ttk.Button(
                endstop_grid,
                text="Min<-Now",
                width=8,
                command=lambda ax=ax: _endstop_set_now(ax, "min"),
            ).grid(row=row_idx, column=3, padx=4, pady=2)
            ttk.Button(
                endstop_grid,
                text="Max<-Now",
                width=8,
                command=lambda ax=ax: _endstop_set_now(ax, "max"),
            ).grid(row=row_idx, column=4, padx=4, pady=2)

        endstop_btns = ttk.Frame(endstop_wrap)
        endstop_btns.pack(anchor="w", padx=6, pady=(0, 6))

        ttk.Button(
            endstop_btns,
            text="Apply",
            command=self._apply_endstop_limits_from_vars,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            endstop_btns,
            text="Save",
            command=self._save_endstop_limits_from_vars,
        ).pack(side=tk.LEFT, padx=4)
        ttk.Button(
            endstop_btns,
            text="Load",
            command=self._load_endstop_limits_into_vars,
        ).pack(side=tk.LEFT, padx=4)

        def _reset_endstops():
            limits = self._default_endstop_limits()
            self.axis_limits = limits
            self._set_endstop_vars_from_limits(limits)
            self._apply_axis_limits_to_ui()

        ttk.Button(
            endstop_btns,
            text="Reset Defaults",
            command=_reset_endstops,
        ).pack(side=tk.LEFT, padx=4)

        tab_vision = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_vision, text="Vision")

        # Shared wrapper for clean alignment
        vision_wrap = ttk.Frame(tab_vision)
        vision_wrap.pack(padx=6, pady=6)

        # Camera and pose windows with equal size
        cam_cfg = _load_camera_config()
        self.cam = CameraCapture(
            vision_wrap,
            camera_index=int(cam_cfg.get("device_index", 0)),
            width=int(cam_cfg.get("width", 640)),
            height=int(cam_cfg.get("height", 480)),
            fps=int(cam_cfg.get("fps", 30)),
            preview_width=300,
            preview_height=225,
        )
        self.board_detector = BoardPose(
            vision_wrap,
            self.cam,
            pattern_size=(7, 7),
            preview_width=300,
            preview_height=225,
        )

        # Side by side with top-edge alignment
        self.cam.get_frame().pack(side="left", padx=8, pady=6, anchor="n")
        self.board_detector.get_frame().pack(side="left", padx=8, pady=6, anchor="n")

        # --- Start button that disappears after launch ---
        btn_row = ttk.Frame(tab_vision)
        btn_row.pack(pady=4)
        btn_start = ttk.Button(btn_row, text="Start Camera")
        btn_start.pack(side=tk.LEFT, padx=4)
        chk_right = ttk.Checkbutton(
            btn_row,
            text="Vision rechts",
            variable=self.vision_right_enabled,
            command=self._toggle_right_vision,
        )
        chk_right.pack(side=tk.LEFT, padx=4)

        def start_cam():
            try:
                self.cam.start()
                btn_start.pack_forget()  # Button ausblenden
            except Exception as e:
                print("[Vision start error]", e)

        btn_start.config(command=start_cam)

        tab_chess = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_chess, text="Chess Vision")
        try:
            self.chess_tab = ChessVisionTab(tab_chess, camera_capture=self.cam, logger=self.log)
            self.chess_tab.pack(fill="both", expand=True)
        except Exception as e:
            ttk.Label(tab_chess, text=f"Chess vision init failed: {e}").pack(anchor="w", padx=6, pady=6)

        # TM  Tab: FluidNC Config / OTA / $$ Inspector
        tab_updater = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_updater, text="Config / OTA")

        # FluidNCUpdaterFrame einbinden
        self.updater_frame = FluidNCUpdaterFrame(tab_updater, default_ip="192.168.25.149")

        # If current backend does not support OTA (e.g., GRBL), disable whole section
        if not self.client.supports_ota:
            for child in tab_updater.winfo_children():
                try:
                    child.configure(state="disabled")
                except Exception:
                    pass



        # --- Limit OTA tab height ---
        tab_updater.update_idletasks()
        tab_updater.configure(height=400)
        tab_updater.pack_propagate(False)
        
        #  Tab: Gamepad (nun unten in pos_tabs)
        tab_gamepad = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_gamepad, text="Gamepad")

        # Gamepad-Integration
        from gamepad_block_v3 import attach_gamepad_tab
        self.stop_gamepad = attach_gamepad_tab(tab_gamepad, self.client, self)

        #  Tab: Kinematics (DH6 / Weltkoordinaten)
        tab_kin = ttk.Frame(pos_tabs)
        # insert directly to the right of the gamepad tab
        pos_tabs.add(tab_kin, text="Kinematics")

        try:
            self.kinematics_tabs = TcpWorldKinematicsTabs(
                tab_kin,        # Parent
                self,           # ExecuteApp -> for axis access / G-code
                self.client     # SerialClient
            )
            self.kinematics_tabs.pack(fill="both", expand=True)
        except TypeError:
            # Alternate order if module is defined differently
            self.kinematics_tabs = TcpWorldKinematicsTabs(
                tab_kin,
                self.client,
                self
            )
            self.kinematics_tabs.pack(fill="both", expand=True)
        self.kinematics_tabs.pack(fill=tk.BOTH, expand=True)

        #  Tab: Fixed TCP Orientation
        tab_fixed_tcp = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_fixed_tcp, text="Fixed TCP")

        tab_pickplace = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_pickplace, text="Pick & Place")
        try:
            self.pickplace_tab = PickPlaceTab(
                tab_pickplace,
                logger=self.log,
                camera_capture=self.cam,
                execute_app=self,
            )
            self.pickplace_tab.pack(fill="both", expand=True)
        except Exception as e:
            ttk.Label(tab_pickplace, text=f"Pick & Place init failed: {e}").pack(anchor="w", padx=6, pady=6)

        self.fixed_tcp_enabled = tk.BooleanVar(value=False)
        self.fixed_tcp_roll = tk.DoubleVar(value=0.0)
        self.fixed_tcp_pitch = tk.DoubleVar(value=0.0)
        self.fixed_tcp_yaw = tk.DoubleVar(value=0.0)
        self.fixed_tcp_step = tk.DoubleVar(value=5.0)
        self.fixed_tcp_feed = tk.DoubleVar(value=3000.0)
        self.fixed_tcp_dx = tk.DoubleVar(value=0.0)
        self.fixed_tcp_dy = tk.DoubleVar(value=0.0)
        self.fixed_tcp_dz = tk.DoubleVar(value=0.0)
        self.fixed_tcp_exec_on_release = tk.BooleanVar(value=True)
        self.tcp_gcode_mode = tk.BooleanVar(value=False)
        self.tcp_gcode_abs = True
        self.fixed_tcp_origin_x = 0.0
        self.fixed_tcp_origin_y = 0.0
        self.fixed_tcp_origin_z = 0.0
        self.fixed_tcp_origin_axes = {ax: 0.0 for ax in AXES}
        self.fixed_tcp_sync_pending = False
        self.fixed_tcp_target_delta = None
        self.fixed_tcp_last_delta = None
        self.fixed_tcp_stable_count = 0
        self.fixed_tcp_user_dragging = False
        self.fixed_tcp_mode = tk.StringVar(value="world")
        self.fixed_tcp_point_dx = tk.DoubleVar(value=0.0)
        self.fixed_tcp_point_dy = tk.DoubleVar(value=0.0)
        self.fixed_tcp_point_dz = tk.DoubleVar(value=100.0)
        self.fixed_tcp_point = None
        self.fixed_tcp_gamepad_mode = tk.BooleanVar(value=False)
        self.fixed_tcp_gp_xy_step = tk.DoubleVar(value=2.0)
        self.fixed_tcp_gp_z_step = tk.DoubleVar(value=1.0)
        self._vis_path_fixed = []
        self._vis_path_kin = []
        self._vis_fixed_frame = None

        fixed_wrap = ttk.LabelFrame(tab_fixed_tcp, text="Fixed TCP Orientation")
        fixed_wrap.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        info = ttk.Label(fixed_wrap, text="Fix current TCP orientation and move in XYZ while keeping it.")
        info.pack(anchor="w", padx=4, pady=(2, 4))

        row_fix = ttk.Frame(fixed_wrap)
        row_fix.pack(fill=tk.X, padx=4, pady=2)

        def _update_fixed_from_tcp():
            tcp = self.get_current_tcp_mm()
            self.fixed_tcp_origin_x = float(tcp.get("X_mm", 0.0))
            self.fixed_tcp_origin_y = float(tcp.get("Y_mm", 0.0))
            self.fixed_tcp_origin_z = float(tcp.get("Z_mm", 0.0))
            try:
                self.fixed_tcp_origin_axes = {
                    ax: float(self.axis_positions.get(ax, 0.0)) for ax in AXES
                }
            except Exception:
                pass
            self.fixed_tcp_roll.set(float(tcp.get("Roll_deg", 0.0)))
            self.fixed_tcp_pitch.set(float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0))))
            self.fixed_tcp_yaw.set(float(tcp.get("Yaw_deg", 0.0)))
            _set_fixed_offsets(0.0, 0.0, 0.0)
            self.fixed_tcp_sync_pending = False
            self.fixed_tcp_target_delta = None
            self.fixed_tcp_last_delta = None
            self.fixed_tcp_stable_count = 0
            self.fixed_tcp_point = None
            try:
                roll = float(self.fixed_tcp_roll.get())
                pitch = float(self.fixed_tcp_pitch.get())
                yaw = float(self.fixed_tcp_yaw.get())
                R = _rpy_to_R(roll, pitch, yaw)
                z_axis = (R[0][2], R[1][2], R[2][2])
                zn = (z_axis[0] ** 2 + z_axis[1] ** 2 + z_axis[2] ** 2) ** 0.5
                if zn > 1e-9:
                    z_axis = (z_axis[0] / zn, z_axis[1] / zn, z_axis[2] / zn)
                    up = (0.0, 0.0, 1.0)
                    if abs(z_axis[2]) > 0.95:
                        up = (0.0, 1.0, 0.0)
                    x_axis = (
                        up[1] * z_axis[2] - up[2] * z_axis[1],
                        up[2] * z_axis[0] - up[0] * z_axis[2],
                        up[0] * z_axis[1] - up[1] * z_axis[0],
                    )
                    xn = (x_axis[0] ** 2 + x_axis[1] ** 2 + x_axis[2] ** 2) ** 0.5
                    if xn > 1e-9:
                        x_axis = (x_axis[0] / xn, x_axis[1] / xn, x_axis[2] / xn)
                        y_axis = (
                            z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
                            z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
                            z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
                        )
                        self._send_vis_fixed_frame(
                            (self.fixed_tcp_origin_x, self.fixed_tcp_origin_y, self.fixed_tcp_origin_z),
                            x_axis,
                            y_axis,
                            z_axis,
                        )
            except Exception:
                pass
            if hasattr(self, "log"):
                self.log("Fixed TCP orientation captured from current pose.")
            _update_fixed_gcode_preview()

        ttk.Button(
            row_fix,
            text="Fix from current TCP",
            command=_update_fixed_from_tcp,
        ).pack(side=tk.LEFT)

        def _reset_fixed_xyz():
            _set_fixed_offsets(0.0, 0.0, 0.0)

        ttk.Button(
            row_fix,
            text="Set XYZ to zero",
            command=_reset_fixed_xyz,
        ).pack(side=tk.LEFT, padx=(6, 0))

        def _home_fixed_xyz():
            _set_fixed_offsets(0.0, 0.0, 0.0)
            origin_axes = getattr(self, "fixed_tcp_origin_axes", None)
            feed = float(self.fixed_tcp_feed.get())
            if origin_axes and hasattr(self, "client"):
                self.client.send_line("G90")
                parts = [f"{ax}{origin_axes.get(ax, 0.0):.3f}" for ax in AXES]
                self.client.send_line("G1 " + " ".join(parts) + f" F{feed:.0f}")
                _request_fixed_delta_sync(target_delta=(0.0, 0.0, 0.0))
                if hasattr(self, "log"):
                    self.log("Fixed TCP home -> stored joint origin.")
                return
            roll = float(self.fixed_tcp_roll.get())
            pitch = float(self.fixed_tcp_pitch.get())
            yaw = float(self.fixed_tcp_yaw.get())
            if hasattr(self, "kinematics_tabs"):
                self.kinematics_tabs.move_tcp_pose(
                    self.fixed_tcp_origin_x,
                    self.fixed_tcp_origin_y,
                    self.fixed_tcp_origin_z,
                    roll,
                    pitch,
                    yaw,
                    feed=feed,
                    allow_out_of_limits=False,
                )

        ttk.Button(
            row_fix,
            text="Home",
            command=_home_fixed_xyz,
        ).pack(side=tk.LEFT, padx=(6, 0))


        def _show_fixed_tcp_help():
            win = tk.Toplevel(self)
            win.title("Fixed TCP Help")
            win.geometry("640x460")
            win.resizable(False, False)
            wrap = ttk.Frame(win, padding=8)
            wrap.pack(fill=tk.BOTH, expand=True)

            ttk.Label(
                wrap,
                text="Fixed TCP: quick guide",
                font=("Segoe UI", 11, "bold"),
            ).pack(anchor="w", pady=(0, 6))

            c = tk.Canvas(wrap, width=600, height=220, bg="white", highlightthickness=1, highlightbackground="#999")
            c.pack(pady=(0, 6))

            # Simple graphic: origin, offsets, tool frame, and point target
            c.create_text(10, 10, anchor="nw", text="Fixed mode flow", fill="#333")
            c.create_oval(60, 70, 90, 100, outline="#333")
            c.create_line(75, 100, 75, 150, arrow="last", fill="#333")
            c.create_text(75, 160, text="TCP", fill="#333")
            c.create_text(50, 55, text="Fix", fill="#333")

            # Offset arrow
            c.create_line(75, 85, 210, 100, arrow="last", fill="#2e7d32", width=2)
            c.create_text(135, 70, text="dx/dy/dz", fill="#2e7d32")

            # Tool frame axes
            c.create_line(260, 120, 320, 120, arrow="last", fill="#1565c0")
            c.create_line(260, 120, 260, 60, arrow="last", fill="#1565c0")
            c.create_text(330, 120, text="tool +X", fill="#1565c0", anchor="w")
            c.create_text(260, 50, text="tool +Z", fill="#1565c0", anchor="s")
            c.create_text(240, 40, text="Tool frame", fill="#1565c0")

            # Point target
            c.create_oval(430, 70, 460, 100, outline="#333")
            c.create_line(260, 120, 445, 85, arrow="last", fill="#ad1457", width=2)
            c.create_text(470, 85, text="TCP Point target", fill="#ad1457", anchor="w")

            ttk.Label(
                wrap,
                text="Controls",
                font=("Segoe UI", 10, "bold"),
            ).pack(anchor="w", pady=(4, 2))

            steps = [
                "Fixed mode: lock Roll/Pitch/Yaw to the fixed values while moving XYZ.",
                "Execute on release: moves are sent only when you release a slider/drag.",
                "G-code TCP mode: incoming G0/G1 X/Y/Z are converted to TCP moves.",
                "Mode=World: offsets are in world XYZ (no rotation applied).",
                "Mode=Tool frame: offsets are applied in the tool's local axes.",
                "Mode=TCP Point: TCP keeps pointing at a fixed 3D point.",
                "Point dX/dY/dZ: defines the TCP Point relative to current TCP.",
            ]
            for s in steps:
                ttk.Label(wrap, text="- " + s).pack(anchor="w")

        ttk.Button(
            row_fix,
            text="Help",
            command=_show_fixed_tcp_help,
        ).pack(side=tk.LEFT, padx=(6, 0))

        row_opts = ttk.Frame(fixed_wrap)
        row_opts.pack(fill=tk.X, padx=4, pady=(2, 2))
        row_opts.columnconfigure(0, weight=1)
        row_opts.columnconfigure(1, weight=1)
        row_opts.columnconfigure(2, weight=1)

        fixed_mode_widgets = []

        chk_fixed_mode = ttk.Checkbutton(
            row_opts,
            text="Fixed mode",
            variable=self.fixed_tcp_enabled,
        )
        chk_fixed_mode.grid(row=0, column=0, sticky="w")

        chk_exec_release = ttk.Checkbutton(
            row_opts,
            text="Execute on release",
            variable=self.fixed_tcp_exec_on_release,
        )
        chk_exec_release.grid(row=0, column=1, sticky="w")
        fixed_mode_widgets.append(chk_exec_release)

        chk_tcp_gcode = ttk.Checkbutton(
            row_opts,
            text="G-code TCP mode (XY(Z) uses fixed orientation)",
            variable=self.tcp_gcode_mode,
        )
        chk_tcp_gcode.grid(row=0, column=2, sticky="w")

        chk_gamepad_ctrl = ttk.Checkbutton(
            row_opts,
            text="Gamepad XYZ",
            variable=self.fixed_tcp_gamepad_mode,
        )
        chk_gamepad_ctrl.grid(row=1, column=0, sticky="w")
        fixed_mode_widgets.append(chk_gamepad_ctrl)

        gp_sens_frame = ttk.Frame(row_opts)
        gp_sens_frame.grid(row=1, column=1, columnspan=2, sticky="w")
        ttk.Label(gp_sens_frame, text="L\u2192XY:", foreground="gray").pack(side=tk.LEFT, padx=(0, 2))
        ent_gp_xy = ttk.Entry(gp_sens_frame, textvariable=self.fixed_tcp_gp_xy_step, width=5, justify="right")
        ent_gp_xy.pack(side=tk.LEFT)
        ttk.Label(gp_sens_frame, text="mm  R\u2192Z:", foreground="gray").pack(side=tk.LEFT, padx=(4, 2))
        ent_gp_z = ttk.Entry(gp_sens_frame, textvariable=self.fixed_tcp_gp_z_step, width=5, justify="right")
        ent_gp_z.pack(side=tk.LEFT)
        ttk.Label(gp_sens_frame, text="mm", foreground="gray").pack(side=tk.LEFT, padx=(2, 6))
        ttk.Label(gp_sens_frame, text="Max±:", foreground="gray").pack(side=tk.LEFT, padx=(4, 2))
        ent_max_dist = ttk.Entry(gp_sens_frame, textvariable=self.fixed_tcp_max_dist, width=5, justify="right")
        ent_max_dist.pack(side=tk.LEFT)
        ttk.Label(gp_sens_frame, text="mm", foreground="gray").pack(side=tk.LEFT, padx=(2, 6))
        fixed_mode_widgets.extend([ent_gp_xy, ent_gp_z, ent_max_dist])

        def _gp_test_go():
            if not self.fixed_tcp_enabled.get():
                return
            try:
                step = max(0.1, float(self.fixed_tcp_gp_xy_step.get()))
            except Exception:
                step = 1.0
            _move_rel(step, 0.0, 0.0)

        btn_gp_test = ttk.Button(gp_sens_frame, text="Test \u25b6", command=_gp_test_go)
        btn_gp_test.pack(side=tk.LEFT)
        fixed_mode_widgets.append(btn_gp_test)

        row_mode = ttk.Frame(fixed_wrap)
        row_mode.pack(fill=tk.X, padx=4, pady=(2, 2))
        ttk.Label(row_mode, text="Mode:").pack(side=tk.LEFT, padx=(0, 6))
        rb_world = ttk.Radiobutton(
            row_mode,
            text="World",
            variable=self.fixed_tcp_mode,
            value="world",
        )
        rb_world.pack(side=tk.LEFT, padx=(0, 10))
        fixed_mode_widgets.append(rb_world)
        rb_tool = ttk.Radiobutton(
            row_mode,
            text="Tool frame",
            variable=self.fixed_tcp_mode,
            value="tool",
        )
        rb_tool.pack(side=tk.LEFT, padx=(0, 10))
        fixed_mode_widgets.append(rb_tool)
        rb_point = ttk.Radiobutton(
            row_mode,
            text="TCP Point",
            variable=self.fixed_tcp_mode,
            value="point",
        )
        rb_point.pack(side=tk.LEFT)
        fixed_mode_widgets.append(rb_point)

        gp_point_lbl = ttk.Label(row_mode, text="", foreground="#1565c0")
        gp_point_lbl.pack(side=tk.LEFT, padx=(12, 0))

        row_point = ttk.Frame(fixed_wrap)
        row_point.pack(fill=tk.X, padx=4, pady=(2, 2))
        ttk.Label(row_point, text="Point dX").pack(side=tk.LEFT, padx=(0, 2))
        ent_pt_dx = ttk.Entry(row_point, textvariable=self.fixed_tcp_point_dx, width=7, justify="right")
        ent_pt_dx.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(row_point, text="dY").pack(side=tk.LEFT, padx=(0, 2))
        ent_pt_dy = ttk.Entry(row_point, textvariable=self.fixed_tcp_point_dy, width=7, justify="right")
        ent_pt_dy.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(row_point, text="dZ").pack(side=tk.LEFT, padx=(0, 2))
        ent_pt_dz = ttk.Entry(row_point, textvariable=self.fixed_tcp_point_dz, width=7, justify="right")
        ent_pt_dz.pack(side=tk.LEFT, padx=(0, 6))
        fixed_mode_widgets.extend([ent_pt_dx, ent_pt_dy, ent_pt_dz])

        def _set_tcp_point_from_current():
            tcp = self.get_current_tcp_mm()
            try:
                roll = float(tcp.get("Roll_deg", 0.0))
                pitch = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
                yaw = float(tcp.get("Yaw_deg", 0.0))
                dxp = float(self.fixed_tcp_point_dx.get())
                dyp = float(self.fixed_tcp_point_dy.get())
                dzp = float(self.fixed_tcp_point_dz.get())
            except Exception:
                return

            R = _rpy_to_R(roll, pitch, yaw)
            # Use actual TCP rotation matrix columns directly (correct for any roll/pitch/yaw)
            x_axis = (R[0][0], R[1][0], R[2][0])
            y_axis = (R[0][1], R[1][1], R[2][1])
            z_axis = (R[0][2], R[1][2], R[2][2])

            dxw = x_axis[0] * dxp + y_axis[0] * dyp + z_axis[0] * dzp
            dyw = x_axis[1] * dxp + y_axis[1] * dyp + z_axis[1] * dzp
            dzw = x_axis[2] * dxp + y_axis[2] * dyp + z_axis[2] * dzp

            try:
                x = float(tcp.get("X_mm", 0.0)) + dxw
                y = float(tcp.get("Y_mm", 0.0)) + dyw
                z = float(tcp.get("Z_mm", 0.0)) + dzw
            except Exception:
                return
            self.fixed_tcp_point = (x, y, z)
            if hasattr(self, "log"):
                self.log(f"TCP Point set to ({x:.2f}, {y:.2f}, {z:.2f}).")

        # Expose for external access (e.g. LT gamepad action)
        self._set_tcp_point_from_current_fn = _set_tcp_point_from_current
        self._update_fixed_from_tcp_fn = _update_fixed_from_tcp

        btn_set_tcp_point = ttk.Button(
            row_point,
            text="Set TCP Point",
            command=lambda: (_set_tcp_point_from_current(), _update_fixed_gcode_preview()),
        )
        btn_set_tcp_point.pack(side=tk.LEFT, padx=(4, 0))
        fixed_mode_widgets.append(btn_set_tcp_point)
        row_vals = ttk.Frame(fixed_wrap)
        row_vals.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(row_vals, text="Roll").pack(side=tk.LEFT, padx=(0, 2))
        ent_roll = ttk.Entry(row_vals, textvariable=self.fixed_tcp_roll, width=6, justify="right")
        ent_roll.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(row_vals, text="Pitch").pack(side=tk.LEFT, padx=(0, 2))
        ent_pitch = ttk.Entry(row_vals, textvariable=self.fixed_tcp_pitch, width=6, justify="right")
        ent_pitch.pack(side=tk.LEFT, padx=(0, 6))
        ttk.Label(row_vals, text="Yaw").pack(side=tk.LEFT, padx=(0, 2))
        ent_yaw = ttk.Entry(row_vals, textvariable=self.fixed_tcp_yaw, width=6, justify="right")
        ent_yaw.pack(side=tk.LEFT, padx=(0, 6))
        fixed_mode_widgets.extend([ent_roll, ent_pitch, ent_yaw])

        row_ctrl = ttk.Frame(fixed_wrap)
        row_ctrl.pack(fill=tk.X, padx=4, pady=(2, 2))
        ttk.Label(row_ctrl, text="Step [mm]").pack(side=tk.LEFT, padx=(0, 4))
        ent_step = ttk.Entry(row_ctrl, textvariable=self.fixed_tcp_step, width=6, justify="right")
        ent_step.pack(side=tk.LEFT, padx=(0, 8))
        ttk.Label(row_ctrl, text="Feed [mm/min]").pack(side=tk.LEFT, padx=(0, 4))
        ent_feed = ttk.Entry(row_ctrl, textvariable=self.fixed_tcp_feed, width=8, justify="right")
        ent_feed.pack(side=tk.LEFT)
        fixed_mode_widgets.extend([ent_step, ent_feed])

        jog = ttk.LabelFrame(fixed_wrap, text="Move XYZ (relative)")
        jog.pack(fill=tk.BOTH, padx=4, pady=(4, 4))

        def _clamp_offset(v):
            try:
                val = float(v)
            except Exception:
                val = 0.0
            try:
                lim = max(10.0, float(self.fixed_tcp_max_dist.get()))
            except Exception:
                lim = 300.0
            return max(-lim, min(lim, val))

        def _get_fixed_origin():
            return (
                float(self.fixed_tcp_origin_x),
                float(self.fixed_tcp_origin_y),
                float(self.fixed_tcp_origin_z),
            )

        def _request_fixed_delta_sync(target_delta=None):
            self.fixed_tcp_sync_pending = True
            self.fixed_tcp_target_delta = target_delta
            self.fixed_tcp_last_delta = None
            self.fixed_tcp_stable_count = 0

        def _set_fixed_offsets(dx, dy, dz):
            self.fixed_tcp_dx.set(_clamp_offset(dx))
            self.fixed_tcp_dy.set(_clamp_offset(dy))
            self.fixed_tcp_dz.set(_clamp_offset(dz))
            try:
                _update_plane_markers()
            except Exception:
                pass
        self._fixed_tcp_set_offsets = _set_fixed_offsets

        def _rpy_to_R(roll_deg, pitch_deg, yaw_deg):
            yaw_r = math.radians(yaw_deg)
            pitch_r = math.radians(pitch_deg)
            roll_r = math.radians(roll_deg)
            cy, sy = math.cos(yaw_r), math.sin(yaw_r)
            cp, sp = math.cos(pitch_r), math.sin(pitch_r)
            cr, sr = math.cos(roll_r), math.sin(roll_r)
            return [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]

        def _rpy_from_R(R):
            r11, r12, r13 = R[0][0], R[0][1], R[0][2]
            r21, r22, r23 = R[1][0], R[1][1], R[1][2]
            r31, r32, r33 = R[2][0], R[2][1], R[2][2]
            pitch = math.degrees(math.atan2(-r31, (r11 * r11 + r21 * r21) ** 0.5))
            roll = math.degrees(math.atan2(r32, r33))
            yaw = math.degrees(math.atan2(r21, r11))
            return roll, pitch, yaw

        def _dot(a, b):
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

        def _cross(a, b):
            return (
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            )

        def _norm(v):
            return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5

        def _apply_rot(R, v):
            return (
                R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
                R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
                R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
            )

        def _apply_rot_T(R, v):
            return (
                R[0][0] * v[0] + R[1][0] * v[1] + R[2][0] * v[2],
                R[0][1] * v[0] + R[1][1] * v[1] + R[2][1] * v[2],
                R[0][2] * v[0] + R[1][2] * v[1] + R[2][2] * v[2],
            )

        def _get_fixed_rpy(tcp):
            if self.fixed_tcp_enabled.get():
                return (
                    float(self.fixed_tcp_roll.get()),
                    float(self.fixed_tcp_pitch.get()),
                    float(self.fixed_tcp_yaw.get()),
                )
            return (
                float(tcp.get("Roll_deg", 0.0)),
                float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0))),
                float(tcp.get("Yaw_deg", 0.0)),
            )

        def _offset_to_world(dx, dy, dz, tcp):
            mode = (self.fixed_tcp_mode.get() or "world").lower()
            if mode == "tool":
                roll, pitch, yaw = _get_fixed_rpy(tcp)
                R = _rpy_to_R(roll, pitch, yaw)
                dx, dy, dz = _apply_rot(R, (dx, dy, dz))
            dz = -dz
            return dx, dy, dz

        def _world_to_offset(dxw, dyw, dzw, tcp):
            mode = (self.fixed_tcp_mode.get() or "world").lower()
            if mode == "tool":
                roll, pitch, yaw = _get_fixed_rpy(tcp)
                R = _rpy_to_R(roll, pitch, yaw)
                dxw, dyw, dzw = _apply_rot_T(R, (dxw, dyw, dzw))
            dzw = -dzw
            return dxw, dyw, dzw
        self._fixed_tcp_world_to_offsets = _world_to_offset

        def _calc_fixed_target():
            tcp = self.get_current_tcp_mm()
            dx = float(self.fixed_tcp_dx.get())
            dy = float(self.fixed_tcp_dy.get())
            dz = float(self.fixed_tcp_dz.get())
            dxw, dyw, dzw = _offset_to_world(dx, dy, dz, tcp)
            ox, oy, oz = _get_fixed_origin()
            x = ox + dxw
            y = oy + dyw
            z = oz + dzw
            roll, pitch, yaw = _get_fixed_rpy(tcp)
            mode = (self.fixed_tcp_mode.get() or "world").lower()
            if mode == "point":
                if self.fixed_tcp_point is None:
                    _set_tcp_point_from_current()
                point = self.fixed_tcp_point
                if point is not None:
                    vx = float(point[0]) - x
                    vy = float(point[1]) - y
                    vz = float(point[2]) - z
                    dist = (vx * vx + vy * vy + vz * vz) ** 0.5
                    if dist > 1e-6:
                        ux, uy, uz = vx / dist, vy / dist, vz / dist
                        z_axis = (ux, uy, uz)
                        ref_roll, ref_pitch, ref_yaw = _get_fixed_rpy(tcp)
                        R_ref = _rpy_to_R(ref_roll, ref_pitch, ref_yaw)
                        x_ref = (R_ref[0][0], R_ref[1][0], R_ref[2][0])
                        dot_xz = _dot(x_ref, z_axis)
                        x_proj = (
                            x_ref[0] - dot_xz * z_axis[0],
                            x_ref[1] - dot_xz * z_axis[1],
                            x_ref[2] - dot_xz * z_axis[2],
                        )
                        xn = _norm(x_proj)
                        if xn < 1e-6:
                            up = (0.0, 0.0, 1.0)
                            if abs(z_axis[2]) > 0.95:
                                up = (0.0, 1.0, 0.0)
                            x_proj = _cross(up, z_axis)
                            xn = _norm(x_proj)
                        if xn > 1e-6:
                            x_axis = (x_proj[0] / xn, x_proj[1] / xn, x_proj[2] / xn)
                            y_axis = _cross(z_axis, x_axis)
                            yn = _norm(y_axis)
                            if yn > 1e-6:
                                y_axis = (y_axis[0] / yn, y_axis[1] / yn, y_axis[2] / yn)
                                x_axis = _cross(y_axis, z_axis)
                                R = [
                                    [x_axis[0], y_axis[0], z_axis[0]],
                                    [x_axis[1], y_axis[1], z_axis[1]],
                                    [x_axis[2], y_axis[2], z_axis[2]],
                                ]
                                roll, pitch, yaw = _rpy_from_R(R)
            return x, y, z, roll, pitch, yaw, dx, dy, dz

        def _set_fixed_gcode_status(ok=None, text=None):
            if not hasattr(self, "fixed_tcp_gcode_status_lbl"):
                return
            if ok is None:
                color = "#777777"
                msg = text or "Endstops: --"
            elif ok:
                color = "#2e7d32"
                msg = text or "Endstops: OK"
            else:
                color = "#b71c1c"
                msg = text or "Endstops: LIMIT"
            if hasattr(self, "fixed_tcp_gcode_status_var"):
                self.fixed_tcp_gcode_status_var.set(msg)
            self.fixed_tcp_gcode_status_lbl.configure(bg=color, fg="white")

        def _emit_fixed_gcode(gcode, lims=None, joints=None):
            if not hasattr(self, "fixed_tcp_gcode_txt"):
                return
            txt = self.fixed_tcp_gcode_txt
            txt.configure(state="normal")
            txt.delete("1.0", tk.END)
            if not gcode:
                txt.insert(tk.END, "--", "plain")
            else:
                axes = ("A", "X", "Y", "B", "Z", "C")
                parts = gcode.split(" ")
                for part in parts:
                    tag = "plain"
                    for ax in axes:
                        if part.startswith(ax):
                            try:
                                val = float(part[1:])
                            except Exception:
                                val = joints.get(ax, None) if joints else None
                            if val is not None and lims:
                                lo, hi = lims.get(ax, (-999, 999))
                                tag = "ok" if lo <= float(val) <= hi else "bad_val"
                            break
                    txt.insert(tk.END, part + " ", tag)
            txt.configure(state="disabled")

        def _update_fixed_gcode_preview():
            if not hasattr(self, "fixed_tcp_gcode_txt"):
                return
            if not self.fixed_tcp_enabled.get():
                _emit_fixed_gcode("Fixed mode off")
                _set_fixed_gcode_status(None, "Endstops: --")
                return
            if not hasattr(self, "kinematics_tabs"):
                _emit_fixed_gcode("No kinematics")
                _set_fixed_gcode_status(None, "Endstops: --")
                return
            try:
                x, y, z, roll, pitch, yaw, _dx, _dy, _dz = _calc_fixed_target()
                feed = float(self.fixed_tcp_feed.get())
            except Exception:
                _emit_fixed_gcode("Preview error")
                _set_fixed_gcode_status(False, "Endstops: n/a")
                return

            res = None
            try:
                res = self.kinematics_tabs.preview_tcp_gcode(
                    x, y, z, roll, pitch, yaw, feed=feed
                )
            except Exception:
                res = None

            if not res:
                _emit_fixed_gcode("Preview error")
                _set_fixed_gcode_status(False, "Endstops: n/a")
                return

            gcode = res.get("gcode", "")
            lims = res.get("limits", {}) or {}
            joints = res.get("joints", {}) or {}
            ok = bool(res.get("ok", False))
            _emit_fixed_gcode(gcode, lims, joints)
            _set_fixed_gcode_status(ok)

        def _apply_fixed_target(execute=False):
            if not self.fixed_tcp_enabled.get():
                return False
            x, y, z, roll, pitch, yaw, dx, dy, dz = _calc_fixed_target()
            if execute and hasattr(self, "kinematics_tabs"):
                _request_fixed_delta_sync(target_delta=(dx, dy, dz))
                prev_skip = getattr(self, "_vis_skip_kin", False)
                self._vis_skip_kin = True
                try:
                    ok = self.kinematics_tabs.move_tcp_pose(
                        x,
                        y,
                        z,
                        roll,
                        pitch,
                        yaw,
                        feed=float(self.fixed_tcp_feed.get()),
                        allow_out_of_limits=False,
                    )
                except Exception as e:
                    self._vis_skip_kin = prev_skip
                    if hasattr(self, "log"):
                        self.log(f"Fixed TCP move error: {e}")
                    return False
                self._vis_skip_kin = prev_skip
                if ok:
                    self._append_vis_path_fixed((x, y, z))
                    return True
                # Fallback: step toward target with small increments
                try:
                    step = max(0.5, float(self.fixed_tcp_step.get()))
                except Exception:
                    step = 5.0
                cur = self.get_current_tcp_mm()
                cx = float(cur.get("X_mm", 0.0))
                cy = float(cur.get("Y_mm", 0.0))
                cz = float(cur.get("Z_mm", 0.0))
                dx = x - cx
                dy = y - cy
                dz = z - cz
                dist = (dx * dx + dy * dy + dz * dz) ** 0.5
                if dist < 1e-6:
                    return False
                ux, uy, uz = dx / dist, dy / dist, dz / dist
                steps = int(dist / step) + 1
                for _ in range(min(steps, 50)):
                    cx += ux * step
                    cy += uy * step
                    cz += uz * step
                    prev_skip = getattr(self, "_vis_skip_kin", False)
                    self._vis_skip_kin = True
                    ok = self.kinematics_tabs.move_tcp_pose(
                        cx,
                        cy,
                        cz,
                        roll,
                        pitch,
                        yaw,
                        feed=float(self.fixed_tcp_feed.get()),
                        allow_out_of_limits=False,
                    )
                    self._vis_skip_kin = prev_skip
                    if not ok:
                        return False
                    self._append_vis_path_fixed((cx, cy, cz))
                return True
            return False
        self._fixed_tcp_apply_target = _apply_fixed_target
        self._fixed_tcp_calc_target = _calc_fixed_target

        def _sync_entry(entry, var):
            entry.delete(0, tk.END)
            entry.insert(0, f"{float(var.get()):.1f}")

        def _bind_entry(entry, var):
            def _on_enter(event=None):
                val = _clamp_offset(entry.get())
                var.set(val)
                _sync_entry(entry, var)
                if self.fixed_tcp_exec_on_release.get():
                    _apply_fixed_target(execute=True)
                return "break"
            entry.bind("<Return>", _on_enter)

        def _move_rel(dx, dy, dz):
            if not self.fixed_tcp_enabled.get():
                return
            cur_dx = _clamp_offset(self.fixed_tcp_dx.get())
            cur_dy = _clamp_offset(self.fixed_tcp_dy.get())
            cur_dz = _clamp_offset(self.fixed_tcp_dz.get())
            _set_fixed_offsets(cur_dx + dx, cur_dy + dy, cur_dz + dz)
            try:
                _apply_fixed_target(execute=True)
            except Exception as e:
                if hasattr(self, "log"):
                    self.log(f"Fixed TCP button move error: {e}")

        def _step_val():
            try:
                return float(self.fixed_tcp_step.get())
            except Exception:
                return 0.0

        def _make_slider_row(parent, label, var):
            row = ttk.Frame(parent)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=label, width=6).pack(side=tk.LEFT)
            scale = ttk.Scale(row, from_=-300.0, to=300.0, variable=var, length=170)
            scale.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)
            ent = ttk.Entry(row, width=7, justify="right")
            ent.pack(side=tk.LEFT, padx=(4, 0))
            _sync_entry(ent, var)
            _bind_entry(ent, var)

            def _on_scale(_val=None):
                _sync_entry(ent, var)
                _update_plane_markers()

            def _on_press(_event=None):
                self.fixed_tcp_user_dragging = True

            def _on_release(_event=None):
                self.fixed_tcp_user_dragging = False
                if not self.fixed_tcp_enabled.get():
                    return
                if self.fixed_tcp_exec_on_release.get():
                    _apply_fixed_target(execute=True)

            def _on_var_change(*_args):
                try:
                    _sync_entry(ent, var)
                    _update_plane_markers()
                except Exception:
                    pass

            scale.configure(command=_on_scale)
            scale.bind("<ButtonPress-1>", _on_press)
            scale.bind("<ButtonRelease-1>", _on_release)
            var.trace_add("write", _on_var_change)
            return scale, ent

        btn_row = ttk.Frame(jog)
        btn_row.pack(pady=(2, 2))
        ttk.Button(btn_row, text="X -", width=6, command=lambda: _move_rel(-_step_val(), 0.0, 0.0)).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="X +", width=6, command=lambda: _move_rel(_step_val(), 0.0, 0.0)).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Y -", width=6, command=lambda: _move_rel(0.0, -_step_val(), 0.0)).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Y +", width=6, command=lambda: _move_rel(0.0, _step_val(), 0.0)).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Z -", width=6, command=lambda: _move_rel(0.0, 0.0, -_step_val())).pack(side=tk.LEFT, padx=4)
        ttk.Button(btn_row, text="Z +", width=6, command=lambda: _move_rel(0.0, 0.0, _step_val())).pack(side=tk.LEFT, padx=4)

        sliders = ttk.Frame(jog)
        sliders.pack(fill=tk.BOTH, expand=True, pady=(2, 0))
        sliders_left = ttk.Frame(sliders)
        sliders_left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sliders_right = ttk.Frame(sliders)
        sliders_right.pack(side=tk.LEFT, padx=(6, 0), fill=tk.Y)

        scale_x, ent_x = _make_slider_row(sliders_left, "X [mm]", self.fixed_tcp_dx)
        scale_y, ent_y = _make_slider_row(sliders_left, "Y [mm]", self.fixed_tcp_dy)
        scale_z, ent_z = _make_slider_row(sliders_left, "Z [mm]", self.fixed_tcp_dz)
        fixed_mode_widgets.extend([scale_x, ent_x, scale_y, ent_y, scale_z, ent_z])

        def _set_all_offsets():
            _set_fixed_offsets(0.0, 0.0, 0.0)
            if self.fixed_tcp_exec_on_release.get():
                _apply_fixed_target(execute=True)

        def _enqueue_fixed_target():
            if not self.fixed_tcp_enabled.get():
                return
            mode = (self.fixed_tcp_mode.get() or "world").lower()
            if mode == "point":
                if hasattr(self, "log"):
                    self.log("Fixed TCP queue: point mode uses live orientation; skipping.")
                return
            x, y, z, _roll, _pitch, _yaw, _dx, _dy, _dz = _calc_fixed_target()
            feed = float(self.fixed_tcp_feed.get())
            if not self.tcp_gcode_mode.get():
                self.tcp_gcode_mode.set(True)
                if hasattr(self, "log"):
                    self.log("Fixed TCP queue: TCP G-code mode enabled.")
            gline = f"G90 G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{feed:.0f}"
            self.enqueue(gline)

        btn_set = ttk.Button(sliders_right, text="Set", width=10,
                             command=lambda: _apply_fixed_target(execute=True))
        btn_set.pack(fill=tk.X, pady=(0, 4))
        btn_set_all = ttk.Button(sliders_right, text="Set All", width=10,
                                 command=_set_all_offsets)
        btn_set_all.pack(fill=tk.X, pady=(0, 4))
        btn_send_queue = ttk.Button(sliders_right, text="Send to Queue", width=10,
                                     command=_enqueue_fixed_target)
        btn_send_queue.pack(fill=tk.X)
        fixed_mode_widgets.extend([btn_set, btn_set_all, btn_send_queue])

        # XY / XZ / YZ fields (drag target around TCP)
        canvas_size = 158
        radius_mm = 200.0
        radius_px = 75.0
        center = canvas_size // 2

        planes_wrap = ttk.Frame(jog)
        planes_wrap.pack(pady=(2, 0), fill=tk.X)

        planes = ttk.Frame(planes_wrap)
        planes.pack(side=tk.LEFT)

        gcode_wrap = ttk.LabelFrame(planes_wrap, text="Target G-code")
        gcode_wrap.pack(side=tk.LEFT, padx=(6, 0), fill=tk.Y)
        self.fixed_tcp_gcode_status_var = tk.StringVar(value="Endstops: --")
        self.fixed_tcp_gcode_status_lbl = tk.Label(
            gcode_wrap,
            textvariable=self.fixed_tcp_gcode_status_var,
            bg="#777777",
            fg="white",
            anchor="center",
        )
        self.fixed_tcp_gcode_status_lbl.pack(fill=tk.X, padx=4, pady=(4, 2))

        self.fixed_tcp_gcode_txt = tk.Text(
            gcode_wrap,
            height=5,
            width=45,
            wrap="none",
            font=("Consolas", 9),
        )
        self.fixed_tcp_gcode_txt.pack(padx=4, pady=(0, 4))
        self.fixed_tcp_gcode_txt.tag_configure("ok", background="#e8ffe8")
        self.fixed_tcp_gcode_txt.tag_configure("bad_val", background="#ffd0d0")
        self.fixed_tcp_gcode_txt.tag_configure("plain", background="white")
        self.fixed_tcp_gcode_txt.configure(state="disabled")

        def _make_plane(parent, label_text):
            wrap = ttk.Frame(parent)
            wrap.pack(side=tk.LEFT, padx=4)
            ttk.Label(wrap, text=label_text).pack()
            c = tk.Canvas(
                wrap,
                width=canvas_size,
                height=canvas_size,
                highlightthickness=1,
                highlightbackground="#666",
            )
            c.pack()
            c.create_oval(center - radius_px, center - radius_px, center + radius_px, center + radius_px, outline="#666")
            c.create_line(center, 0, center, canvas_size, fill="#444")
            c.create_line(0, center, canvas_size, center, fill="#444")
            dot = c.create_oval(center - 4, center - 4, center + 4, center + 4, fill="#d32f2f", outline="")
            return c, dot

        xy, xy_dot = _make_plane(planes, "XY")
        xz, xz_dot = _make_plane(planes, "XZ")
        yz, yz_dot = _make_plane(planes, "YZ")
        fixed_mode_widgets.extend([xy, xz, yz])

        def _update_plane_markers():
            dx = _clamp_offset(self.fixed_tcp_dx.get())
            dy = _clamp_offset(self.fixed_tcp_dy.get())
            dz = _clamp_offset(self.fixed_tcp_dz.get())

            def _pos(a, b):
                px = center + (a / radius_mm) * radius_px
                py = center - (b / radius_mm) * radius_px
                return px, py

            px, py = _pos(dx, dy)
            xy.coords(xy_dot, px - 4, py - 4, px + 4, py + 4)

            px, py = _pos(dx, dz)
            xz.coords(xz_dot, px - 4, py - 4, px + 4, py + 4)

            px, py = _pos(dy, dz)
            yz.coords(yz_dot, px - 4, py - 4, px + 4, py + 4)

        def _set_from_event(event, a_axis, b_axis):
            if not self.fixed_tcp_enabled.get():
                return
            dx_px = event.x - center
            dy_px = center - event.y
            r = (dx_px ** 2 + dy_px ** 2) ** 0.5
            if r > radius_px and r > 1e-6:
                scale = radius_px / r
                dx_px *= scale
                dy_px *= scale
            a = (dx_px / radius_px) * radius_mm
            b = (dy_px / radius_px) * radius_mm
            if a_axis == "X":
                self.fixed_tcp_dx.set(a)
            elif a_axis == "Y":
                self.fixed_tcp_dy.set(a)
            elif a_axis == "Z":
                self.fixed_tcp_dz.set(a)
            if b_axis == "X":
                self.fixed_tcp_dx.set(b)
            elif b_axis == "Y":
                self.fixed_tcp_dy.set(b)
            elif b_axis == "Z":
                self.fixed_tcp_dz.set(b)
            _update_plane_markers()

        def _bind_plane(canvas, a_axis, b_axis):
            def _on_drag(event):
                self.fixed_tcp_user_dragging = True
                _set_from_event(event, a_axis, b_axis)

            def _on_release(event):
                _set_from_event(event, a_axis, b_axis)
                self.fixed_tcp_user_dragging = False
                if not self.fixed_tcp_enabled.get():
                    return
                if self.fixed_tcp_exec_on_release.get():
                    _apply_fixed_target(execute=True)

            canvas.bind("<B1-Motion>", _on_drag)
            canvas.bind("<ButtonRelease-1>", _on_release)

        _bind_plane(xy, "X", "Y")
        _bind_plane(xz, "X", "Z")
        _bind_plane(yz, "Y", "Z")

        def _sync_fixed_tcp_modes(*_args):
            if getattr(self, "_fixed_tcp_guard", False):
                return
            self._fixed_tcp_guard = True
            try:
                enabled = bool(self.fixed_tcp_enabled.get())
                if not enabled:
                    if self.fixed_tcp_exec_on_release.get():
                        self.fixed_tcp_exec_on_release.set(False)
                    if self.tcp_gcode_mode.get():
                        self.tcp_gcode_mode.set(False)
                    if self.fixed_tcp_gamepad_mode.get():
                        self.fixed_tcp_gamepad_mode.set(False)
                else:
                    if not self.fixed_tcp_exec_on_release.get():
                        self.fixed_tcp_exec_on_release.set(True)
                if self.tcp_gcode_mode.get() and not enabled:
                    self.fixed_tcp_enabled.set(True)
                    enabled = True
                state = "normal" if enabled else "disabled"
                for w in fixed_mode_widgets:
                    try:
                        w.configure(state=state)
                    except Exception:
                        pass
            finally:
                self._fixed_tcp_guard = False

        self.fixed_tcp_enabled.trace_add("write", _sync_fixed_tcp_modes)
        self.tcp_gcode_mode.trace_add("write", _sync_fixed_tcp_modes)
        _sync_fixed_tcp_modes()

        def _update_gp_point_notice(*_):
            if self.fixed_tcp_mode.get() == "point" and self.fixed_tcp_gamepad_mode.get():
                gp_point_lbl.configure(text="\u2192 Spitze zeigt auf Fixpunkt")
            else:
                gp_point_lbl.configure(text="")
            # Reset RPY continuity cache when gamepad mode is toggled
            if not self.fixed_tcp_gamepad_mode.get():
                self._gp_tcp_prev_rpy = None

        self.fixed_tcp_mode.trace_add("write", _update_gp_point_notice)
        self.fixed_tcp_gamepad_mode.trace_add("write", _update_gp_point_notice)

        def _auto_enable_gamepad_xyz(*_):
            """Auto-enable Gamepad XYZ when Fixed TCP is manually turned on."""
            if self.fixed_tcp_enabled.get() and not self.fixed_tcp_gamepad_mode.get():
                self.fixed_tcp_gamepad_mode.set(True)

        self.fixed_tcp_enabled.trace_add("write", _auto_enable_gamepad_xyz)

        def _fixed_preview_refresh(*_args):
            _update_fixed_gcode_preview()

        self.fixed_tcp_roll.trace_add("write", _fixed_preview_refresh)
        self.fixed_tcp_pitch.trace_add("write", _fixed_preview_refresh)
        self.fixed_tcp_yaw.trace_add("write", _fixed_preview_refresh)
        self.fixed_tcp_feed.trace_add("write", _fixed_preview_refresh)
        self.fixed_tcp_mode.trace_add("write", _fixed_preview_refresh)
        self.fixed_tcp_enabled.trace_add("write", _fixed_preview_refresh)
        self.fixed_tcp_dx.trace_add("write", _fixed_preview_refresh)
        self.fixed_tcp_dy.trace_add("write", _fixed_preview_refresh)
        self.fixed_tcp_dz.trace_add("write", _fixed_preview_refresh)

        _update_plane_markers()


        # --- Tab: Plane G2/G3 ---
        tab_plane = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_plane, text="GCODE")

        self.plane_model = {
            "origin": (0.0, 0.0, 0.0),
            "u": (1.0, 0.0, 0.0),
            "v": (0.0, 1.0, 0.0),
            "n": (0.0, 0.0, 1.0),
            "rpy": (0.0, 0.0, 0.0),
        }
        self.plane_defined = False

        self.plane_origin_x = tk.DoubleVar(value=0.0)
        self.plane_origin_y = tk.DoubleVar(value=0.0)
        self.plane_origin_z = tk.DoubleVar(value=0.0)
        self.plane_u_x = tk.DoubleVar(value=1.0)
        self.plane_u_y = tk.DoubleVar(value=0.0)
        self.plane_u_z = tk.DoubleVar(value=0.0)
        self.plane_v_x = tk.DoubleVar(value=0.0)
        self.plane_v_y = tk.DoubleVar(value=1.0)
        self.plane_v_z = tk.DoubleVar(value=0.0)
        self.plane_n_x = tk.DoubleVar(value=0.0)
        self.plane_n_y = tk.DoubleVar(value=0.0)
        self.plane_n_z = tk.DoubleVar(value=1.0)
        self.plane_roll = tk.DoubleVar(value=0.0)
        self.plane_pitch = tk.DoubleVar(value=0.0)
        self.plane_yaw = tk.DoubleVar(value=0.0)
        self.plane_abs_mode = tk.BooleanVar(value=True)
        self.plane_dir = tk.StringVar(value="G2")
        self.plane_end_u = tk.DoubleVar(value=20.0)
        self.plane_end_v = tk.DoubleVar(value=0.0)
        self.plane_center_i = tk.DoubleVar(value=10.0)
        self.plane_center_j = tk.DoubleVar(value=0.0)
        self.plane_w = tk.DoubleVar(value=0.0)
        self.plane_feed = tk.DoubleVar(value=3000.0)
        self.plane_seg_len = tk.DoubleVar(value=2.0)
        # G-code file interpreter state
        self.plane_gcode_lines = []
        self.plane_gcode_path = tk.StringVar(value="")
        self.plane_gcode_status = tk.StringVar(value="No file loaded.")
        self.plane_gcode_progress = tk.StringVar(value="")
        self.plane_scale_x = tk.DoubleVar(value=1.0)
        self.plane_scale_y = tk.DoubleVar(value=1.0)
        self.plane_mirror_x = tk.BooleanVar(value=False)
        self.plane_mirror_y = tk.BooleanVar(value=False)
        self.plane_rotate = tk.StringVar(value="0°")
        self.plane_offset_u = tk.DoubleVar(value=0.0)
        self.plane_offset_v = tk.DoubleVar(value=0.0)
        self.plane_dry_run = tk.BooleanVar(value=True)
        self.plane_gcode_worker = None
        self.plane_gcode_stop_event = threading.Event()
        self.plane_preview_canvas = None

        def _plane_log(msg):
            if hasattr(self, "log"):
                self.log(msg)
            else:
                print(msg)

        def _plane_dot(a, b):
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

        def _plane_cross(a, b):
            return (
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            )

        def _plane_norm(v):
            return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5

        def _plane_normalize(v, default):
            n = _plane_norm(v)
            if n < 1e-9:
                return default
            return (v[0] / n, v[1] / n, v[2] / n)

        def _plane_rpy_to_R(roll_deg, pitch_deg, yaw_deg):
            yaw_r = math.radians(yaw_deg)
            pitch_r = math.radians(pitch_deg)
            roll_r = math.radians(roll_deg)
            cy, sy = math.cos(yaw_r), math.sin(yaw_r)
            cp, sp = math.cos(pitch_r), math.sin(pitch_r)
            cr, sr = math.cos(roll_r), math.sin(roll_r)
            return [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ]

        def _plane_get_origin():
            ox = float(self.plane_origin_x.get())
            oy = float(self.plane_origin_y.get())
            oz = float(self.plane_origin_z.get())
            self.plane_model["origin"] = (ox, oy, oz)
            return ox, oy, oz

        def _plane_get_rpy():
            roll = float(self.plane_roll.get())
            pitch = float(self.plane_pitch.get())
            yaw = float(self.plane_yaw.get())
            self.plane_model["rpy"] = (roll, pitch, yaw)
            return roll, pitch, yaw

        def _plane_get_axes():
            u = (
                float(self.plane_u_x.get()),
                float(self.plane_u_y.get()),
                float(self.plane_u_z.get()),
            )
            v = (
                float(self.plane_v_x.get()),
                float(self.plane_v_y.get()),
                float(self.plane_v_z.get()),
            )
            u = _plane_normalize(u, (1.0, 0.0, 0.0))
            dot_vu = _plane_dot(v, u)
            v = (v[0] - dot_vu * u[0], v[1] - dot_vu * u[1], v[2] - dot_vu * u[2])
            v = _plane_normalize(v, (0.0, 1.0, 0.0))
            n = _plane_cross(u, v)
            n = _plane_normalize(n, (0.0, 0.0, 1.0))
            return u, v, n

        def _plane_sync_axes():
            u, v, n = _plane_get_axes()
            self.plane_u_x.set(u[0])
            self.plane_u_y.set(u[1])
            self.plane_u_z.set(u[2])
            self.plane_v_x.set(v[0])
            self.plane_v_y.set(v[1])
            self.plane_v_z.set(v[2])
            self.plane_n_x.set(n[0])
            self.plane_n_y.set(n[1])
            self.plane_n_z.set(n[2])
            self.plane_model["u"] = u
            self.plane_model["v"] = v
            self.plane_model["n"] = n
            return u, v, n

        def _plane_set_model(origin, u, v, n, rpy):
            ox, oy, oz = origin
            u = _plane_normalize(u, (1.0, 0.0, 0.0))
            v = _plane_normalize(v, (0.0, 1.0, 0.0))
            n = _plane_normalize(n, (0.0, 0.0, 1.0))
            self.plane_origin_x.set(ox)
            self.plane_origin_y.set(oy)
            self.plane_origin_z.set(oz)
            self.plane_u_x.set(u[0])
            self.plane_u_y.set(u[1])
            self.plane_u_z.set(u[2])
            self.plane_v_x.set(v[0])
            self.plane_v_y.set(v[1])
            self.plane_v_z.set(v[2])
            self.plane_n_x.set(n[0])
            self.plane_n_y.set(n[1])
            self.plane_n_z.set(n[2])
            roll, pitch, yaw = rpy
            self.plane_roll.set(roll)
            self.plane_pitch.set(pitch)
            self.plane_yaw.set(yaw)
            self.plane_model["origin"] = (ox, oy, oz)
            self.plane_model["u"] = u
            self.plane_model["v"] = v
            self.plane_model["n"] = n
            self.plane_model["rpy"] = (roll, pitch, yaw)
            self.plane_defined = True

        def _plane_send_frame():
            try:
                origin = _plane_get_origin()
                u, v, n = _plane_sync_axes()
                if hasattr(self, "_send_vis_fixed_frame"):
                    self._send_vis_fixed_frame(origin, u, v, n)
            except Exception:
                pass

        def _plane_set_from_current():
            try:
                tcp = self.get_current_tcp_mm()
            except Exception:
                tcp = None
            if not tcp:
                _plane_log("Plane: no TCP pose available.")
                return
            ox = float(tcp.get("X_mm", 0.0))
            oy = float(tcp.get("Y_mm", 0.0))
            oz = float(tcp.get("Z_mm", 0.0))
            roll = float(tcp.get("Roll_deg", 0.0))
            pitch = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
            yaw = float(tcp.get("Yaw_deg", 0.0))
            R = _plane_rpy_to_R(roll, pitch, yaw)
            u = (R[0][0], R[1][0], R[2][0])
            v = (R[0][1], R[1][1], R[2][1])
            n = (R[0][2], R[1][2], R[2][2])
            _plane_set_model((ox, oy, oz), u, v, n, (roll, pitch, yaw))
            self.plane_w.set(0.0)
            _plane_log("Plane defined from current TCP.")
            _plane_send_frame()

        def _plane_set_world_xy():
            try:
                tcp = self.get_current_tcp_mm()
            except Exception:
                tcp = None
            if not tcp:
                _plane_log("Plane: no TCP pose available.")
                return
            ox = float(tcp.get("X_mm", 0.0))
            oy = float(tcp.get("Y_mm", 0.0))
            oz = float(tcp.get("Z_mm", 0.0))
            roll = float(tcp.get("Roll_deg", 0.0))
            pitch = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
            yaw = float(tcp.get("Yaw_deg", 0.0))
            _plane_set_model(
                (ox, oy, oz),
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0),
                (roll, pitch, yaw),
            )
            self.plane_w.set(0.0)
            _plane_log("Plane set to world XY at current TCP.")
            _plane_send_frame()

        def _plane_arc_data():
            if not self.plane_defined:
                _plane_log("Plane not defined. Use 'Set plane from current TCP'.")
                return None
            origin = _plane_get_origin()
            u_axis, v_axis, n_axis = _plane_sync_axes()
            roll, pitch, yaw = _plane_get_rpy()
            try:
                tcp = self.get_current_tcp_mm()
            except Exception:
                tcp = None
            if not tcp:
                _plane_log("Plane: no TCP pose available.")
                return None
            px = float(tcp.get("X_mm", 0.0))
            py = float(tcp.get("Y_mm", 0.0))
            pz = float(tcp.get("Z_mm", 0.0))
            rel = (px - origin[0], py - origin[1], pz - origin[2])
            u0 = _plane_dot(rel, u_axis)
            v0 = _plane_dot(rel, v_axis)
            w0 = _plane_dot(rel, n_axis)
            w = float(self.plane_w.get())
            if abs(w0 - w) > 0.5:
                _plane_log(f"Plane: current W offset is {w0:.2f}mm, target {w:.2f}mm.")

            if self.plane_abs_mode.get():
                u1 = float(self.plane_end_u.get())
                v1 = float(self.plane_end_v.get())
            else:
                u1 = u0 + float(self.plane_end_u.get())
                v1 = v0 + float(self.plane_end_v.get())

            cx = u0 + float(self.plane_center_i.get())
            cy = v0 + float(self.plane_center_j.get())
            r0 = math.hypot(u0 - cx, v0 - cy)
            r1 = math.hypot(u1 - cx, v1 - cy)
            if r0 < 1e-6:
                _plane_log("Plane arc: radius too small.")
                return None
            if abs(r0 - r1) > 0.5:
                _plane_log(f"Plane arc: radius mismatch (r0={r0:.3f}, r1={r1:.3f}).")
                return None

            start = math.atan2(v0 - cy, u0 - cx)
            end = math.atan2(v1 - cy, u1 - cx)
            if abs(u1 - u0) < 1e-6 and abs(v1 - v0) < 1e-6:
                delta = -2.0 * math.pi if self.plane_dir.get() == "G2" else 2.0 * math.pi
            else:
                delta = end - start
                if self.plane_dir.get() == "G2":
                    if delta >= 0.0:
                        delta -= 2.0 * math.pi
                else:
                    if delta <= 0.0:
                        delta += 2.0 * math.pi

            arc_len = abs(delta) * r0
            try:
                feed = float(self.plane_feed.get())
            except Exception:
                feed = 3000.0
            return {
                "origin": origin,
                "u_axis": u_axis,
                "v_axis": v_axis,
                "n_axis": n_axis,
                "rpy": (roll, pitch, yaw),
                "u0": u0,
                "v0": v0,
                "w": w,
                "u1": u1,
                "v1": v1,
                "cx": cx,
                "cy": cy,
                "r0": r0,
                "arc_len": arc_len,
                "start": start,
                "delta": delta,
                "feed": feed,
            }

        def _plane_arc_points():
            arc = _plane_arc_data()
            if not arc:
                return None
            origin = arc["origin"]
            u_axis = arc["u_axis"]
            v_axis = arc["v_axis"]
            n_axis = arc["n_axis"]
            w = arc["w"]
            cx = arc["cx"]
            cy = arc["cy"]
            r0 = arc["r0"]
            start = arc["start"]
            delta = arc["delta"]
            arc_len = arc["arc_len"]
            feed = arc["feed"]
            roll, pitch, yaw = arc["rpy"]

            try:
                seg_len = float(self.plane_seg_len.get())
            except Exception:
                seg_len = 2.0
            seg_len = max(0.5, seg_len)
            steps = max(3, int(math.ceil(arc_len / seg_len)))
            pts = []
            for i in range(1, steps + 1):
                ang = start + delta * (i / steps)
                uu = cx + r0 * math.cos(ang)
                vv = cy + r0 * math.sin(ang)
                x = origin[0] + uu * u_axis[0] + vv * v_axis[0] + w * n_axis[0]
                y = origin[1] + uu * u_axis[1] + vv * v_axis[1] + w * n_axis[1]
                z = origin[2] + uu * u_axis[2] + vv * v_axis[2] + w * n_axis[2]
                pts.append((x, y, z))
            _plane_log(
                f"Plane arc: {len(pts)} segments, len={arc_len:.1f}mm, dir={self.plane_dir.get()}"
            )
            return pts, (roll, pitch, yaw), feed

        def _plane_detect_native_plane(n_axis):
            ax = abs(n_axis[0])
            ay = abs(n_axis[1])
            az = abs(n_axis[2])
            tol = 0.95
            if az >= ay and az >= ax and az >= tol:
                return {"gplane": "G17", "a": ("X", 0), "b": ("Y", 1), "c": ("Z", 2), "offsets": ("I", "J")}
            if ay >= ax and ay >= az and ay >= tol:
                return {"gplane": "G18", "a": ("X", 0), "b": ("Z", 2), "c": ("Y", 1), "offsets": ("I", "K")}
            if ax >= ay and ax >= az and ax >= tol:
                return {"gplane": "G19", "a": ("Y", 1), "b": ("Z", 2), "c": ("X", 0), "offsets": ("J", "K")}
            return None

        def _plane_world_axis_value(origin, u_axis, v_axis, n_axis, uu, vv, ww, idx):
            return (
                origin[idx]
                + uu * u_axis[idx]
                + vv * v_axis[idx]
                + ww * n_axis[idx]
            )

        def _plane_arc_queue_native():
            arc = _plane_arc_data()
            if not arc:
                return

            plane = _plane_detect_native_plane(arc["n_axis"])
            if not plane:
                _plane_log("Native G2/G3 requires an almost-canonical plane normal (XY/XZ/YZ).")
                return

            origin = arc["origin"]
            u_axis = arc["u_axis"]
            v_axis = arc["v_axis"]
            n_axis = arc["n_axis"]
            w = arc["w"]
            u0 = arc["u0"]
            v0 = arc["v0"]
            u1 = arc["u1"]
            v1 = arc["v1"]
            cx = arc["cx"]
            cy = arc["cy"]
            feed = arc["feed"]

            an, ai = plane["a"]
            bn, bi = plane["b"]
            cn, ci = plane["c"]
            off_a, off_b = plane["offsets"]

            start_a = _plane_world_axis_value(origin, u_axis, v_axis, n_axis, u0, v0, w, ai)
            start_b = _plane_world_axis_value(origin, u_axis, v_axis, n_axis, u0, v0, w, bi)
            end_a = _plane_world_axis_value(origin, u_axis, v_axis, n_axis, u1, v1, w, ai)
            end_b = _plane_world_axis_value(origin, u_axis, v_axis, n_axis, u1, v1, w, bi)
            end_c = _plane_world_axis_value(origin, u_axis, v_axis, n_axis, u1, v1, w, ci)
            center_a = _plane_world_axis_value(origin, u_axis, v_axis, n_axis, cx, cy, w, ai)
            center_b = _plane_world_axis_value(origin, u_axis, v_axis, n_axis, cx, cy, w, bi)
            off_val_a = center_a - start_a
            off_val_b = center_b - start_b

            gcode = (
                f"{self.plane_dir.get()} "
                f"{an}{end_a:.3f} {bn}{end_b:.3f} {cn}{end_c:.3f} "
                f"{off_a}{off_val_a:.3f} {off_b}{off_val_b:.3f} "
                f"F{feed:.0f}"
            )
            self.enqueue("G90")
            self.enqueue(plane["gplane"])
            self.enqueue(gcode)
            _plane_log(f"Native arc queued: {plane['gplane']} + {self.plane_dir.get()} ({an}{bn}{cn})")

        def _plane_queue_gcode_example(filename, fallback_lines, label):
            base_dir_local = os.path.dirname(os.path.abspath(__file__))
            example_path = os.path.join(base_dir_local, "data", "examples", filename)
            lines = []
            try:
                with open(example_path, "r", encoding="utf-8") as f:
                    lines = [ln.strip() for ln in f.readlines() if ln.strip()]
            except Exception:
                lines = list(fallback_lines)
            for ln in lines:
                self.enqueue(ln)
            _plane_log(f"Queued {label} ({len(lines)} lines) from {example_path}.")

        def _plane_queue_3dp_example_g17():
            _plane_queue_gcode_example(
                "plane_g2g3_3dp_example.gcode",
                [
                    "; 3D printer G2/G3 quick example (G17 XY)",
                    "G21",
                    "G90",
                    "G17",
                    "G0 X20.000 Y20.000 Z0.300 F3000",
                    "G2 X60.000 Y20.000 I20.000 J0.000 F1200",
                    "G3 X20.000 Y20.000 I-20.000 J0.000 F1200",
                ],
                "3DP example G17",
            )

        def _plane_queue_3dp_example_g18():
            _plane_queue_gcode_example(
                "plane_g2g3_3dp_example_g18.gcode",
                [
                    "; 3D printer G2/G3 quick example (G18 XZ)",
                    "G21",
                    "G90",
                    "G18",
                    "G0 X20.000 Y0.300 Z20.000 F3000",
                    "G2 X60.000 Z20.000 I20.000 K0.000 F1200",
                    "G3 X20.000 Z20.000 I-20.000 K0.000 F1200",
                ],
                "3DP example G18",
            )

        def _plane_queue_3dp_example_g19():
            _plane_queue_gcode_example(
                "plane_g2g3_3dp_example_g19.gcode",
                [
                    "; 3D printer G2/G3 quick example (G19 YZ)",
                    "G21",
                    "G90",
                    "G19",
                    "G0 X0.300 Y20.000 Z20.000 F3000",
                    "G2 Y60.000 Z20.000 J20.000 K0.000 F1200",
                    "G3 Y20.000 Z20.000 J-20.000 K0.000 F1200",
                ],
                "3DP example G19",
            )

        def _plane_queue_3dp_examples_all():
            self.enqueue("; ---- Dry-run bundle start: G17/G18/G19 ----")
            self.enqueue("; ---- Example G17 ----")
            _plane_queue_3dp_example_g17()
            self.enqueue("; ---- Example G18 ----")
            _plane_queue_3dp_example_g18()
            self.enqueue("; ---- Example G19 ----")
            _plane_queue_3dp_example_g19()
            self.enqueue("; ---- Dry-run bundle end ----")
            _plane_log("Queued dry-run bundle: G17 + G18 + G19.")

        def _plane_arc_send():
            data = _plane_arc_points()
            if not data:
                return
            if not hasattr(self, "kinematics_tabs"):
                _plane_log("Plane arc: no kinematics available.")
                return
            pts, rpy, feed = data
            roll, pitch, yaw = rpy
            ok_count = 0
            for x, y, z in pts:
                ok = self.kinematics_tabs.move_tcp_pose(
                    x,
                    y,
                    z,
                    roll,
                    pitch,
                    yaw,
                    feed=feed,
                    allow_out_of_limits=False,
                )
                if not ok:
                    _plane_log("Plane arc aborted (IK/limits).")
                    break
                ok_count += 1
            _plane_log(f"Plane arc sent: {ok_count}/{len(pts)} segments.")

        def _plane_arc_queue():
            data = _plane_arc_points()
            if not data:
                return
            if not hasattr(self, "kinematics_tabs") or not hasattr(self.kinematics_tabs, "preview_tcp_gcode"):
                _plane_log("Plane arc: no preview/queue available.")
                return
            pts, rpy, feed = data
            roll, pitch, yaw = rpy
            count = 0
            for x, y, z in pts:
                res = self.kinematics_tabs.preview_tcp_gcode(x, y, z, roll, pitch, yaw, feed=feed)
                if not res or not res.get("gcode"):
                    _plane_log("Plane arc queue aborted (preview failed).")
                    break
                if res.get("ok") is False:
                    _plane_log("Plane arc queue aborted (limits).")
                    break
                self.enqueue(res["gcode"])
                count += 1
            _plane_log(f"Plane arc queued: {count}/{len(pts)} segments.")

        # ---- G-code file interpreter ----

        def _plane_parse_line(raw):
            line = raw
            if ";" in line:
                line = line[:line.index(";")]
            line = line.strip().upper()
            if not line:
                return None
            tokens = line.split()
            if not tokens:
                return None
            cmd = tokens[0]
            params = {}
            for tok in tokens[1:]:
                if len(tok) >= 2 and tok[0].isalpha():
                    try:
                        params[tok[0]] = float(tok[1:])
                    except ValueError:
                        pass
            return {"cmd": cmd, "params": params}

        def _plane_exec_gcode(dry_run):
            lines = self.plane_gcode_lines
            if not lines:
                _plane_log("G-code: no file loaded.")
                return
            if not self.plane_defined:
                _plane_log("G-code: plane not defined. Set plane from TCP first.")
                return
            if not dry_run and not hasattr(self, "kinematics_tabs"):
                _plane_log("G-code: kinematics not available.")
                return
            origin = _plane_get_origin()
            u_axis, v_axis, n_axis = _plane_sync_axes()
            roll, pitch, yaw = _plane_get_rpy()
            scale_x = max(1e-6, float(self.plane_scale_x.get()))
            scale_y = max(1e-6, float(self.plane_scale_y.get()))
            _mx = -1.0 if self.plane_mirror_x.get() else 1.0
            _my = -1.0 if self.plane_mirror_y.get() else 1.0
            _rot = self.plane_rotate.get()
            off_u = float(self.plane_offset_u.get())
            off_v = float(self.plane_offset_v.get())
            seg_len = max(0.5, float(self.plane_seg_len.get()))
            stop = self.plane_gcode_stop_event
            # interpreter state
            abs_mode = True
            metric = True
            cur_u = 0.0
            cur_v = 0.0
            cur_w = 0.0
            cur_feed = float(self.plane_feed.get())
            move_count = 0
            total = len(lines)

            def _transform_uv(rx, ry):
                """Scale + mirror + rotate (no offset). None means axis not specified."""
                ff = 25.4 if not metric else 1.0
                u_r = rx * ff * scale_x * _mx if rx is not None else None
                v_r = ry * ff * scale_y * _my if ry is not None else None
                if u_r is not None and v_r is not None and _rot != "0°":
                    if _rot == "90°":    u_r, v_r = -v_r, u_r
                    elif _rot == "180°": u_r, v_r = -u_r, -v_r
                    elif _rot == "270°": u_r, v_r = v_r, -u_r
                return u_r, v_r

            def to_uv(rx, ry, rz):
                u_t, v_t = _transform_uv(rx, ry)
                ff = 25.4 if not metric else 1.0
                u = u_t + off_u if u_t is not None else None
                v = v_t + off_v if v_t is not None else None
                w = rz * ff * (scale_x + scale_y) * 0.5 if rz is not None else None
                return u, v, w

            def world_xyz(uu, vv, ww):
                x = origin[0] + uu * u_axis[0] + vv * v_axis[0] + ww * n_axis[0]
                y = origin[1] + uu * u_axis[1] + vv * v_axis[1] + ww * n_axis[1]
                z = origin[2] + uu * u_axis[2] + vv * v_axis[2] + ww * n_axis[2]
                return x, y, z

            def do_move(u1, v1, w1, feed):
                nonlocal cur_u, cur_v, cur_w, move_count
                if u1 is None: u1 = cur_u
                if v1 is None: v1 = cur_v
                if w1 is None: w1 = cur_w
                if not dry_run:
                    wx, wy, wz = world_xyz(u1, v1, w1)
                    ok = self.kinematics_tabs.move_tcp_pose(wx, wy, wz, roll, pitch, yaw, feed=feed)
                    if not ok:
                        return False
                move_count += 1
                cur_u, cur_v, cur_w = u1, v1, w1
                return True

            def do_arc(u1, v1, w1, ci, cj, direction, feed):
                nonlocal cur_u, cur_v, cur_w, move_count
                if u1 is None: u1 = cur_u
                if v1 is None: v1 = cur_v
                if w1 is None: w1 = cur_w
                cx = cur_u + ci
                cy = cur_v + cj
                r0 = math.hypot(cur_u - cx, cur_v - cy)
                r1 = math.hypot(u1 - cx, v1 - cy)
                if r0 < 1e-6:
                    _plane_log(f"G-code arc: radius too small, skipped.")
                    cur_u, cur_v, cur_w = u1, v1, w1
                    return True
                if abs(r0 - r1) > 0.5:
                    _plane_log(f"G-code arc: radius mismatch r0={r0:.3f} r1={r1:.3f}, skipped.")
                    cur_u, cur_v, cur_w = u1, v1, w1
                    return True
                start_a = math.atan2(cur_v - cy, cur_u - cx)
                end_a = math.atan2(v1 - cy, u1 - cx)
                if abs(u1 - cur_u) < 1e-6 and abs(v1 - cur_v) < 1e-6:
                    delta = -2 * math.pi if direction == "G2" else 2 * math.pi
                else:
                    delta = end_a - start_a
                    if direction == "G2":
                        if delta >= 0: delta -= 2 * math.pi
                    else:
                        if delta <= 0: delta += 2 * math.pi
                arc_len = abs(delta) * r0
                steps = max(3, int(math.ceil(arc_len / seg_len)))
                for i in range(1, steps + 1):
                    if stop.is_set():
                        return False
                    ang = start_a + delta * (i / steps)
                    uu = cx + r0 * math.cos(ang)
                    vv = cy + r0 * math.sin(ang)
                    if not dry_run:
                        wx, wy, wz = world_xyz(uu, vv, w1)
                        ok = self.kinematics_tabs.move_tcp_pose(wx, wy, wz, roll, pitch, yaw, feed=feed)
                        if not ok:
                            return False
                    move_count += 1
                cur_u, cur_v, cur_w = u1, v1, w1
                return True

            for lineno, raw in enumerate(lines, 1):
                if stop.is_set():
                    _plane_log(f"G-code stopped at line {lineno}.")
                    break
                parsed = _plane_parse_line(raw)
                if not parsed:
                    continue
                cmd = parsed["cmd"]
                p = parsed["params"]
                if lineno % 100 == 0 or lineno == total:
                    self.plane_gcode_progress.set(f"Line {lineno}/{total}  moves={move_count}")
                if cmd == "G20":
                    metric = False
                elif cmd == "G21":
                    metric = True
                elif cmd == "G90":
                    abs_mode = True
                elif cmd == "G91":
                    abs_mode = False
                elif cmd in ("G17", "G18", "G19", "G28", "G92"):
                    pass  # plane select / home / set position: ignored
                elif cmd in ("G0", "G1"):
                    if "F" in p:
                        cur_feed = p["F"] * (25.4 if not metric else 1.0)
                    u1, v1, w1 = to_uv(p.get("X"), p.get("Y"), p.get("Z"))
                    if not abs_mode:
                        if u1 is not None: u1 += cur_u
                        if v1 is not None: v1 += cur_v
                        if w1 is not None: w1 += cur_w
                    feed = cur_feed if cmd == "G1" else min(cur_feed, 6000.0)
                    if not do_move(u1, v1, w1, feed):
                        _plane_log(f"G-code aborted at line {lineno} (move failed).")
                        break
                elif cmd in ("G2", "G3"):
                    if "F" in p:
                        cur_feed = p["F"] * (25.4 if not metric else 1.0)
                    ci_t, cj_t = _transform_uv(p.get("I", 0.0), p.get("J", 0.0))
                    ci = ci_t if ci_t is not None else 0.0
                    cj = cj_t if cj_t is not None else 0.0
                    u1, v1, w1 = to_uv(p.get("X"), p.get("Y"), p.get("Z"))
                    if not abs_mode:
                        if u1 is not None: u1 += cur_u
                        if v1 is not None: v1 += cur_v
                    if not do_arc(u1, v1, w1, ci, cj, cmd, cur_feed):
                        _plane_log(f"G-code aborted at line {lineno} (arc failed).")
                        break
                # M, T, S, N, E: silently ignored
            self.plane_gcode_progress.set(f"Done — {move_count} moves / {total} lines")
            _plane_log(f"G-code {'dry-run' if dry_run else 'run'} complete: {move_count} moves.")

        def _plane_gcode_load():
            import tkinter.filedialog as fd
            path = fd.askopenfilename(
                title="Load G-code file",
                filetypes=[("G-code", "*.gcode *.nc *.gc *.txt"), ("All files", "*.*")],
            )
            if not path:
                return
            try:
                with open(path, "r", encoding="utf-8", errors="replace") as f:
                    self.plane_gcode_lines = [ln.rstrip("\n") for ln in f]
                self.plane_gcode_path.set(path)
                self.plane_gcode_status.set(f"{len(self.plane_gcode_lines)} lines loaded.")
                self.plane_gcode_progress.set("")
                _plane_log(f"G-code loaded: {path} ({len(self.plane_gcode_lines)} lines)")
            except Exception as e:
                _plane_log(f"G-code load failed: {e}")

        def _plane_gcode_run():
            if self.plane_gcode_worker and self.plane_gcode_worker.is_alive():
                _plane_log("G-code: already running.")
                return
            self.plane_gcode_stop_event.clear()
            self.plane_gcode_progress.set("Starting…")
            import threading as _t
            self.plane_gcode_worker = _t.Thread(
                target=_plane_exec_gcode,
                args=(bool(self.plane_dry_run.get()),),
                daemon=True,
            )
            self.plane_gcode_worker.start()

        def _plane_gcode_stop():
            self.plane_gcode_stop_event.set()
            _plane_log("G-code stop requested.")

        def _plane_gcode_preview():
            """Parse loaded G-code and draw the UV path on the preview canvas."""
            lines = self.plane_gcode_lines
            canvas = self.plane_preview_canvas
            if canvas is None:
                return
            if not lines:
                canvas.delete("all")
                canvas.create_text(150, 100, text="No file loaded.", fill="gray")
                return

            scale_x = max(1e-6, float(self.plane_scale_x.get()))
            scale_y = max(1e-6, float(self.plane_scale_y.get()))
            _mx = -1.0 if self.plane_mirror_x.get() else 1.0
            _my = -1.0 if self.plane_mirror_y.get() else 1.0
            _rot = self.plane_rotate.get()
            off_u = float(self.plane_offset_u.get())
            off_v = float(self.plane_offset_v.get())
            seg_len = max(0.5, float(self.plane_seg_len.get()))
            metric = True
            abs_mode = True
            cur_u = 0.0
            cur_v = 0.0

            def _txuv(rx, ry):
                ff = 25.4 if not metric else 1.0
                u_r = rx * ff * scale_x * _mx if rx is not None else None
                v_r = ry * ff * scale_y * _my if ry is not None else None
                if u_r is not None and v_r is not None and _rot != "0°":
                    if _rot == "90°":    u_r, v_r = -v_r, u_r
                    elif _rot == "180°": u_r, v_r = -u_r, -v_r
                    elif _rot == "270°": u_r, v_r = v_r, -u_r
                return u_r, v_r

            def tuv(rx, ry):
                u_t, v_t = _txuv(rx, ry)
                u = u_t + off_u if u_t is not None else None
                v = v_t + off_v if v_t is not None else None
                return u, v

            draw_segs = []  # (u0, v0, u1, v1, is_rapid)

            for raw in lines:
                parsed = _plane_parse_line(raw)
                if not parsed:
                    continue
                cmd = parsed["cmd"]
                p = parsed["params"]
                if cmd == "G20":   metric = False
                elif cmd == "G21": metric = True
                elif cmd == "G90": abs_mode = True
                elif cmd == "G91": abs_mode = False
                elif cmd in ("G0", "G1"):
                    u1, v1 = tuv(p.get("X"), p.get("Y"))
                    if not abs_mode:
                        if u1 is not None: u1 += cur_u
                        if v1 is not None: v1 += cur_v
                    if u1 is None: u1 = cur_u
                    if v1 is None: v1 = cur_v
                    draw_segs.append((cur_u, cur_v, u1, v1, cmd == "G0"))
                    cur_u, cur_v = u1, v1
                elif cmd in ("G2", "G3"):
                    ci_t, cj_t = _txuv(p.get("I", 0.0), p.get("J", 0.0))
                    ci = ci_t if ci_t is not None else 0.0
                    cj = cj_t if cj_t is not None else 0.0
                    u1, v1 = tuv(p.get("X"), p.get("Y"))
                    if not abs_mode:
                        if u1 is not None: u1 += cur_u
                        if v1 is not None: v1 += cur_v
                    if u1 is None: u1 = cur_u
                    if v1 is None: v1 = cur_v
                    cx = cur_u + ci
                    cy = cur_v + cj
                    r0 = math.hypot(cur_u - cx, cur_v - cy)
                    if r0 < 1e-6:
                        draw_segs.append((cur_u, cur_v, u1, v1, False))
                        cur_u, cur_v = u1, v1
                        continue
                    start_a = math.atan2(cur_v - cy, cur_u - cx)
                    end_a = math.atan2(v1 - cy, u1 - cx)
                    if abs(u1 - cur_u) < 1e-6 and abs(v1 - cur_v) < 1e-6:
                        delta = -2 * math.pi if cmd == "G2" else 2 * math.pi
                    else:
                        delta = end_a - start_a
                        if cmd == "G2":
                            if delta >= 0: delta -= 2 * math.pi
                        else:
                            if delta <= 0: delta += 2 * math.pi
                    steps = max(3, int(math.ceil(abs(delta) * r0 / seg_len)))
                    prev_u, prev_v = cur_u, cur_v
                    for i in range(1, steps + 1):
                        ang = start_a + delta * (i / steps)
                        uu = cx + r0 * math.cos(ang)
                        vv = cy + r0 * math.sin(ang)
                        draw_segs.append((prev_u, prev_v, uu, vv, False))
                        prev_u, prev_v = uu, vv
                    cur_u, cur_v = u1, v1

            if not draw_segs:
                canvas.delete("all")
                canvas.create_text(150, 100, text="No drawable moves found.", fill="gray")
                return

            all_u = [s[0] for s in draw_segs] + [s[2] for s in draw_segs]
            all_v = [s[1] for s in draw_segs] + [s[3] for s in draw_segs]
            min_u, max_u = min(all_u), max(all_u)
            min_v, max_v = min(all_v), max(all_v)
            cw = canvas.winfo_width() or 300
            ch = canvas.winfo_height() or 260
            pad = 14
            w = cw - 2 * pad
            h = ch - 2 * pad
            du = max_u - min_u or 1.0
            dv = max_v - min_v or 1.0
            sc = min(w / du, h / dv)

            def _cx(u): return pad + (u - min_u) * sc
            def _cy(v): return pad + h - (v - min_v) * sc  # flip Y (G-code Y up → canvas Y down)

            canvas.delete("all")
            canvas.create_rectangle(pad - 1, pad - 1, pad + w + 1, pad + h + 1, outline="#ddd")
            for u0, v0, u1, v1, rapid in draw_segs:
                x0, y0 = _cx(u0), _cy(v0)
                x1, y1 = _cx(u1), _cy(v1)
                if rapid:
                    canvas.create_line(x0, y0, x1, y1, fill="#c0c0c0", dash=(4, 3))
                else:
                    canvas.create_line(x0, y0, x1, y1, fill="#1a5fb4", width=1)
            # mark G-code origin
            ox, oy = _cx(off_u), _cy(off_v)
            canvas.create_oval(ox - 3, oy - 3, ox + 3, oy + 3, fill="#e01b24", outline="")
            canvas.create_text(
                cw // 2, ch - 4,
                text=f"UV extent  {du:.1f} × {dv:.1f} mm",
                fill="#555", font=("TkDefaultFont", 8),
            )

        def _plane_show_help():
            import tkinter.scrolledtext as _st
            top = tk.Toplevel()
            top.title("GCODE Tab – Help")
            top.geometry("620x520")
            txt = _st.ScrolledText(top, wrap=tk.WORD, font=("Consolas", 10))
            txt.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
            help_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "GCODE_help.txt")
            try:
                with open(help_path, "r", encoding="utf-8") as _f:
                    content = _f.read()
            except Exception as e:
                content = f"Help file not found.\nExpected: {help_path}\n\n{e}"
            txt.insert("1.0", content)
            txt.config(state=tk.DISABLED)

        # ---- end G-code interpreter ----

        plane_wrap = ttk.LabelFrame(tab_plane, text="Plane definition")
        plane_wrap.pack(fill=tk.X, expand=False, padx=4, pady=4)

        ttk.Label(
            plane_wrap,
            text="Define a UV plane from current TCP orientation. G2/G3 run in that plane.",
        ).pack(anchor="w", padx=4, pady=(2, 4))

        plane_btns = ttk.Frame(plane_wrap)
        plane_btns.pack(anchor="w", padx=4, pady=(0, 4))
        ttk.Button(
            plane_btns,
            text="Set plane from current TCP",
            command=_plane_set_from_current,
        ).pack(side=tk.LEFT)
        ttk.Button(
            plane_btns,
            text="Plane = world XY",
            command=_plane_set_world_xy,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            plane_btns,
            text="Normalize axes",
            command=_plane_sync_axes,
        ).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Button(
            plane_btns,
            text="Send plane frame",
            command=_plane_send_frame,
        ).pack(side=tk.LEFT, padx=(6, 0))

        plane_grid = ttk.Frame(plane_wrap)
        plane_grid.pack(anchor="w", padx=4, pady=(0, 4))

        def _plane_vec_row(row, label, vx, vy, vz):
            ttk.Label(plane_grid, text=label, width=8).grid(row=row, column=0, sticky="w")
            ttk.Entry(plane_grid, textvariable=vx, width=8, justify="right").grid(
                row=row, column=1, padx=2, pady=1
            )
            ttk.Entry(plane_grid, textvariable=vy, width=8, justify="right").grid(
                row=row, column=2, padx=2, pady=1
            )
            ttk.Entry(plane_grid, textvariable=vz, width=8, justify="right").grid(
                row=row, column=3, padx=2, pady=1
            )

        ttk.Label(plane_grid, text="X").grid(row=0, column=1)
        ttk.Label(plane_grid, text="Y").grid(row=0, column=2)
        ttk.Label(plane_grid, text="Z").grid(row=0, column=3)
        _plane_vec_row(1, "Origin", self.plane_origin_x, self.plane_origin_y, self.plane_origin_z)
        _plane_vec_row(2, "U axis", self.plane_u_x, self.plane_u_y, self.plane_u_z)
        _plane_vec_row(3, "V axis", self.plane_v_x, self.plane_v_y, self.plane_v_z)
        _plane_vec_row(4, "N axis", self.plane_n_x, self.plane_n_y, self.plane_n_z)

        rpy_row = ttk.Frame(plane_wrap)
        rpy_row.pack(anchor="w", padx=4, pady=(0, 4))
        ttk.Label(rpy_row, text="Tool RPY (deg)").pack(side=tk.LEFT)
        ttk.Entry(rpy_row, textvariable=self.plane_roll, width=8, justify="right").pack(
            side=tk.LEFT, padx=(6, 2)
        )
        ttk.Entry(rpy_row, textvariable=self.plane_pitch, width=8, justify="right").pack(
            side=tk.LEFT, padx=2
        )
        ttk.Entry(rpy_row, textvariable=self.plane_yaw, width=8, justify="right").pack(
            side=tk.LEFT, padx=2
        )

        plane_nb = ttk.Notebook(tab_plane)
        plane_nb.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        # ---- Tab: G-code file ----
        tab_gfile = ttk.Frame(plane_nb)
        plane_nb.add(tab_gfile, text="G-code file")

        file_row = ttk.Frame(tab_gfile)
        file_row.pack(fill=tk.X, padx=4, pady=(6, 2))
        ttk.Label(file_row, text="File:").pack(side=tk.LEFT)
        ttk.Entry(file_row, textvariable=self.plane_gcode_path, width=30).pack(
            side=tk.LEFT, padx=4, fill=tk.X, expand=True
        )
        ttk.Button(file_row, text="Browse…", command=_plane_gcode_load).pack(side=tk.LEFT)

        ttk.Label(tab_gfile, textvariable=self.plane_gcode_status, foreground="gray").pack(
            anchor="w", padx=6, pady=(0, 2)
        )

        # Horizontal split: controls (left) | preview canvas (right)
        main_frame = ttk.Frame(tab_gfile)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        left_frame = ttk.Frame(main_frame, width=268)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 4))
        left_frame.pack_propagate(False)

        right_frame = ttk.Frame(main_frame)
        right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- Transform controls ---
        tf = ttk.LabelFrame(left_frame, text="Transform")
        tf.pack(fill=tk.X, padx=0, pady=(0, 4))

        tf_r1 = ttk.Frame(tf)
        tf_r1.pack(anchor="w", padx=4, pady=(4, 1))
        ttk.Label(tf_r1, text="Scale X:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(tf_r1, textvariable=self.plane_scale_x, width=6, justify="right").pack(
            side=tk.LEFT, padx=(2, 10)
        )
        ttk.Label(tf_r1, text="Scale Y:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(tf_r1, textvariable=self.plane_scale_y, width=6, justify="right").pack(
            side=tk.LEFT, padx=(2, 0)
        )

        tf_r2 = ttk.Frame(tf)
        tf_r2.pack(anchor="w", padx=4, pady=1)
        ttk.Label(tf_r2, text="Origin U:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(tf_r2, textvariable=self.plane_offset_u, width=6, justify="right").pack(
            side=tk.LEFT, padx=(2, 10)
        )
        ttk.Label(tf_r2, text="Origin V:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(tf_r2, textvariable=self.plane_offset_v, width=6, justify="right").pack(
            side=tk.LEFT, padx=(2, 0)
        )

        tf_r3 = ttk.Frame(tf)
        tf_r3.pack(anchor="w", padx=4, pady=1)
        ttk.Checkbutton(tf_r3, text="Mirror X", variable=self.plane_mirror_x).pack(
            side=tk.LEFT, padx=(0, 10)
        )
        ttk.Checkbutton(tf_r3, text="Mirror Y", variable=self.plane_mirror_y).pack(
            side=tk.LEFT
        )

        tf_r4 = ttk.Frame(tf)
        tf_r4.pack(anchor="w", padx=4, pady=1)
        ttk.Label(tf_r4, text="Rotate:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Combobox(
            tf_r4, textvariable=self.plane_rotate,
            values=["0°", "90°", "180°", "270°"],
            width=6, state="readonly",
        ).pack(side=tk.LEFT, padx=(2, 0))

        tf_r5 = ttk.Frame(tf)
        tf_r5.pack(anchor="w", padx=4, pady=(1, 4))
        ttk.Label(tf_r5, text="SegLen:", width=8, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(tf_r5, textvariable=self.plane_seg_len, width=6, justify="right").pack(
            side=tk.LEFT, padx=(2, 10)
        )
        ttk.Label(tf_r5, text="Feed:", width=6, anchor="w").pack(side=tk.LEFT)
        ttk.Entry(tf_r5, textvariable=self.plane_feed, width=6, justify="right").pack(
            side=tk.LEFT, padx=(2, 0)
        )

        # Run controls
        run_row = ttk.Frame(left_frame)
        run_row.pack(anchor="w", padx=0, pady=(2, 2))
        ttk.Checkbutton(run_row, text="Dry run", variable=self.plane_dry_run).pack(
            side=tk.LEFT
        )
        ttk.Button(run_row, text="Run", command=_plane_gcode_run).pack(
            side=tk.LEFT, padx=(8, 4)
        )
        ttk.Button(run_row, text="Stop", command=_plane_gcode_stop).pack(side=tk.LEFT)

        ttk.Label(left_frame, textvariable=self.plane_gcode_progress, foreground="gray").pack(
            anchor="w", padx=0, pady=(0, 4)
        )

        btn_row = ttk.Frame(left_frame)
        btn_row.pack(anchor="w", padx=0, pady=2)
        ttk.Button(btn_row, text="Preview", command=_plane_gcode_preview).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(btn_row, text="Help", command=_plane_show_help).pack(side=tk.LEFT)

        # --- Preview canvas ---
        pv_canvas = tk.Canvas(right_frame, bg="white", cursor="crosshair", bd=1, relief=tk.SUNKEN)
        pv_canvas.pack(fill=tk.BOTH, expand=True)
        self.plane_preview_canvas = pv_canvas

        # ---- Tab: Single arc ----
        tab_arc = ttk.Frame(plane_nb)
        plane_nb.add(tab_arc, text="Single arc")

        arc_row0 = ttk.Frame(tab_arc)
        arc_row0.pack(anchor="w", padx=4, pady=(6, 0))
        ttk.Checkbutton(
            arc_row0,
            text="Absolute UV (G90)",
            variable=self.plane_abs_mode,
        ).pack(side=tk.LEFT)
        ttk.Radiobutton(
            arc_row0,
            text="G2 CW",
            variable=self.plane_dir,
            value="G2",
        ).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Radiobutton(
            arc_row0,
            text="G3 CCW",
            variable=self.plane_dir,
            value="G3",
        ).pack(side=tk.LEFT, padx=(4, 0))

        arc_row1 = ttk.Frame(tab_arc)
        arc_row1.pack(anchor="w", padx=4, pady=2)
        ttk.Label(arc_row1, text="End U").pack(side=tk.LEFT)
        ttk.Entry(arc_row1, textvariable=self.plane_end_u, width=8, justify="right").pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Label(arc_row1, text="End V").pack(side=tk.LEFT)
        ttk.Entry(arc_row1, textvariable=self.plane_end_v, width=8, justify="right").pack(
            side=tk.LEFT, padx=(4, 0)
        )

        arc_row2 = ttk.Frame(tab_arc)
        arc_row2.pack(anchor="w", padx=4, pady=2)
        ttk.Label(arc_row2, text="Center I").pack(side=tk.LEFT)
        ttk.Entry(arc_row2, textvariable=self.plane_center_i, width=8, justify="right").pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Label(arc_row2, text="Center J").pack(side=tk.LEFT)
        ttk.Entry(arc_row2, textvariable=self.plane_center_j, width=8, justify="right").pack(
            side=tk.LEFT, padx=(4, 0)
        )

        arc_row3 = ttk.Frame(tab_arc)
        arc_row3.pack(anchor="w", padx=4, pady=2)
        ttk.Label(arc_row3, text="W offset").pack(side=tk.LEFT)
        ttk.Entry(arc_row3, textvariable=self.plane_w, width=8, justify="right").pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Label(arc_row3, text="Feed").pack(side=tk.LEFT)
        ttk.Entry(arc_row3, textvariable=self.plane_feed, width=8, justify="right").pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Label(arc_row3, text="SegLen").pack(side=tk.LEFT)
        ttk.Entry(arc_row3, textvariable=self.plane_seg_len, width=8, justify="right").pack(
            side=tk.LEFT, padx=(4, 0)
        )

        arc_row4 = ttk.Frame(tab_arc)
        arc_row4.pack(anchor="w", padx=4, pady=(4, 2))
        ttk.Button(arc_row4, text="Execute arc", command=_plane_arc_send).pack(side=tk.LEFT)
        ttk.Button(arc_row4, text="Queue arc", command=_plane_arc_queue).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(arc_row4, text="Queue native G2/G3", command=_plane_arc_queue_native).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(arc_row4, text="Queue 3DP G17", command=_plane_queue_3dp_example_g17).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(arc_row4, text="Queue 3DP G18", command=_plane_queue_3dp_example_g18).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(arc_row4, text="Queue 3DP G19", command=_plane_queue_3dp_example_g19).pack(
            side=tk.LEFT, padx=(6, 0)
        )
        ttk.Button(arc_row4, text="Queue ALL 3DP", command=_plane_queue_3dp_examples_all).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        # Tab: Commands
        tab_commands = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_commands, text="Commands")



        # Tab: Simulation / Pose
        tab_pose = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_pose, text="Simulation / Pose")

        # =====================================================
        #  RoboSim visualizer integration (moved from top tab)
        # =====================================================
        try:
            import psutil
        except Exception:
            psutil = None

        def _get_base_dir():
            """Resolve base directory for both Python and packaged EXE."""
            if getattr(sys, 'frozen', False):  # running as .exe (PyInstaller)
                return os.path.dirname(sys.executable)
            return os.path.dirname(os.path.abspath(__file__))

        BASE_DIR = _get_base_dir()
        VISUALIZER_SCRIPT = os.path.join(BASE_DIR, "robosim_visualizer_v90.py")
        VISUALIZER_EXE    = os.path.join(BASE_DIR, "robosim_visualizer_v90.exe")
        VISUALIZER_TAG    = "RoboSim Control Center"

        def _log_visualizer(msg: str):
            try:
                if hasattr(self, "log"):
                    self.log(msg)
                    return
            except Exception:
                pass
            print(msg)

        def _find_visualizer_process():
            """Suche laufende RoboSim-Prozesse, um Doppelstart zu verhindern."""
            if psutil is None:
                _log_visualizer("1  psutil missing; process check disabled.")
                return None
            try:
                for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
                    cmdline = " ".join(p.info.get("cmdline") or [])
                    if (
                        "robosim_visualizer_v90" in cmdline
                        or VISUALIZER_TAG in cmdline
                        or "RoboSim" in (p.info.get("name") or "")
                    ):
                        return p
            except Exception:
                pass
            return None

        def start_visualizer_window():
            """Start the RoboSim visualizer as a separate process (single instance)."""
            existing = _find_visualizer_process()
            if existing:
                _log_visualizer(f"1  Visualizer is already running (PID={existing.pid}) - no new start.")
                return

            try:
                # If EXE exists (e.g., in dist folder), prefer it
                if os.path.exists(VISUALIZER_EXE):
                    _log_visualizer(f" Starting visualizer EXE: {VISUALIZER_EXE}")
                    subprocess.Popen([VISUALIZER_EXE], cwd=BASE_DIR)
                elif os.path.exists(VISUALIZER_SCRIPT):
                    if getattr(sys, "frozen", False):
                        _log_visualizer(" Visualizer EXE missing; frozen build cannot start .py script.")
                        return
                    _log_visualizer(f" Starting visualizer PY: {VISUALIZER_SCRIPT}")
                    log_path = os.path.join(BASE_DIR, "robosim_visualizer_start.log")
                    try:
                        log_fp = open(log_path, "a", encoding="utf-8")
                    except Exception:
                        log_fp = None
                    subprocess.Popen(
                        [sys.executable, VISUALIZER_SCRIPT],
                        cwd=BASE_DIR,
                        stdout=log_fp or subprocess.DEVNULL,
                        stderr=log_fp or subprocess.STDOUT,
                    )
                    if log_fp:
                        log_fp.close()
                    _log_visualizer(f"1  Visualizer startup log: {log_path}")
                else:
                    _log_visualizer(f" No visualizer found in {BASE_DIR}")
                    return

                time.sleep(0.5)
                _log_visualizer("... Visualizer started successfully.")
                try:
                    self.after(700, self._send_vis_robot_profile)
                except Exception:
                    pass
            except Exception as e:
                _log_visualizer(f" Visualizer start failed: {e}")

        # ---- Add UI into tab ----
        ttk.Label(tab_pose, text="RoboSim Visualizer", font=("Segoe UI", 12, "bold")).pack(pady=10)
        ttk.Button(
            tab_pose,
            text="Open Simulation (external process)",
            command=start_visualizer_window
        ).pack(pady=20)


        # =========================
        #   Command reference (moved from top tab bar)
        # =========================
        frame_text = ttk.Frame(tab_commands)
        frame_text.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        # Scrollbarer Textbereich
        text_commands = tk.Text(frame_text, wrap="word", height=6)
        text_commands.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(frame_text, orient="vertical", command=text_commands.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text_commands.config(yscrollcommand=scrollbar.set)

        # ---- Content (copied from top tab) ----
        command_reference = """\
  FluidNC / G-code command reference
====================================

 Machine control
-----------------
$X                  - Unlock after alarm
$H                  - Home all axes
$HX / $HY / $HZ     - Home specific axis
$$                  - Show current parameters (settings)
$G                  - Show active G-code modes
$I                  - Firmware/system information
!                   - Feed hold (pause)
~                   - Resume
Ctrl+X ()          - Software reset / emergency stop

TM  Motion commands
-----------------
G90                 - Absolute coordinates
G91                 - Relative coordinates
G92 X0 Y0 Z0 A0 B0  - Set current position as zero
G0 X10              - Rapid move to X10
G1 X10 F500         - Move to X10 with feedrate 500
G4 P2               - Dwell for 2 seconds

 Axis and limit configuration
------------------------------
$130..$134          - Max travel for X..B (soft limits)
$120..$124          - Accelerations (mm/s2)
$110..$114          - Max feedrate per axis (mm/min)
$20 / $21           - Enable soft/hard limits
$N / $N+            - Configure startup commands

  Tool / gripper / spindle
---------------------------
M3 S0               - Open gripper / spindle on (S=0)
M3 S1000            - Close gripper / full speed
M4                  - Spindle counter-clockwise
M5                  - Stop spindle

 Diagnostics and status
------------------------
(TM)                   - Current status (MPos, endstops)
$#                  - Show coordinate systems
$Help               - Help / available commands
$Startup/Show       - Show startup file
$Erase/All          - Erase all saved settings
$CD / $Dir          - Browse file system (older versions)
$PrintConfig        - Show currently loaded YAML

34 Misc
-------
Ctrl-X              - Restart / reset
$Report/State       - Print current machine status
$Report/Startup     - Show startup file loaded at boot
"""

        text_commands.insert("1.0", command_reference)
        text_commands.configure(state="disabled")  # Nur Lesen

        
        # ============================================
        #  Content for "Manual Axis Control" tab
        # ============================================
        posf = ttk.LabelFrame(tab_manual, text="Position / Manual Axis Control")
        posf.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)


        # ======================================================
        # HEADER-ZEILE: LiveMove | Absolute | Poll | ManualSpeed
        # ======================================================
        header = ttk.Frame(posf)
        header.pack(fill=tk.X, pady=(2, 4))

        # --- Live Move default ON ---
        self.live_move = tk.BooleanVar(value=True)
        ttk.Checkbutton(header, text="Live Move (on release)",
                        variable=self.live_move).pack(side=tk.LEFT, padx=(4, 10))

        # --- Absolute (G90) ---
        ttk.Checkbutton(header, text="Absolute (G90)",
                        variable=self.mode_absolute).pack(side=tk.LEFT, padx=(0, 10))

        # --- Poll Positions ---
        ttk.Checkbutton(header, text="Poll Positions ((TM))",
                        variable=self.poll_positions).pack(side=tk.LEFT, padx=(0, 25))
        ttk.Button(header, text="Zero (G92)", command=self.manual_zero_current_pose).pack(side=tk.RIGHT, padx=(0, 6))


        # ======================================================
        # MANUAL SPEED FACTOR  in der gleichen Zeile
        # ======================================================
        style = ttk.Style()

        # ---- Style for normal buttons ----
        style.configure(
            "SpeedButton.TButton",
            padding=(6, 3),          # Innenpadding (links/rechts, oben/unten)
            font=("Segoe UI", 9),    # font size
        )

        # ---- Style for selected button ----
        style.configure(
            "SpeedButtonSelected.TButton",
            padding=(6, 3),
            font=("Segoe UI", 9, "bold"),
            background="#333333",
            foreground="white"
        )

        # ---- Force width/height in pixels ----
        style.layout("SpeedButton.TButton", [
            ('Button.border', {'sticky': 'nswe', 'children': [
                ('Button.padding', {'sticky': 'nswe', 'children': [
                    ('Button.label', {'sticky': 'nswe'})
                ]})
            ]})
        ])

        style.layout("SpeedButtonSelected.TButton", style.layout("SpeedButton.TButton"))


        # Label
        ttk.Label(header, text="Manual Speed:", width=12).pack(side=tk.LEFT, padx=(4, 6))

        self.manual_speed_factor = tk.DoubleVar(value=1.0)
        factor_buttons = {}

        def _set_manual_factor(val):
            self.manual_speed_factor.set(val)

            # --- Set button colors ---
            for denom, btn in factor_buttons.items():
                # float-safe comparison
                if abs(val - 1/denom) < 1e-9:
                    btn.configure(style="SpeedButtonSelected.TButton")
                else:
                    btn.configure(style="SpeedButton.TButton")

            # Log only if logger exists
            if hasattr(self, "txt_log"):
                self.log(f"Manual Speed Factor: 1/{int(1/val)}")


        # ---- Buttons erzeugen ----
        for denom in (1, 5, 10, 20, 50, 100):
            b = ttk.Button(
                header,
                text=f"1/{denom}",
                style="SpeedButton.TButton",
                command=lambda d=denom: _set_manual_factor(1/d)
            )

            # ---- Size & spacing ----
            b.pack(side=tk.LEFT, padx=4, pady=2)
            b.configure(width=6)

            factor_buttons[denom] = b


        # ---- DEFAULT: 1/20 ----
        _set_manual_factor(1/20)



        # ======================================================
        # MANUAL AXIS CONTROL  Buttons oben, Slider unten
        # ======================================================
        self.axis_labels = {}
        self.axis_vars = {}
        self.axis_entries = {}
        self.axis_endstop_icons = {}
        self.axis_limit_labels = {}
        self.axis_scales = {}
        self.axis_scale_redraw = {}


        def clamp_with_limits(ax, v):
            try:
                val = float(v)
            except:
                val = 0.0
            lo, hi = self._effective_axis_limits(ax)
            return max(lo, min(hi, val))


        # ======================================================
        # Hilfsfunktion: Nach jeder Bewegung ALLE Felder aktualisieren
        # ======================================================
        def update_all_positions():
            """Update all axes using the latest status data."""
            st = getattr(self.client, "last_status", None)
            if not st or "MPos" not in st:
                return
            for ax, pos in st["MPos"].items():
                if ax not in AXES:
                    continue
                self.axis_positions[ax] = pos
                self.axis_vars[ax].set(pos)
                if ax in self.axis_entries:
                    e = self.axis_entries[ax]
                    e.delete(0, tk.END)
                    e.insert(0, f"{pos:.3f}")
                if ax in self.axis_labels:
                    self.axis_labels[ax].config(text=f"{pos:.3f}")
        self.update_all_positions = update_all_positions



        # ======================================================
        # 1) BUTTONBLOCK (JE 6 REIHEN)
        # ======================================================
        for ax in AXES:

            row = ttk.Frame(posf)
            row.pack(fill=tk.X, pady=1)

            # Axis label
            ttk.Label(row, text=ax, width=3).pack(side=tk.LEFT)

            # mpos-Anzeige
            lbl = ttk.Label(row, text="0.000", width=10, anchor="e")
            lbl.pack(side=tk.LEFT, padx=5)
            self.axis_labels[ax] = lbl

            # Endstop-Kreis
            c = tk.Canvas(row, width=14, height=14, highlightthickness=0)
            oval = c.create_oval(2, 2, 12, 12, fill="#999999")
            c.pack(side=tk.LEFT, padx=5)
            self.axis_endstop_icons[ax] = (c, oval, "unknown")


            # ---------------------------------------------------
            # Prozent-Buttons (Null statt "0%")
            # ---------------------------------------------------
            def send_percent(ax=ax, pct=0):
                lo, hi = self._manual_axis_limits(ax)
                target = lo + (hi - lo) * (pct + 100) / 200.0
                target = clamp_with_limits(ax, target)

                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G90")
                self.client.send_line(f"G1 {ax}{target:.3f} F{F:.0f}")

                self.log(f"{ax}  {target:.3f} ({pct:+d}%) @F={F}")
                self.update_all_positions()

            pct_frame = ttk.Frame(row, relief="solid", borderwidth=1)
            pct_frame.pack(side=tk.LEFT, padx=4)

            for pct in (-100, -50, 0, 50, 100):
                btn_text = "0" if pct == 0 else f"{pct:+d}%"
                b = ttk.Button(
                    pct_frame, text=btn_text,
                    width=6, style="FlatMini.TButton",
                    command=lambda ax=ax, pct=pct: send_percent(ax, pct)
                )
                if pct == 0:
                    b.configure(style="FlatMiniSelected.TButton")
                b.pack(side=tk.LEFT, padx=1)


            # ---------------------------------------------------
            # Jog Buttons (relativ)
            # ---------------------------------------------------
            jog_frame = ttk.Frame(row, relief="solid", borderwidth=1)
            jog_frame.pack(side=tk.LEFT, padx=4)

            neg = [10, 5, 1, 0.5, 0.1]
            pos = [0.1, 0.5, 1, 5, 10]

            def jog(ax=ax, dist=0):
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G91")
                self.client.send_line(f"G1 {ax}{dist:.3f} F{F:.0f}")
                self.client.send_line("G90")

                self.log(f"{ax} jog {dist:+} @F={F}")
                self.update_all_positions()

            for step in neg:
                ttk.Button(
                    jog_frame, text=f"{-step:g}",
                    width=4, style="FlatMini.TButton",
                    command=lambda ax=ax, step=step: jog(ax, -step)
                ).pack(side=tk.LEFT, padx=1)

            for step in pos:
                ttk.Button(
                    jog_frame, text=f"+{step:g}",
                    width=4, style="FlatMini.TButton",
                    command=lambda ax=ax, step=step: jog(ax, +step)
                ).pack(side=tk.LEFT, padx=1)

        # ======================================================
        # 2) SLIDERBLOCK
        # ======================================================
        for ax in AXES:

            slider_row = ttk.Frame(posf)
            slider_row.pack(fill=tk.X, pady=(3, 3))

            ttk.Label(slider_row, text=ax, width=9).pack(side=tk.LEFT)

            lo, hi = self._manual_axis_limits(ax)
            var = tk.DoubleVar(value=0.0)
            self.axis_vars[ax] = var

            # --- Canvas + Null-Linie ---
            slider_wrap = tk.Frame(slider_row)
            slider_wrap.pack(side=tk.LEFT, padx=4)

            canvas = tk.Canvas(slider_wrap, width=600, height=22, highlightthickness=0)
            canvas.pack()

            # ---- Axis scale in canvas ----
            scale_width = 600
            scale_height = 22

            # Mapping function (value -> pixel)
            def val_to_x(v):
                return (v - lo) / (hi - lo) * scale_width

            # Null-Linie
            zero_x = val_to_x(0)
            canvas.create_line(zero_x, 0, zero_x, scale_height, fill="black", width=2)

            # major ticks: always at 10-step intervals
            for v in range(int(lo - lo % 10), int(hi) + 1, 10):
                x = val_to_x(v)
                canvas.create_line(x, scale_height - 14, x, scale_height, fill="black", width=2)

            def _redraw_axis_scale(lo_val, hi_val, canvas=canvas, width=scale_width, height=scale_height):
                canvas.delete("all")
                span = hi_val - lo_val
                if abs(span) < 1e-9:
                    span = 1.0

                def val_to_x(v):
                    return (v - lo_val) / span * width

                zero_x = val_to_x(0.0)
                canvas.create_line(zero_x, 0, zero_x, height, fill="black", width=2)

                start = int(lo_val - (lo_val % 10))
                end = int(hi_val) + 1
                for v in range(start, end, 10):
                    x = val_to_x(v)
                    canvas.create_line(x, height - 14, x, height, fill="black", width=2)

            _redraw_axis_scale(lo, hi)

            # --- Slider ---
            s = ttk.Scale(canvas, from_=lo, to=hi, variable=var,
                          orient=tk.HORIZONTAL, length=600)
            s.place(x=0, y=0)
            self.axis_scales[ax] = s
            self.axis_scale_redraw[ax] = _redraw_axis_scale

            # --- Entry ---
            ent = ttk.Entry(slider_row, width=7)
            ent.insert(0, "0.0")
            ent.pack(side=tk.LEFT, padx=4)
            self.axis_entries[ax] = ent
            self.axis_entry_order.append(ent)

            # -----------------------
            # Wert lesen + clamp
            # -----------------------
            def _get(ax=ax, ent=ent, var=var):
                try:
                    v = float(ent.get().replace(",", "."))
                except:
                    v = var.get()
                v = clamp_with_limits(ax, v)
                ent.delete(0, tk.END)
                ent.insert(0, f"{v:.3f}")
                return v

            # -----------------------
            # Slider->Entry Sync (HAT bisher gefehlt)
            # -----------------------
            def on_var_write(*_ignored, ax=ax, ent=ent, var=var):
                # do not overwrite during active input
                if getattr(self, "_user_editing", {}).get(ax, False):
                    return
                ent.delete(0, tk.END)
                ent.insert(0, f"{var.get():.3f}")

            var.trace_add("write", on_var_write)

            # --- ENTER sendet sofort ---
            def _on_enter(event=None, ax=ax, ent=ent, var=var):
                val = _get()
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G90")
                self.client.send_line(f"G1 {ax}{val:.3f} F{F:.0f}")
                self.log(f"{ax}  {val:.3f} @F={F}")
                # nach Bewegung aktualisieren
                self.update_position_display()
                return "break"
            ent.bind("<Return>", _on_enter)

            # --- TAB Navigation ---
            def _focus_and_select(w):
                w.focus_set()
                w.after(1, lambda: w.selection_range(0, tk.END))

            ent.bind("<FocusIn>", lambda e, ent=ent: ent.after(1, lambda: ent.selection_range(0, tk.END)))
            ent.bind("<Tab>", lambda e, ent=ent: (_focus_and_select(
                self.axis_entry_order[(self.axis_entry_order.index(ent)+1) % len(self.axis_entry_order)]
            ), "break")[1])
            ent.bind("<Shift-Tab>", lambda e, ent=ent: (_focus_and_select(
                self.axis_entry_order[(self.axis_entry_order.index(ent)-1) % len(self.axis_entry_order)]
            ), "break")[1])
            ent.bind("<ISO_Left_Tab>", lambda e, ent=ent: (_focus_and_select(
                self.axis_entry_order[(self.axis_entry_order.index(ent)-1) % len(self.axis_entry_order)]
            ), "break")[1])

            # --- Buttons ToQ / ToCLI / Send ---
            btn_block = ttk.Frame(slider_row)
            btn_block.pack(side=tk.LEFT, padx=2)

            def _btn_toQ(ax=ax, ent=ent, var=var):
                v = _get()
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.enqueue(f"G1 {ax}{v:.3f} F{F:.0f}")
                self.update_position_display()

            def _btn_toCLI(ax=ax, ent=ent, var=var):
                v = _get()
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.entry_cmd.delete(0, tk.END)
                self.entry_cmd.insert(0, f"G1 {ax}{v:.3f} F{F:.0f}")

            def _btn_send(ax=ax, ent=ent, var=var):
                v = _get()
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G90")
                self.client.send_line(f"G1 {ax}{v:.3f} F{F:.0f}")
                self.log(f"{ax}  {v:.3f} @F={F}")
                self.update_position_display()

            # --- Neue Funktion: komplette Pose  Queue ---
            def _btn_pose_toQ(ax=ax):
                # Collect all axis values
                parts = []
                for ax2 in AXES:
                    ent2 = self.axis_entries.get(ax2)
                    if not ent2:
                        continue
                    try:
                        v = float(ent2.get().replace(",", "."))
                    except:
                        v = self.axis_vars[ax2].get()

                    lo2, hi2 = self._effective_axis_limits(ax2)
                    v = max(lo2, min(hi2, v))  # clamp

                    parts.append(f"{ax2}{v:.3f}")

                # Feedrate
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()

                # Kompletten G1 erzeugen
                gline = "G1 " + " ".join(parts) + f" F{F:.0f}"

                # In Queue schieben
                self.enqueue(gline)
                self.log(f"Pose  Queue: {gline}")


            ttk.Button(btn_block, text="ToQ",   width=4, style="FlatMini.TButton", command=_btn_toQ).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_block, text="ToCLI", width=5, style="FlatMini.TButton", command=_btn_toCLI).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_block, text="Send",  width=5, style="FlatMini.TButton", command=_btn_send).pack(side=tk.LEFT, padx=1)



            # --- Slider LiveMove ---
            def _press(e, ax=ax):
                self._user_editing[ax] = True
                self._start_val = self.axis_vars[ax].get()

            def _release(e, ax=ax, ent=ent):
                self._user_editing[ax] = False
                val = self.axis_vars[ax].get()

                if not self.live_move.get():
                    return
                if abs(val - getattr(self, "_start_val", val)) < MOTION_EPS:
                    return

                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G90")
                self.client.send_line(f"G1 {ax}{val:.3f} F{F:.0f}")
                self.log(f"{ax} slide  {val:.3f} @F={F}")
                self.update_position_display()

            s.bind("<ButtonPress-1>", _press)
            s.bind("<ButtonRelease-1>", _release)

        # =====================================================
        # TM  SPEED control with 3 MaxSpeed modes (Low/Mid/High)
        # =====================================================
        spdf = ttk.LabelFrame(wrap, text="Speed", width=240, height=220)
        spdf.pack(side=tk.LEFT, fill=tk.Y, padx=2, pady=2)
        spdf.pack_propagate(False)

        # --- interne Werte ---
        self.speed_val = tk.DoubleVar(value=500)
        self.max_feed = tk.DoubleVar(value=15000.0)
        self.speed_mode = tk.StringVar(value="Mid")

        # --- Header with title + current value ---
        hdr = ttk.Frame(spdf)
        hdr.pack(fill="x", pady=(4, 2))
        ttk.Label(hdr, text="Speed:", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=(8, 2))
        self.lbl_speed_val = ttk.Label(hdr, text=f"{self.speed_val.get():.1f}", width=6, anchor="e")
        self.lbl_speed_val.pack(side=tk.LEFT, padx=(2, 8))

        # --- MaxSpeed-Mode Buttons (Low / Mid / High) ---
        mode_frame = ttk.Frame(spdf)
        mode_frame.pack(pady=(0, 4))
        style = ttk.Style()
        style.configure("SpeedMode.TButton", font=("Segoe UI", 8))
        def _set_mode(mode):
            modes = {"Low": 6000.0, "Mid": 15000.0, "High": 50000.0}
            self.speed_mode.set(mode)
            self.max_feed.set(modes[mode])
            self.log(f"Max-Speed-Mode: {mode} ({modes[mode]} mm/min)")
        for m in ("Low", "Mid", "High"):
            ttk.Button(mode_frame, text=m, width=6, style="SpeedMode.TButton",
                       command=lambda mm=m: _set_mode(mm)).pack(side=tk.LEFT, padx=2)

        # --- Speed-Slider (01000) ---
        speed_slider = ttk.Scale(
            spdf, from_=0, to=1000, variable=self.speed_val,
            orient=tk.HORIZONTAL, length=220
        )
        speed_slider.pack(padx=8, pady=(0, 6))

        # --- Prozent-Buttons ---
        pct_frame = ttk.Frame(spdf)
        pct_frame.pack(pady=(0, 0))
        style.configure("SpeedPct.TButton", font=("Segoe UI", 8))
        for pct in (0, 20, 40, 50, 60, 80, 100):
            ttk.Button(
                pct_frame,
                text=f"{pct}%",
                width=3,
                style="SpeedPct.TButton",
                command=lambda p=pct: self._set_speed_percent(p)
            ).pack(side=tk.LEFT, padx=0)

        # --- Label-Update ---
        def _update_speed_label(*_):
            self.lbl_speed_val.config(text=f"{self.speed_val.get():.1f}")
        self.speed_val.trace_add("write", _update_speed_label)

        # --- Speed-Mapping & interne Funktionen ---
        def map_speed(val: float) -> float:
            return (val / 1000.0) * self.max_feed.get()

        def _set_speed_percent_local(pct: int):
            val = max(0, min(1000, int(10 * pct)))
            self.speed_val.set(val)
            mm = map_speed(val)
            self.lbl_speed_val.config(text=f"{self.speed_val.get():.1f}")
            self.log(f"Speed set to {pct}% ({val}/1000 -> {mm:.0f} mm/min @ {self.speed_mode.get()})")

        self._set_speed_percent = _set_speed_percent_local

        # =====================================================
        #   HOMING / CONTROL  34 Matrix IM Speed/Accel-Frame
        # =====================================================
        style.configure("HomingCtrl.TButton", padding=(6, 1), font=("Segoe UI", 9))
        ctrl = ttk.LabelFrame(spdf, text="Homing / Control")
        ctrl.pack(fill=tk.X, padx=6, pady=(8, 4))
        btns = [
            ("Home $H", lambda: self.send_now("$H"), "homing_global"),
            ("Home $HX", lambda: self.send_now("$HX"), "homing_axis"),
            ("Home $HY", lambda: self.send_now("$HY"), "homing_axis"),
            ("Home $HZ", lambda: self.send_now("$HZ"), "homing_axis"),
            ("Home $HA", lambda: self.send_now("$HA"), "homing_axis"),
            ("Home $HB", lambda: self.send_now("$HB"), "homing_axis"),
            ("Home $HC", lambda: self.send_now("$HC"), "homing_axis"),
            ("Unlock $X", lambda: self.send_now("$X"), "other"),
            ("Zero", self.manual_zero_current_pose, "other"),
            ("Go Home", self.goto_home, "other"),
            ("Home->Q", self.enqueue_home, "other"),
            ("Gripper Open", lambda: self.send_now("M3 S0"), "other"),
            ("Gripper Close", lambda: self.send_now("M3 S1000"), "other"),
            ("Pose 1", lambda: self.send_now("G1 X45 Y90 Z45 A0 B0 C0 F6000"), "other"),
            ("Pose 2", lambda: self.send_now("G1 X45 Y90 Z-45 A0 B0 C0 F6000"), "other"),
        ]
        self._homing_global_buttons = []
        self._homing_axis_buttons = []
        # Buttons im Grid (3 Spalten x 4 Zeilen)
        for i, (txt, cmd, kind) in enumerate(btns):
            r, c = divmod(i, 3)
            btn = ttk.Button(ctrl, text=txt, width=10, command=cmd, style="HomingCtrl.TButton")
            btn.grid(row=r, column=c, padx=2, pady=1, sticky="ew")
            if kind == "homing_global":
                self._homing_global_buttons.append(btn)
            elif kind == "homing_axis":
                self._homing_axis_buttons.append(btn)

        for c in range(3):
            ctrl.columnconfigure(c, weight=1)

        # =====================================================
        #  Farbiger STATUS-BLOCK direkt unter Tilt
        # (own StringVar, do not overwrite status_var)
        # =====================================================
        self.status_block_var = tk.StringVar(value="Status: -")
        self.lbl_status_block = tk.Label(
            spdf,
            textvariable=self.status_block_var,
            font=("Segoe UI", 10, "bold"),
            width=22,
            height=2,
            relief="groove",
            bg="#777777",
            fg="white",
            anchor="w",
            padx=8
        )
        self.lbl_status_block.pack(fill="x", padx=8, pady=(4, 8))

        # =====================================================
        # 2 Endstop-Anzeige  kompakt unter Status
        # =====================================================
        endstop_frame = ttk.Frame(spdf)
        endstop_frame.pack(fill="x", padx=8, pady=(0, 4))

        self.endstop_indicators = {}

        for ax in AXES:
            col = ttk.Frame(endstop_frame)
            col.pack(side=tk.LEFT, expand=True, padx=3)

            # Kreis (Canvas)
            c = tk.Canvas(col, width=20, height=20, highlightthickness=0)
            oval = c.create_oval(3, 3, 17, 17, fill="#999999", outline="")
            c.pack()
            # Label unter Kreis
            ttk.Label(col, text=ax, font=("Segoe UI", 8)).pack(pady=(0, 0))

            self.endstop_indicators[ax] = (c, oval, "unknown")

        # =====================================================
        #  TCP Pose  zentrale 6DOF FK Anzeige
        # =====================================================

        self.tcp_panel = tcp_pose_module.TcpPosePanel(
            master=spdf,
            client=self.client,
            geom_dh=tcp_pose_module.GEOM_DH,
            title="",
            poll_interval_ms=150
        )
        self.tcp_panel.pack(
            fill="x",
            padx=8,
            pady=(6, 10)
        )
        style.configure("TcpQueue.TButton", padding=(6, 1), font=("Segoe UI", 9))
        style.configure("TcpQueueSmall.TButton", padding=(4, 1), font=("Segoe UI", 8))
        ttk.Button(
            spdf,
            text="Pose to Queue",
            command=self.add_current_pose_to_queue,
            style="TcpQueue.TButton",
        ).pack(fill="x", padx=8, pady=(0, 4))
        def _queue_kinematics_sequence():
            if hasattr(self, "kinematics_tabs"):
                for attr in ("queue_sequence", "_send_preview_to_queue"):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()
            if hasattr(self, "log"):
                self.log(" No kinematics queue function found.")
        ttk.Button(
            spdf,
            text="Sequence to Queue",
            command=_queue_kinematics_sequence,
            style="TcpQueue.TButton",
        ).pack(fill="x", padx=8, pady=(0, 4))
        def _queue_grip_open():
            self.enqueue("M3 S0")

        def _queue_grip_close():
            self.enqueue("M3 S1000")

        grip_row = ttk.Frame(spdf)
        grip_row.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Button(
            grip_row,
            text="Gripper Open\nTo Queue",
            command=_queue_grip_open,
            style="TcpQueueSmall.TButton",
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 2))
        ttk.Button(
            grip_row,
            text="Gripper Close\nTo Queue",
            command=_queue_grip_close,
            style="TcpQueueSmall.TButton",
        ).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2, 0))
        def _on_tcp_pose_changed(pose):
            if not getattr(self, "fixed_tcp_sync_pending", False):
                return
            if getattr(self, "fixed_tcp_user_dragging", False):
                return
            try:
                ox = float(self.fixed_tcp_origin_x)
                oy = float(self.fixed_tcp_origin_y)
                oz = float(self.fixed_tcp_origin_z)
                dxw = float(pose.get("X_mm", 0.0)) - ox
                dyw = float(pose.get("Y_mm", 0.0)) - oy
                dzw = float(pose.get("Z_mm", 0.0)) - oz
            except Exception:
                return
            if hasattr(self, "_fixed_tcp_world_to_offsets"):
                try:
                    dx, dy, dz = self._fixed_tcp_world_to_offsets(dxw, dyw, dzw, pose)
                except Exception:
                    dx, dy, dz = dxw, dyw, dzw
            else:
                dx, dy, dz = dxw, dyw, dzw
            if hasattr(self, "_fixed_tcp_set_offsets"):
                self._fixed_tcp_set_offsets(dx, dy, dz)
            else:
                self.fixed_tcp_dx.set(dx)
                self.fixed_tcp_dy.set(dy)
                self.fixed_tcp_dz.set(dz)
            last = self.fixed_tcp_last_delta
            if last is not None:
                ddx = dx - last[0]
                ddy = dy - last[1]
                ddz = dz - last[2]
                if (ddx * ddx + ddy * ddy + ddz * ddz) ** 0.5 < 0.05:
                    self.fixed_tcp_stable_count += 1
                else:
                    self.fixed_tcp_stable_count = 0
            self.fixed_tcp_last_delta = (dx, dy, dz)
            target = self.fixed_tcp_target_delta
            if target is not None:
                tdx = dx - target[0]
                tdy = dy - target[1]
                tdz = dz - target[2]
                if (tdx * tdx + tdy * tdy + tdz * tdz) ** 0.5 < 0.5:
                    self.fixed_tcp_sync_pending = False
                    return
            if self.fixed_tcp_stable_count >= 2:
                self.fixed_tcp_sync_pending = False
        try:
            self.tcp_panel.on_pose_changed(_on_tcp_pose_changed)
        except Exception:
            pass

        # Enable panel (start update loop)
        try:
            self.tcp_panel.start()
        except:
            pass

        # ---- Set All Buttons ----
        allf = ttk.Frame(posf)
        allf.pack(pady=6)
        # ---- Vision rechts (rechts neben Speed) ----
        self.vision_right_frame = ttk.LabelFrame(wrap, text="Vision rechts")
        self.vision_right_label = ttk.Label(self.vision_right_frame)
        self.vision_right_label.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self._toggle_right_vision()
        self.after(80, self._tick_right_vision)
        ttk.Button(allf, text="To Queue", command=_btn_pose_toQ).pack(side=tk.LEFT, padx=2)
        ttk.Button(allf, text="Set All Now", command=self.set_all_now).pack(side=tk.LEFT, padx=2)




        # ========================
        # -- PROGRAM / QUEUE --
        # ========================
        prog = ttk.LabelFrame(outer, text="Program / Queue")
        prog.pack(fill=tk.BOTH, expand=True, pady=6)

        # ---- Command Line ----
        cline = ttk.Frame(prog)
        cline.pack(fill=tk.X, padx=2, pady=(2, 2))
        ttk.Label(cline, text="Command Line (G-Code):").pack(side=tk.LEFT)
        self.entry_cmd = ttk.Entry(cline)
        self.entry_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(cline, text="To Queue", command=self.cli_add_to_program).pack(side=tk.LEFT, padx=3)
        ttk.Button(cline, text="Send Now", command=self.cli_send_now).pack(side=tk.LEFT, padx=3)

        # ---- Log Window ----
        self.txt_log = tk.Text(prog, height=4)
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # ---- Queue List ----
        self.listbox_queue = tk.Listbox(prog, height=5)
        self.listbox_queue.pack(fill=tk.X, padx=6, pady=(0, 6))

        # ---- Queue Buttons ----
        pbtn = ttk.Frame(prog)
        pbtn.pack(fill=tk.X, padx=6, pady=(2, 8))
        ttk.Button(pbtn, text="Load (To Queue)", command=self.load_program).pack(side=tk.LEFT)
        ttk.Button(pbtn, text="Save Queue", command=self.save_program).pack(side=tk.LEFT, padx=2)
        ttk.Button(pbtn, text=" Clear Queue", command=self.clear_program).pack(side=tk.LEFT, padx=2)      
        ttk.Button(pbtn, text="Run", command=self.start_run).pack(side=tk.LEFT, padx=2)
        ttk.Button(pbtn, text="Pause (!)", command=self.pause_run).pack(side=tk.LEFT, padx=2)
        ttk.Button(pbtn, text="Resume (~)", command=self.resume_run).pack(side=tk.LEFT, padx=2)
        ttk.Button(pbtn, text="Stop / Abort", command=self.stop_abort).pack(side=tk.LEFT, padx=6)
        ttk.Checkbutton(pbtn, text="Repeat", variable=self.repeat).pack(side=tk.LEFT, padx=6)
        ttk.Label(pbtn, text="x").pack(side=tk.LEFT, padx=(2, 0))
        ttk.Entry(pbtn, textvariable=self.repeat_times, width=4, justify="center").pack(side=tk.LEFT, padx=(2, 8))
        ttk.Label(pbtn, text="Runs:").pack(side=tk.LEFT, padx=2)
        self.lbl_repeat_status = ttk.Label(pbtn, textvariable=self.repeat_count)
        self.lbl_repeat_status.pack(side=tk.LEFT)
        self._init_cli_history()
        self._init_queue_edit()


    def _toggle_right_vision(self):
        if not hasattr(self, "vision_right_frame"):
            return
        if bool(self.vision_right_enabled.get()):
            self.vision_right_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(2, 0), pady=2)
        else:
            self.vision_right_frame.pack_forget()

    def _tick_right_vision(self):
        try:
            if bool(self.vision_right_enabled.get()):
                vis = None
                if hasattr(self, "board_detector"):
                    vis = getattr(self.board_detector, "last_vis", None)
                if vis is not None and hasattr(self, "vision_right_label"):
                    img = Image.fromarray(vis)
                    resampling = getattr(Image, "Resampling", Image)
                    img = img.resize((300, 225), resampling.LANCZOS)
                    imgtk = ImageTk.PhotoImage(image=img)
                    self.vision_right_label.imgtk = imgtk
                    self.vision_right_label.configure(image=imgtk)
        except Exception:
            pass
        self.after(80, self._tick_right_vision)


    # expose live tcp pose to children
    def get_current_tcp_mm(self):
        try:
            return self.tcp_panel.get_current_tcp_mm()
        except:
            return {"X_mm":0,"Y_mm":0,"Z_mm":0,"Roll_deg":0,"Pitch_deg":0,"Yaw_deg":0}

    def set_gamepad_config_ui(self, ui):
        self.gamepad_config_ui = ui

    def get_profile_section(self, key, default=None):
        if isinstance(self.profile_data, dict):
            if key in self.profile_data:
                return self.profile_data.get(key)
        return default

    def set_profile_section(self, key, value, save=True):
        if not isinstance(self.profile_data, dict):
            self.profile_data = {"name": self.profile_name, "version": 1}
        self.profile_data[key] = value
        if save:
            config_profiles.save_profile(self.profile_name, self.profile_data, base_dir=BASE_DIR)

    def save_profile(self):
        if isinstance(self.profile_data, dict):
            config_profiles.save_profile(self.profile_name, self.profile_data, base_dir=BASE_DIR)

    def apply_profile(self, profile_name, profile_data):
        self.profile_name = profile_name
        self.profile_data = profile_data if isinstance(profile_data, dict) else {"name": profile_name, "version": 1}
        dh_model = self.profile_data.get("dh_model")
        if dh_model:
            tcp_pose_module.set_dh_model_from_dict(dh_model)
        self.GEOM_DH = tcp_pose_module.GEOM_DH.copy()
        if hasattr(self, "tcp_panel"):
            self.tcp_panel.set_geom_dh(self.GEOM_DH)
        self.axis_limits = self._load_endstop_limits(create_if_missing=True)
        self.manual_limits = self._load_manual_limits(create_if_missing=True)
        self._set_endstop_vars_from_limits(self.axis_limits)
        self._apply_axis_limits_to_ui()
        if hasattr(self, "kinematics_tabs"):
            try:
                self.kinematics_tabs.refresh_dh_table()
            except Exception:
                pass
        if self.gamepad_config_ui:
            try:
                self.gamepad_config_ui.reload_into_ui()
            except Exception:
                pass
        self._apply_profile_runtime_flags(log_note=True)
        self._send_vis_robot_profile()

    def get_gamepad_config(self):
        data = self.get_profile_section("gamepad", default={})
        return dict(data) if isinstance(data, dict) else {}

    def save_gamepad_config(self, data):
        if not isinstance(data, dict):
            return False, "Gamepad config invalid."
        self.set_profile_section("gamepad", data, save=True)
        return True, f"Config saved -> {self.profile_path}"

    def save_cam_to_base(self, data):
        if not isinstance(data, dict):
            return False, "cam_to_base invalid."
        self.set_profile_section("cam_to_base", data, save=True)
        return True, f"cam_to_base saved -> {self.profile_path}"

    def save_dh_model_to_profile(self, model):
        if not isinstance(model, dict):
            return False, "DH model invalid."
        self.set_profile_section("dh_model", model, save=True)
        tcp_pose_module.set_dh_model_from_dict(model)
        self.GEOM_DH = tcp_pose_module.GEOM_DH.copy()
        if hasattr(self, "tcp_panel"):
            self.tcp_panel.set_geom_dh(self.GEOM_DH)
        return True, "DH parameters saved to profile and reloaded."

    def _apply_profile_runtime_flags(self, log_note=True):
        self.profile_has_endstops = _profile_has_endstops(self.profile_name, self.profile_data)
        axis_home_attr = getattr(self.client, "supports_axis_homing", False)
        can_axis_home = bool(axis_home_attr() if callable(axis_home_attr) else axis_home_attr)
        global_home_attr = getattr(self.client, "supports_global_homing", True)
        can_global_home = bool(global_home_attr() if callable(global_home_attr) else global_home_attr)
        for btn in getattr(self, "_homing_global_buttons", []):
            try:
                btn.configure(state=("normal" if can_global_home else "disabled"))
            except Exception:
                pass
        for btn in getattr(self, "_homing_axis_buttons", []):
            try:
                btn.configure(state=("normal" if can_axis_home else "disabled"))
            except Exception:
                pass
        if log_note and hasattr(self, "log") and not self.profile_has_endstops:
            self.log("Profile without endstops: run 'Zero (G92)' after manual zeroing.")

    def _send_vis_robot_profile(self):
        if not UDP_MIRROR or not getattr(self.client, "udp_sock", None):
            return
        try:
            dh_model = self.get_profile_section("dh_model", default={})
            geom = tcp_pose_module.derive_visualizer_geometry_mm(dh_model if isinstance(dh_model, dict) else None)
            limits_deg = {}
            dh_rows = []
            post_transform = {}
            if isinstance(dh_model, dict):
                joints_by_axis = {}
                for j in dh_model.get("joints", []):
                    ax = str(j.get("axis", "")).strip().upper()
                    if not ax:
                        continue
                    joints_by_axis[ax] = j
                    q_min = j.get("q_min")
                    q_max = j.get("q_max")
                    if q_min is None or q_max is None:
                        continue
                    try:
                        limits_deg[ax] = [math.degrees(float(q_min)), math.degrees(float(q_max))]
                    except Exception:
                        continue

                src_order = dh_model.get("joint_order", [])
                order = []
                seen = set()
                if isinstance(src_order, list):
                    for raw_ax in src_order:
                        ax = str(raw_ax).strip().upper()
                        if not ax or ax in seen:
                            continue
                        order.append(ax)
                        seen.add(ax)
                for raw_ax in joints_by_axis.keys():
                    ax = str(raw_ax).strip().upper()
                    if not ax or ax in seen:
                        continue
                    order.append(ax)
                    seen.add(ax)

                for ax in order:
                    j = joints_by_axis.get(ax)
                    if not isinstance(j, dict):
                        continue
                    try:
                        dh_rows.append(
                            {
                                "axis": ax,
                                "alpha_deg": float(math.degrees(float(j.get("alpha", 0.0)))),
                                "a_mm": float(j.get("a", 0.0)) * 1000.0,
                                "d_mm": float(j.get("d", 0.0)) * 1000.0,
                                "theta_offset_deg": float(math.degrees(float(j.get("theta_offset", 0.0)))),
                            }
                        )
                    except Exception:
                        continue

                raw_post = dh_model.get("post_transform", {})
                if isinstance(raw_post, dict):
                    sim_theta_offset_deg = {}
                    raw_sim = raw_post.get("sim_theta_offset_deg", {})
                    if isinstance(raw_sim, dict):
                        for raw_ax, raw_val in raw_sim.items():
                            ax = str(raw_ax).strip().upper()
                            if not ax:
                                continue
                            try:
                                sim_theta_offset_deg[ax] = float(raw_val)
                            except Exception:
                                continue
                    sim_theta_scale = {}
                    raw_scale = raw_post.get("sim_theta_scale", {})
                    if isinstance(raw_scale, dict):
                        for raw_ax, raw_val in raw_scale.items():
                            ax = str(raw_ax).strip().upper()
                            if not ax:
                                continue
                            try:
                                sim_theta_scale[ax] = float(raw_val)
                            except Exception:
                                continue
                    post_transform = {
                        "mirror_x": bool(raw_post.get("mirror_x", False)),
                        "sim_theta_offset_deg": sim_theta_offset_deg,
                        "sim_theta_scale": sim_theta_scale,
                    }
            msg = {
                "type": "robot_profile",
                "profile": self.profile_name,
                "has_endstops": bool(self.profile_has_endstops),
                "joint_order": list(geom.get("joint_order", [])),
                "geometry_mm": {
                    "base_height_mm": float(geom.get("base_height_mm", 240.0)),
                    "L1_mm": float(geom.get("L1_mm", 230.0)),
                    "L2_mm": float(geom.get("L2_mm", 250.0)),
                    "L_tool_mm": float(geom.get("L_tool_mm", 180.0)),
                },
                "dh_rows": dh_rows,
                "post_transform": post_transform,
                "limits_deg": limits_deg,
            }
            self.client.udp_sock.sendto((json.dumps(msg) + "\n").encode("utf-8"), UDP_ADDR)
        except Exception:
            pass

    def _default_endstop_limits(self):
        out = {}
        for ax in AXES:
            out[ax] = tuple(DEFAULT_ENDSTOP_LIMITS.get(ax, AXIS_LIMITS_DEFAULT[ax]))
        return out

    def _default_manual_limits(self):
        return dict(self._default_endstop_limits())

    def _normalize_manual_limits(self, limits):
        return self._normalize_endstop_limits(limits)

    def _normalize_endstop_limits(self, limits):
        out = {}
        defaults = self._default_endstop_limits()
        for ax in AXES:
            raw = limits.get(ax, defaults[ax]) if isinstance(limits, dict) else defaults[ax]
            if isinstance(raw, dict):
                lo = raw.get("min", defaults[ax][0])
                hi = raw.get("max", defaults[ax][1])
            elif isinstance(raw, (list, tuple)) and len(raw) >= 2:
                lo, hi = raw[0], raw[1]
            else:
                lo, hi = defaults[ax]
            try:
                lo = float(lo)
                hi = float(hi)
            except Exception:
                lo, hi = defaults[ax]
            if lo > hi:
                lo, hi = hi, lo
            out[ax] = (lo, hi)
        return out

    def _load_endstop_limits(self, create_if_missing=True):
        data = None
        if isinstance(self.profile_data, dict):
            data = self.profile_data.get("endstops")
        axes = None
        if isinstance(data, dict):
            axes = data.get("axes", data)
        if axes is None:
            axes = self._default_endstop_limits()
            if create_if_missing:
                self._write_endstop_limits(self._normalize_endstop_limits(axes))
        return self._normalize_endstop_limits(axes)

    def _load_manual_limits(self, create_if_missing=True):
        data = None
        if isinstance(self.profile_data, dict):
            data = self.profile_data.get("manual_limits")
        axes = None
        if isinstance(data, dict):
            axes = data.get("axes", data)
        if axes is None:
            axes = self._default_manual_limits()
            if create_if_missing:
                self._write_manual_limits(self._normalize_manual_limits(axes))
        return self._normalize_manual_limits(axes)

    def _write_endstop_limits(self, limits):
        payload = {
            "version": 1,
            "units": "deg",
            "axes": {
                ax: {"min": float(limits[ax][0]), "max": float(limits[ax][1])}
                for ax in AXES
            },
        }
        try:
            self.profile_data["endstops"] = payload
            config_profiles.save_profile(self.profile_name, self.profile_data, base_dir=BASE_DIR)
        except Exception as e:
            if hasattr(self, "log"):
                self.log(f"[Endstops] Save failed: {e}")

    def _write_manual_limits(self, limits):
        payload = {
            "version": 1,
            "units": "deg",
            "axes": {
                ax: {"min": float(limits[ax][0]), "max": float(limits[ax][1])}
                for ax in AXES
            },
        }
        try:
            self.profile_data["manual_limits"] = payload
            config_profiles.save_profile(self.profile_name, self.profile_data, base_dir=BASE_DIR)
        except Exception as e:
            if hasattr(self, "log"):
                self.log(f"[ManualLimits] Save failed: {e}")

    def _set_endstop_vars_from_limits(self, limits):
        if not hasattr(self, "endstop_vars"):
            return
        for ax in AXES:
            lo, hi = limits.get(ax, DEFAULT_ENDSTOP_LIMITS.get(ax, (-180.0, 180.0)))
            vmin, vmax = self.endstop_vars.get(ax, (None, None))
            if vmin is not None:
                vmin.set(f"{lo:.3f}")
            if vmax is not None:
                vmax.set(f"{hi:.3f}")

    def _get_endstop_limits_from_vars(self):
        out = {}
        defaults = self._default_endstop_limits()
        for ax in AXES:
            vmin, vmax = self.endstop_vars.get(ax, (None, None))
            lo = defaults[ax][0]
            hi = defaults[ax][1]
            if vmin is not None:
                try:
                    lo = float(vmin.get().replace(",", "."))
                except Exception:
                    lo = defaults[ax][0]
            if vmax is not None:
                try:
                    hi = float(vmax.get().replace(",", "."))
                except Exception:
                    hi = defaults[ax][1]
            out[ax] = (lo, hi)
        return self._normalize_endstop_limits(out)

    def _apply_axis_limits_to_ui(self):
        if not hasattr(self, "axis_scales"):
            return
        for ax in AXES:
            manual_lo, manual_hi = self._manual_axis_limits(ax)
            scale = self.axis_scales.get(ax)
            if scale:
                scale.configure(from_=manual_lo, to=manual_hi)
            redraw = self.axis_scale_redraw.get(ax)
            if redraw:
                redraw(manual_lo, manual_hi)
            if ax in self.axis_vars:
                try:
                    val = float(self.axis_vars[ax].get())
                except Exception:
                    val = 0.0
                lo, hi = self._effective_axis_limits(ax)
                val = max(lo, min(hi, val))
                self.axis_vars[ax].set(val)
                ent = self.axis_entries.get(ax)
                if ent:
                    ent.delete(0, tk.END)
                    ent.insert(0, f"{val:.3f}")

    def _apply_endstop_limits_from_vars(self):
        limits = self._get_endstop_limits_from_vars()
        self.axis_limits = limits
        self._apply_axis_limits_to_ui()
        if hasattr(self, "log"):
            self.log("[Endstops] Limits applied.")

    def _load_endstop_limits_into_vars(self):
        limits = self._load_endstop_limits(create_if_missing=True)
        self.axis_limits = limits
        self._set_endstop_vars_from_limits(limits)
        self._apply_axis_limits_to_ui()
        if hasattr(self, "log"):
            self.log("[Endstops] Limits loaded.")

    def _save_endstop_limits_from_vars(self):
        limits = self._get_endstop_limits_from_vars()
        self.axis_limits = limits
        self._write_endstop_limits(limits)
        self._apply_axis_limits_to_ui()
        if hasattr(self, "log"):
            self.log("[Endstops] Limits saved.")

    def _manual_axis_limits(self, ax):
        limits = getattr(self, "manual_limits", {}) or {}
        lo, hi = limits.get(ax, DEFAULT_ENDSTOP_LIMITS.get(ax, AXIS_LIMITS_DEFAULT[ax]))
        try:
            lo = float(lo)
            hi = float(hi)
        except Exception:
            lo, hi = DEFAULT_ENDSTOP_LIMITS.get(ax, AXIS_LIMITS_DEFAULT[ax])
        if lo > hi:
            lo, hi = hi, lo
        return lo, hi

    def _effective_axis_limits(self, ax):
        lo, hi = self.axis_limits.get(ax, DEFAULT_ENDSTOP_LIMITS.get(ax, AXIS_LIMITS_DEFAULT[ax]))
        if ax in self.hw_limits:
            hw_lo, hw_hi = self.hw_limits[ax]
            use_hw = True
            if self.endstop_limits_enabled and lo < 0 and hw_lo >= 0:
                use_hw = False
            if use_hw:
                lo = max(lo, hw_lo)
                hi = min(hi, hw_hi)
        return lo, hi


    # ---------- Helpers / Logging ----------
    def log(self, s: str):
        self.txt_log.insert(tk.END, s + "\n")
        self.txt_log.see(tk.END)

    def send_now(self, g: str):
        try:
            if self._handle_tcp_gcode(g, source="CLI"):
                return
            self.client.send_line(g)
            self.log("TX: " + g)
        except Exception as e:
            self.log("Send error: " + str(e))

    def insert_delay(self):
        """Ask for a delay and insert G4 P<time> into the queue."""
        try:
            t = tk.simpledialog.askfloat("Insert Delay", "Delay in seconds:", minvalue=0.0, maxvalue=3600.0)
            if t is None:
                return
            g = f"G4 P{t:.3f} ; Delay {t:.3f}s"
            self.enqueue(g)
            self.log(f" Delay {t:.3f}s -> added to queue")
        except Exception as e:
            self.log(f"Delay error: {e}")

    # ---------- CLI ----------
    def _parse_cli(self):
        txt = self.entry_cmd.get().strip()
        if not txt: raise ValueError("Empty input.")
        return [ln.strip() for ln in txt.splitlines() if ln.strip()]

    def _init_cli_history(self):
        self.cli_history = []
        self.cli_hist_index = -1
        self.entry_cmd.bind("<Return>", self._on_cli_return)
        self.entry_cmd.bind("<Up>", self._on_cli_up)
        self.entry_cmd.bind("<Down>", self._on_cli_down)

    def _add_to_history(self, cmd: str):
        if cmd and (not self.cli_history or self.cli_history[-1] != cmd):
            self.cli_history.append(cmd)
        self.cli_hist_index = len(self.cli_history)

    def _on_cli_return(self, event=None):
        cmd = self.entry_cmd.get().strip()
        if not cmd: return "break"
        self._add_to_history(cmd)
        try:
            self.client.send_line(cmd)
            self.log(f"TX (CLI ): {cmd}")
        except Exception as e:
            self.log(f"Send error: {e}")
        self.entry_cmd.delete(0, tk.END)
        return "break"

    def _on_cli_up(self, event=None):
        if not self.cli_history: return "break"
        if self.cli_hist_index > 0: self.cli_hist_index -= 1
        self.entry_cmd.delete(0, tk.END)
        self.entry_cmd.insert(0, self.cli_history[self.cli_hist_index])
        return "break"

    def _on_cli_down(self, event=None):
        if not self.cli_history: return "break"
        if self.cli_hist_index < len(self.cli_history) - 1:
            self.cli_hist_index += 1
            self.entry_cmd.delete(0, tk.END)
            self.entry_cmd.insert(0, self.cli_history[self.cli_hist_index])
        else:
            self.cli_hist_index = len(self.cli_history)
            self.entry_cmd.delete(0, tk.END)
        return "break"

    def cli_add_to_program(self):
        try:
            lines = self._parse_cli()
            for ln in lines: self.enqueue(ln); self._add_to_history(ln)
            self.log(f"CLI: added {len(lines)} line(s) to queue.")
        except Exception as e:
            messagebox.showerror("CLI Error", str(e))

    def cli_send_now(self):
        try:
            lines = self._parse_cli()
            sent = 0
            for ln in lines:
                try:
                    s = (ln or "").strip()
                    s = s.split(";", 1)[0].strip()
                    s = re.sub(r"\(.*(TM)\)", "", s).strip()
                    if not s:
                        continue
                    if self._handle_tcp_gcode(s, source="CLI"):
                        self._add_to_history(ln)
                        sent += 1
                        continue
                    self.client.send_line(s); self._add_to_history(ln)
                    self.log("TX (CLI): " + s)
                    sent += 1
                except Exception as e:
                    self.log("Send Error: " + str(e))
            messagebox.showinfo("Send Now", f"Sent {sent}/{len(lines)} line(s).")
        except Exception as e:
            messagebox.showerror("CLI Error", str(e))


    def _handle_tcp_gcode(self, line: str, source: str = "CLI") -> bool:
        if not self.tcp_gcode_mode.get():
            return False
        s = (line or "").strip()
        if not s:
            return False
        # Strip comments
        s = s.split(";", 1)[0]
        s = re.sub(r"\(.*(TM)\)", "", s)
        u = s.upper()

        if "G90" in u:
            self.tcp_gcode_abs = True
            self.log(f"[TCP GCODE] Mode: G90 ({source})")
            return True
        if "G91" in u:
            self.tcp_gcode_abs = False
            self.log(f"[TCP GCODE] Mode: G91 ({source})")
            return True

        if not ("G0" in u or "G1" in u):
            return False

        vals = {}
        for m in re.finditer(r"([XYZF])\s*([-+](TM)[0-9]*\.(TM)[0-9]+)", u):
            vals[m.group(1)] = float(m.group(2))

        if not any(k in vals for k in ("X", "Y", "Z")):
            return False

        tcp = self.get_current_tcp_mm()
        cx = float(tcp.get("X_mm", 0.0))
        cy = float(tcp.get("Y_mm", 0.0))
        cz = float(tcp.get("Z_mm", 0.0))

        if self.tcp_gcode_abs:
            tx = vals.get("X", cx)
            ty = vals.get("Y", cy)
            tz = vals.get("Z", cz)
        else:
            tx = cx + vals.get("X", 0.0)
            ty = cy + vals.get("Y", 0.0)
            tz = cz + vals.get("Z", 0.0)

        roll = float(self.fixed_tcp_roll.get())
        pitch = float(self.fixed_tcp_pitch.get())
        yaw = float(self.fixed_tcp_yaw.get())
        feed = float(vals.get("F", self.fixed_tcp_feed.get()))

        if hasattr(self, "kinematics_tabs"):
            ok = self.kinematics_tabs.move_tcp_pose(
                tx, ty, tz, roll, pitch, yaw, feed=feed, allow_out_of_limits=False
            )
            if ok:
                self._append_vis_path_fixed((tx, ty, tz))
            self.log(f"[TCP GCODE] {'OK' if ok else 'FAIL'}: X{tx:.3f} Y{ty:.3f} Z{tz:.3f} F{feed:.0f}")
            return True
        return False

    # ---------- Queue ops ----------
    def enqueue(self, gline: str):
        g = gline.strip()
        if not g: return
        self.program.append(g)
        self.listbox_queue.insert(tk.END, g)
        self.log("Queued: " + g)

    def _append_vis_path_fixed(self, pt):
        try:
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
        except Exception:
            return
        try:
            a = float(self.axis_positions.get("A", 0.0))
        except Exception:
            a = 0.0
        self._vis_path_fixed.append((x, y, z, a))
        if len(self._vis_path_fixed) > 400:
            self._vis_path_fixed = self._vis_path_fixed[-400:]
        self._send_vis_path("path_fixed", self._vis_path_fixed)

    def _append_vis_path_kin(self, pt):
        try:
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
        except Exception:
            return
        try:
            a = float(self.axis_positions.get("A", 0.0))
        except Exception:
            a = 0.0
        self._vis_path_kin.append((x, y, z, a))
        if len(self._vis_path_kin) > 400:
            self._vis_path_kin = self._vis_path_kin[-400:]
        self._send_vis_path("path_kin", self._vis_path_kin)

    def _send_vis_path(self, path_type, pts):
        if not UDP_MIRROR or not getattr(self.client, "udp_sock", None):
            return
        try:
            msg = json.dumps({
                "type": path_type,
                "pts": [[float(a), float(b), float(c), float(d)] for a, b, c, d in pts],
            })
            self.client.udp_sock.sendto(msg.encode("utf-8"), UDP_ADDR)
        except Exception:
            pass

    def _send_vis_fixed_frame(self, origin, x_axis, y_axis, z_axis):
        if not UDP_MIRROR or not getattr(self.client, "udp_sock", None):
            return
        try:
            msg = json.dumps({
                "type": "fixed_frame",
                "origin": [float(origin[0]), float(origin[1]), float(origin[2])],
                "x": [float(x_axis[0]), float(x_axis[1]), float(x_axis[2])],
                "y": [float(y_axis[0]), float(y_axis[1]), float(y_axis[2])],
                "z": [float(z_axis[0]), float(z_axis[1]), float(z_axis[2])],
            })
            self.client.udp_sock.sendto(msg.encode("utf-8"), UDP_ADDR)
        except Exception:
            pass

    def clear_program(self):
        self.program.clear()
        self.listbox_queue.delete(0, tk.END)
        self.log("Queue cleared.")

    def load_program(self):
        path = filedialog.askopenfilename(filetypes=[("GCode/All", "*.gcode *.nc *.txt *.ngc *.tap *.*")])
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = [ln.strip() for ln in f.readlines()]
            n = 0
            for ln in lines:
                if ln and not ln.startswith(";"):
                    self.enqueue(ln); n += 1
            self.log(f"Loaded {n} lines from {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def save_program(self):
        if not self.program:
            messagebox.showinfo("Save Queue", "Queue is empty."); return
        path = filedialog.asksaveasfilename(defaultextension=".gcode", filetypes=[("GCode", "*.gcode"), ("Text", "*.txt")])
        if not path: return
        try:
            with open(path, "w", encoding="utf-8") as f:
                for ln in self.program: f.write(ln + "\n")
            self.log(f"Saved {len(self.program)} lines to {path}")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    # ---------- Queue Edit (DnD / Edit / Insert / Delete) ----------
    def _init_queue_edit(self):
        lb = self.listbox_queue
        lb.bind("<Delete>", self._queue_delete)
        lb.bind("<Insert>", self._queue_insert)
        lb.bind("<Double-Button-1>", self._queue_edit_item)

        # Drag&Drop Reorder
        lb.bind("<Button-1>", self._queue_on_click)
        lb.bind("<B1-Motion>", self._queue_on_drag)
        lb.bind("<ButtonRelease-1>", self._queue_on_drop)
        self._drag_index = None

    def _queue_delete(self, event=None):
        sel = self.listbox_queue.curselection()
        if not sel: return "break"
        idx = sel[0]
        self.listbox_queue.delete(idx)
        del self.program[idx]
        self.log(f"Deleted line {idx}")
        return "break"

    def _queue_insert(self, event=None):
        idx = self.listbox_queue.curselection()
        ins = idx[0] if idx else tk.END
        cmd = simpledialog.askstring("Insert G-Code", "G-Code Zeile:")
        if cmd:
            if ins == tk.END:
                self.program.append(cmd); self.listbox_queue.insert(tk.END, cmd)
            else:
                self.program.insert(ins, cmd); self.listbox_queue.insert(ins, cmd)
            self.log("Inserted: " + cmd)
        return "break"

    def _queue_edit_item(self, event=None):
        sel = self.listbox_queue.curselection()
        if not sel: return
        idx = sel[0]
        old = self.program[idx]
        new = simpledialog.askstring("Edit G-Code", "G-Code Zeile bearbeiten:", initialvalue=old)
        if new and new.strip() != old:
            self.program[idx] = new.strip()
            self.listbox_queue.delete(idx)
            self.listbox_queue.insert(idx, new.strip())
            self.listbox_queue.selection_set(idx)
            self.log(f"Edited line {idx}")

    def _queue_on_click(self, event):
        self._drag_index = self.listbox_queue.nearest(event.y)

    def _queue_on_drag(self, event):
        if self._drag_index is None: return
        i = self.listbox_queue.nearest(event.y)
        if i < 0 or i >= self.listbox_queue.size(): return
        if i == self._drag_index: return
        # Swap in listbox
        txt_from = self.listbox_queue.get(self._drag_index)
        txt_to   = self.listbox_queue.get(i)
        self.listbox_queue.delete(self._drag_index)
        self.listbox_queue.insert(self._drag_index, txt_to)
        self.listbox_queue.delete(i)
        self.listbox_queue.insert(i, txt_from)
        # Swap in program
        self.program[self._drag_index], self.program[i] = self.program[i], self.program[self._drag_index]
        self.listbox_queue.selection_clear(0, tk.END)
        self.listbox_queue.selection_set(i)
        self._drag_index = i

    def _queue_on_drop(self, event):
        self._drag_index = None

    def clamp_with_limits(self, ax, v):
        """Clamp a target position to configured axis limits."""
        try:
            val = float(v)
        except Exception:
            val = 0.0
        lo, hi = self._effective_axis_limits(ax)
        return max(lo, min(hi, val))

    # ============================================================
    # ACHSENFUNKTIONEN: Queue / CLI / DirectSend
    # ============================================================

    def axis_to_queue(self, ax: str, val: float):
        """Insert one axis move as a G1 command into the queue."""
        try:
            val = float(val)
            g = f"G1 {ax}{val:.3f} F{map_speed(int(self.speed_val.get()))}"
            self.enqueue(g)
            self.log(f"[ToQ] {g}")
        except Exception as e:
            self.log(f"[ToQ ERROR] {e}")

    def axis_to_cli(self, ax: str, val: float):
        """Write axis move into CLI field."""
        try:
            val = float(val)
            g = f"G1 {ax}{val:.3f} F{map_speed(int(self.speed_val.get()))}"
            self.entry_cmd.delete(0, tk.END)
            self.entry_cmd.insert(0, g)
            self.log(f"[ToCLI] {g}")
        except Exception as e:
            self.log(f"[ToCLI ERROR] {e}")

    def axis_direct_send(self, ax: str, val: float):
        """Send G1 directly (without queue)."""
        try:
            val = float(val)
            f = map_speed(int(self.speed_val.get()))
            g = f"G1 {ax}{val:.3f} F{f}"

            self.client.send_line("$X")
            time.sleep(0.02)
            self.client.send_line("G90")
            time.sleep(0.02)

            self.client.send_line(g)
            self.axis_positions[ax] = val
            self.axis_vars[ax].set(val)
            self.update_position_display()

            self.log(f"[SEND] {g}")
        except Exception as e:
            self.log(f"[Send ERROR] {e}")

    def do_zero(self, persist_reference=False, source="zero"):
        try:
            pre_pose = {ax: float(self.axis_positions.get(ax, 0.0)) for ax in AXES}
            self.client.send_line("G92 X0 Y0 Z0 A0 B0 C0")
            for ax in AXES:
                self.axis_positions[ax] = 0.0
                if ax in self.axis_vars:
                    self.axis_vars[ax].set(0.0)
                ent = self.axis_entries.get(ax)
                if ent is not None:
                    ent.delete(0, tk.END)
                    ent.insert(0, "0.000")
            self.update_position_display()
            if persist_reference:
                payload = {
                    "source": str(source),
                    "profile": self.profile_name,
                    "pre_g92_machine_pose": pre_pose,
                    "timestamp": time.time(),
                }
                self.set_profile_section("manual_zero_reference", payload, save=True)
                self.log("Zero reference saved + G92 applied.")
            else:
                self.log("Zero All (G92)")
            return True
        except Exception as e:
            self.log("Zero error: " + str(e))
            return False

    def manual_zero_current_pose(self):
        return self.do_zero(persist_reference=True, source="manual_zero")

    def goto_home(self):
        try:
            # Original: gemappter Feed/Accel
            f = map_speed(int(self.speed_val.get()))
            a = map_speed(int(self.accel_val.get()))

            # ---- Limit only for GOTO HOME ----
            f_home = min(f, MAX_HOME_FEED)

            self.client.send_line("$X"); time.sleep(0.02)   # entsperren
            self.client.send_line("G90"); time.sleep(0.02)  # absolut

            g = f"G1 X0 Y0 Z0 A0 B0 C0 F{f_home}"
            self.client.send_line(g)

            self.log(f"TX: {g} (Goto Home, F_user={f}, F_home={f_home}, A={a})")

        except Exception as e:
            self.log("Goto-home error: " + str(e))

    def enqueue_home(self):
        f = map_speed(int(self.speed_val.get()))
        g = f"G90 G1 X0 Y0 Z0 A0 B0 C0 F{f}"
        self.enqueue(g)
        self.log(" To Home -> added to queue")

    def enqueue_axis(self, ax: str, val: float):
        g = self._make_move_line({ax: val})
        self.enqueue(g)

    def set_all_to_queue(self):
        targets = {}
        for ax in AXES:
            try: v = float(self.axis_entries[ax].get())
            except: v = 0.0
            # clamp inkl. Endstop-/HW-Limits
            lo, hi = self._effective_axis_limits(ax)
            v = max(lo, min(hi, v))
            targets[ax] = v
        g = self._make_move_line(targets)
        self.enqueue(g)
        self.log("To Queue: " + g)

    # ============================================================
    # POSE -> QUEUE (for gamepad & button " To Queue")
    # ============================================================
    def add_current_pose_to_queue(self):
        """
        Read all axis entries from the Manual tab, clamp to limits,
        build a G1 line, and enqueue it.
        """
        try:
            parts = []

            for ax in AXES:
                ent = self.axis_entries.get(ax)
                if ent is None:
                    self.log(f"[add_current_pose_to_queue] Missing entry for {ax}")
                    return

                raw = ent.get().strip().replace(",", ".")
                if not raw:
                    self.log(f"[add_current_pose_to_queue] Empty entry for {ax}")
                    return
                try:
                    v = float(raw)
                except Exception:
                    self.log(f"[add_current_pose_to_queue] Invalid value for {ax}: {raw}")
                    return

                lo, hi = self._effective_axis_limits(ax)
                v = max(lo, min(hi, v))
                parts.append(f"{ax}{v:.3f}")

            base_F = map_speed(int(self.speed_val.get()))
            msf = getattr(self, "manual_speed_factor", None)
            if msf is not None:
                base_F *= float(msf.get())
            F = max(1.0, base_F)

            gline = "G1 " + " ".join(parts) + f" F{F:.0f}"
            self.enqueue(gline)
            self.log(f"Pose -> Queue (Manual): {gline}")

        except Exception as e:
            self.log(f"[add_current_pose_to_queue ERROR] {e}")


    def set_all_now(self):
        targets = {}
        for ax in AXES:
            try: v = float(self.axis_entries[ax].get())
            except: v = 0.0
            lo, hi = self._effective_axis_limits(ax)
            v = max(lo, min(hi, v))
            targets[ax] = v
        g = self._make_move_line(targets)
        try:
            self.client.send_line("G90" if self.mode_absolute.get() else "G91")
            time.sleep(0.01)
            self.client.send_line(g)
            self.log("TX (Set All Now): " + g)
        except Exception as e:
            self.log("Send error Set All Now: " + str(e))

    # ---------- Worker / Steuerung ----------
    def start_run(self):
        if not self.program:
            self.log("   Queue is empty.")
            return
        self.abort_event.clear()
        self.paused = False
        self.repeat_count.set(0)
        self.log(" RUN started")
        self.run_event.set()

    def pause_run(self):
        """Pausiert aktuelle Bewegung (Feed Hold)."""
        try:
            self.client.send_line("!")
            self.paused = True
            self.log("  Paused (Feed Hold sent)")
        except Exception as e:
            self.log(f"Pause error: {e}")

    def resume_run(self):
        """Setzt pausierte Bewegung fort."""
        if not self.paused:
            self.log("1  No active pause.")
            return
        try:
            self.client.send_line("~")
            self.paused = False
            self.log(" Resume (~) sent")
        except Exception as e:
            self.log(f"Resume error: {e}")

    def stop_abort(self):
        """Full abort: stop queue, reset events, and release control immediately."""
        self.log(" STOP/ABORT pressed")
        self.abort_event.set()
        self.run_event.clear()
        self.paused = False
        self._awaiting_ack = False
        try:
            self.client.send_ctrl_x()  # hard reset is intentional here
            self.log(" Ctrl-X (reset) sent")
        except Exception as e:
            self.log(f"Abort error: {e}")
        self.log("... System ready for new commands")

    # Alias for gamepad shortcut (compatibility)
    def emergency_stop(self):
        """Stop alias for gamepad compatibility."""
        self.stop_abort()




    def worker(self):
        """Background thread for queue execution."""
        while True:
            self.run_event.wait()
            if self.abort_event.is_set():
                time.sleep(0.05)
                continue
            if not self.program:
                time.sleep(0.05)
                continue

            lines = list(self.program)
            self.log(f" Starting program execution ({len(lines)} line(s))")
            self._awaiting_ack = False

            for i, g in enumerate(lines, start=1):
                # check pause/abort
                while self.paused and not self.abort_event.is_set():
                    time.sleep(0.05)
                if self.abort_event.is_set() or not self.run_event.is_set():
                    break

                f_now = map_speed(int(self.speed_val.get()))
                a_now = map_speed(int(self.accel_val.get()))
                if "G1" in g or "G0" in g:
                    import re
                    g = re.sub(r"F[0-9\.]+", f"F{f_now}", g) if "F" in g else f"{g} F{f_now}"

                self.current_cmd = g
                self._awaiting_ack = True
                self.log(f"[{i}/{len(lines)}] TX: {g} (F={f_now}, A={a_now})")

                if self._handle_tcp_gcode(g, source="Queue"):
                    self._awaiting_ack = False
                    continue

                try:
                    self.client.send_line(g)
                except Exception as e:
                    self.log(f" TX error: {e}")
                    self._awaiting_ack = False
                    continue

                deadline = time.time() + DEFAULT_TIMEOUT
                while time.time() < deadline:
                    if self.abort_event.is_set() or not self.run_event.is_set():
                        self._awaiting_ack = False
                        break
                    if not self._awaiting_ack:
                        break
                    time.sleep(0.02)
                else:
                    self.log(f"   Timeout on: {g}")
                    self._awaiting_ack = False

                self.current_cmd = None

            # End/abort/repeat
            if self.abort_event.is_set():
                self.log(" Program execution aborted")
                self.abort_event.clear()
                self.run_event.clear()
                self.paused = False
                self._awaiting_ack = False
                continue

            if self.repeat.get() and not self.abort_event.is_set():
                self.repeat_count.set(self.repeat_count.get() + 1)
                total = max(1, self.repeat_times.get())

                if self.repeat_count.get() < total:
                    self.log(f" Repeating queue ({self.repeat_count.get()}/{total})")
                    continue
                else:
                    self.log(f"... Repeat finished ({self.repeat_count.get()}/{total})")


            self.run_event.clear()
            self.paused = False
            self._awaiting_ack = False
            self.log("... Queue finished - ready for next start")

    def _update_status_block(self, state: str):
        """Update the status block color/text based on machine state."""
        st = (state or "-").lower()
        color = "#777777"   # neutral
        if "idle" in st:
            color = "#2e7d32"   # green
        elif "run" in st:
            color = "#1565c0"   # blau
        elif "hold" in st:
            color = "#f9a825"   # gelb
        elif "alarm" in st or "error" in st:
            color = "#b71c1c"   # rot
        elif "home" in st:
            color = "#00897b"   # turquoise
        elif "disconnect" in st:
            color = "#616161"   # grau

        self.status_block_var.set(f"Status: {state}")
        # tk.Label (not ttk) -> bg works reliably
        self.lbl_status_block.configure(bg=color, fg=("black" if color == "#f9a825" else "white"))


    # ---------- RX / Status ----------
    def on_serial_line(self, line: str):
        if not line: return
        if line == "ok":
            if self._awaiting_ack: self._awaiting_ack = False
            return
        if line.startswith("error:"):
            self.log("RX: " + line)
            if self._awaiting_ack: self._awaiting_ack = False
            return


        # $$ soft-limit parser (works for FluidNC and GRBL)
        for ax, rx in SOFTMAX_RE.items():
            m = rx.match(line)
            if m:
                try:
                    max_travel = float(m.group(1))
                    self.hw_limits[ax] = (0.0, max_travel)
                    if ax in self.axis_limit_labels:
                        self.axis_limit_labels[ax].config(text=f"[0..{max_travel:.0f}]")
                except:
                    pass


        # =========================================================
        #  Shared status parser for FluidNC & GRBL
        # Format typischerweise:
        #   <Idle|MPos:0.000,0.000,0.000|FS:0,0|Pn:X>
        #   <Run|MPos:...|FS:...>
        # =========================================================
        if line.startswith("<") and line.endswith(">"):
            payload = line[1:-1]
            parts = payload.split("|")
            if not parts:
                return

            state = parts[0]
            self._update_status_block(state)

            mpos = {}
            wpos = {}
            endstop_active = set()

            for part in parts[1:]:
                if part.startswith("MPos:"):
                    nums = part[5:].split(",")
                    for idx, ax in enumerate(AXES):
                        if idx < len(nums):
                            try:
                                mpos[ax] = float(nums[idx])
                            except ValueError:
                                pass

                elif part.startswith("WPos:"):
                    nums = part[5:].split(",")
                    for idx, ax in enumerate(AXES):
                        if idx < len(nums):
                            try:
                                wpos[ax] = float(nums[idx])
                            except ValueError:
                                pass

                elif part.startswith("Pn:") and self.client.supports_endstops:
                    endstop_active = set(part[3:])

            # --- Endstops are meaningful only with FluidNC ---
            if self.client.supports_endstops and endstop_active is not None:
                for ax in AXES:
                    if ax in self.axis_endstop_icons:
                        c, oval, state_old = self.axis_endstop_icons[ax]
                        new_state = "tripped" if ax in endstop_active else "free"
                        if new_state != state_old:
                            color = "#b71c1c" if new_state == "tripped" else "#2e7d32"
                            c.itemconfig(oval, fill=color)
                            self.axis_endstop_icons[ax] = (c, oval, new_state)

                    if hasattr(self, "endstop_indicators") and ax in self.endstop_indicators:
                        c2, oval2, state2 = self.endstop_indicators[ax]
                        new_state = "tripped" if ax in endstop_active else "free"
                        if new_state != state2:
                            color = "#b71c1c" if new_state == "tripped" else "#2e7d32"
                            c2.itemconfig(oval2, fill=color)
                            self.endstop_indicators[ax] = (c2, oval2, new_state)

            # --- Achspositionen aktualisieren (MPos bevorzugt) ---
            pose_src = mpos or wpos
            abs_pose = {}

            updated = False
            for ax, val in pose_src.items():
                if ax not in AXES:
                    continue
                abs_pose[ax] = val
                if abs(val - self.axis_positions.get(ax, 0.0)) > MOTION_EPS and not self._user_editing.get(ax, False):
                    self.axis_positions[ax] = val
                    if ax in self.axis_vars:
                        self.axis_vars[ax].set(val)
                    updated = True
            if updated:
                self.update_position_display()

            # keep status for other parts
            self.client.last_status = {"state": state, "MPos": mpos, "WPos": wpos}

            # --- UDP abs-Positionsbroadcast ---
            if UDP_MIRROR:
                try:
                    import json, time as _time

                    # Basis: das, was vom Controller kam
                    full_pose = dict(abs_pose)

                    # fill missing axes with last known GUI position
                    for ax in AXES:
                        if ax not in full_pose and ax in self.axis_positions:
                            full_pose[ax] = float(self.axis_positions[ax])

                    if full_pose:
                        msg = {"type": "abs", "timestamp": _time.time()}
                        msg.update(full_pose)
                        self.client.udp_sock.sendto((json.dumps(msg) + "\n").encode("utf-8"), UDP_ADDR)

                except Exception as e:
                    self.log(f"[UDP abs send fail] {e}")



        # Sonstiges
            if not (
                line.startswith("<") or
                line == "ok" or
                line.startswith("[MSG:") or
                line.startswith("[GC:") or
                "MPos:" in line or
                "WPos:" in line or
                "FS:"   in line or
                "Ov:"   in line
            ):
                self.log("RX: " + line)
    # ---------- $$ Parser ----------
    def request_and_parse_settings(self):
        """Send '$$' and parse the responses (soft limits, etc.)."""
        self.log("TX: $$ (request settings)")
        try:
            # clear old HW limits display
            for ax in self.hw_limits:
                self.axis_limit_labels[ax].configure(text="")
            self.hw_limits.clear()

            # Anfrage senden
            self.client.send_line("$$")
        except Exception as e:
            self.log("$$ error: " + str(e))


    def update_position_display(self):
        for ax in AXES:
            self.axis_labels[ax].configure(text=f"{self.axis_positions[ax]:.3f}")


    # ============================================================
    # GAMEPAD  Fixed-TCP XYZ movement (called via after() from gamepad thread)
    # ============================================================
    def move_fixed_tcp_gamepad(self, dx, dy, dz, feed=3000):
        """Accumulate incremental XYZ offset and execute a fixed-TCP IK move.
        Called on the main thread via widget.after(0, ...) from the gamepad thread.
        Uses the supplied feed rate (from speed_val) instead of fixed_tcp_feed so
        the robot buffer stays pre-filled and motion is smooth."""
        if not getattr(self, "fixed_tcp_enabled", None) or not self.fixed_tcp_enabled.get():
            return
        if not getattr(self, "fixed_tcp_gamepad_mode", None) or not self.fixed_tcp_gamepad_mode.get():
            return
        try:
            try:
                _lim = max(10.0, float(self.fixed_tcp_max_dist.get()))
            except Exception:
                _lim = 300.0
            def _c(v):
                return max(-_lim, min(_lim, float(v)))
            self.fixed_tcp_dx.set(_c(self.fixed_tcp_dx.get() + dx))
            self.fixed_tcp_dy.set(_c(self.fixed_tcp_dy.get() + dy))
            self.fixed_tcp_dz.set(_c(self.fixed_tcp_dz.get() + dz))
            if hasattr(self, "_fixed_tcp_calc_target") and hasattr(self, "kinematics_tabs"):
                x, y, z, roll, pitch, yaw, _, _, _ = self._fixed_tcp_calc_target()
                # Normalize RPY to be closest to previous values → prevents ±180 jumps
                prev_rpy = getattr(self, "_gp_tcp_prev_rpy", None)
                if prev_rpy is not None:
                    def _wrap_closest(angle, ref):
                        d = (angle - ref + 180.0) % 360.0 - 180.0
                        return ref + d
                    roll = _wrap_closest(roll, prev_rpy[0])
                    pitch = _wrap_closest(pitch, prev_rpy[1])
                    yaw = _wrap_closest(yaw, prev_rpy[2])
                self._gp_tcp_prev_rpy = (roll, pitch, yaw)
                prev = getattr(self, "_vis_skip_kin", False)
                self._vis_skip_kin = True
                try:
                    self.kinematics_tabs.move_tcp_pose(
                        x, y, z, roll, pitch, yaw,
                        feed=int(feed),
                        allow_out_of_limits=False,
                    )
                finally:
                    self._vis_skip_kin = prev
        except Exception as e:
            try:
                self.log(f"[Gamepad TCP] {e}")
            except Exception:
                pass

    # ============================================================
    # GAMEPAD  Multi-Mode Cycle  (RT = next mode)
    # Modes: 0=Normal  1=Fixed/Tool  2=Fixed/Point
    # ============================================================
    _GP_MODE_LABELS = ["Normal", "Fixed/Tool", "Fixed/Point"]
    _GP_MODE_COLORS = ["#455a64", "#1565c0", "#6a1b9a"]

    def set_gamepad_mode_cycle(self, mode: int):
        """Cycle to the given gamepad mode and update UI state."""
        global _gp_mode_var, _gp_mode_lbl
        mode = int(mode) % 3
        self.gamepad_mode_cycle.set(mode)
        label = self._GP_MODE_LABELS[mode]
        color = self._GP_MODE_COLORS[mode]
        try:
            _gp_mode_var.set(f"GP: {label}")
            _gp_mode_lbl.configure(bg=color)
        except Exception:
            pass
        self._gp_tcp_prev_rpy = None
        if mode == 0:
            if getattr(self, "fixed_tcp_gamepad_mode", None):
                self.fixed_tcp_gamepad_mode.set(False)
            if getattr(self, "fixed_tcp_enabled", None):
                self.fixed_tcp_enabled.set(False)
        elif mode == 1:
            if getattr(self, "fixed_tcp_enabled", None):
                self.fixed_tcp_enabled.set(True)
            if getattr(self, "fixed_tcp_mode", None):
                self.fixed_tcp_mode.set("tool")
            if getattr(self, "fixed_tcp_gamepad_mode", None):
                self.fixed_tcp_gamepad_mode.set(True)
        elif mode == 2:
            if getattr(self, "fixed_tcp_enabled", None):
                self.fixed_tcp_enabled.set(True)
            if getattr(self, "fixed_tcp_mode", None):
                self.fixed_tcp_mode.set("point")
            if getattr(self, "fixed_tcp_gamepad_mode", None):
                self.fixed_tcp_gamepad_mode.set(True)
        try:
            self.log(f"[Gamepad] Mode \u2192 {label}")
        except Exception:
            pass

    # ============================================================
    # GAMEPAD  Fix from current TCP  (LT full press)
    # ============================================================
    def fix_tcp_from_gamepad(self):
        """LT: capture current TCP as fixed origin (same as GUI button) & optionally set Point."""
        mode = int(self.gamepad_mode_cycle.get()) if getattr(self, "gamepad_mode_cycle", None) else 0
        # Call the same function as the GUI "Fix from current TCP" button
        fn = getattr(self, "_update_fixed_from_tcp_fn", None)
        if fn is not None:
            try:
                fn()
            except Exception:
                pass
        self._gp_tcp_prev_rpy = None
        # In Point mode: additionally set the TCP point target from current position
        if mode == 2:
            pt_fn = getattr(self, "_set_tcp_point_from_current_fn", None)
            if pt_fn is not None:
                try:
                    pt_fn()
                except Exception:
                    pass
        try:
            self.log("[Gamepad] Fix from current TCP")
        except Exception:
            pass

    # ============================================================
    # GAMEPAD  Tool Roll  (D-Pad left/right in fixed modes 1 & 2)
    # ============================================================
    def rotate_fixed_tcp_roll(self, delta_deg: float):
        """Increment fixed TCP roll angle and re-apply (tool rotation in fixed modes)."""
        if not getattr(self, "fixed_tcp_enabled", None) or not self.fixed_tcp_enabled.get():
            return
        try:
            cur_roll = float(self.fixed_tcp_roll.get())
            self.fixed_tcp_roll.set(cur_roll + delta_deg)
            if hasattr(self, "_fixed_tcp_apply_target"):
                self._fixed_tcp_apply_target(execute=True)
        except Exception as e:
            try:
                self.log(f"[Gamepad Roll] {e}")
            except Exception:
                pass

    # ============================================================
    # GAMEPAD  ExecuteApp Dispatcher
    # (called from gamepad_block_v3_x)
    # ============================================================
    def on_gamepad_trigger(self, name, payload):
        """
        Central dispatcher for gamepad actions.
        'name' is a string like 'tcp_add_pose_to_queue',
        'payload' is an optional dict.
        """
        try:
            self.log(f"[GAMEPAD] Trigger  {name}  payload={payload}")
        except Exception:
            print(f"[GAMEPAD] Trigger  {name}  payload={payload}")

        # ---------------------------------------------------------
        # 1) add current pose to queue
        # ---------------------------------------------------------
        if name == "tcp_add_pose_to_queue":
            if hasattr(self, "add_current_pose_to_queue"):
                return self.add_current_pose_to_queue()
            self.log("   Missing method: add_current_pose_to_queue()")
            return

        # ---------------------------------------------------------
        # 2) Preview der TCP-Sequenz
        # ---------------------------------------------------------
        elif name == "tcp_seq_preview":
            # 1) check kinematics tabs
            if hasattr(self, "kinematics_tabs"):
                for attr in (
                    "gamepad_preview_sequence",
                    "preview_sequence",
                    "preview_tcp_sequence",
                    "seq_preview",     # common name
                    "preview",         # fallback
                ):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()

            # 2) Fallback: direkt ExecuteApp Preview
            if hasattr(self, "seq_preview"):
                return self.seq_preview()

            self.log("   No preview function found (neither in kinematics_tabs nor ExecuteApp).")
            return

        # ---------------------------------------------------------
        # 3) execute sequence
        # ---------------------------------------------------------
        elif name == "tcp_seq_execute_cli":
            if hasattr(self, "kinematics_tabs"):
                for attr in ("gamepad_execute_sequence", "execute_sequence_cli", "execute_tcp_sequence"):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()
            # Fallback: komplette Queue abfahren
            if hasattr(self, "start_run"):
                return self.start_run()
            self.log("   No execute function found for TCP sequence.")
            return

        # ---------------------------------------------------------
        # 4) set TCP as IK reference
        # ---------------------------------------------------------
        elif name == "tcp_set_tcp_as_reference":
            if hasattr(self, "kinematics_tabs"):
                for attr in ("set_tcp_as_reference", "use_current_tcp_as_ref"):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()
            self.log("   No function found to set TCP reference.")
            return

        # ---------------------------------------------------------
        # 4b) TCP -> mask, Generate (without Execute)
        # ---------------------------------------------------------
        elif name == "tcp_pose_to_mask_and_generate":
            if hasattr(self, "kinematics_tabs"):
                for attr in ("pose_to_mask_and_generate", "generate_from_current_pose"):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()
            self.log("   No generate function found for TCP.")
            return

        # ---------------------------------------------------------
        # 5) Solve & Run komplette TCP-Sequenz
        # ---------------------------------------------------------
        elif name == "tcp_solve_seq_and_execute":
            if hasattr(self, "kinematics_tabs"):
                for attr in ("solve_and_execute_sequence", "solve_and_run_tcp_sequence"):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()
            self.log("   No solve-and-run function found in kinematics_tabs.")
            return

        # ---------------------------------------------------------
        # 6) Unbekannt
        # ---------------------------------------------------------
        else:
            self.log(f"   Unknown gamepad trigger: {name}")
            return



    def _tick_status_poll(self):
        if self.poll_positions.get():
            try: self.client.send_line(self.client.status_query_line())
            except Exception as e:
                print("[Warn]", e)
        cur = (self.status_block_var.get() or "")
        delay = 150 if ("Run" in cur or "Hold" in cur) else 250
        self.after(delay, self._tick_status_poll)

# ---- App-Instanz ----
execute_app = ExecuteApp(root, client, ACTIVE_PROFILE["name"], ACTIVE_PROFILE["data"])
execute_app.pack(fill="both", expand=True)

# Klick auf Gamepad-Modus-Label cyclet den Modus (wie RT am Controller)
def _gp_label_click(_event=None):
    cur = int(execute_app.gamepad_mode_cycle.get())
    execute_app.set_gamepad_mode_cycle((cur + 1) % 3)
_gp_mode_lbl.bind("<Button-1>", _gp_label_click)

def on_close():
    try:
        # TCP-Panel stoppen
        if hasattr(execute_app, "tcp_panel"):
            try:
                execute_app.tcp_panel.stop()
            except:
                pass

        # Gamepad stoppen
        if hasattr(execute_app, "stop_gamepad"):
            try:
                execute_app.stop_gamepad()
            except Exception as e:
                print("[on_close] stop_gamepad error:", e)

        # Seriell trennen
        try:
            client.disconnect()
        except Exception as e:
            print("[on_close] client.disconnect error:", e)

    except Exception as e:
        print("[on_close ERROR]", e)

    root.destroy()
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
