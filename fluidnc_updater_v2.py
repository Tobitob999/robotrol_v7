# fluidnc_updater_v2.py
# Standalone FluidNC OTA + $$ Inspector + Config Manager
# Ready to integrate as a tab/frame.
# Authors: ChatGPT & Tobias Kraemer
# Date: 2025-11-01

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading, time, json, re, os
import urllib.parse
import requests

DEFAULT_IP = "192.168.25.149"  # <- Deine Vorgabe ("pre IP")

# --------------- Low-level HTTP Client ---------------

class FluidNCClient:
    """HTTP helper for FluidNC WebAPI / web UI endpoints."""
    def __init__(self, host: str, timeout=5.0):
        self.host = host.strip()
        self.base = f"http://{self.host}"
        self.timeout = float(timeout)

    def set_host(self, host: str):
        self.host = host.strip()
        self.base = f"http://{self.host}"

    def command(self, line: str) -> str:
        """Send a plain command via /command?plain=..."""
        url = f"{self.base}/command"
        params = {"plain": line}
        r = requests.get(url, params=params, timeout=self.timeout)
        r.raise_for_status()
        return r.text.strip()

    def is_alive(self) -> bool:
        try:
            txt = self.command("$I")
            return bool(txt)
        except Exception:
            return False

    def get_dollars(self) -> list[str]:
        out = self.command("$$")
        return [ln.strip() for ln in out.splitlines() if ln.strip()]

    def list_localfs(self) -> list[str]:
        txt = self.command("$LocalFS/List")
        files = []
        try:
            data = json.loads(txt)
            if isinstance(data, list):
                files = [str(x) for x in data]
            elif isinstance(data, dict) and "files" in data:
                files = [f.get("name", "") for f in data["files"] if isinstance(f, dict)]
        except Exception:
            for ln in txt.splitlines():
                s = ln.strip().split()[-1] if ln.strip() else ""
                if s and not s.startswith("ok"):
                    files.append(s)
        return sorted({f for f in files if f})

    def get_active_config(self) -> str | None:
        try:
            txt = self.command("$Config/Filename")
            m = re.search(r"([A-Za-z0-9_\-/.]+\.yaml)", txt)
            if m:
                return m.group(1)
            s = txt.strip()
            if s.endswith(".yaml") or s.endswith(".yml"):
                return s
        except Exception:
            pass
        return None

    def set_active_config(self, filename: str) -> str:
        cmd = f"$Config/Filename={filename}"
        return self.command(cmd)

    #  hier kommt jetzt die fehlende reboot()-Methode
    def reboot(self) -> bool:
        """Reboot FluidNC (fire & forget). A network error means success."""
        try:
            url = f"{self.base}/command"
            params = {"plain": "$System/Control=RESTART"}
            r = requests.get(url, params=params, timeout=self.timeout)
            # If response is 'ok', that is already good
            if r.ok:
                txt = r.text.lower()
                if "ok" in txt or "restart" in txt:
                    return True
            return False
        except requests.exceptions.ReadTimeout:
            # Timeout = Reboot erfolgreich (ESP hat Verbindung abgebrochen)
            return True
        except requests.exceptions.ConnectionError:
            # Connection lost = reboot triggered
            return True
        except Exception:
            return False






    # ---- Uploads (firmware + yaml) ----
    def upload_firmware(self, bin_path: str) -> str:
        """
        OTA firmware update.
        Tries common endpoints: /update, /ota, /firmware
        """
        endpoints = ["/update", "/ota", "/firmware"]
        fn = os.path.basename(bin_path)
        with open(bin_path, "rb") as f:
            data = f.read()
        for ep in endpoints:
            try:
                url = f"{self.base}{ep}"
                # Common ArduinoOTA/ESP WebUpdate expects multipart/form-data with field 'firmware' or 'file'
                files = [
                    ("firmware", (fn, data, "application/octet-stream")),
                ]
                r = requests.post(url, files=files, timeout=max(30.0, self.timeout))
                if r.ok:
                    return f"OK via {ep}: {r.text.strip()}"
            except Exception as e:
                last_err = str(e)
        # Fallback: try /update with 'file'
        try:
            url = f"{self.base}/update"
            files = [("file", (fn, data, "application/octet-stream"))]
            r = requests.post(url, files=files, timeout=max(30.0, self.timeout))
            if r.ok:
                return f"OK via /update(file): {r.text.strip()}"
            return f"Upload failed: HTTP {r.status_code} {r.text[:200]}"
        except Exception as e:
            return f"Upload error: {e}"

    def upload_config_yaml(self, yaml_path: str, remote_name: str | None = None) -> str:
        """
        Upload a YAML config to the ESP's local filesystem.
        Compatible with FluidNC 3.53.9.
        """
        fn = remote_name or os.path.basename(yaml_path)
        with open(yaml_path, "rb") as f:
            data = f.read()

        # 1 Modern REST API (FluidNC 3.9+)
        try:
            url = f"{self.base}/api/files/localfs/{urllib.parse.quote(fn)}"
            files = {"file": (fn, data, "text/yaml")}
            r = requests.post(url, files=files, timeout=max(30.0, self.timeout))
            if r.ok:
                return f"OK via /api/files/localfs: {r.text.strip() or 'success'}"
        except Exception as e:
            last_err = str(e)

        # 2 Legacy WebUI /upload
        try:
            url = f"{self.base}/upload"
            files = {"upload": (fn, data, "text/yaml")}
            params = {"path": "/"}
            r = requests.post(url, files=files, params=params, timeout=max(30.0, self.timeout))
            if r.ok:
                return f"OK via /upload: {r.text.strip() or 'success'}"
        except Exception as e:
            last_err = str(e)

        # 3 LittleFS /edit fallback
        try:
            url = f"{self.base}/edit"
            files = {"data": (fn, data, "text/yaml")}
            params = {"path": "/", "name": fn}
            r = requests.post(url, files=files, params=params, timeout=max(30.0, self.timeout))
            if r.ok:
                return f"OK via /edit: {r.text.strip() or 'success'}"
        except Exception as e:
            last_err = str(e)

        return f"Upload failed (no compatible endpoint, last error: {last_err})"





# --------------- $$ Parsing helpers ---------------

AXES = ["X", "Y", "Z", "A", "B", "C"]

SETTINGS_MAP = {
    # steps/mm per axis
    "steps": {"X": 100, "Y": 101, "Z": 102, "A": 103, "B": 104, "C": 105},
    # max rate (mm/min or deg/min)
    "max_rate": {"X": 110, "Y": 111, "Z": 112, "A": 113, "B": 114, "C": 115},
    # accel (mm/s^2 or deg/s^2)
    "accel": {"X": 120, "Y": 121, "Z": 122, "A": 123, "B": 124, "C": 125},
    # max travel (soft limit extents)
    "max_travel": {"X": 130, "Y": 131, "Z": 132, "A": 133, "B": 134, "C": 135},
}

# Common GRBL/FluidNC flags
FLAG_KEYS = {
    "soft_limits": 20,  # 0/1
    "hard_limits": 21,  # 0/1 (may be unsupported)
    "homing": 22,       # 0/1
    "invert_mask": 3,   # bitmask
    "status_report": 10 # variant-specific
}

def parse_dollars(lines: list[str]) -> dict:
    """
    Parse $$ lines into a dict {int_code: float|str}
    Accepts lines like "$100=250.000 (x, step/mm)"
    """
    out = {}
    pat = re.compile(r"^\s*\$([0-9]+)\s*=\s*([^\s]+)")
    for ln in lines:
        m = pat.match(ln)
        if not m:
            continue
        code = int(m.group(1))
        raw = m.group(2)
        try:
            val = float(raw)
        except Exception:
            val = raw
        out[code] = val
    return out


# --------------- Tk Frame ---------------
class FluidNCUpdaterFrame(ttk.Frame):
    def __init__(self, master, default_ip=DEFAULT_IP):
        super().__init__(master)
        self.pack(fill="both", expand=False)
        self.client = FluidNCClient(default_ip)
        self._build_ui()
        self._lock = threading.Lock()

        #  Start auto IP detection after GUI init (main thread -> worker thread)
        self.after(300, self._start_auto_detect)

    # --------------- Auto-detect IP (network only, no UI) ---------------

    def _auto_detect_ip(self) -> str | None:
        """
        First tries DEFAULT_IP, then scans typical subnets for
        a FluidNC device (GET /commandGRIP CLOSEplain=$I).
        Returns found IP as a string or None.
        """

        # 1 Erst die vordefinierte IP direkt testen
        try:
            self.client.set_host(DEFAULT_IP)
            if self.client.is_alive():
                return DEFAULT_IP
        except Exception:
            pass

        # 2 Typische lokale Subnetze (bei Bedarf anpassen/erweitern)
        bases = [
            "192.168.0.",
            "192.168.1.",
            "192.168.2.",
            "192.168.25.",   # dein Bereich
            "192.168.178.",  # Fritz!Box-Standard in DE
        ]

        timeout = 0.5  # slightly more generous than 0.15s

        for base in bases:
            for i in range(1, 255):
                host = f"{base}{i}"
                url = f"http://{host}/command"
                params = {"plain": "$I"}
                try:
                    r = requests.get(url, params=params, timeout=timeout)
                    if not r.ok:
                        continue

                    txt = r.text.strip()
                    # Do not be too strict: any meaningful response counts
                    if txt:
                        return host
                except Exception:
                    continue

        return None

    def _start_auto_detect(self):
        """Start network scan in a background thread and update
        danach die UI im Mainthread via .after(...)."""

        def worker():
            ip = self._auto_detect_ip()

            def apply_result():
                if ip:
                    # UI & Client im Hauptthread aktualisieren
                    self.ip_var.set(ip)
                    self.client.set_host(ip)
                    ok = self.client.is_alive()
                    self._set_connected(ok)
                    if ok:
                        self._ui_info(f"Automatically connected to {ip}")
                    else:
                        self._ui_info(f"IP auto-detected ({ip}), but no response to $I.")
                else:
                    self._ui_info("Automatic IP scan: no device found, using DEFAULT_IP.")

            # Return result to Tk main thread
            self.after(0, apply_result)

        threading.Thread(target=worker, daemon=True).start()

    # --------------- UI-Bau ---------------

    def _build_ui(self):
        # --- Header / Connect ---
        head = ttk.LabelFrame(self, text="Connection")
        head.pack(fill="x", padx=8, pady=6)
        ttk.Label(head, text="Host/IP:").pack(side="left", padx=(6, 4))

        self.ip_var = tk.StringVar(value=self.client.host)
        ttk.Entry(head, textvariable=self.ip_var, width=24).pack(side="left")

        ttk.Button(head, text="Connect / Check", command=self._on_connect).pack(side="left", padx=6)

        self.conn_status = ttk.Label(head, text=" Disconnected", foreground="#b71c1c")
        self.conn_status.pack(side="right", padx=8)

        # --- Notebook ---
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=8, pady=6)

        # Tab $$ / Status
        self.tab_status = ttk.Frame(nb)
        nb.add(self.tab_status, text="$$ Status")

        # Tab Configs
        self.tab_cfg = ttk.Frame(nb)
        nb.add(self.tab_cfg, text="Configs")

        # Tab OTA
        self.tab_ota = ttk.Frame(nb)
        nb.add(self.tab_ota, text="OTA / Uploads")

        self._build_tab_status()
        self._build_tab_configs()
        self._build_tab_ota()

    # ----------------- UI helpers -----------------

    def _set_connected(self, ok: bool):
        self.conn_status.configure(
            text=" Connected" if ok else " Disconnected",
            foreground=("#2e7d32" if ok else "#b71c1c")
        )

    def _with_thread(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()

    def _on_connect(self):
        def work():
            host = self.ip_var.get().strip()
            if not host:
                # UI-Update wieder in den Hauptthread geben
                self.after(0, lambda: self._ui_info("Please enter host/IP."))
                return

            self.client.set_host(host)
            ok = self.client.is_alive()

            def apply():
                self._set_connected(ok)
                self._ui_info("Connected." if ok else "No response.")
            self.after(0, apply)

        self._with_thread(work)

    def _ui_info(self, s: str):
        """Log text to the log text field (if available)."""
        try:
            self.log.insert("end", s + "\n")
            self.log.see("end")
        except Exception:
            # At startup, before log widget creation, silently ignore
            pass



    # --------- Tab: $$ Status ----------
    def _build_tab_status(self):
        top = ttk.Frame(self.tab_status)
        top.pack(fill="x", pady=(2, 2))
        ttk.Button(top, text="Refresh $$", command=self._refresh_dollars).pack(side="left", padx=6)
        self.last_refresh = ttk.Label(top, text="")
        self.last_refresh.pack(side="left")

        # Soft/Hard/Homing flags
        flagsf = ttk.LabelFrame(self.tab_status, text="Machine Limits / Flags")
        flagsf.pack(fill="x", padx=2, pady=2)

        # Correct parent is flagsf, not "parent"
        wrap = ttk.Frame(flagsf)
        wrap.pack(fill="x", padx=2, pady=2)

        self.var_soft   = tk.StringVar(value="Soft Limits: ")
        self.var_hard   = tk.StringVar(value="Hard Limits: ")
        self.var_homing = tk.StringVar(value="Homing: ")
        self.var_invert = tk.StringVar(value="Invert Mask ($3): ")

        ttk.Label(wrap, textvariable=self.var_soft,   width=20, anchor="w").grid(row=0, column=0, padx=4)
        ttk.Label(wrap, textvariable=self.var_hard,   width=20, anchor="w").grid(row=0, column=1, padx=4)
        ttk.Label(wrap, textvariable=self.var_homing, width=20, anchor="w").grid(row=0, column=2, padx=4)
        ttk.Label(wrap, textvariable=self.var_invert, width=25, anchor="w").grid(row=0, column=3, padx=4)


        # Axis table
        tablef = ttk.LabelFrame(self.tab_status, text="Per-Axis Settings")
        tablef.pack(fill="both", expand=True, padx=2, pady=(0,8))

        cols = ("Axis", "Steps/mm", "Max Rate", "Accel", "Max Travel")
        self.tree = ttk.Treeview(tablef, columns=cols, show="headings", height=8)
        for c, w in zip(cols, (70, 110, 110, 110, 110)):
            self.tree.heading(c, text=c)
            self.tree.column(c, width=w, anchor="center")
        self.tree.pack(fill="both", expand=True)

        btns = ttk.Frame(self.tab_status)
        btns.pack(fill="x", padx=2, pady=2)
        ttk.Button(btns, text="Export $$ to file...", command=self._export_dollars).pack(side="left")

    # --------- Tab: Configs ----------
    def _build_tab_configs(self):
        ctrl = ttk.Frame(self.tab_cfg)
        ctrl.pack(fill="x", pady=(2, 2))
        ttk.Button(ctrl, text="List LocalFS", command=self._refresh_files).pack(side="left", padx=6)
        ttk.Button(ctrl, text="Get Active", command=self._get_active).pack(side="left", padx=6)
        ttk.Button(ctrl, text="Set Active from selection", command=self._set_active_from_sel).pack(side="left", padx=6)
        ttk.Button(ctrl, text="Reboot", command=self._reboot).pack(side="left", padx=6)

        info = ttk.Frame(self.tab_cfg)
        info.pack(fill="x", padx=2)
        ttk.Label(info, text="Active config:").pack(side="left")
        self.var_active = tk.StringVar(value="")
        ttk.Label(info, textvariable=self.var_active, foreground="#00695c", font=("Segoe UI", 10, "bold")).pack(side="left", padx=6)

        lf = ttk.LabelFrame(self.tab_cfg, text="Flash files")
        lf.pack(fill="both", expand=True, padx=2, pady=2)
        self.files_list = tk.Listbox(lf, height=10)
        self.files_list.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(lf, orient="vertical", command=self.files_list.yview)
        self.files_list.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")

    # --------- Tab: OTA / Uploads ----------
    def _build_tab_ota(self):
        # Firmware
        fwf = ttk.LabelFrame(self.tab_ota, text="Firmware OTA (.bin)")
        fwf.pack(fill="x", padx=2, pady=(2,2))
        self.var_fw = tk.StringVar(value="")
        ttk.Entry(fwf, textvariable=self.var_fw, width=60).pack(side="left", padx=6, pady=6)
        ttk.Button(fwf, text="Choose...", command=self._pick_fw).pack(side="left", padx=6)
        ttk.Button(fwf, text="Upload Firmware", command=self._upload_fw).pack(side="left", padx=6)

        # YAML
        yaml_f = ttk.LabelFrame(self.tab_ota, text="Config YAML Upload")
        yaml_f.pack(fill="x", padx=2, pady=(2,2))
        self.var_yaml = tk.StringVar(value="")
        ttk.Entry(yaml_f, textvariable=self.var_yaml, width=60).pack(side="left", padx=6, pady=6)
        ttk.Button(yaml_f, text="Choose...", command=self._pick_yaml).pack(side="left", padx=6)
        ttk.Button(yaml_f, text="Upload YAML", command=self._upload_yaml).pack(side="left", padx=6)

        # Log
        logf = ttk.LabelFrame(self.tab_ota, text="Log")
        logf.pack(fill="both", expand=True, padx=2, pady=(2,2))
        self.log = tk.Text(logf, height=2)
        self.log.pack(fill="both", expand=True)

    # ----------------- UI actions -----------------

    def _set_connected(self, ok: bool):
        self.conn_status.configure(text=" Connected" if ok else " Disconnected",
                                   foreground=("#2e7d32" if ok else "#b71c1c"))

    def _with_thread(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()

    def _on_connect(self):
        def work():
            host = self.ip_var.get().strip()
            if not host:
                self._ui_info("Please enter host/IP.")
                return
            self.client.set_host(host)
            ok = self.client.is_alive()
            self._set_connected(ok)
            if ok:
                self._ui_info("Connected.")
            else:
                self._ui_info("No response.")
        self._with_thread(work)

    def _ui_info(self, s):
        try:
            self.log.insert("end", s + "\n")
            self.log.see("end")
        except Exception:
            pass

    def _refresh_dollars(self):
        def work():
            try:
                lines = self.client.get_dollars()
                parsed = parse_dollars(lines)
                self._fill_flags(parsed)
                self._fill_axes(parsed)
                self.last_refresh.config(text=time.strftime("Aktualisiert: %H:%M:%S"))
                self._ui_info("$$ aktualisiert.")
            except Exception as e:
                messagebox.showerror("$$ Error", str(e))
        self._with_thread(work)

    def _fill_flags(self, parsed: dict):
        def flag(code):
            v = parsed.get(code, None)
            return int(v) if isinstance(v, (int, float)) else v

        soft = flag(FLAG_KEYS["soft_limits"])
        hard = flag(FLAG_KEYS["hard_limits"])
        hom = flag(FLAG_KEYS["homing"])
        inv = parsed.get(FLAG_KEYS["invert_mask"], None)

        self.var_soft.set(f"Soft Limits ($20): {'ON' if soft==1 else ('OFF' if soft==0 else '')}")
        if hard in (0,1):
            self.var_hard.set(f"Hard Limits ($21): {'ON' if hard==1 else 'OFF'}")
        else:
            self.var_hard.set("Hard Limits ($21): ")
        self.var_homing.set(f"Homing ($22): {'ON' if hom==1 else ('OFF' if hom==0 else '')}")
        if inv is None:
            self.var_invert.set("Invert Mask ($3): ")
        else:
            try:
                mask = int(inv)
                self.var_invert.set(f"Invert Mask ($3): {mask} (bin {mask:08b})")
            except Exception:
                self.var_invert.set(f"Invert Mask ($3): {inv}")

    def _fill_axes(self, parsed: dict):
        self.tree.delete(*self.tree.get_children())
        def get_val(code):
            v = parsed.get(code, None)
            return f"{v:.3f}" if isinstance(v, (int, float, float)) else (str(v) if v is not None else "")

        for ax in AXES:
            steps = get_val(SETTINGS_MAP["steps"][ax])
            rate  = get_val(SETTINGS_MAP["max_rate"][ax])
            acc   = get_val(SETTINGS_MAP["accel"][ax])
            trav  = get_val(SETTINGS_MAP["max_travel"][ax])
            self.tree.insert("", "end", values=(ax, steps, rate, acc, trav))

    def _export_dollars(self):
        try:
            lines = self.client.get_dollars()
            path = filedialog.asksaveasfilename(defaultextension=".txt",
                filetypes=[("Text", "*.txt"), ("All", "*.*")],
                title="Save $$ dump")
            if not path:
                return
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines))
            messagebox.showinfo("Export", f"Saved: {path}")
        except Exception as e:
            messagebox.showerror("Export Error", str(e))

    def _refresh_files(self):
        def work():
            try:
                files = self.client.list_localfs()
                self.files_list.delete(0, "end")
                for fn in files:
                    fn = fn.split("|")[0].strip()   # <--- NEU
                    self.files_list.insert("end", fn)
                self._ui_info(f"{len(files)} Dateien gelistet.")
            except Exception as e:
                messagebox.showerror("List Error", str(e))
        self._with_thread(work)

    def _get_active(self):
        def work():
            name = self.client.get_active_config()
            self.var_active.set(name or "")
            if name:
                self._ui_info(f"Aktive Config: {name}")
            else:
                self._ui_info("Aktive Config: unbekannt")
        self._with_thread(work)

    def _set_active_from_sel(self):
        sel = self.files_list.curselection()
        if not sel:
            messagebox.showwarning("Set Active", "Please select a file in the list.")
            return
        filename = self.files_list.get(sel[0])
        if not (filename.endswith(".yaml") or filename.endswith(".yml")):
            if not messagebox.askyesno("Confirm", f"'{filename}' is not a YAML file. Set it as active config anywayGRIP CLOSE"):
                return
        def work():
            try:
                resp = self.client.set_active_config(filename)
                self._ui_info(f"SetActive  {resp}")
                self.var_active.set(filename)
                time.sleep(1.0)
                ok = self.client.reboot()
                self._ui_info("Reboot command sent." if ok else "Reboot failed.")
            except Exception as e:
                messagebox.showerror("Set Active Error", str(e))
        self._with_thread(work)
        
    def _reboot(self) -> bool:
        cmds = [
            "$System/Control=RESTART",
            "$Restart",
            "$Reset",
        ]
        for cmd in cmds:
            try:
                resp = self.client.command(cmd)
                if "ok" in resp.lower() or "restart" in resp.lower():
                    return True
            except:
                continue
        return False





    def _pick_fw(self):
        path = filedialog.askopenfilename(filetypes=[("Firmware BIN", "*.bin"), ("All", "*.*")])
        if path:
            self.var_fw.set(path)

    def _upload_fw(self):
        path = self.var_fw.get().strip()
        if not path or not os.path.exists(path):
            messagebox.showwarning("Firmware", "Please select a .bin file.")
            return
        def work():
            try:
                msg = self.client.upload_firmware(path)
                self._ui_info(msg)
                messagebox.showinfo("Firmware", msg)
            except Exception as e:
                messagebox.showerror("Firmware Upload Error", str(e))
        self._with_thread(work)

    def _pick_yaml(self):
        path = filedialog.askopenfilename(filetypes=[("YAML", "*.yaml *.yml"), ("All", "*.*")])
        if path:
            self.var_yaml.set(path)

    def _upload_yaml(self):
        path = self.var_yaml.get().strip()
        if not path or not os.path.exists(path):
            messagebox.showwarning("YAML", "Please select a YAML file.")
            return
        def work():
            try:
                msg = self.client.upload_config_yaml(path)
                self._ui_info(msg)
                messagebox.showinfo("YAML Upload", msg)
            except Exception as e:
                messagebox.showerror("YAML Upload Error", str(e))
        self._with_thread(work)


# --------------- Standalone launcher ---------------

if __name__ == "__main__":
    root = tk.Tk()
    root.title("FluidNC Updater / $$ Inspector / Config Manager")
    try:
        style = ttk.Style()
        style.theme_use("clam")
    except Exception:
        pass
    app = FluidNCUpdaterFrame(root, default_ip=DEFAULT_IP)
    root.geometry("880x400")
    root.mainloop()
