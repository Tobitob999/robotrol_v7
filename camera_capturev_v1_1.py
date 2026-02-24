# ============================================================================================
#  Camera Capture Module v1.1
# Logitech/USB camera integration for RoboControl
# ============================================================================================
# Funktionen:
#    OpenCV-basierter Video-Stream (Threaded)
#    Live-Preview in Tkinter (auch als separater Tab)
#    Access to latest frames (for chessboard detection)
#     Camera detection + selection menu (internal/external)
# ============================================================================================

import cv2
import threading
import time
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk


class CameraCapture:
    def __init__(
        self,
        master=None,
        camera_index=0,
        width=640,
        height=480,
        fps=30,
        preview_width=300,
        preview_height=225,
    ):
        self.master = master
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.preview_width = preview_width
        self.preview_height = preview_height
        self.frame = None
        self.running = False

        # --- UI: frame + image label ---
        self.ui_frame = ttk.Frame(master)

        #  NEW: camera selection dropdown
        self.combo_var = tk.StringVar()
        self.available_cameras = self._list_cameras()
        self.camera_map = {f"Camera {i}": i for i in self.available_cameras}

        combo_label = ttk.Label(self.ui_frame, text=" Select camera:")
        combo_label.pack(pady=(5, 0))
        self.combo_box = ttk.Combobox(
            self.ui_frame, textvariable=self.combo_var, values=list(self.camera_map.keys()), state="readonly"
        )
        self.combo_box.pack(pady=(0, 5))

        if self.available_cameras:
            if self.camera_index in self.available_cameras:
                idx = self.available_cameras.index(self.camera_index)
            else:
                idx = 0
                self.camera_index = self.available_cameras[0]
            self.combo_box.current(idx)

        self.combo_box.bind("<<ComboboxSelected>>", self._on_camera_change)
        #  ENDE

        self.label = ttk.Label(self.ui_frame)
        self.label.pack(padx=4, pady=4)

        # --- Threading ---
        self.capture_thread = None
        self._lock = threading.Lock()

    # ============================================================
    #  Detect camera list
    # ============================================================
    def _list_cameras(self, max_index=6):
        found = []
        for i in range(max_index):
            cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
            if cap.isOpened():
                found.append(i)
                cap.release()
        return found
    #  ENDE

    # ============================================================
    #  Camera switch
    # ============================================================
    def _on_camera_change(self, event=None):
        if self.running:
            self.stop()
        cam_name = self.combo_var.get()
        self.camera_index = self.camera_map.get(cam_name, 0)
        self.start()
    #  ENDE

    # ============================================================
    #  Start / stop
    # ============================================================
    def start(self):
        if self.running:
            return
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.running = True
        self.capture_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.capture_thread.start()
        self._tk_update()

    def stop(self):
        self.running = False
        time.sleep(0.1)
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()

    # ============================================================
    #  Capture Loop
    # ============================================================
    def _update_loop(self):
        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                with self._lock:
                    self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            time.sleep(1.0 / self.fps)

    # ============================================================
    #  Update Tkinter Preview
    # ============================================================
    def _tk_update(self):
        if not self.running:
            return
        with self._lock:
            if self.frame is not None:
                img = Image.fromarray(self.frame)
                if self.preview_width > 0 and self.preview_height > 0:
                    resampling = getattr(Image, "Resampling", Image)
                    img = img.resize((int(self.preview_width), int(self.preview_height)), resampling.LANCZOS)
                imgtk = ImageTk.PhotoImage(image=img)
                self.label.imgtk = imgtk
                self.label.configure(image=imgtk)
        self.master.after(33, self._tk_update)  # ~30 FPS UI Refresh

    # ============================================================
    #  Access latest frame (e.g., for board detection)
    # ============================================================
    def get_latest_frame(self):
        with self._lock:
            return None if self.frame is None else self.frame.copy()

    # ============================================================
    #  UI-Helfer
    # ============================================================
    def get_frame(self):
        return self.ui_frame


# ============================================================================================
#  Test mode - start standalone
# ============================================================================================
if __name__ == "__main__":
    root = tk.Tk()
    root.title(" Vision  Camera Preview")
    cam = CameraCapture(root)
    cam.get_frame().pack()
    cam.start()

    def on_close():
        cam.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
