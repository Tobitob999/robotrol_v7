# ============================================================================================
#  Board Pose Module v1.0
# 3D pose estimation (solvePnP) + status display below camera image
# ============================================================================================

import cv2
import json
import numpy as np
import os
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import ttk
from perception.tnt_detector import detect_tnt_contour


class BoardPose:
    def __init__(
        self,
        master,
        camera,
        pattern_size=(7, 7),
        square_size_mm=30.0,
        preview_width=300,
        preview_height=225,
    ):
        self.master = master
        self.camera = camera
        self.pattern_size = pattern_size
        self.square_size = square_size_mm
        self.preview_width = int(preview_width) if preview_width else 0
        self.preview_height = int(preview_height) if preview_height else 0
        self.running = True
        self._tnt_cfg = None
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            cfg_path = os.path.join(base_dir, "configs", "tnt.json")
            with open(cfg_path, "r", encoding="utf-8-sig") as f:
                self._tnt_cfg = json.load(f)
        except Exception:
            self._tnt_cfg = None

        # If no camera instance was provided, create one locally.
        if camera is None:
            from camera_capturev_v1_1 import CameraCapture
            self.camera = CameraCapture(master, width=640, height=480)
        else:
            self.camera = camera

        self.last_H = None
        self.last_rvec = None
        self.last_tvec = None
        self.last_vis = None
        self.status = "none"

        # =============== UI Frames ===============
        self.ui_frame = ttk.Frame(master)

        # Camera image
        self.label_img = ttk.Label(self.ui_frame)
        self.label_img.pack(padx=4, pady=(4, 0))

        # Status section
        status_frame = ttk.Frame(self.ui_frame)
        status_frame.pack(fill="x", pady=(2, 2))

        self.indicators = {}
        for name, color in [("Detected", "green"), ("Lost", "orange"), ("Not detected", "red")]:
            f = ttk.Frame(status_frame, width=60)
            f.pack(side=tk.LEFT, padx=5)
            c = tk.Canvas(f, width=16, height=16, highlightthickness=0)
            oval = c.create_oval(2, 2, 14, 14, fill="#555555", outline="")
            c.pack(side=tk.LEFT)
            ttk.Label(f, text=name).pack(side=tk.LEFT, padx=3)
            self.indicators[name] = (c, oval, color)

        # Pose display
        self.pose_text = tk.StringVar(value="Pose: -")
        ttk.Label(self.ui_frame, textvariable=self.pose_text, font=("Consolas", 9)).pack(fill="x", padx=6, pady=(2, 6))

        # --- Control buttons for live feed ---
        ctrlf = ttk.Frame(self.ui_frame)
        ctrlf.pack(fill="x", pady=(0, 6))

        self.paused = False
        ttk.Button(ctrlf, text="Resume", width=10, command=self.resume_feed).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrlf, text="Stop", width=10, command=self.stop).pack(side=tk.LEFT, padx=4)


        self._update_ui()

    # ============================================================
    def start_camera(self):
        if hasattr(self, "camera") and self.camera:
            try:
                self.camera.start()
                self.running = True
                self._update_ui()
            except Exception as e:
                print("[Vision start error]", e)

    def stop_camera(self):
        if hasattr(self, "camera") and self.camera:
            try:
                self.camera.stop()
                self.running = False
            except Exception as e:
                print("[Vision stop error]", e)


    # ============================================================
    # Main update loop
    # ============================================================
    def _update_ui(self):
        if not self.running:
            return

        if self.paused:
            # paused: do not fetch new frames
            self.master.after(200, self._update_ui)
            return

        frame = self.camera.get_latest_frame()
        if frame is None:
            self.master.after(100, self._update_ui)
            return

        vis = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        # ---------- Chessboard Detection ----------
        try:
            found, corners = cv2.findChessboardCornersSB(
                gray, self.pattern_size,
                flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
            )
        except Exception:
            found, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

        rvec, tvec = None, None
        tracking_lost = False
        offset = -self.square_size / 1.0  # inherited from v2.4

        if found:
            # Subpixel-Refinement
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Objektpunkte
            objp = np.zeros((np.prod(self.pattern_size), 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
            objp *= self.square_size

            # solvePnP -> camera pose
            try:
                ok, rvec, tvec = cv2.solvePnP(objp, corners2, self._camera_matrix(gray), None)
                if ok:
                    self.last_rvec, self.last_tvec = rvec, tvec
            except Exception:
                pass

            txt = " Chessboard detected (pose active)"
            color = (0, 255, 0)
            self.status = "ok"
            self.last_H = self._estimate_homography(objp[:, :2], corners2)
        else:
            if self.last_H is not None:
                txt = " Tracking lost  cached pose"
                color = (0, 255, 255)
                tracking_lost = True
                self.status = "lost"
                rvec, tvec = self.last_rvec, self.last_tvec
            else:
                txt = " No chessboard detected"
                color = (0, 100, 255)
                self.status = "none"

        # ---------- Overlay ----------
#        cv2.putText(vis, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if self._tnt_cfg is not None:
            try:
                contour, _ = detect_tnt_contour(frame, self._tnt_cfg, camera=self.camera)
                if contour is not None:
                    cv2.drawContours(vis, [contour], -1, (0, 0, 255), 2)
            except Exception:
                pass
        if self.last_H is not None:
            self._draw_virtual_board(vis, self.last_H, offset)
        self.last_vis = vis

        # ---------- Pose display ----------
        if rvec is not None and tvec is not None:
            R, _ = cv2.Rodrigues(rvec)
            yaw, pitch, roll = self._rotation_to_euler(R)
            tx, ty, tz = tvec.flatten()
            self.pose_text.set(
                f"Pose  X={tx:7.1f} mm  Y={ty:7.1f} mm  Z={tz:7.1f} mm   "
                f"Yaw={yaw:6.1f}  Pitch={pitch:6.1f}  Roll={roll:6.1f}"
            )

        self._update_indicators()

        # ---------- Anzeige ----------
        display = vis
        if self.preview_width > 0 and self.preview_height > 0:
            h, w = vis.shape[:2]
            if h > 0 and w > 0:
                scale = min(self.preview_width / float(w), self.preview_height / float(h))
                new_w = max(1, int(round(w * scale)))
                new_h = max(1, int(round(h * scale)))
                if new_w != w or new_h != h:
                    display = cv2.resize(vis, (new_w, new_h), interpolation=cv2.INTER_AREA)

        img = Image.fromarray(display)
        imgtk = ImageTk.PhotoImage(image=img)
        self.label_img.imgtk = imgtk
        self.label_img.configure(image=imgtk)

        self.master.after(80, self._update_ui)

    # ============================================================
    # Utilities
    # ============================================================
    def _camera_matrix(self, gray):
        """Assume a simple pinhole camera with f  800 px."""
        h, w = gray.shape[:2]
        f = 0.8 * w
        cx, cy = w / 2, h / 2
        return np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float32)

    def _estimate_homography(self, objp, corners):
        return cv2.findHomography(objp, corners)[0]

    def _rotation_to_euler(self, R):
        """Convert rotation matrix to Euler angles (yaw, pitch, roll)."""
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            yaw = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
            pitch = np.degrees(np.arctan2(-R[2, 0], sy))
            roll = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        else:
            yaw = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
            pitch = np.degrees(np.arctan2(-R[2, 0], sy))
            roll = 0
        return yaw, pitch, roll

    # ============================================================
    # Draw virtual board overlay
    # ============================================================
    def _draw_virtual_board(self, img, H, offset):
        fields = 8
        step = self.square_size
        for i in range(fields + 1):
            p1 = np.array([[offset, i * step + offset]], dtype=np.float32)
            p2 = np.array([[fields * step + offset, i * step + offset]], dtype=np.float32)
            pts = cv2.perspectiveTransform(np.array([p1, p2]), H)
            cv2.line(img, tuple(pts[0][0].astype(int)), tuple(pts[1][0].astype(int)), (0, 255, 100), 2)

        for j in range(fields + 1):
            p1 = np.array([[j * step + offset, offset]], dtype=np.float32)
            p2 = np.array([[j * step + offset, fields * step + offset]], dtype=np.float32)
            pts = cv2.perspectiveTransform(np.array([p1, p2]), H)
            cv2.line(img, tuple(pts[0][0].astype(int)), tuple(pts[1][0].astype(int)), (0, 255, 100), 2)

    # ============================================================
    # Status indicators
    # ============================================================
    def _update_indicators(self):
        for name, (c, oval, color) in self.indicators.items():
            c.itemconfig(oval, fill="#444444")  # Reset

        if self.status == "ok":
            self.indicators["Detected"][0].itemconfig(self.indicators["Detected"][1], fill="limegreen")
        elif self.status == "lost":
            self.indicators["Lost"][0].itemconfig(self.indicators["Lost"][1], fill="gold")
        else:
            self.indicators["Not detected"][0].itemconfig(self.indicators["Not detected"][1], fill="red")

    # ============================================================
    # Query helper
    # ============================================================
    def get_field_position(self, field_name: str):
        """
        Return world coordinates (x, y, z) for a chess square.
        Square label like 'E2' -> center point coordinate.
        """
        if self.last_H is None:
            return None

        col = ord(field_name[0].upper()) - ord('A')
        row = 8 - int(field_name[1])
        step = self.square_size
        p = np.array([[col * step + step / 2, row * step + step / 2]], dtype=np.float32)
        dst = cv2.perspectiveTransform(np.array([p]), self.last_H)[0][0]
        return tuple(dst)

    # ============================================================
    # Control
    # ============================================================
    def get_frame(self):
        return self.ui_frame

    def set_tnt_cfg(self, cfg: dict):
        if isinstance(cfg, dict):
            self._tnt_cfg = cfg


    def pause_feed(self):
        """Pause live feed (keep current frame)."""
        self.paused = True
        self.pose_text.set(" Live feed paused")

    def resume_feed(self):
        """Resume live feed while ensuring only one loop is active."""
        if not self.running:
            # if previously stopped: restart
            self.running = True

        if not self.paused:
            return  # already active, avoid double start
        self.paused = False
        self.pose_text.set(" Live feed active")

        # Keep only one loop active: start after a short delay to avoid
        # duplicate .after() cycles.
        self.master.after(100, self._resume_safe)

    def _resume_safe(self):
        """Helper: verify loop may continue safely."""
        if self.running and not self.paused:
            self._update_ui()

    def stop(self):
        """Stop vision entirely."""
        self.running = False
        self.paused = True
        self.pose_text.set(" Vision stopped")



# ============================================================================================
# Test mode
# ============================================================================================
if __name__ == "__main__":
    from camera_capturev_v1_1 import CameraCapture
    root = tk.Tk()
    root.title(" Board Pose v1.0 - 3D Pose + Status")

    cam = CameraCapture(root, width=640, height=480)
    cam.start()

    bd = BoardPose(root, cam)
    bd.get_frame().pack(padx=4, pady=4)

    def on_close():
        bd.stop()
        cam.stop()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

# ============================================================================================
# BoardDetectorCalibration v1.0
# Camera-robot calibration (chessboard alignment)
# ============================================================================================
class BoardDetectorCalibration(ttk.Frame):
    def __init__(self, master, board_detector_pose, execute_app):
        super().__init__(master)
        self.pack(fill="both", expand=True)
        self.board_detector_pose = board_detector_pose
        self.execute_app = execute_app
        self.points_cam = []
        self.points_robot = []
        self.R = None
        self.t = None
        self._build_ui()

    def _build_ui(self):
        ttk.Label(self, text=" Camera-Robot Calibration",
                  font=("Segoe UI", 11, "bold")).pack(pady=8)
        btnf = ttk.Frame(self)
        btnf.pack(pady=6)
        for f in ["A1", "H1", "A8", "H8"]:
            ttk.Button(btnf, text=f"Capture {f}",
                       command=lambda f=f: self.capture_point(f)).pack(side=tk.LEFT, padx=4)

        ttk.Button(self, text=" Calibrate (compute R|t)",
                   command=self.compute_transform).pack(pady=8)
        ttk.Button(self, text=" Save",
                   command=self.save_transform).pack(pady=4)

        self.text_out = tk.Text(self, height=10, width=90, font=("Consolas", 9))
        self.text_out.pack(padx=8, pady=6, fill="both", expand=True)
        self._log("No points captured yet.\n")

    def _log(self, msg):
        self.text_out.insert("end", msg + "\n")
        self.text_out.see("end")

    def capture_point(self, field):
        try:
            p_cam = np.array(self.board_detector_pose.get_field_position(field), dtype=float)
            if len(p_cam) == 2:
                p_cam = np.append(p_cam, 0.0)
            p_tcp = self.execute_app.get_current_tcp_mm()
            p_robot = np.array([p_tcp["X_mm"], p_tcp["Y_mm"], p_tcp["Z_mm"]], dtype=float)
            self.points_cam.append(p_cam)
            self.points_robot.append(p_robot)
            self._log(f"Captured {field}: CAM {np.round(p_cam,1)} -> ROBOT {np.round(p_robot,1)}")
        except Exception as e:
            messagebox.showerror("Capture Error", str(e))
            self._log(f"[] Capture failed: {e}")

    def compute_transform(self):
        if len(self.points_cam) < 3:
            messagebox.showwarning("Notice", "At least 3 points are required.")
            return
        try:
            pts_cam = np.array(self.points_cam)[:, :2]
            pts_robot = np.array(self.points_robot)[:, :2]
            H, mask = cv2.estimateAffine2D(pts_cam, pts_robot)
            if H is None:
                raise ValueError("Affine2D could not be computed.")
            R = np.eye(3)
            R[:2, :2] = H[:2, :2]
            t = np.zeros(3)
            t[:2] = H[:, 2]
            self.R, self.t = R, t
            pts_est = (pts_cam @ H[:2, :2].T) + H[:, 2]
            rmse = np.mean(np.linalg.norm(pts_est - pts_robot, axis=1))
            self._log(f" Calibration successful! RMSE={rmse:.2f} mm\nR=\n{np.round(R,3)}\nt={np.round(t,2)}")
        except Exception as e:
            messagebox.showerror("Calibration Error", str(e))
            self._log(f"[] Calibration failed: {e}")

    def save_transform(self):
        if self.R is None or self.t is None:
            messagebox.showwarning("Notice", "No valid calibration has been computed yet.")
            return
        data = {"R": self.R.tolist(), "t": self.t.tolist()}
        if hasattr(self.execute_app, "save_cam_to_base"):
            ok, msg = self.execute_app.save_cam_to_base(data)
            if not ok:
                messagebox.showerror("Error", msg)
                self._log(f"[Error] {msg}")
                return
            messagebox.showinfo("Saved", msg)
            self._log(msg)
        else:
            with open("cam_to_base.json", "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Saved", " cam_to_base.json created.")
            self._log(" cam_to_base.json saved.")
