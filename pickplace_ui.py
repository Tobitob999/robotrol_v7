import json
import math
import os
import threading
import time
import tkinter as tk
from tkinter import ttk

try:
    import autocalib_v1 as _autocalib
    _AUTOCALIB_AVAILABLE = True
except ImportError:
    _AUTOCALIB_AVAILABLE = False

from control import config as config_loader
from control.app import build_pipeline
from learning.tnt_self_learning import TntSelfLearningManager
from control.transforms import (
    make_transform,
    matmul,
    invert_transform,
    normalize_vector,
    cross,
    extract_translation,
)
from perception.camera import Frame
from simulation.simulation_loop import SimulationRunner


class _CameraCaptureAdapter:
    def __init__(self, camera_capture):
        self._camera_capture = camera_capture
        self.color_order = "rgb"

    def get_frame(self):
        image = None
        if self._camera_capture is not None:
            image = self._camera_capture.get_latest_frame()
        if image is None:
            raise RuntimeError("No camera frame available")
        return Frame(image=image, timestamp=time.time())


class PickPlaceTab(ttk.Frame):
    def __init__(self, master, logger=None, camera_capture=None, execute_app=None):
        super().__init__(master)
        self._logger = logger
        self._pipeline = None
        self._configs = None
        self._worker = None
        self._stop_event = threading.Event()
        self._base_dir = os.path.abspath(os.path.dirname(__file__))
        self._camera_capture = camera_capture

        self._status_var = tk.StringVar(value="Not initialized")
        self._stack_index_var = tk.StringVar(value="Stack index: n/a")
        self._run_count = tk.IntVar(value=1)
        self._sim_cycles = tk.IntVar(value=1000)
        self._detect_var = tk.StringVar(value="No detection yet.")
        self._execute_app = execute_app
        self._learner = None
        self._learning_enabled_var = tk.BooleanVar(value=False)
        self._learning_mode_var = tk.StringVar(value="shadow")
        self._learning_status_var = tk.StringVar(value="Learning: disabled")
        self._learning_event_count = 0
        self._last_cycle_ctx = {}

        self._pattern_cols_var = tk.IntVar(value=7)
        self._pattern_rows_var = tk.IntVar(value=7)
        self._square_size_var = tk.DoubleVar(value=32.0)
        self._marker_axis_var = tk.StringVar(value="z")
        self._marker_sign_var = tk.IntVar(value=1)
        self._tnt_h1_low_var = tk.IntVar(value=0)
        self._tnt_h1_high_var = tk.IntVar(value=12)
        self._tnt_h2_low_var = tk.IntVar(value=168)
        self._tnt_h2_high_var = tk.IntVar(value=180)
        self._tnt_s_min_var = tk.IntVar(value=40)
        self._tnt_v_min_var = tk.IntVar(value=30)
        self._tnt_min_area_var = tk.IntVar(value=500)
        self._tnt_kernel_var = tk.IntVar(value=3)

        self._test_pitch_adapt_var = tk.BooleanVar(value=False)
        self._test_max_pitch_var = tk.DoubleVar(value=35.0)

        # Pick & Place Test
        self._ppt_from_sq_var = tk.StringVar(value="e4")
        self._ppt_to_sq_var = tk.StringVar(value="d5")
        self._ppt_z_grasp_var = tk.DoubleVar(value=5.0)
        self._ppt_open_s_var = tk.IntVar(value=0)
        self._ppt_close_s_var = tk.IntVar(value=1000)
        self._ppt_pause_ms_var = tk.IntVar(value=500)

        self._base_T_board_vars = None
        self._base_T_cam_vars = None
        self._marker_T_obj_vars = None
        self._board_points = {}
        self._board_point_vars = {}
        self._board_square_points = {}
        self._cam_calib_objpoints = []
        self._cam_calib_imgpoints = []
        self._cam_calib_image_size = None
        self._cam_calib_pattern = None
        self._cam_calib_square = None
        self._cam_calib_result = None
        self._cam_calib_status = tk.StringVar(value="No samples")
        self._cam_calib_result_var = tk.StringVar(value="")

        self._build_ui()
        self._refresh_learning_from_manager()
        self._load_calibration_into_ui()

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky="ew", padx=6, pady=4)
        header.columnconfigure(3, weight=1)

        ttk.Button(header, text="Init Pipeline", command=self._init_pipeline).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(header, text="Stop", command=self._stop).grid(row=0, column=1, padx=(0, 6))
        ttk.Label(header, text="Status:").grid(row=0, column=2, sticky="w")
        ttk.Label(header, textvariable=self._status_var).grid(row=0, column=3, sticky="w")

        tabs = ttk.Notebook(self)
        # keep reference for diagnostics/fixes
        self._tabs = tabs
        # Limit notebook height so the Pick & Place tab does not force the main
        # window to grow so large that lower UI buttons become invisible.
        # 600px is a reasonable default; adjust if your screen is very small.
        try:
            tabs.configure(height=600)
        except Exception:
            pass
        # schedule a diagnostic check shortly after the UI is mapped
        try:
            self.after(150, self._dump_layout_sizes)
        except Exception:
            pass
        tabs.grid(row=1, column=0, sticky="nsew", padx=6, pady=4)
        self.rowconfigure(1, weight=1)

        run_tab = ttk.Frame(tabs)
        sim_tab = ttk.Frame(tabs)
        cfg_tab = ttk.Frame(tabs)
        calib_tab = ttk.Frame(tabs)
        help_tab = ttk.Frame(tabs)
        tabs.add(run_tab, text="Run")
        tabs.add(sim_tab, text="Simulation")
        tabs.add(cfg_tab, text="Config")
        tabs.add(calib_tab, text="Calibration")
        tabs.add(help_tab, text="Help")

        self._build_run_tab(run_tab)
        self._build_sim_tab(sim_tab)
        self._build_cfg_tab(cfg_tab)
        self._build_calib_tab(calib_tab)
        self._build_help_tab(help_tab)

    def _build_run_tab(self, tab):
        tab.columnconfigure(1, weight=1)
        btns = ttk.Frame(tab)
        btns.grid(row=0, column=0, sticky="w", padx=6, pady=6)

        ttk.Button(btns, text="Run 1 Cycle", command=self._run_one_cycle).grid(row=0, column=0, padx=(0, 6))
        ttk.Label(btns, text="Cycles:").grid(row=0, column=1, padx=(0, 4))
        ttk.Entry(btns, textvariable=self._run_count, width=6).grid(row=0, column=2, padx=(0, 6))
        ttk.Button(btns, text="Run N Cycles", command=self._run_n_cycles).grid(row=0, column=3)
        ttk.Button(btns, text="Reset Stack", command=self._reset_stack_index).grid(row=0, column=4, padx=(6, 0))
        ttk.Label(btns, textvariable=self._stack_index_var).grid(row=0, column=5, padx=(10, 0), sticky="w")

        learn = ttk.LabelFrame(tab, text="Self-Learning (TNT)")
        learn.grid(row=1, column=0, sticky="ew", padx=6, pady=(0, 6))
        ttk.Checkbutton(
            learn,
            text="Enable",
            variable=self._learning_enabled_var,
            command=self._on_learning_toggle,
        ).grid(row=0, column=0, padx=6, pady=4, sticky="w")
        ttk.Label(learn, text="Mode:").grid(row=0, column=1, padx=(6, 2), pady=4, sticky="w")
        mode = ttk.Combobox(learn, textvariable=self._learning_mode_var, values=["shadow", "active"], width=8, state="readonly")
        mode.grid(row=0, column=2, padx=2, pady=4, sticky="w")
        mode.bind("<<ComboboxSelected>>", lambda _e: self._on_learning_mode_change())
        ttk.Button(learn, text="Reset Policy", command=self._reset_learning_policy).grid(row=0, column=3, padx=6, pady=4)
        ttk.Button(learn, text="Show Status", command=self._show_learning_status).grid(row=0, column=4, padx=6, pady=4)
        ttk.Label(learn, textvariable=self._learning_status_var).grid(row=0, column=5, padx=6, pady=4, sticky="w")
        learn.columnconfigure(5, weight=1)

        log_frame = ttk.LabelFrame(tab, text="Pick & Place Log")
        log_frame.grid(row=2, column=0, sticky="nsew", padx=6, pady=6)
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self._log_text = tk.Text(log_frame, height=12, wrap="word")
        self._log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self._log_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self._log_text.configure(yscrollcommand=scrollbar.set)

        tab.rowconfigure(2, weight=1)

    def _build_sim_tab(self, tab):
        tab.columnconfigure(1, weight=1)
        row = ttk.Frame(tab)
        row.grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Label(row, text="Cycles:").grid(row=0, column=0, padx=(0, 4))
        ttk.Entry(row, textvariable=self._sim_cycles, width=8).grid(row=0, column=1, padx=(0, 6))
        ttk.Button(row, text="Run Simulation", command=self._run_simulation).grid(row=0, column=2)

    def _build_cfg_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        self._cfg_text = tk.Text(tab, height=18, wrap="none")
        self._cfg_text.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

    def _build_calib_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        tabs = ttk.Notebook(tab)
        tabs.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        camera_tab = ttk.Frame(tabs)
        base_tab = ttk.Frame(tabs)
        marker_tab = ttk.Frame(tabs)
        detect_tab = ttk.Frame(tabs)
        autocalib_tab = ttk.Frame(tabs)
        tabs.add(camera_tab, text="Camera")
        tabs.add(base_tab, text="Base-Cam")
        tabs.add(marker_tab, text="Marker-Obj")
        tabs.add(detect_tab, text="Perception")
        tabs.add(autocalib_tab, text="Auto-Calib")

        # Base-Cam has many stacked frames – wrap in scrollable canvas so it
        # doesn't force the outer Notebook (and main window) to grow vertically.
        base_tab.columnconfigure(0, weight=1)
        base_tab.rowconfigure(0, weight=1)
        _bc_canvas = tk.Canvas(base_tab, highlightthickness=0, height=420)
        _bc_vsb = ttk.Scrollbar(base_tab, orient="vertical", command=_bc_canvas.yview)
        _bc_canvas.configure(yscrollcommand=_bc_vsb.set)
        _bc_vsb.grid(row=0, column=1, sticky="ns")
        _bc_canvas.grid(row=0, column=0, sticky="nsew")
        base_inner = ttk.Frame(_bc_canvas)
        _bc_win = _bc_canvas.create_window((0, 0), window=base_inner, anchor="nw")
        base_inner.bind(
            "<Configure>",
            lambda e: _bc_canvas.configure(scrollregion=_bc_canvas.bbox("all")),
        )
        _bc_canvas.bind(
            "<Configure>",
            lambda e: _bc_canvas.itemconfigure(_bc_win, width=e.width),
        )
        _bc_canvas.bind(
            "<MouseWheel>",
            lambda e: _bc_canvas.yview_scroll(int(-1 * (e.delta / 120)), "units"),
        )

        self._build_base_cam_tab(base_inner)
        self._build_camera_tab(camera_tab)
        self._build_marker_tab(marker_tab)
        self._build_detect_tab(detect_tab)
        self._build_autocalib_tab(autocalib_tab)

    def _build_base_cam_tab(self, tab):
        info = ttk.Label(tab, text="Compute base_T_cam using a chessboard and a known base_T_board.")
        info.pack(anchor="w", padx=6, pady=(6, 2))

        params = ttk.Frame(tab)
        params.pack(anchor="w", padx=6, pady=4)
        ttk.Label(params, text="Pattern cols:").grid(row=0, column=0, padx=4, pady=2, sticky="w")
        ttk.Entry(params, textvariable=self._pattern_cols_var, width=6).grid(row=0, column=1, padx=4, pady=2)
        ttk.Label(params, text="Pattern rows:").grid(row=0, column=2, padx=4, pady=2, sticky="w")
        ttk.Entry(params, textvariable=self._pattern_rows_var, width=6).grid(row=0, column=3, padx=4, pady=2)
        ttk.Label(params, text="Square size (mm):").grid(row=0, column=4, padx=4, pady=2, sticky="w")
        ttk.Entry(params, textvariable=self._square_size_var, width=8).grid(row=0, column=5, padx=4, pady=2)

        btns = ttk.Frame(tab)
        btns.pack(anchor="w", padx=6, pady=4)
        btns_r1 = ttk.Frame(btns)
        btns_r1.pack(anchor="w")
        ttk.Button(btns_r1, text="Load Config", command=self._load_calibration_into_ui).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns_r1, text="Save Board Config", command=self._save_board_config).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns_r1, text="Compute base_T_cam", command=self._compute_base_T_cam).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns_r1, text="Save base_T_cam", command=self._save_base_T_cam).pack(side=tk.LEFT, padx=4)
        btns_r2 = ttk.Frame(btns)
        btns_r2.pack(anchor="w", pady=(2, 0))
        ttk.Button(btns_r2, text="Validate base_T_cam", command=self._validate_base_T_cam_once).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns_r2, text="Simulate calibration", command=self._simulate_base_cam_noise).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns_r2, text="Test Board Moves", command=self._test_board_moves).pack(side=tk.LEFT, padx=4)

        grid = ttk.Frame(tab)
        grid.pack(fill="x", padx=6, pady=6)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        board_frame, self._base_T_board_vars = self._make_matrix_grid(grid, "base_T_board", readonly=False)
        cam_frame, self._base_T_cam_vars = self._make_matrix_grid(grid, "base_T_cam", readonly=True)
        board_frame.grid(row=0, column=0, sticky="n", padx=4)
        cam_frame.grid(row=0, column=1, sticky="n", padx=4)

        capture = ttk.LabelFrame(tab, text="Board Corner Capture (TCP)")
        capture.pack(fill="x", padx=6, pady=6)
        ttk.Label(
            capture,
            text=(
                "Capture three inner corners: P0=(0,0), P1=(cols-1,0), P2=(0,rows-1). "
                "Use the same corner order as the camera view."
            ),
        ).pack(anchor="w", padx=6, pady=(4, 2))

        btn_row = ttk.Frame(capture)
        btn_row.pack(anchor="w", padx=6, pady=2)
        state = "normal" if self._execute_app is not None else "disabled"
        ttk.Button(btn_row, text="Capture P0 (0,0)", command=lambda: self._capture_board_point("p0"), state=state).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(btn_row, text="Capture P1 (X+)", command=lambda: self._capture_board_point("p1"), state=state).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(btn_row, text="Capture P2 (Y+)", command=lambda: self._capture_board_point("p2"), state=state).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(btn_row, text="Compute base_T_board", command=self._compute_base_T_board_from_points).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(btn_row, text="Clear Points", command=self._clear_board_points).pack(side=tk.LEFT, padx=4)

        points = ttk.Frame(capture)
        points.pack(anchor="w", padx=6, pady=(2, 6))
        self._board_point_vars = {
            "p0": tk.StringVar(value="not set"),
            "p1": tk.StringVar(value="not set"),
            "p2": tk.StringVar(value="not set"),
        }
        ttk.Label(points, text="P0:").grid(row=0, column=0, sticky="w", padx=(0, 4))
        ttk.Label(points, textvariable=self._board_point_vars["p0"]).grid(row=0, column=1, sticky="w")
        ttk.Label(points, text="P1:").grid(row=1, column=0, sticky="w", padx=(0, 4))
        ttk.Label(points, textvariable=self._board_point_vars["p1"]).grid(row=1, column=1, sticky="w")
        ttk.Label(points, text="P2:").grid(row=2, column=0, sticky="w", padx=(0, 4))
        ttk.Label(points, textvariable=self._board_point_vars["p2"]).grid(row=2, column=1, sticky="w")

        sq_frame = ttk.LabelFrame(tab, text="Square Fit (manual TCP)")
        sq_frame.pack(fill="x", padx=6, pady=6)
        self._square_name_var = tk.StringVar(value="e4")
        ttk.Label(sq_frame, text="Square:").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(sq_frame, textvariable=self._square_name_var, width=6).grid(row=0, column=1, sticky="w", padx=4, pady=2)
        ttk.Button(sq_frame, text="Capture Square", command=self._capture_square_point, state=state).grid(
            row=0, column=2, sticky="w", padx=4, pady=2
        )
        ttk.Button(sq_frame, text="Clear Squares", command=self._clear_square_points).grid(
            row=0, column=3, sticky="w", padx=4, pady=2
        )
        ttk.Button(sq_frame, text="Fit base_T_board", command=self._fit_base_T_board_from_squares).grid(
            row=0, column=4, sticky="w", padx=4, pady=2
        )
        ttk.Label(sq_frame, text="Alternative to P0/P1/P2: capture 3+ named squares (a1..h8) for a least-squares fit of base_T_board.").grid(row=1, column=0, columnspan=5, sticky="w", padx=4, pady=(2, 4))

        test_frame = ttk.LabelFrame(tab, text="Board Test Targets")
        test_frame.pack(fill="x", padx=6, pady=6)
        self._test_squares_var = tk.StringVar(value="a1 h1 a8 h8 e4 d5")
        self._test_z_safe_var = tk.DoubleVar(value=80.0)
        self._test_z_touch_var = tk.DoubleVar(value=10.0)
        self._test_dry_run_var = tk.BooleanVar(value=True)
        ttk.Label(test_frame, text="Squares (e.g. a1 h1 e4) or x,y(mm):").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(test_frame, textvariable=self._test_squares_var, width=40).grid(row=0, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(test_frame, text="Z safe:").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(test_frame, textvariable=self._test_z_safe_var, width=8).grid(row=1, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(test_frame, text="Z touch:").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(test_frame, textvariable=self._test_z_touch_var, width=8).grid(row=2, column=1, sticky="w", padx=4, pady=2)
        ttk.Checkbutton(test_frame, text="Dry run (no motion)", variable=self._test_dry_run_var).grid(
            row=3, column=0, sticky="w", padx=4, pady=(2, 2)
        )
        ttk.Checkbutton(
            test_frame, text="Pitch adaptation (tilt outward from board center)",
            variable=self._test_pitch_adapt_var,
        ).grid(row=4, column=0, columnspan=2, sticky="w", padx=4, pady=(2, 0))
        ttk.Label(test_frame, text="Max pitch (°):").grid(row=5, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(test_frame, textvariable=self._test_max_pitch_var, width=8).grid(row=5, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(test_frame, text="Use this to verify calibration/kinematics before pick.").grid(
            row=6, column=0, columnspan=2, sticky="w", padx=4, pady=(2, 4)
        )

        self._build_pick_place_test_frame(tab)

    # ------------------------------------------------------------------
    # Pick & Place Test UI
    # ------------------------------------------------------------------
    def _build_pick_place_test_frame(self, tab):
        pp = ttk.LabelFrame(tab, text="Pick & Place Test")
        pp.pack(fill="x", padx=6, pady=6)

        ttk.Label(pp, text="From square:").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(pp, textvariable=self._ppt_from_sq_var, width=6).grid(row=0, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(pp, text="To square:").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(pp, textvariable=self._ppt_to_sq_var, width=6).grid(row=1, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(pp, text="Z grasp (mm):").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(pp, textvariable=self._ppt_z_grasp_var, width=8).grid(row=2, column=1, sticky="w", padx=4, pady=2)
        ttk.Label(pp, text="(board frame, depth to grasp)").grid(row=2, column=2, sticky="w", padx=4, pady=2)

        ttk.Label(pp, text="Gripper open S:").grid(row=3, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(pp, textvariable=self._ppt_open_s_var, width=8).grid(row=3, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(pp, text="Gripper close S:").grid(row=4, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(pp, textvariable=self._ppt_close_s_var, width=8).grid(row=4, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(pp, text="Gripper pause (ms):").grid(row=5, column=0, sticky="w", padx=4, pady=2)
        ttk.Entry(pp, textvariable=self._ppt_pause_ms_var, width=8).grid(row=5, column=1, sticky="w", padx=4, pady=2)

        ttk.Label(pp, text="Uses Z safe, Dry run, Pitch adapt from above.").grid(
            row=6, column=0, columnspan=3, sticky="w", padx=4, pady=(2, 2)
        )
        ttk.Button(pp, text="Run Pick & Place", command=self._run_pick_place_test).grid(
            row=7, column=0, columnspan=2, sticky="w", padx=4, pady=(4, 6)
        )

    def _build_camera_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        info = ttk.Label(
            tab,
            text=(
                "Calibrate camera intrinsics using the chessboard pattern. "
                "Capture 10+ samples with varied board orientations and distances (minimum 5 required). "
                "Target RMS reprojection error: < 0.5 px."
            ),
            wraplength=520,
        )
        info.pack(anchor="w", padx=6, pady=(6, 2))

        btns = ttk.Frame(tab)
        btns.pack(anchor="w", padx=6, pady=4)
        ttk.Button(btns, text="Capture Sample", command=self._capture_camera_sample).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Compute Intrinsics", command=self._compute_camera_intrinsics).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Save to camera.json", command=self._save_camera_intrinsics).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Clear Samples", command=self._reset_camera_calib).pack(side=tk.LEFT, padx=4)

        ttk.Label(tab, textvariable=self._cam_calib_status).pack(anchor="w", padx=6, pady=(2, 2))
        ttk.Label(tab, textvariable=self._cam_calib_result_var, font=("Consolas", 9), justify="left").pack(
            anchor="w", padx=6, pady=(2, 6)
        )

    def _get_tcp_position_mm(self):
        if self._execute_app is None or not hasattr(self._execute_app, "get_current_tcp_mm"):
            raise RuntimeError("Robot TCP not available")
        tcp = self._execute_app.get_current_tcp_mm()
        return [
            float(tcp.get("X_mm", 0.0)),
            float(tcp.get("Y_mm", 0.0)),
            float(tcp.get("Z_mm", 0.0)),
        ]

    def _capture_board_point(self, key: str):
        try:
            point = self._get_tcp_position_mm()
            self._board_points[key] = point
            if key in self._board_point_vars:
                self._board_point_vars[key].set(f"{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}")
            self._log(f"Board point {key} captured at {point}.")
            # Append capture to JSON log for verification/simulation
            try:
                entry = {
                    "timestamp": time.time(),
                    "type": "board_point",
                    "key": key,
                    "tcp_mm": point,
                }
                # attempt to include joint values if available
                joints = None
                try:
                    get_j = getattr(self._execute_app, "get_current_joint_list", None)
                    if callable(get_j):
                        joints = get_j()
                    else:
                        kt = getattr(self._execute_app, "kinematics_tabs", None)
                        if kt is not None and hasattr(kt, "_get_current_joint_list"):
                            joints = kt._get_current_joint_list()
                except Exception:
                    joints = None
                if joints is not None:
                    entry["joints_rad"] = joints
                    try:
                        # If possible, set the kinematics initial-joint guess for next move
                        kt = getattr(self._execute_app, "kinematics_tabs", None)
                        if kt is not None:
                            kt._initial_joints_next = list(joints)
                    except Exception:
                        pass
                self._append_capture_log(entry)
            except Exception:
                pass
        except Exception as exc:
            self._log(f"Capture board point failed: {exc}")

    def _clear_board_points(self):
        self._board_points = {}
        for var in self._board_point_vars.values():
            var.set("not set")
        self._log("Board points cleared.")

    def _capture_square_point(self):
        try:
            name = (self._square_name_var.get() or "").strip().lower()
            if len(name) < 2:
                raise RuntimeError("Square name required (e.g. e4).")
            file_ch = name[0]
            rank_ch = name[1]
            if file_ch < "a" or file_ch > "h" or not rank_ch.isdigit() or not (1 <= int(rank_ch) <= 8):
                raise RuntimeError("Square must be a1..h8.")
            point = self._get_tcp_position_mm()
            self._board_square_points[name[:2]] = point
            self._log(f"Square {name} captured at {point}.")
            try:
                entry = {
                    "timestamp": time.time(),
                    "type": "square_point",
                    "square": name[:2],
                    "tcp_mm": point,
                }
                joints = None
                try:
                    get_j = getattr(self._execute_app, "get_current_joint_list", None)
                    if callable(get_j):
                        joints = get_j()
                    else:
                        kt = getattr(self._execute_app, "kinematics_tabs", None)
                        if kt is not None and hasattr(kt, "_get_current_joint_list"):
                            joints = kt._get_current_joint_list()
                except Exception:
                    joints = None
                if joints is not None:
                    entry["joints_rad"] = joints
                    try:
                        kt = getattr(self._execute_app, "kinematics_tabs", None)
                        if kt is not None:
                            kt._initial_joints_next = list(joints)
                    except Exception:
                        pass
                self._append_capture_log(entry)
            except Exception:
                pass
        except Exception as exc:
            self._log(f"Capture square failed: {exc}")

    def _clear_square_points(self):
        self._board_square_points = {}
        self._log("Square points cleared.")

    # ------------------------------------------------------------------
    # Plausibility checks for base_T_board
    # ------------------------------------------------------------------

    def _check_base_T_board_point_geometry(self, p0, p1, p2, x_len, y_len):
        """Check geometric consistency of the three captured corner points."""
        vx = [p1[i] - p0[i] for i in range(3)]
        vy = [p2[i] - p0[i] for i in range(3)]
        dist_x = math.sqrt(sum(v * v for v in vx))
        dist_y = math.sqrt(sum(v * v for v in vy))

        # Distance check ±20 %
        tol = 0.20
        if abs(dist_x - x_len) > x_len * tol:
            self._log(
                f"  [WARN] P0→P1 distance {dist_x:.1f} mm deviates from expected "
                f"{x_len:.1f} mm by >{tol*100:.0f}% — wrong corner or square-size?"
            )
        if abs(dist_y - y_len) > y_len * tol:
            self._log(
                f"  [WARN] P0→P2 distance {dist_y:.1f} mm deviates from expected "
                f"{y_len:.1f} mm by >{tol*100:.0f}% — wrong corner or square-size?"
            )

        # Orthogonality check: angle between vx and vy should be 90° ± 15°
        if dist_x > 1e-6 and dist_y > 1e-6:
            cos_a = sum(vx[i] * vy[i] for i in range(3)) / (dist_x * dist_y)
            cos_a = max(-1.0, min(1.0, cos_a))
            angle_deg = math.degrees(math.acos(abs(cos_a)))
            deviation = abs(90.0 - angle_deg)
            if deviation > 15.0:
                self._log(
                    f"  [WARN] Angle P0→P1 / P0→P2 = {angle_deg:.1f}° (expected 90°, "
                    f"deviation {deviation:.1f}°) — P1/P2 likely not orthogonal axes. "
                    f"Check P0/P1/P2 order in help sketch."
                )
            else:
                self._log(f"  [OK]  P0/P1/P2 orthogonality: {angle_deg:.1f}° (±{deviation:.1f}° from 90°)")

    def _check_base_T_board_plausibility(self, base_T_board):
        """Common plausibility checks on the computed 4×4 base_T_board matrix."""
        # Rotation sub-matrix
        R = [[base_T_board[r][c] for c in range(3)] for r in range(3)]

        # Det(R) must be +1 for proper rotation
        det = (
            R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
            - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
            + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0])
        )
        if det < 0.5:
            self._log(
                f"  [WARN] det(R) = {det:.3f} — rotation matrix invalid "
                f"(reflection or degenerate). Check P0/P1/P2 order."
            )
        else:
            self._log(f"  [OK]  det(R) = {det:.4f}")

        # Board Z-axis (3rd column of R) should be roughly vertical in base frame.
        # R[2][2] = dot(base_Z, board_Z); |R[2][2]| > cos(45°)=0.707 means <45° tilt.
        board_z_in_base_z = abs(R[2][2])
        tilt_deg = math.degrees(math.acos(min(1.0, board_z_in_base_z)))
        if tilt_deg > 45.0:
            self._log(
                f"  [WARN] Board normal tilted {tilt_deg:.1f}° from base-Z — "
                f"board may not be horizontal or axes are swapped."
            )
        else:
            self._log(f"  [OK]  Board normal tilt from vertical: {tilt_deg:.1f}°")

        # Compare with previously stored calibration (if any)
        try:
            if self._configs is None:
                return
            calib = self._configs.get("calibration", {})
            old_T = calib.get("base_T_board")
            if not old_T:
                return
            # Translation delta
            t_new = [base_T_board[r][3] for r in range(3)]
            t_old = [old_T[r][3] for r in range(3)]
            t_delta = math.sqrt(sum((t_new[i] - t_old[i]) ** 2 for i in range(3)))
            if t_delta > 200.0:
                self._log(
                    f"  [WARN] Board origin moved {t_delta:.1f} mm vs stored calibration "
                    f"— board repositioned or wrong TCP?"
                )
            else:
                self._log(f"  [OK]  Origin delta vs stored: {t_delta:.1f} mm")

            # Rotation delta angle: trace method
            R_old = [[old_T[r][c] for c in range(3)] for r in range(3)]
            # R_delta = R_new @ R_old^T
            trace_delta = sum(
                sum(R[i][k] * R_old[j][k] for k in range(3)) * (1 if i == j else 0)
                for i in range(3) for j in range(3)
            )
            # Simpler: direct trace of R_new @ R_old^T
            trace_val = sum(
                sum(R[i][k] * R_old[j][k] for k in range(3))
                for i in range(3) for j in range(3) if i == j
            )
            cos_theta = (trace_val - 1.0) / 2.0
            cos_theta = max(-1.0, min(1.0, cos_theta))
            rot_delta_deg = math.degrees(math.acos(cos_theta))
            if rot_delta_deg > 30.0:
                self._log(
                    f"  [WARN] Board orientation changed {rot_delta_deg:.1f}° vs stored "
                    f"— board rotated or P0/P1/P2 in different order than last time?"
                )
            else:
                self._log(f"  [OK]  Rotation delta vs stored: {rot_delta_deg:.1f}°")
        except Exception as exc:
            self._log(f"  [INFO] Could not compare with stored calibration: {exc}")

    def _compute_base_T_board_from_points(self):
        try:
            p0 = self._board_points.get("p0")
            p1 = self._board_points.get("p1")
            p2 = self._board_points.get("p2")
            if not (p0 and p1 and p2):
                raise RuntimeError("Capture P0, P1, and P2 first.")
            cols = int(self._pattern_cols_var.get())
            rows = int(self._pattern_rows_var.get())
            square = float(self._square_size_var.get())
            if cols < 2 or rows < 2:
                raise RuntimeError("Pattern size must be at least 2x2.")
            x_len = (cols - 1) * square
            y_len = (rows - 1) * square

            vx = [p1[i] - p0[i] for i in range(3)]
            vy = [p2[i] - p0[i] for i in range(3)]
            x_axis = normalize_vector(vx)
            y_axis = normalize_vector(vy)
            if x_axis is None or y_axis is None:
                raise RuntimeError("Board points are too close or invalid.")
            z_axis = normalize_vector(cross(x_axis, y_axis))
            if z_axis is None:
                raise RuntimeError("Board points are collinear.")
            y_axis = normalize_vector(cross(z_axis, x_axis))
            if y_axis is None:
                raise RuntimeError("Failed to orthogonalize board axes.")

            R = [
                [x_axis[0], y_axis[0], z_axis[0]],
                [x_axis[1], y_axis[1], z_axis[1]],
                [x_axis[2], y_axis[2], z_axis[2]],
            ]
            base_T_board = make_transform(R, p0)
            self._set_matrix_vars(self._base_T_board_vars, base_T_board)

            pred_p1 = [p0[i] + x_axis[i] * x_len for i in range(3)]
            pred_p2 = [p0[i] + y_axis[i] * y_len for i in range(3)]
            err1 = math.sqrt(sum((pred_p1[i] - p1[i]) ** 2 for i in range(3)))
            err2 = math.sqrt(sum((pred_p2[i] - p2[i]) ** 2 for i in range(3)))
            self._log(f"base_T_board computed. err_x={err1:.2f} mm err_y={err2:.2f} mm")
            self._log("Plausibility check:")
            self._check_base_T_board_point_geometry(p0, p1, p2, x_len, y_len)
            self._check_base_T_board_plausibility(base_T_board)
        except Exception as exc:
            self._log(f"Compute base_T_board failed: {exc}")

    def _fit_base_T_board_from_squares(self):
        try:
            if len(self._board_square_points) < 3:
                raise RuntimeError("Capture at least 3 squares.")
            cols = int(self._pattern_cols_var.get())
            rows = int(self._pattern_rows_var.get())
            square = float(self._square_size_var.get())
            if cols < 2 or rows < 2:
                raise RuntimeError("Pattern size must be at least 2x2.")

            import numpy as np

            board_pts = []
            base_pts = []
            for name, base_pt in self._board_square_points.items():
                file_ch = name[0]
                rank = int(name[1])
                x, y = self._board_square_center(file_ch, rank)
                board_pts.append([x, y, 0.0])
                base_pts.append(list(base_pt))
            B = np.array(board_pts, dtype=np.float64)
            A = np.array(base_pts, dtype=np.float64)

            # Kabsch: find R,t that maps B -> A
            B_cent = B.mean(axis=0)
            A_cent = A.mean(axis=0)
            B0 = B - B_cent
            A0 = A - A_cent
            H = B0.T @ A0
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[2, :] *= -1
                R = Vt.T @ U.T
            t = A_cent - R @ B_cent

            base_T_board = [
                [float(R[0, 0]), float(R[0, 1]), float(R[0, 2]), float(t[0])],
                [float(R[1, 0]), float(R[1, 1]), float(R[1, 2]), float(t[1])],
                [float(R[2, 0]), float(R[2, 1]), float(R[2, 2]), float(t[2])],
                [0.0, 0.0, 0.0, 1.0],
            ]
            self._set_matrix_vars(self._base_T_board_vars, base_T_board)

            # RMS error
            B_fit = (R @ B.T).T + t
            err = np.linalg.norm(B_fit - A, axis=1).mean()
            self._log(f"base_T_board fit from squares. avg_err={err:.2f} mm")
            self._log("Plausibility check:")
            self._check_base_T_board_plausibility(base_T_board)
        except Exception as exc:
            self._log(f"Fit base_T_board failed: {exc}")

    def _parse_board_targets(self, text):
        tokens = [t.strip() for t in (text or "").replace(";", " ").split() if t.strip()]
        out = []
        for tok in tokens:
            if "," in tok:
                parts = tok.split(",")
                if len(parts) >= 2:
                    try:
                        out.append(("xy", float(parts[0]), float(parts[1])))
                        continue
                    except Exception:
                        pass
            if len(tok) >= 2:
                file_ch = tok[0].lower()
                rank_ch = tok[1]
                if "a" <= file_ch <= "h" and rank_ch.isdigit():
                    out.append(("sq", file_ch, int(rank_ch)))
        return out

    def _board_square_center(self, file_ch, rank):
        cols = int(self._pattern_cols_var.get())
        rows = int(self._pattern_rows_var.get())
        square = float(self._square_size_var.get())
        if cols < 2 or rows < 2:
            raise RuntimeError("Pattern size must be at least 2x2.")
        file_idx = ord(file_ch) - ord("a")
        rank_idx = int(rank) - 1
        if file_idx < 0 or file_idx > 7 or rank_idx < 0 or rank_idx > 7:
            raise RuntimeError("Square out of range.")
        x = (file_idx + 0.5) * square
        y = (rank_idx + 0.5) * square
        return x, y

    def _board_to_base(self, x_mm, y_mm, z_mm):
        base_T_board = self._read_matrix_vars(self._base_T_board_vars)
        T = make_transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [x_mm, y_mm, z_mm])
        base_T = matmul(base_T_board, T)
        return [base_T[0][3], base_T[1][3], base_T[2][3]]

    def _compute_approach_rpy(self, sq_x, sq_y, roll0, pitch0, yaw0):
        """
        When pitch adaptation is enabled, tilt the tool outward from the board
        centre by up to max_pitch_deg at the board corners.

        sq_x, sq_y : square centre in board frame (mm).
        Returns modified (roll, pitch, yaw) in degrees.
        """
        if not self._test_pitch_adapt_var.get():
            return roll0, pitch0, yaw0

        max_pitch = float(self._test_max_pitch_var.get())
        cols = int(self._pattern_cols_var.get())
        rows = int(self._pattern_rows_var.get())
        sq = float(self._square_size_var.get())

        # Board centre and half-diagonal (max reachable distance from centre)
        cx = cols * sq / 2.0
        cy = rows * sq / 2.0
        max_d = math.sqrt(cx * cx + cy * cy)
        if max_d < 1e-6:
            return roll0, pitch0, yaw0

        dx = sq_x - cx
        dy = sq_y - cy
        d = math.sqrt(dx * dx + dy * dy)
        tilt_deg = max_pitch * (d / max_d)
        if tilt_deg < 0.1:
            return roll0, pitch0, yaw0

        # Tilt axis in board frame: perpendicular to (dx,dy) in board XY-plane.
        # Rotating around (-dy, dx, 0) by tilt_deg leans the tool outward toward
        # the square's direction from the board centre.
        tilt_axis_board = normalize_vector([-dy, dx, 0.0])
        if tilt_axis_board is None:
            return roll0, pitch0, yaw0

        # Transform tilt axis from board frame to base frame via base_T_board rotation.
        base_T_board = self._read_matrix_vars(self._base_T_board_vars)
        R_board = [[base_T_board[r][c] for c in range(3)] for r in range(3)]
        kx = sum(R_board[0][c] * tilt_axis_board[c] for c in range(3))
        ky = sum(R_board[1][c] * tilt_axis_board[c] for c in range(3))
        kz = sum(R_board[2][c] * tilt_axis_board[c] for c in range(3))

        # Rodrigues rotation matrix around unit axis (kx,ky,kz) by tilt_deg
        theta = math.radians(tilt_deg)
        c, s = math.cos(theta), math.sin(theta)
        t = 1.0 - c
        R_tilt = [
            [t*kx*kx + c,      t*kx*ky - s*kz, t*kx*kz + s*ky],
            [t*kx*ky + s*kz,   t*ky*ky + c,    t*ky*kz - s*kx],
            [t*kx*kz - s*ky,   t*ky*kz + s*kx, t*kz*kz + c   ],
        ]

        # Compose: R_new = R_tilt @ R_original
        R0 = self._rpy_to_R(roll0, pitch0, yaw0)
        R_new = [
            [sum(R_tilt[i][k] * R0[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)
        ]
        roll_new, pitch_new, yaw_new = self._rpy_from_R(R_new)
        return roll_new, pitch_new, yaw_new

    def _test_board_moves(self):
        def _task():
            try:
                if self._execute_app is None or not hasattr(self._execute_app, "kinematics_tabs"):
                    raise RuntimeError("Kinematics UI not available")
                self._ensure_pipeline()
                kt = self._execute_app.kinematics_tabs

                # --- Verify live MPos is available before any movement ---
                client = getattr(self._execute_app, "client", None)
                st = getattr(client, "last_status", None) or {}
                mpos = st.get("MPos") or {}
                if not mpos:
                    raise RuntimeError(
                        "No current machine position (MPos) available. "
                        "Is the robot connected and homed?"
                    )
                # Prime axis_positions with live MPos so IK starts from actual position
                # kt.world is the TcpKinematicsFrame that owns exec / _get_current_joint_list
                kw = kt.world
                for ax, val in mpos.items():
                    if ax in ("A", "B", "C", "X", "Y", "Z"):
                        kw.exec.axis_positions[ax] = float(val)
                joints_init = kw._get_current_joint_list()
                self._log(f"Board test: live joints from MPos = {joints_init}")

                targets = self._parse_board_targets(self._test_squares_var.get())
                if not targets:
                    raise RuntimeError("No test targets specified.")
                z_safe = float(self._test_z_safe_var.get())
                z_touch = float(self._test_z_touch_var.get())
                dry_run = bool(self._test_dry_run_var.get())
                roll0, pitch0, yaw0 = self._current_rpy()
                feed = self._get_speed_slider_feed(60.0)
                pitch_adapt = self._test_pitch_adapt_var.get()
                self._log(
                    f"Board test dry_run={dry_run} z_safe={z_safe} z_touch={z_touch} "
                    f"pitch_adapt={pitch_adapt}"
                )

                # --- Safe-Z retreat at current X/Y before approaching any board target ---
                if not dry_run:
                    tcp_now = self._execute_app.get_current_tcp_mm()
                    x_now = float(tcp_now.get("X_mm", 0.0))
                    y_now = float(tcp_now.get("Y_mm", 0.0))
                    self._log(
                        f"Board test: retreating to safe Z={z_safe} "
                        f"at current X={x_now:.1f} Y={y_now:.1f}"
                    )
                    # Provide explicit joint hint so IK starts from actual position
                    kw._initial_joints_next = list(joints_init)
                    ok = kt.move_tcp_pose(x_now, y_now, z_safe, roll0, pitch0, yaw0, feed=feed)
                    if not ok:
                        raise RuntimeError("Initial safe-Z retreat failed")
                    self._wait_for_idle()

                for item in targets:
                    if item[0] == "xy":
                        x_mm, y_mm = item[1], item[2]
                    else:
                        x_mm, y_mm = self._board_square_center(item[1], item[2])

                    # Per-square orientation: tilt outward from board centre if enabled
                    roll, pitch, yaw = self._compute_approach_rpy(x_mm, y_mm, roll0, pitch0, yaw0)

                    p_safe = self._board_to_base(x_mm, y_mm, z_safe)
                    p_touch = self._board_to_base(x_mm, y_mm, z_touch)
                    self._log(
                        f"Test target {item}: base_safe={p_safe} base_touch={p_touch} "
                        f"rpy=({roll:.1f},{pitch:.1f},{yaw:.1f})"
                    )
                    preview_safe = kt.preview_tcp_gcode(
                        p_safe[0], p_safe[1], p_safe[2], roll, pitch, yaw, feed=feed
                    )
                    if not preview_safe or not preview_safe.get("ok", False):
                        raise RuntimeError(f"Preview failed for safe target: {preview_safe}")
                    preview_touch = kt.preview_tcp_gcode(
                        p_touch[0], p_touch[1], p_touch[2], roll, pitch, yaw, feed=feed
                    )
                    if not preview_touch or not preview_touch.get("ok", False):
                        raise RuntimeError(f"Preview failed for touch target: {preview_touch}")
                    self._log(f"Preview safe: {preview_safe.get('gcode')}")
                    self._log(f"Preview touch: {preview_touch.get('gcode')}")
                    if not dry_run:
                        # Refresh joint hint from live MPos before each safe move
                        st2 = getattr(client, "last_status", None) or {}
                        mpos2 = st2.get("MPos") or {}
                        if mpos2:
                            for ax, val in mpos2.items():
                                if ax in ("A", "B", "C", "X", "Y", "Z"):
                                    kw.exec.axis_positions[ax] = float(val)
                            kw._initial_joints_next = kw._get_current_joint_list()
                        ok = kt.move_tcp_pose(
                            p_safe[0], p_safe[1], p_safe[2], roll, pitch, yaw, feed=feed
                        )
                        if not ok:
                            raise RuntimeError("Move to safe failed")
                        self._wait_for_idle()
                        # Refresh joint hint from live MPos before touch move
                        st3 = getattr(client, "last_status", None) or {}
                        mpos3 = st3.get("MPos") or {}
                        if mpos3:
                            for ax, val in mpos3.items():
                                if ax in ("A", "B", "C", "X", "Y", "Z"):
                                    kw.exec.axis_positions[ax] = float(val)
                            kw._initial_joints_next = kw._get_current_joint_list()
                        ok = kt.move_tcp_pose(
                            p_touch[0], p_touch[1], p_touch[2], roll, pitch, yaw, feed=feed
                        )
                        if not ok:
                            raise RuntimeError("Move to touch failed")
                        self._wait_for_idle()
                self._log("Board test moves complete.")
            except Exception as exc:
                self._log(f"Board test moves failed: {exc}")
        self._start_worker(_task)

    # ------------------------------------------------------------------
    # Pick & Place Test — motion sequence
    # ------------------------------------------------------------------
    def _run_pick_place_test(self):
        def _task():
            try:
                # -- guard --
                if self._execute_app is None or not hasattr(self._execute_app, "kinematics_tabs"):
                    raise RuntimeError("Kinematics UI not available")
                self._ensure_pipeline()
                kt = self._execute_app.kinematics_tabs

                # -- live MPos → joint hints --
                client = getattr(self._execute_app, "client", None)
                st = getattr(client, "last_status", None) or {}
                mpos = st.get("MPos") or {}
                if not mpos:
                    raise RuntimeError(
                        "No current machine position (MPos) available. "
                        "Is the robot connected and homed?"
                    )
                kw = kt.world
                for ax, val in mpos.items():
                    if ax in ("A", "B", "C", "X", "Y", "Z"):
                        kw.exec.axis_positions[ax] = float(val)
                joints_init = kw._get_current_joint_list()
                self._log(f"Pick & Place: live joints = {joints_init}")

                # -- read parameters --
                from_sq = self._ppt_from_sq_var.get().strip().lower()
                to_sq   = self._ppt_to_sq_var.get().strip().lower()
                z_safe  = float(self._test_z_safe_var.get())
                z_grasp = float(self._ppt_z_grasp_var.get())
                open_s  = int(self._ppt_open_s_var.get())
                close_s = int(self._ppt_close_s_var.get())
                pause_s = float(self._ppt_pause_ms_var.get()) / 1000.0
                dry_run = bool(self._test_dry_run_var.get())
                feed    = self._get_speed_slider_feed(60.0)

                # -- parse squares --
                def _sq(s):
                    if len(s) < 2:
                        raise RuntimeError(f"Invalid square '{s}'")
                    f, r = s[0], s[1]
                    if not ("a" <= f <= "h" and r.isdigit()):
                        raise RuntimeError(f"'{s}' is not a valid square (a1..h8)")
                    return self._board_square_center(f, int(r))

                pick_x, pick_y   = _sq(from_sq)
                place_x, place_y = _sq(to_sq)

                # -- orientation (with optional pitch adaptation) --
                roll0, pitch0, yaw0 = self._current_rpy()
                pick_r, pick_p, pick_y_ = self._compute_approach_rpy(
                    pick_x, pick_y, roll0, pitch0, yaw0
                )
                place_r, place_p, place_y_ = self._compute_approach_rpy(
                    place_x, place_y, roll0, pitch0, yaw0
                )

                # -- compute 4 waypoints in base frame --
                pick_safe  = self._board_to_base(pick_x,  pick_y,  z_safe)
                pick_grasp = self._board_to_base(pick_x,  pick_y,  z_grasp)
                place_safe = self._board_to_base(place_x, place_y, z_safe)
                place_grasp = self._board_to_base(place_x, place_y, z_grasp)

                self._log(
                    f"Pick & Place: {from_sq}->{to_sq}  z_safe={z_safe} "
                    f"z_grasp={z_grasp} feed={feed:.0f} dry={dry_run}"
                )
                self._log(f"  pick_safe  = {[f'{v:.1f}' for v in pick_safe]}")
                self._log(f"  pick_grasp = {[f'{v:.1f}' for v in pick_grasp]}")
                self._log(f"  place_safe = {[f'{v:.1f}' for v in place_safe]}")
                self._log(f"  place_grasp= {[f'{v:.1f}' for v in place_grasp]}")

                # -- IK preview all 4 (fail early) --
                for label, pos, r, p, y in [
                    ("pick_safe",   pick_safe,   pick_r,  pick_p,  pick_y_),
                    ("pick_grasp",  pick_grasp,  pick_r,  pick_p,  pick_y_),
                    ("place_safe",  place_safe,  place_r, place_p, place_y_),
                    ("place_grasp", place_grasp, place_r, place_p, place_y_),
                ]:
                    res = kt.preview_tcp_gcode(pos[0], pos[1], pos[2], r, p, y, feed=feed)
                    if not res or not res.get("ok", False):
                        raise RuntimeError(f"IK preview failed for {label}: {res}")
                    self._log(f"  IK ok [{label}]")

                if dry_run:
                    self._log("Dry run: all IK previews passed. No motion.")
                    return

                # -- helpers --
                def _refresh():
                    st2 = getattr(client, "last_status", None) or {}
                    m2 = st2.get("MPos") or {}
                    if m2:
                        for ax, val in m2.items():
                            if ax in ("A", "B", "C", "X", "Y", "Z"):
                                kw.exec.axis_positions[ax] = float(val)
                        kw._initial_joints_next = kw._get_current_joint_list()

                def _move(label, pos, r, p, y):
                    _refresh()
                    self._log(f"  -> {label}")
                    ok = kt.move_tcp_pose(pos[0], pos[1], pos[2], r, p, y, feed=feed)
                    if not ok:
                        raise RuntimeError(f"Move to {label} failed")
                    self._wait_for_idle()

                # ===== PICK =====
                self._log("=== PICK ===")
                _move("pick_safe", pick_safe, pick_r, pick_p, pick_y_)

                self._log(f"  Gripper OPEN (M3 S{open_s})")
                self._execute_app.send_now(f"M3 S{open_s}")
                time.sleep(pause_s)

                _move("pick_grasp", pick_grasp, pick_r, pick_p, pick_y_)

                self._log(f"  Gripper CLOSE (M3 S{close_s})")
                self._execute_app.send_now(f"M3 S{close_s}")
                time.sleep(pause_s)

                _move("pick_safe (lift)", pick_safe, pick_r, pick_p, pick_y_)

                # ===== TRANSIT =====
                self._log("=== TRANSIT ===")
                _move("place_safe", place_safe, place_r, place_p, place_y_)

                # ===== PLACE =====
                self._log("=== PLACE ===")
                _move("place_grasp", place_grasp, place_r, place_p, place_y_)

                self._log(f"  Gripper OPEN (M3 S{open_s})")
                self._execute_app.send_now(f"M3 S{open_s}")
                time.sleep(pause_s)

                _move("place_safe (retreat)", place_safe, place_r, place_p, place_y_)

                self._log("Pick & Place complete.")

            except Exception as exc:
                self._log(f"Pick & Place failed: {exc}")
        self._start_worker(_task)

    def _reset_camera_calib(self):
        self._cam_calib_objpoints = []
        self._cam_calib_imgpoints = []
        self._cam_calib_image_size = None
        self._cam_calib_pattern = None
        self._cam_calib_square = None
        self._cam_calib_result = None
        self._cam_calib_status.set("No samples")
        self._cam_calib_result_var.set("")
        self._log("Camera calibration samples cleared.")

    def _append_capture_log(self, entry: dict):
        """Append a capture entry to data/capture_poses.json (creates file if missing)."""
        try:
            data_dir = os.path.join(self._base_dir, "data")
            os.makedirs(data_dir, exist_ok=True)
            path = os.path.join(data_dir, "capture_poses.json")
            existing = []
            if os.path.isfile(path):
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        existing = json.load(f) or []
                except Exception:
                    existing = []
            existing.append(entry)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(existing, f, indent=2)
        except Exception:
            pass

    def _capture_camera_sample(self):
        try:
            self._ensure_configs()
            cols = int(self._pattern_cols_var.get())
            rows = int(self._pattern_rows_var.get())
            square = float(self._square_size_var.get())
            if cols < 2 or rows < 2:
                raise RuntimeError("Pattern size must be at least 2x2.")
            if self._cam_calib_pattern and self._cam_calib_pattern != (cols, rows):
                self._reset_camera_calib()
            if self._cam_calib_square and abs(self._cam_calib_square - square) > 1e-6:
                self._reset_camera_calib()

            cam_cfg = self._configs.get("camera", {})
            image, color_order = self._get_calibration_image(cam_cfg)
            if image is None:
                raise RuntimeError("No camera frame available")

            import cv2
            import numpy as np

            if len(image.shape) == 3:
                if color_order == "rgb":
                    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                else:
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
            try:
                flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
                found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
            except Exception:
                found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
            if not found:
                raise RuntimeError("Chessboard not found")

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objp = np.zeros((rows * cols, 3), np.float32)
            objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
            objp *= square

            self._cam_calib_pattern = (cols, rows)
            self._cam_calib_square = square
            self._cam_calib_objpoints.append(objp)
            self._cam_calib_imgpoints.append(corners2)
            self._cam_calib_image_size = (gray.shape[1], gray.shape[0])
            self._cam_calib_status.set(f"Samples: {len(self._cam_calib_objpoints)}")
            self._log("Camera calibration sample captured.")
        except Exception as exc:
            self._log(f"Capture sample failed: {exc}")

    def _compute_camera_intrinsics(self):
        try:
            if len(self._cam_calib_objpoints) < 5:
                raise RuntimeError("Capture at least 5 samples first.")
            if not self._cam_calib_image_size:
                raise RuntimeError("Missing image size.")

            import cv2

            rms, camera_matrix, dist, _, _ = cv2.calibrateCamera(
                self._cam_calib_objpoints,
                self._cam_calib_imgpoints,
                self._cam_calib_image_size,
                None,
                None,
            )
            fx = float(camera_matrix[0][0])
            fy = float(camera_matrix[1][1])
            cx = float(camera_matrix[0][2])
            cy = float(camera_matrix[1][2])
            dist_list = [float(x) for x in dist.reshape(-1).tolist()]

            self._cam_calib_result = {
                "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
                "distortion": dist_list,
                "image_size": list(self._cam_calib_image_size),
                "rms": float(rms),
            }
            self._cam_calib_status.set(f"RMS: {rms:.4f}  Samples: {len(self._cam_calib_objpoints)}")
            self._cam_calib_result_var.set(
                f"fx={fx:.3f} fy={fy:.3f} cx={cx:.3f} cy={cy:.3f}\n"
                f"dist={dist_list}"
            )
            self._log("Camera intrinsics computed.")
        except Exception as exc:
            self._log(f"Compute intrinsics failed: {exc}")

    def _save_camera_intrinsics(self):
        try:
            self._ensure_configs()
            if not self._cam_calib_result:
                raise RuntimeError("No intrinsics computed.")
            cam_cfg = self._configs.get("camera", {})
            cam_cfg["intrinsics"] = dict(self._cam_calib_result["intrinsics"])
            cam_cfg["distortion"] = list(self._cam_calib_result["distortion"])
            cam_cfg["image_size"] = list(self._cam_calib_result["image_size"])
            self._write_config("camera", cam_cfg)
            self._log("Camera intrinsics saved. Re-init the pipeline to apply.")
        except Exception as exc:
            self._log(f"Save intrinsics failed: {exc}")

    def _build_marker_tab(self, tab):
        info = ttk.Label(
            tab,
            text=(
                "Define marker_T_obj: transform from marker frame to object center. "
                "Face axis = the cube axis that points toward the marker face. "
                "Sign = +1 if the marker faces in the positive axis direction, -1 if negative."
            ),
            wraplength=520,
        )
        info.pack(anchor="w", padx=6, pady=(6, 2))

        params = ttk.Frame(tab)
        params.pack(anchor="w", padx=6, pady=4)
        ttk.Label(params, text="Face axis:").grid(row=0, column=0, padx=4, pady=2, sticky="w")
        ttk.Combobox(params, textvariable=self._marker_axis_var, values=["x", "y", "z"], width=4, state="readonly").grid(
            row=0, column=1, padx=4, pady=2
        )
        ttk.Label(params, text="Sign:").grid(row=0, column=2, padx=4, pady=2, sticky="w")
        ttk.Combobox(params, textvariable=self._marker_sign_var, values=[-1, 1], width=4, state="readonly").grid(
            row=0, column=3, padx=4, pady=2
        )

        btns = ttk.Frame(tab)
        btns.pack(anchor="w", padx=6, pady=4)
        ttk.Button(btns, text="Load Config", command=self._load_calibration_into_ui).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Compute marker_T_obj", command=self._compute_marker_T_obj).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Save marker_T_obj", command=self._save_marker_T_obj).pack(side=tk.LEFT, padx=4)

        frame, self._marker_T_obj_vars = self._make_matrix_grid(tab, "marker_T_obj", readonly=False)
        frame.pack(anchor="w", padx=6, pady=6)

    def _build_detect_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        info = ttk.Label(tab, text="Run a single detection and show the object pose.")
        info.pack(anchor="w", padx=6, pady=(6, 2))
        btns = ttk.Frame(tab)
        btns.pack(anchor="w", padx=6, pady=4)
        ttk.Button(btns, text="Detect Once", command=self._detect_once).pack(side=tk.LEFT, padx=4)
        ttk.Button(btns, text="Pick up (Test)", command=self._pickup_test).pack(side=tk.LEFT, padx=4)
        ttk.Label(tab, textvariable=self._detect_var, font=("Consolas", 9)).pack(anchor="w", padx=6, pady=4)

        filt = ttk.LabelFrame(tab, text="TNT Filter")
        filt.pack(fill="x", padx=6, pady=6)
        ttk.Label(
            filt,
            text=(
                "H1/H2: Red hue ranges (0-179). "
                "S min/V min: Minimum saturation/brightness. "
                "Min area: Minimum object area in pixels. "
                "Kernel: Morphology kernel size (odd number)."
            ),
            wraplength=520,
        ).pack(anchor="w", padx=6, pady=(4, 2))

        def _slider_row(parent, label, var, from_, to_, step=1):
            row = ttk.Frame(parent)
            row.pack(fill="x", padx=6, pady=2)
            ttk.Label(row, text=label, width=14).pack(side=tk.LEFT)
            scale = ttk.Scale(
                row,
                from_=from_,
                to=to_,
                variable=var,
                orient=tk.HORIZONTAL,
                length=200,
                value=var.get(),
                command=lambda _v, v=var, s=step: v.set(int(round(float(_v) / s) * s)),
            )
            scale.pack(side=tk.LEFT, padx=4)
            scale.bind("<ButtonRelease-1>", lambda _e: self._apply_tnt_filters())
            entry = ttk.Entry(row, textvariable=var, width=6, justify="right")
            entry.pack(side=tk.LEFT)
            entry.bind("<Return>", lambda _e: self._apply_tnt_filters())

        _slider_row(filt, "H1 low", self._tnt_h1_low_var, 0, 179)
        _slider_row(filt, "H1 high", self._tnt_h1_high_var, 0, 179)
        _slider_row(filt, "H2 low", self._tnt_h2_low_var, 0, 179)
        _slider_row(filt, "H2 high", self._tnt_h2_high_var, 0, 179)
        _slider_row(filt, "S min", self._tnt_s_min_var, 0, 255)
        _slider_row(filt, "V min", self._tnt_v_min_var, 0, 255)
        _slider_row(filt, "Min area", self._tnt_min_area_var, 0, 5000)
        _slider_row(filt, "Kernel", self._tnt_kernel_var, 3, 15, step=2)

        filt_btns = ttk.Frame(filt)
        filt_btns.pack(anchor="w", padx=6, pady=(4, 6))
        ttk.Button(filt_btns, text="Apply Filters", command=self._apply_tnt_filters).pack(side=tk.LEFT, padx=4)
        ttk.Button(filt_btns, text="Save Filters", command=self._save_tnt_filters).pack(side=tk.LEFT, padx=4)

    def _build_autocalib_tab(self, tab):
        """
        Auto-Calib sub-tab: automatic eye-to-hand hand-eye calibration.

        Workflow:
          1. Mount an ArUco marker on the TCP flange.
          2. Configure marker dict / ID / side length.
          3. Move robot to ≥ 8 diverse poses (vary orientation ≥ 10° each step).
          4. Click "Capture Pose" at each pose.
          5. Click "Solve" → base_T_cam is computed.
          6. Verify RMS error, then click "Apply to Base-Cam".
        """
        import numpy as np
        from PIL import Image, ImageTk

        tab.columnconfigure(0, weight=1)

        if not _AUTOCALIB_AVAILABLE:
            ttk.Label(
                tab,
                text="autocalib_v1 module not found.\n"
                     "Ensure autocalib_v1.py is in the project folder.",
                foreground="red",
            ).pack(padx=10, pady=20)
            return

        # ── State ─────────────────────────────────────────────────────────────
        self._ac_session  = None   # AutoCalibSession instance
        self._ac_result   = None   # last solved 4×4 base_T_cam (np.ndarray)
        self._ac_imgtk    = None   # keep PhotoImage alive (gc protection)

        self._ac_dict_var   = tk.StringVar(value="DICT_4X4_50")
        self._ac_id_var     = tk.StringVar(value="0")
        self._ac_size_var   = tk.StringVar(value="50")
        self._ac_status_var = tk.StringVar(value="0 samples  (need ≥ 5)   diversity: 0.0°")
        self._ac_rms_var    = tk.StringVar(value="RMS: —")
        self._ac_mat_vars   = [
            [tk.StringVar(value="—") for _ in range(4)] for _ in range(4)
        ]

        # ── Marker config ──────────────────────────────────────────────────────
        mf = ttk.LabelFrame(tab, text="Marker (attached to TCP flange)")
        mf.pack(fill=tk.X, padx=6, pady=(6, 4))
        mr = ttk.Frame(mf)
        mr.pack(anchor="w", padx=6, pady=4)
        ttk.Label(mr, text="ArUco dict:").pack(side=tk.LEFT)
        ttk.Combobox(
            mr, textvariable=self._ac_dict_var,
            values=_autocalib.ARUCO_DICT_NAMES,
            width=14, state="readonly",
        ).pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(mr, text="Marker ID:").pack(side=tk.LEFT)
        ttk.Entry(mr, textvariable=self._ac_id_var, width=5, justify="right").pack(
            side=tk.LEFT, padx=(4, 12)
        )
        ttk.Label(mr, text="Side (mm):").pack(side=tk.LEFT)
        ttk.Entry(mr, textvariable=self._ac_size_var, width=6, justify="right").pack(
            side=tk.LEFT, padx=(4, 0)
        )

        # ── Capture section ────────────────────────────────────────────────────
        cf = ttk.LabelFrame(tab, text="Capture")
        cf.pack(fill=tk.X, padx=6, pady=4)
        ttk.Label(
            cf,
            text=(
                "Move robot so the marker is fully visible, then click Capture Pose.\n"
                "Vary orientation ≥ 10° between captures (tilt, rotate around Z)."
            ),
            wraplength=460, foreground="gray",
        ).pack(anchor="w", padx=6, pady=(4, 2))
        ttk.Label(cf, textvariable=self._ac_status_var).pack(anchor="w", padx=6, pady=(0, 4))
        cb = ttk.Frame(cf)
        cb.pack(anchor="w", padx=6, pady=(0, 6))
        btn_capture = ttk.Button(cb, text="Capture Pose", command=lambda: _ac_capture())
        btn_capture.pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(cb, text="Clear", command=lambda: _ac_clear()).pack(side=tk.LEFT)

        # ── Preview canvas ─────────────────────────────────────────────────────
        pf = ttk.LabelFrame(tab, text="Last captured frame")
        pf.pack(fill=tk.X, padx=6, pady=4)
        ac_canvas = tk.Canvas(pf, width=240, height=160, bg="#111", highlightthickness=0)
        ac_canvas.pack(padx=4, pady=4)

        # ── Result section ─────────────────────────────────────────────────────
        rf = ttk.LabelFrame(tab, text="Result  base_T_cam")
        rf.pack(fill=tk.X, padx=6, pady=4)
        rb_row = ttk.Frame(rf)
        rb_row.pack(anchor="w", padx=6, pady=(6, 2))
        btn_solve = ttk.Button(rb_row, text="Solve", state="disabled", command=lambda: _ac_solve())
        btn_solve.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Label(rb_row, textvariable=self._ac_rms_var, foreground="gray").pack(side=tk.LEFT)

        mat_frame = ttk.Frame(rf)
        mat_frame.pack(anchor="w", padx=6, pady=2)
        for r in range(4):
            for c in range(4):
                ttk.Label(
                    mat_frame, textvariable=self._ac_mat_vars[r][c],
                    width=10, anchor="e", relief=tk.GROOVE, padding=(2, 1),
                ).grid(row=r, column=c, padx=1, pady=1)

        apply_row = ttk.Frame(rf)
        apply_row.pack(anchor="w", padx=6, pady=(4, 8))
        btn_apply = ttk.Button(
            apply_row, text="Apply to Base-Cam", state="disabled",
            command=lambda: _ac_apply(),
        )
        btn_apply.pack(side=tk.LEFT)

        # ── Refresh status ─────────────────────────────────────────────────────
        def _ac_refresh_status():
            sess = self._ac_session
            n   = sess.n_samples()   if sess else 0
            div = sess.diversity_score() if sess else 0.0
            min_s = _autocalib.AutoCalibSession.MIN_SAMPLES
            suf = "✓" if n >= min_s else f"(need ≥ {min_s})"
            self._ac_status_var.set(f"{n} samples  {suf}   diversity: {div:.1f}°")
            btn_solve.config(
                state="normal" if (sess and sess.can_solve()) else "disabled"
            )

        # ── Show frame in canvas ───────────────────────────────────────────────
        def _ac_show_frame(frame_rgb):
            try:
                img = Image.fromarray(frame_rgb)
                img.thumbnail((240, 160), Image.LANCZOS)
                self._ac_imgtk = ImageTk.PhotoImage(img)
                ac_canvas.delete("all")
                ac_canvas.create_image(120, 80, image=self._ac_imgtk, anchor="center")
            except Exception:
                pass

        # ── Capture ────────────────────────────────────────────────────────────
        def _ac_capture():
            import cv2 as _cv2

            frame_rgb = None
            try:
                frame_rgb = self._camera_capture.get_latest_frame()
            except Exception:
                pass
            if frame_rgb is None:
                self._ac_status_var.set("Camera not available — start camera first.")
                return

            frame_bgr = _cv2.cvtColor(frame_rgb, _cv2.COLOR_RGB2BGR)

            try:
                tcp = self._execute_app.get_current_tcp_mm()
                x  = float(tcp.get("X_mm", 0.0))
                y  = float(tcp.get("Y_mm", 0.0))
                z  = float(tcp.get("Z_mm", 0.0))
                ro = float(tcp.get("Roll_deg", 0.0))
                pi = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
                ya = float(tcp.get("Yaw_deg", 0.0))
            except Exception as e:
                self._ac_status_var.set(f"TCP read error: {e}")
                return

            cam_cfg = self._configs.get("camera", {})
            K, D = self._camera_model_from_cfg(cam_cfg)

            dict_name  = self._ac_dict_var.get()
            marker_id  = int(self._ac_id_var.get())
            size_mm    = float(self._ac_size_var.get())
            aruco_dict = _autocalib.get_aruco_dict(dict_name)

            if self._ac_session is None:
                self._ac_session = _autocalib.AutoCalibSession(
                    K, D, aruco_dict, marker_id, size_mm,
                )

            found, annotated_bgr = self._ac_session.add_capture(
                frame_bgr, x, y, z, ro, pi, ya,
            )

            vis_rgb = _cv2.cvtColor(annotated_bgr, _cv2.COLOR_BGR2RGB)
            _ac_show_frame(vis_rgb)

            if not found:
                self._ac_status_var.set(
                    f"Marker {marker_id} not detected — adjust position or lighting."
                )
            else:
                _ac_refresh_status()

        # ── Clear ──────────────────────────────────────────────────────────────
        def _ac_clear():
            if self._ac_session is not None:
                self._ac_session.clear()
            self._ac_result = None
            self._ac_rms_var.set("RMS: —")
            for row in self._ac_mat_vars:
                for v in row:
                    v.set("—")
            btn_apply.config(state="disabled")
            ac_canvas.delete("all")
            _ac_refresh_status()

        # ── Solve ──────────────────────────────────────────────────────────────
        def _ac_solve():
            if self._ac_session is None or not self._ac_session.can_solve():
                return
            try:
                base_T_cam = self._ac_session.solve()
                rms = self._ac_session.reprojection_error_mm(base_T_cam)
                self._ac_result = base_T_cam
                self._ac_rms_var.set(f"RMS: {rms:.2f} mm")
                for r in range(4):
                    for c in range(4):
                        self._ac_mat_vars[r][c].set(f"{base_T_cam[r, c]:.4f}")
                btn_apply.config(state="normal")
            except Exception as e:
                self._ac_rms_var.set(f"Solve error: {e}")

        # ── Apply ──────────────────────────────────────────────────────────────
        def _ac_apply():
            if self._ac_result is None:
                return
            mat = self._ac_result.tolist()
            transforms = self._configs.get("transforms", {})
            transforms["base_T_cam"] = mat
            self._write_config("transforms", transforms)
            try:
                self._set_matrix_vars(self._base_T_cam_vars, mat)
            except Exception:
                pass
            self._ac_rms_var.set(self._ac_rms_var.get() + "  ✓ applied")
            btn_apply.config(state="disabled")

        _ac_refresh_status()

    def _build_help_tab(self, tab):
        tab.columnconfigure(0, weight=1)
        tab.rowconfigure(0, weight=1)
        text = (
            "Setup + Calibration (v7.1) - Step by step\n"
            "\n"
            "A) Basic setup\n"
            "1) Connect the camera and start it in the Vision tab.\n"
            "2) Prepare the chessboard: flat, known square size (mm), board fixed.\n"
            "3) Pattern size (cols/rows) = number of INNER corners (not total squares).\n"
            "4) Open Calibration -> Base-Cam and set the pattern parameters.\n"
            "\n"
            "B) Camera intrinsics (once per camera, redo after lens change)\n"
            "5) Open Calibration -> Camera.\n"
            "6) Capture 10+ samples with varied board orientations and distances.\n"
            "   Minimum 5 samples required; 15+ recommended for best accuracy.\n"
            "7) Click Compute Intrinsics. Target RMS reprojection error: < 0.5 px.\n"
            "8) Click Save to camera.json.\n"
            "9) Re-init pipeline (Init Pipeline) to apply new intrinsics.\n"
            "\n"
            "C) base_T_board (Board origin expressed in Base frame)\n"
            "10) Open Calibration -> Base-Cam.\n"
            "11) Verify pattern cols/rows and square size (mm).\n"
            "12) Move TCP tip to P0=(0,0) inner corner, click Capture P0.\n"
            "13) Move TCP tip to P1=(cols-1,0) inner corner, click Capture P1.\n"
            "14) Move TCP tip to P2=(0,rows-1) inner corner, click Capture P2.\n"
            "15) Click Compute base_T_board. Target: err_x and err_y < 2 mm.\n"
            "16) Click Save Board Config (saves pattern params + base_T_board matrix).\n"
            "    Note: P1 defines +X, P2 defines +Y. Order must match camera view.\n"
            "\n"
            "D) base_T_cam (Camera pose expressed in Base frame)\n"
            "17) Keep the chessboard visible and sharp in the camera feed.\n"
            "18) Click Compute base_T_cam. Target reproj_rmse: < 1.0 px.\n"
            "19) Click Validate base_T_cam. Target: pos_err <= 3 mm, rot_err <= 1.5 deg.\n"
            "20) Click Simulate calibration (Monte Carlo 200x, noise=0.35 px).\n"
            "    Target: t_err p95 < 2 mm, R_err p95 < 0.5 deg.\n"
            "21) Verify matrix and click Save base_T_cam (configs/transforms.json).\n"
            "\n"
            "E) marker_T_obj (Object center relative to marker frame)\n"
            "22) Open Calibration -> Marker-Obj.\n"
            "23) Set Face axis = cube axis pointing toward the marker; Sign = +1 or -1.\n"
            "24) Click Compute marker_T_obj, then Save marker_T_obj (configs/markers.json).\n"
            "\n"
            "F) Perception test\n"
            "25) Open Calibration -> Perception.\n"
            "26) Click Detect Once. Verify position, quaternion and confidence.\n"
            "\n"
            "Troubleshooting\n"
            "- Board not found: check lighting, focus, pattern cols/rows, square size.\n"
            "- High RMS intrinsics: add more varied samples or check board flatness.\n"
            "- Large pos/rot errors: recalibrate intrinsics or re-capture board corners.\n"
            "- Rotated pose: verify P0/P1/P2 order matches camera view orientation.\n"
            "- Wrong base_T_cam: board moved after base_T_board was captured.\n"
            "\n"
            "Sketch (board calibration — positions)\n"
            "P0 = (0,0)  P1 = (cols-1,0)  P2 = (0,rows-1)\n"
            "\n"
            "Positionierung (kurz):\n"
            "- Robot Basis: Wähle festen Robot-Nullpunkt R0; Robot Z zeigt nach oben; Robot +X zeigt in Arbeitsrichtung (zum Board).\n"
            "- Board: Lege das Schachbrett flach auf die Arbeitsfläche. Markiere P0 physisch; P1 definiert +X (Spalten), P2 definiert +Y (Reihen).\n"
            "- Kamera: In diesem Setup ist die Kamera unterhalb des Boards montiert (Kamera blickt nach oben).\n"
            "  Richte die optische Achse auf die Board-Mitte; Bild-X möglichst parallel zu Board +X.\n"
            "  Bei Unterbau: achte auf Bildrotation/Invertierung (evtl. Flip) und korrigiere P0/P1/P2-Reihenfolge falls nötig.\n"
            "\n"
            "Schema (Top-View, Kamera unterhalb):\n"
            "  P2 o----o P1   <- +X (cols)\n"
            "  |\n"
            "  P0\n"
            "     ^\n"
            "     | (optische Achse)\n"
            "  Camera (unten)\n"
            "\n"
            "Robot base -> (Robot +X toward board; Robot Z up)\n"
            "\n"
            "Prüf-Schritte:\n"
            "1) Detect Once: Visualisiere die projizierten Board-Achsen im Kamerabild; Board +X sollte mit Robot +X übereinstimmen.\n"
            "2) Bei ~90°-Fehler: prüfe P0/P1/P2-Reihenfolge, Bildrotation (Kamera-Mount) und Tool-Frame-Konventionen.\n"
        )
        help_text = tk.Text(tab, wrap="word")
        help_text.insert("1.0", text)
        help_text.configure(state="disabled")
        help_text.grid(row=0, column=0, sticky="nsew", padx=8, pady=8)

    def _dump_layout_sizes(self):
        """Diagnostic: log requested heights of each Notebook tab and apply a defensive max-height.

        This helps identify which sub-frame is forcing the main window to grow and applies
        a safe upper bound so bottom controls remain visible.
        """
        try:
            nb = getattr(self, "_tabs", None)
            if nb is None:
                return
            try:
                screen_h = self.winfo_toplevel().winfo_screenheight()
            except Exception:
                screen_h = 800
            entries = []
            for tab_id in nb.tabs():
                try:
                    w = nb.nametowidget(tab_id)
                except Exception:
                    continue
                name = nb.tab(tab_id, "text") or str(w)
                req_h = w.winfo_reqheight()
                req_w = w.winfo_reqwidth()
                entries.append((name, req_w, req_h))
            # log the findings (both to UI log and to console for diagnostics)
            for name, rw, rh in entries:
                msg = f"LayoutDiag: tab '{name}' req_w={rw} req_h={rh}"
                try:
                    self._log(msg)
                except Exception:
                    pass
                try:
                    print(msg)
                except Exception:
                    pass

            # defensive enforcement: don't let notebook exceed most of the screen
            max_allowed = max(300, int(screen_h - 200))
            # current configured height attempt
            try:
                cur_h = int(nb.cget("height"))
            except Exception:
                cur_h = None
            # if any tab wants more than allowed, reduce notebook height
            if any(rh > max_allowed for _, _, rh in entries):
                try:
                    nb.configure(height=min(600, max_allowed))
                    msg2 = f"LayoutDiag: enforced notebook max height {min(600,max_allowed)}"
                    try:
                        self._log(msg2)
                    except Exception:
                        pass
                    try:
                        print(msg2)
                    except Exception:
                        pass
                except Exception:
                    pass
        except Exception:
            pass

    def _make_matrix_grid(self, parent, title, readonly=False):
        frame = ttk.LabelFrame(parent, text=title)
        vars_grid = []
        for r in range(4):
            row_vars = []
            for c in range(4):
                var = tk.StringVar(value="0.0")
                ent = ttk.Entry(frame, textvariable=var, width=9, justify="right")
                if readonly:
                    ent.configure(state="readonly")
                ent.grid(row=r, column=c, padx=2, pady=2)
                row_vars.append(var)
            vars_grid.append(row_vars)
        return frame, vars_grid

    def _set_matrix_vars(self, vars_grid, matrix):
        if vars_grid is None or matrix is None:
            return
        for r in range(4):
            for c in range(4):
                try:
                    val = float(matrix[r][c])
                except Exception:
                    val = 0.0
                vars_grid[r][c].set(f"{val:.6f}")

    def _read_matrix_vars(self, vars_grid):
        matrix = []
        for r in range(4):
            row = []
            for c in range(4):
                raw = vars_grid[r][c].get().replace(",", ".")
                row.append(float(raw))
            matrix.append(row)
        return matrix

    def _identity_matrix(self):
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _ensure_configs(self):
        if self._configs is None:
            self._configs = config_loader.load_all_configs(self._base_dir)
            self._render_config()

    def _write_config(self, key, data):
        rel_path = config_loader.CONFIG_FILES.get(key)
        if not rel_path:
            raise RuntimeError(f"Unknown config key: {key}")
        path = os.path.join(self._base_dir, rel_path)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        self._configs[key] = data
        self._render_config()

    def _load_calibration_into_ui(self):
        try:
            self._ensure_configs()
            calib = self._configs.get("calibration", {})
            pattern = calib.get("board_pattern", {})
            size = pattern.get("pattern_size", [7, 7])
            if len(size) >= 2:
                self._pattern_cols_var.set(int(size[0]))
                self._pattern_rows_var.set(int(size[1]))
            self._square_size_var.set(float(pattern.get("square_size_mm", 30.0)))
            base_T_board = calib.get("base_T_board", self._identity_matrix())
            transforms = self._configs.get("transforms", {})
            base_T_cam = transforms.get("base_T_cam", self._identity_matrix())
            markers = self._configs.get("markers", {})
            marker_T_obj = markers.get("marker_T_obj", self._identity_matrix())
            self._set_matrix_vars(self._base_T_board_vars, base_T_board)
            self._set_matrix_vars(self._base_T_cam_vars, base_T_cam)
            self._set_matrix_vars(self._marker_T_obj_vars, marker_T_obj)
            self._load_tnt_into_ui()
            self._log("Calibration config loaded.")
        except Exception as exc:
            self._log(f"Calibration load failed: {exc}")

    def _save_board_config(self):
        try:
            self._ensure_configs()
            cols = int(self._pattern_cols_var.get())
            rows = int(self._pattern_rows_var.get())
            square = float(self._square_size_var.get())
            base_T_board = self._read_matrix_vars(self._base_T_board_vars)
            calib = self._configs.get("calibration", {})
            calib["board_pattern"] = {"pattern_size": [cols, rows], "square_size_mm": square}
            calib["base_T_board"] = base_T_board
            self._write_config("calibration", calib)
            self._log("Calibration board config saved.")
        except Exception as exc:
            self._log(f"Save board config failed: {exc}")

    def _compute_base_T_cam(self):
        def _task():
            try:
                self._ensure_configs()
                cols = int(self._pattern_cols_var.get())
                rows = int(self._pattern_rows_var.get())
                square = float(self._square_size_var.get())
                base_T_board = self._read_matrix_vars(self._base_T_board_vars)
                cam_cfg = self._configs.get("camera", {})
                camera_matrix, dist = self._camera_model_from_cfg(cam_cfg)
                objp, corners2 = self._detect_board_points(cols, rows, square, cam_cfg)

                import cv2
                ok, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist)
                if not ok:
                    raise RuntimeError("solvePnP failed")

                proj, _ = cv2.projectPoints(objp, rvec, tvec, camera_matrix, dist)
                reproj = proj.reshape(-1, 2) - corners2.reshape(-1, 2)
                reproj_rmse = float((reproj[:, 0] ** 2 + reproj[:, 1] ** 2).mean() ** 0.5)

                R_cam, _ = cv2.Rodrigues(rvec)
                cam_T_board = make_transform(R_cam.tolist(), tvec.reshape(-1).tolist())
                base_T_cam = matmul(base_T_board, invert_transform(cam_T_board))

                self._set_matrix_vars(self._base_T_cam_vars, base_T_cam)
                self._set_status("base_T_cam computed")
                self._log(f"base_T_cam computed from chessboard. reproj_rmse={reproj_rmse:.3f} px")
            except Exception as exc:
                self._set_status("base_T_cam error")
                self._log(f"Compute base_T_cam failed: {exc}")
        self._start_worker(_task)

    @staticmethod
    def _rot_error_deg(R_a, R_b):
        t = (
            R_a[0][0] * R_b[0][0] + R_a[1][0] * R_b[1][0] + R_a[2][0] * R_b[2][0]
            + R_a[0][1] * R_b[0][1] + R_a[1][1] * R_b[1][1] + R_a[2][1] * R_b[2][1]
            + R_a[0][2] * R_b[0][2] + R_a[1][2] * R_b[1][2] + R_a[2][2] * R_b[2][2]
        )
        cos_theta = max(-1.0, min(1.0, (t - 1.0) * 0.5))
        return math.degrees(math.acos(cos_theta))

    def _camera_model_from_cfg(self, cam_cfg):
        import numpy as np

        intr = cam_cfg.get("intrinsics", {})
        fx = float(intr.get("fx", 800.0))
        fy = float(intr.get("fy", 800.0))
        image_size = cam_cfg.get("image_size", [640, 480])
        w = float(image_size[0]) if isinstance(image_size, list) and len(image_size) >= 2 else 640.0
        h = float(image_size[1]) if isinstance(image_size, list) and len(image_size) >= 2 else 480.0
        cx = float(intr.get("cx", w / 2.0))
        cy = float(intr.get("cy", h / 2.0))
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
        dist = np.array(cam_cfg.get("distortion", [0, 0, 0, 0, 0]), dtype=np.float32)
        return camera_matrix, dist

    def _detect_board_points(self, cols, rows, square, cam_cfg):
        image, color_order = self._get_calibration_image(cam_cfg)
        if image is None:
            raise RuntimeError("No camera frame available")

        import cv2
        import numpy as np

        if len(image.shape) == 3:
            if color_order == "rgb":
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            else:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        try:
            flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
            found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
        except Exception:
            found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
        if not found:
            raise RuntimeError("Chessboard not found")

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        objp = np.zeros((rows * cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        objp *= float(square)
        return objp, corners2

    def _validate_base_T_cam_once(self):
        def _task():
            try:
                self._ensure_configs()
                cols = int(self._pattern_cols_var.get())
                rows = int(self._pattern_rows_var.get())
                square = float(self._square_size_var.get())
                base_T_board_cfg = self._read_matrix_vars(self._base_T_board_vars)
                base_T_cam_cfg = self._read_matrix_vars(self._base_T_cam_vars)
                cam_cfg = self._configs.get("camera", {})
                objp, corners2 = self._detect_board_points(cols, rows, square, cam_cfg)
                camera_matrix, dist = self._camera_model_from_cfg(cam_cfg)

                import cv2
                ok, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist)
                if not ok:
                    raise RuntimeError("solvePnP failed")
                R_cam, _ = cv2.Rodrigues(rvec)
                cam_T_board_meas = make_transform(R_cam.tolist(), tvec.reshape(-1).tolist())
                base_T_board_meas = matmul(base_T_cam_cfg, cam_T_board_meas)

                t_cfg = extract_translation(base_T_board_cfg)
                t_meas = extract_translation(base_T_board_meas)
                pos_err = math.sqrt(sum((t_cfg[i] - t_meas[i]) ** 2 for i in range(3)))

                R_cfg = [row[:3] for row in base_T_board_cfg[:3]]
                R_meas = [row[:3] for row in base_T_board_meas[:3]]
                ang_err = self._rot_error_deg(R_cfg, R_meas)
                status = "OK" if pos_err <= 3.0 and ang_err <= 1.5 else "WARN"
                self._log(
                    f"base_T_cam validation ({status}): board_pos_err={pos_err:.2f} mm "
                    f"board_rot_err={ang_err:.2f} deg"
                )
            except Exception as exc:
                self._log(f"Validate base_T_cam failed: {exc}")

        self._start_worker(_task)

    def _simulate_base_cam_noise(self):
        def _task():
            try:
                self._ensure_configs()
                cols = int(self._pattern_cols_var.get())
                rows = int(self._pattern_rows_var.get())
                square = float(self._square_size_var.get())
                base_T_board = self._read_matrix_vars(self._base_T_board_vars)
                base_T_cam = self._read_matrix_vars(self._base_T_cam_vars)
                cam_cfg = self._configs.get("camera", {})

                import cv2
                import numpy as np

                camera_matrix, dist = self._camera_model_from_cfg(cam_cfg)
                cam_T_board = matmul(invert_transform(base_T_cam), base_T_board)
                R = np.array([row[:3] for row in cam_T_board[:3]], dtype=np.float32)
                t = np.array(extract_translation(cam_T_board), dtype=np.float32).reshape(3, 1)
                rvec, _ = cv2.Rodrigues(R)

                objp = np.zeros((rows * cols, 3), np.float32)
                objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
                objp *= float(square)
                ideal, _ = cv2.projectPoints(objp, rvec, t, camera_matrix, dist)
                ideal = ideal.reshape(-1, 2)

                rng = np.random.default_rng(6300)
                sigma_px = 0.35
                trials = 200
                trans_err = []
                rot_err = []
                for _ in range(trials):
                    noisy = ideal + rng.normal(0.0, sigma_px, ideal.shape)
                    noisy = noisy.astype(np.float32).reshape(-1, 1, 2)
                    ok, rvec_n, tvec_n = cv2.solvePnP(objp, noisy, camera_matrix, dist)
                    if not ok:
                        continue
                    R_n, _ = cv2.Rodrigues(rvec_n)
                    cam_T_board_n = make_transform(R_n.tolist(), tvec_n.reshape(-1).tolist())
                    base_T_cam_n = matmul(base_T_board, invert_transform(cam_T_board_n))

                    t_true = extract_translation(base_T_cam)
                    t_est = extract_translation(base_T_cam_n)
                    trans_err.append(math.sqrt(sum((t_true[i] - t_est[i]) ** 2 for i in range(3))))

                    R_true = [row[:3] for row in base_T_cam[:3]]
                    R_est = [row[:3] for row in base_T_cam_n[:3]]
                    rot_err.append(self._rot_error_deg(R_true, R_est))

                if not trans_err:
                    raise RuntimeError("No valid Monte-Carlo trials.")

                trans_arr = np.array(trans_err, dtype=float)
                rot_arr = np.array(rot_err, dtype=float)
                self._log(
                    "Simulation base_T_cam (200x, noise=0.35 px): "
                    f"t_err mean={trans_arr.mean():.2f} mm p95={np.percentile(trans_arr,95):.2f} mm, "
                    f"R_err mean={rot_arr.mean():.2f} deg p95={np.percentile(rot_arr,95):.2f} deg"
                )
            except Exception as exc:
                self._log(f"Simulate calibration failed: {exc}")

        self._start_worker(_task)

    def _save_base_T_cam(self):
        try:
            self._ensure_configs()
            base_T_cam = self._read_matrix_vars(self._base_T_cam_vars)
            transforms = self._configs.get("transforms", {})
            transforms["base_T_cam"] = base_T_cam
            self._write_config("transforms", transforms)
            if self._pipeline is not None:
                self._pipeline.context.perception.base_T_cam = base_T_cam
            self._log("base_T_cam saved.")
        except Exception as exc:
            self._log(f"Save base_T_cam failed: {exc}")

    def _compute_marker_T_obj(self):
        try:
            self._ensure_configs()
            cube_cfg = self._configs.get("cube", {})
            edge = float(cube_cfg.get("edge_length_mm", 50.0))
            axis = (self._marker_axis_var.get() or "z").lower()
            sign = int(self._marker_sign_var.get())
            normal = [0.0, 0.0, 0.0]
            if axis == "x":
                normal[0] = float(sign)
            elif axis == "y":
                normal[1] = float(sign)
            else:
                normal[2] = float(sign)

            t = [-normal[0] * edge / 2.0, -normal[1] * edge / 2.0, -normal[2] * edge / 2.0]
            R = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
            marker_T_obj = make_transform(R, t)
            self._set_matrix_vars(self._marker_T_obj_vars, marker_T_obj)
            self._log("marker_T_obj computed (rotation assumes marker axes align with object axes).")
        except Exception as exc:
            self._log(f"Compute marker_T_obj failed: {exc}")

    def _save_marker_T_obj(self):
        try:
            self._ensure_configs()
            marker_T_obj = self._read_matrix_vars(self._marker_T_obj_vars)
            markers = self._configs.get("markers", {})
            markers["marker_T_obj"] = marker_T_obj
            self._write_config("markers", markers)
            if self._pipeline is not None:
                self._pipeline.context.perception.marker_T_obj = marker_T_obj
            self._log("marker_T_obj saved.")
        except Exception as exc:
            self._log(f"Save marker_T_obj failed: {exc}")

    def _detect_once(self):
        def _task():
            try:
                self._ensure_pipeline()
                self._attach_camera_capture()
                pose = self._pipeline.context.perception.detect_object_pose()
                msg = (
                    f"pos={pose.position_mm} quat={pose.quaternion_xyzw} "
                    f"conf={pose.confidence:.3f}"
                )
                self._detect_var.set(msg)
                self._log(f"Detect ok: {msg}")
                self._update_detection_overlay()
            except Exception as exc:
                self._detect_var.set(f"Detect failed: {exc}")
                self._log(f"Detect failed: {exc}")
        self._start_worker(_task)

    def _load_tnt_into_ui(self):
        try:
            tnt_cfg = self._configs.get("tnt", {})
            ranges = tnt_cfg.get("hsv_red_ranges", [])
            if len(ranges) >= 1 and len(ranges[0]) >= 6:
                r1 = ranges[0]
                self._tnt_h1_low_var.set(int(r1[0]))
                self._tnt_s_min_var.set(int(r1[1]))
                self._tnt_v_min_var.set(int(r1[2]))
                self._tnt_h1_high_var.set(int(r1[3]))
            if len(ranges) >= 2 and len(ranges[1]) >= 6:
                r2 = ranges[1]
                self._tnt_h2_low_var.set(int(r2[0]))
                self._tnt_h2_high_var.set(int(r2[3]))
            self._tnt_min_area_var.set(int(tnt_cfg.get("min_area_px", 500)))
            self._tnt_kernel_var.set(int(tnt_cfg.get("morph_kernel", 3)))
        except Exception as exc:
            self._log(f"TNT load failed: {exc}")

    def _build_tnt_config(self):
        h1_low = int(self._tnt_h1_low_var.get())
        h1_high = int(self._tnt_h1_high_var.get())
        h2_low = int(self._tnt_h2_low_var.get())
        h2_high = int(self._tnt_h2_high_var.get())
        s_min = int(self._tnt_s_min_var.get())
        v_min = int(self._tnt_v_min_var.get())
        min_area = int(self._tnt_min_area_var.get())
        kernel = int(self._tnt_kernel_var.get())
        if kernel % 2 == 0:
            kernel += 1
        tnt_cfg = dict(self._configs.get("tnt", {}))
        tnt_cfg["hsv_red_ranges"] = [
            [h1_low, s_min, v_min, h1_high, 255, 255],
            [h2_low, s_min, v_min, h2_high, 255, 255],
        ]
        tnt_cfg["min_area_px"] = min_area
        tnt_cfg["morph_kernel"] = kernel
        return tnt_cfg

    def _apply_tnt_filters(self):
        try:
            self._ensure_configs()
            tnt_cfg = self._build_tnt_config()
            self._configs["tnt"] = tnt_cfg
            if self._pipeline is not None:
                self._pipeline.context.perception.tnt_cfg = tnt_cfg
            if self._execute_app is not None and hasattr(self._execute_app, "board_detector"):
                try:
                    self._execute_app.board_detector.set_tnt_cfg(tnt_cfg)
                except Exception:
                    pass
            self._log("TNT filters applied.")
        except Exception as exc:
            self._log(f"Apply TNT filters failed: {exc}")

    def _save_tnt_filters(self):
        try:
            self._ensure_configs()
            tnt_cfg = self._build_tnt_config()
            self._write_config("tnt", tnt_cfg)
            if self._pipeline is not None:
                self._pipeline.context.perception.tnt_cfg = tnt_cfg
            if self._execute_app is not None and hasattr(self._execute_app, "board_detector"):
                try:
                    self._execute_app.board_detector.set_tnt_cfg(tnt_cfg)
                except Exception:
                    pass
            self._log("TNT filters saved.")
        except Exception as exc:
            self._log(f"Save TNT filters failed: {exc}")

    def _rpy_from_R(self, R):
        sy = math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0])
        singular = sy < 1e-6
        if not singular:
            roll = math.degrees(math.atan2(R[2][1], R[2][2]))
            pitch = math.degrees(math.atan2(-R[2][0], sy))
            yaw = math.degrees(math.atan2(R[1][0], R[0][0]))
        else:
            roll = math.degrees(math.atan2(-R[1][2], R[1][1]))
            pitch = math.degrees(math.atan2(-R[2][0], sy))
            yaw = 0.0
        return roll, pitch, yaw

    @staticmethod
    def _wrap_angle_deg(angle):
        return ((angle + 180.0) % 360.0) - 180.0

    def _rpy_to_R(self, roll_deg, pitch_deg, yaw_deg):
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

    def _current_rpy(self):
        tcp = self._execute_app.get_current_tcp_mm()
        roll = float(tcp.get("Roll_deg", 0.0))
        pitch = float(tcp.get("Pitch_deg", tcp.get("Tilt_deg", 0.0)))
        yaw = float(tcp.get("Yaw_deg", 0.0))
        R = self._rpy_to_R(roll, pitch, yaw)
        # If tool Z points up, flip orientation to point down.
        if R[2][2] > 0.0:
            roll = self._wrap_angle_deg(roll + 180.0)
        return roll, pitch, yaw

    def _get_speed_slider_feed(self, fallback: float) -> float:
        if self._execute_app is None:
            return fallback
        try:
            speed_val = getattr(self._execute_app, "speed_val", None)
            max_feed = getattr(self._execute_app, "max_feed", None)
            if speed_val is None or max_feed is None:
                return fallback
            val = float(speed_val.get())
            max_f = float(max_feed.get())
            feed = (val / 1000.0) * max_f
            msf = getattr(self._execute_app, "manual_speed_factor", None)
            if msf is not None:
                try:
                    feed *= float(msf.get())
                except Exception:
                    pass
            if feed <= 0.0:
                return fallback
            return feed
        except Exception:
            return fallback

    def _wait_for_idle(self, timeout_s=30.0):
        if self._execute_app is None or not hasattr(self._execute_app, "client"):
            return True
        start = time.time()
        while time.time() - start < timeout_s:
            try:
                st = getattr(self._execute_app.client, "last_status", None) or {}
                state = str(st.get("state", "")).lower()
                if "idle" in state or state == "":
                    return True
            except Exception:
                pass
            time.sleep(0.05)
        return False

    def _pickup_test(self):
        def _task():
            try:
                if self._execute_app is None or not hasattr(self._execute_app, "kinematics_tabs"):
                    raise RuntimeError("Kinematics UI not available")
                self._ensure_pipeline()
                self._attach_camera_capture()

                pose = self._pipeline.context.perception.detect_object_pose()
                self._update_detection_overlay()
                cube_cfg = self._configs.get("cube", {})
                obj_pos = extract_translation(pose.matrix)
                self._check_calibration_sanity(obj_pos)
                normal = self._resolve_grasp_normal(cube_cfg)
                base_T_cam = None
                try:
                    base_T_cam = getattr(self._pipeline.context.perception, "base_T_cam", None)
                except Exception:
                    base_T_cam = None
                grasp_offset = float(cube_cfg.get("grasp_offset_mm", 0.0))
                approach_dist = float(cube_cfg.get("approach_distance_mm", 0.0))
                lift_dist = float(cube_cfg.get("lift_distance_mm", 0.0))
                grasp_pos = [
                    obj_pos[0] + normal[0] * grasp_offset,
                    obj_pos[1] + normal[1] * grasp_offset,
                    obj_pos[2] + normal[2] * grasp_offset,
                ]
                approach_pos = [
                    grasp_pos[0] + normal[0] * approach_dist,
                    grasp_pos[1] + normal[1] * approach_dist,
                    grasp_pos[2] + normal[2] * approach_dist,
                ]
                lift_pos = [
                    grasp_pos[0],
                    grasp_pos[1],
                    grasp_pos[2] + lift_dist,
                ]
                self._log(
                    f"Pick plan: obj={obj_pos} approach={approach_pos} grasp={grasp_pos} lift={lift_pos}"
                )
                if base_T_cam:
                    self._log(f"base_T_cam: {base_T_cam}")
                for label, p in [("approach", approach_pos), ("grasp", grasp_pos), ("lift", lift_pos)]:
                    if not self._check_workspace_bounds(p, label=label):
                        raise RuntimeError(f"Workspace check failed for {label} target")

                robot_cfg = self._configs.get("robot", {})
                base_feed = self._get_speed_slider_feed(float(robot_cfg.get("approach_speed", 60.0)))
                approach_feed = base_feed
                grasp_feed = base_feed
                lift_feed = base_feed

                def _move_pos_split(pos, fast_feed, slow_feed):
                    roll, pitch, yaw = self._current_rpy()
                    tcp = self._execute_app.get_current_tcp_mm()
                    start_pos = [
                        float(tcp.get("X_mm", 0.0)),
                        float(tcp.get("Y_mm", 0.0)),
                        float(tcp.get("Z_mm", 0.0)),
                    ]
                    mid = [
                        start_pos[0] + 0.9 * (pos[0] - start_pos[0]),
                        start_pos[1] + 0.9 * (pos[1] - start_pos[1]),
                        start_pos[2] + 0.9 * (pos[2] - start_pos[2]),
                    ]
                    ok = self._execute_app.kinematics_tabs.move_tcp_pose(
                        mid[0],
                        mid[1],
                        mid[2],
                        roll,
                        pitch,
                        yaw,
                        feed=fast_feed,
                    )
                    if not ok:
                        return False
                    self._wait_for_idle()
                    return self._execute_app.kinematics_tabs.move_tcp_pose(
                        pos[0],
                        pos[1],
                        pos[2],
                        roll,
                        pitch,
                        yaw,
                        feed=slow_feed,
                    )

                self._execute_app.send_now("M3 S0")
                time.sleep(0.5)  # post-gripper-open: wait for servo to fully open
                slow_feed = 300.0
                if not _move_pos_split(approach_pos, approach_feed, slow_feed):
                    raise RuntimeError("Approach move failed")
                self._wait_for_idle()
                if not _move_pos_split(grasp_pos, grasp_feed, slow_feed):
                    raise RuntimeError("Grasp move failed")
                self._execute_app.send_now("M3 S1000")
                self._wait_for_idle()
                if not _move_pos_split(lift_pos, lift_feed, slow_feed):
                    raise RuntimeError("Lift move failed")
                self._log("Pick up test complete.")
            except Exception as exc:
                self._log(f"Pick up test failed: {exc}")
        self._start_worker(_task)

    def _resolve_grasp_normal(self, cube_cfg):
        axis = str(cube_cfg.get("grasp_normal_axis", "z")).strip().lower()
        sign = float(cube_cfg.get("grasp_normal_sign", 1.0))
        sign = 1.0 if sign >= 0 else -1.0
        if axis == "x":
            return [1.0 * sign, 0.0, 0.0]
        if axis == "y":
            return [0.0, 1.0 * sign, 0.0]
        return [0.0, 0.0, 1.0 * sign]

    def _check_board_bounds(self, pos, margin_mm=20.0, z_tol_mm=80.0):
        try:
            calib = self._configs.get("calibration", {})
            pattern = calib.get("board_pattern", {})
            cols = int(pattern.get("pattern_size", [0, 0])[0])
            rows = int(pattern.get("pattern_size", [0, 0])[1])
            square = float(pattern.get("square_size_mm", 0.0))
            base_T_board = calib.get("base_T_board")
            if not (cols and rows and square and base_T_board):
                return True
            board_w = (cols - 1) * square
            board_h = (rows - 1) * square
            p = [float(pos[0]), float(pos[1]), float(pos[2])]
            p_board = matmul(invert_transform(base_T_board), make_transform([[1,0,0],[0,1,0],[0,0,1]], p))
            bx, by, bz = p_board[0][3], p_board[1][3], p_board[2][3]
            self._log(f"Board coords: x={bx:.2f} y={by:.2f} z={bz:.2f}")
            if bx < -margin_mm or bx > board_w + margin_mm:
                return False
            if by < -margin_mm or by > board_h + margin_mm:
                return False
            if abs(bz) > z_tol_mm:
                return False
            return True
        except Exception:
            return True

    def _check_workspace_bounds(self, pos, label="target"):
        try:
            robot_cfg = self._configs.get("robot", {})
            ws = robot_cfg.get("workspace", {})
            if not bool(ws.get("enabled", False)):
                return True
            margin = float(ws.get("margin_mm", 0.0))
            x_min, x_max = ws.get("x", [None, None])
            y_min, y_max = ws.get("y", [None, None])
            z_min, z_max = ws.get("z", [None, None])
            x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
            if x_min is not None and x < float(x_min) - margin:
                self._log(f"Workspace: {label} x={x:.2f} < {x_min}")
                return False
            if x_max is not None and x > float(x_max) + margin:
                self._log(f"Workspace: {label} x={x:.2f} > {x_max}")
                return False
            if y_min is not None and y < float(y_min) - margin:
                self._log(f"Workspace: {label} y={y:.2f} < {y_min}")
                return False
            if y_max is not None and y > float(y_max) + margin:
                self._log(f"Workspace: {label} y={y:.2f} > {y_max}")
                return False
            if z_min is not None and z < float(z_min) - margin:
                self._log(f"Workspace: {label} z={z:.2f} < {z_min}")
                return False
            if z_max is not None and z > float(z_max) + margin:
                self._log(f"Workspace: {label} z={z:.2f} > {z_max}")
                return False
            return True
        except Exception:
            return True

    def _check_calibration_sanity(self, obj_pos):
        if not self._check_board_bounds(obj_pos):
            raise RuntimeError("Detected pose outside board bounds (calibration mismatch)")
        try:
            base_T_cam = None
            if self._pipeline is not None:
                base_T_cam = getattr(self._pipeline.context.perception, "base_T_cam", None)
            if not base_T_cam:
                raise RuntimeError("base_T_cam missing")
            R = [row[:3] for row in base_T_cam[:3]]
            # Orthonormal check (R^T R ≈ I) and det ~ 1
            rt = [[R[0][0], R[1][0], R[2][0]],
                  [R[0][1], R[1][1], R[2][1]],
                  [R[0][2], R[1][2], R[2][2]]]
            def dot(a, b):
                return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
            cols = [rt[0], rt[1], rt[2]]
            ortho_err = abs(dot(cols[0], cols[1])) + abs(dot(cols[0], cols[2])) + abs(dot(cols[1], cols[2]))
            det = (
                R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1]) -
                R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0]) +
                R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0])
            )
            if ortho_err > 0.05 or abs(det - 1.0) > 0.05:
                self._log(f"base_T_cam sanity: ortho_err={ortho_err:.3f} det={det:.3f}")
        except RuntimeError:
            raise
        except Exception:
            pass

    def _ui(self, fn):
        self.after(0, fn)

    def _log(self, message: str):
        if self._logger:
            try:
                self._logger(message)
            except Exception:
                pass
        def _write():
            self._log_text.insert(tk.END, message + "\n")
            self._log_text.see(tk.END)
        self._ui(_write)

    def _set_status(self, text: str):
        self._ui(lambda: self._status_var.set(text))

    def _refresh_stack_index(self):
        idx = "n/a"
        try:
            if self._pipeline is not None:
                idx = str(int(getattr(self._pipeline.context, "stack_index", 0)))
        except Exception:
            idx = "n/a"
        self._ui(lambda: self._stack_index_var.set(f"Stack index: {idx}"))

    def _reset_stack_index(self):
        try:
            self._ensure_pipeline()
            self._pipeline.context.stack_index = 0
            self._refresh_stack_index()
            self._log("Stack index reset to 0.")
        except Exception as exc:
            self._log(f"Stack index reset failed: {exc}")

    def _init_pipeline(self):
        try:
            self._pipeline, self._configs = build_pipeline(self._base_dir)
            self._attach_camera_capture()
            lr = self._ensure_learner()
            lr.sync_baseline_from_configs(self._configs)
            self._refresh_learning_from_manager()
            sim_cfg = self._configs.get("simulation", {})
            cycles = int(sim_cfg.get("cycles", 1000))
            self._sim_cycles.set(cycles)
            self._render_config()
            self._load_calibration_into_ui()
            self._refresh_stack_index()
            self._set_status("Pipeline ready")
            self._log("Pipeline initialized.")
        except Exception as exc:
            self._set_status("Init failed")
            self._log(f"Init failed: {exc}")

    def _ensure_pipeline(self):
        if self._pipeline is None:
            self._init_pipeline()
        if self._pipeline is None:
            raise RuntimeError("Pipeline not initialized")
        self._attach_camera_capture()

    def _attach_camera_capture(self):
        if self._pipeline is None:
            return
        if self._camera_capture is None:
            return
        try:
            current = getattr(self._pipeline.context.perception, "camera", None)
            restart_needed = False
            if current is not None and current is not self._camera_capture:
                cap = getattr(current, "cap", None)
                if cap is not None:
                    try:
                        cap.release()
                        restart_needed = True
                    except Exception:
                        pass
            self._pipeline.context.perception.camera = _CameraCaptureAdapter(self._camera_capture)
            try:
                running = bool(getattr(self._camera_capture, "running", False))
                if restart_needed and running:
                    self._camera_capture.stop()
                    self._camera_capture.start()
                elif not running:
                    self._camera_capture.start()
            except Exception:
                pass
        except Exception:
            pass

    def _update_detection_overlay(self):
        try:
            if self._camera_capture is None:
                return
            image = self._camera_capture.get_latest_frame()
            if image is None:
                return
            import cv2
            from perception.tnt_detector import detect_tnt_contour
            tnt_cfg = self._configs.get("tnt", {}) if self._configs else {}
            cnt, bgr = detect_tnt_contour(image, tnt_cfg, camera=_CameraCaptureAdapter(self._camera_capture))
            if bgr is None:
                return
            if cnt is not None:
                cv2.drawContours(bgr, [cnt], -1, (255, 0, 0), 2)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            if self._execute_app is not None and hasattr(self._execute_app, "board_detector"):
                self._execute_app.board_detector.last_vis = rgb
        except Exception:
            pass

    def _ensure_learner(self):
        if self._learner is None:
            self._learner = TntSelfLearningManager(self._base_dir, logger=self._log)
        return self._learner

    def _refresh_learning_from_manager(self):
        lr = self._ensure_learner()
        st = lr.get_status()
        self._learning_enabled_var.set(bool(st.get("enabled", False)))
        self._learning_mode_var.set(str(st.get("mode", "shadow")))
        self._learning_status_var.set(
            f"Learning: {st.get('mode', 'shadow')} "
            f"eps={st.get('epsilon', 0.0):.3f} "
            f"succ={st.get('successes', 0)}/{st.get('attempts', 0)}"
        )

    def _on_learning_toggle(self):
        lr = self._ensure_learner()
        lr.set_enabled(bool(self._learning_enabled_var.get()))
        lr.save_settings()
        self._refresh_learning_from_manager()
        self._log(f"Learning enabled set to {bool(self._learning_enabled_var.get())}.")

    def _on_learning_mode_change(self):
        lr = self._ensure_learner()
        lr.set_mode(self._learning_mode_var.get())
        lr.save_settings()
        self._refresh_learning_from_manager()
        self._log(f"Learning mode set to {self._learning_mode_var.get()}.")

    def _show_learning_status(self):
        lr = self._ensure_learner()
        st = lr.get_status()
        self._log(
            "Learning status: "
            f"mode={st['mode']} enabled={st['enabled']} epsilon={st['epsilon']:.3f} "
            f"success_rate={st['success_rate']:.2%} attempts={st['attempts']} "
            f"failure_streak={st.get('failure_streak', 0)} last_reward={st.get('last_reward', 0.0):+.2f} "
            f"context={st.get('active_context', 'global')}"
        )

    def _reset_learning_policy(self):
        try:
            self._ensure_configs()
            lr = self._ensure_learner()
            lr.reset_policy_from_configs(self._configs)
            self._refresh_learning_from_manager()
            self._log("Learning policy reset from current config.")
        except Exception as exc:
            self._log(f"Learning reset failed: {exc}")

    def _learning_before_cycle(self, simulated: bool):
        if self._configs is None:
            return
        lr = self._ensure_learner()
        info = lr.before_cycle(self._configs, simulated=simulated, context_hint=self._last_cycle_ctx)
        self._learning_event_count += 1
        should_log = (not simulated) or (self._learning_event_count % 50 == 0)
        if info.get("applied"):
            if should_log:
                self._log(
                f"Learning candidate applied (mode={info.get('mode')}, explore={info.get('explore')})."
                )
        elif info.get("enabled"):
            if should_log:
                self._log(
                f"Learning candidate evaluated in shadow mode (explore={info.get('explore')})."
                )

    def _make_cycle_context(self, duration_s: float, err):
        ctx = {"duration_s": float(duration_s)}
        try:
            obj_pose = getattr(self._pipeline.context, "object_pose", None)
            if obj_pose is not None and hasattr(obj_pose, "confidence"):
                ctx["confidence"] = float(obj_pose.confidence)
        except Exception:
            pass
        if err is not None:
            ctx["error_type"] = type(err).__name__
        return ctx

    def _learning_after_cycle(self, success: bool, err, cycle_ctx=None):
        lr = self._ensure_learner()
        msg = "" if err is None else str(err)
        ctx = cycle_ctx or {}
        out = lr.after_cycle(bool(success), msg, cycle_ctx=ctx)
        self._last_cycle_ctx = dict(ctx)
        if out.get("updated"):
            st = lr.get_status()
            suffix = ""
            if out.get("rolled_back"):
                suffix += " rollback"
            if out.get("mode_changed"):
                suffix += f" mode={st.get('mode')}"
            self._learning_status_var.set(
                f"Learning: {st.get('mode')} "
                f"eps={out.get('epsilon', 0.0):.3f} "
                f"reward={out.get('reward', 0.0):+.2f}"
                f" fs={st.get('failure_streak', 0)}"
                f" ctx={st.get('active_context', 'global')}{suffix}"
            )

    def _start_worker(self, target):
        if self._worker and self._worker.is_alive():
            self._log("Worker already running.")
            return
        self._stop_event.clear()
        self._worker = threading.Thread(target=target, daemon=True)
        self._worker.start()

    def _run_one_cycle(self):
        def _task():
            try:
                self._ensure_pipeline()
                self._learning_before_cycle(simulated=False)
                t0 = time.time()
                ok = self._pipeline.run_cycle()
                dt = time.time() - t0
                err = self._pipeline.state_machine.last_error
                self._learning_after_cycle(ok, err, cycle_ctx=self._make_cycle_context(dt, err))
                self._refresh_stack_index()
                if ok:
                    self._set_status("Cycle ok")
                    self._log("Cycle success.")
                else:
                    self._set_status("Cycle failed")
                    self._log(f"Cycle failed: {err}")
            except Exception as exc:
                self._set_status("Cycle error")
                self._log(f"Cycle error: {exc}")
        self._start_worker(_task)

    def _run_n_cycles(self):
        def _task():
            try:
                self._ensure_pipeline()
                total = max(1, int(self._run_count.get()))
                successes = 0
                for i in range(total):
                    if self._stop_event.is_set():
                        self._log("Run stopped by user.")
                        break
                    self._learning_before_cycle(simulated=False)
                    t0 = time.time()
                    ok = self._pipeline.run_cycle()
                    dt = time.time() - t0
                    err = self._pipeline.state_machine.last_error
                    self._learning_after_cycle(ok, err, cycle_ctx=self._make_cycle_context(dt, err))
                    self._refresh_stack_index()
                    if not ok:
                        self._set_status("Cycle failed")
                        self._log(f"Cycle {i + 1} failed: {err}")
                        break
                    successes += 1
                    self._set_status(f"Success {successes}/{total}")
                self._log(f"Run complete: {successes}/{total} successes.")
            except Exception as exc:
                self._set_status("Run error")
                self._log(f"Run error: {exc}")
        self._start_worker(_task)

    def _run_simulation(self):
        def _task():
            try:
                self._ensure_pipeline()
                cycles = max(1, int(self._sim_cycles.get()))
                runner = SimulationRunner(self._pipeline, self._configs.get("perception", {}))
                def _before(_i):
                    self._learning_before_cycle(simulated=True)

                def _cb(_i, ok, err, meta):
                    self._learning_after_cycle(ok, err, cycle_ctx=meta or {})

                successes = runner.run(cycles, per_cycle=_cb, before_cycle=_before)
                self._refresh_stack_index()
                self._set_status(f"Simulation {successes}/{cycles}")
                self._log(f"Simulation complete: {successes}/{cycles} successes.")
            except Exception as exc:
                self._set_status("Simulation error")
                self._log(f"Simulation error: {exc}")
        self._start_worker(_task)

    def _stop(self):
        self._stop_event.set()
        if self._pipeline is not None:
            try:
                self._pipeline.context.safe_stop()
            except Exception:
                pass
        self._set_status("Stopped")
        self._log("Stop requested.")

    def _render_config(self):
        if self._configs is None:
            return
        payload = json.dumps(self._configs, indent=2, sort_keys=True)
        self._cfg_text.delete("1.0", tk.END)
        self._cfg_text.insert(tk.END, payload)

    def _get_calibration_image(self, cam_cfg):
        if self._camera_capture is not None:
            image = self._camera_capture.get_latest_frame()
            if image is not None:
                return image, "rgb"
            raise RuntimeError("Camera capture not running or no frame available")
        camera = None
        if self._pipeline is not None:
            camera = self._pipeline.context.perception.camera
        if camera is None:
            from perception.camera import build_camera
            camera = build_camera(cam_cfg)
        frame = None
        try:
            frame = camera.get_frame()
            color_order = getattr(camera, "color_order", "bgr")
            return frame.image, color_order
        finally:
            cap = getattr(camera, "cap", None)
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass

