import os
import tkinter as tk
from tkinter import ttk

from chess.vision_pipeline import detect_chess_state, load_config


class ChessVisionTab(ttk.Frame):
    def __init__(self, master, camera_capture=None, logger=None):
        super().__init__(master)
        self._camera_capture = camera_capture
        self._logger = logger
        self._base_dir = os.path.abspath(os.path.dirname(__file__))
        self._config = load_config(self._base_dir)
        self._status_var = tk.StringVar(value="Idle")
        self._fen_var = tk.StringVar(value="")
        self._board_text = None
        self._build_ui()

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky="ew", padx=6, pady=4)
        header.columnconfigure(2, weight=1)

        ttk.Button(header, text="Load Config", command=self._reload_config).grid(row=0, column=0, padx=(0, 6))
        ttk.Button(header, text="Detect State", command=self._detect_state).grid(row=0, column=1, padx=(0, 6))
        ttk.Label(header, textvariable=self._status_var).grid(row=0, column=2, sticky="w")

        fen_frame = ttk.LabelFrame(self, text="FEN")
        fen_frame.grid(row=1, column=0, sticky="ew", padx=6, pady=4)
        ttk.Entry(fen_frame, textvariable=self._fen_var, width=80).pack(fill="x", padx=6, pady=6)

        board_frame = ttk.LabelFrame(self, text="Board")
        board_frame.grid(row=2, column=0, sticky="nsew", padx=6, pady=4)
        board_frame.columnconfigure(0, weight=1)
        board_frame.rowconfigure(0, weight=1)
        self._board_text = tk.Text(board_frame, height=10, wrap="none")
        self._board_text.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)

        self.rowconfigure(2, weight=1)

    def _reload_config(self):
        try:
            self._config = load_config(self._base_dir)
            self._set_status("Config loaded")
        except Exception as exc:
            self._set_status(f"Config load failed: {exc}")

    def _get_image(self):
        if self._camera_capture is None:
            return None
        return self._camera_capture.get_latest_frame()

    def _detect_state(self):
        image = self._get_image()
        if image is None:
            self._set_status("No camera frame")
            return
        state, detection = detect_chess_state(image, self._config, color_order="rgb")
        if state is None:
            self._set_status("Board not found")
            return
        self._fen_var.set(state.fen)
        self._render_board(state.board)
        self._set_status(f"Confidence {state.confidence:.2f}")

    def _render_board(self, board):
        if self._board_text is None:
            return
        lines = []
        size = len(board)
        for r in range(size):
            line = " ".join(board[r])
            lines.append(line)
        self._board_text.delete("1.0", tk.END)
        self._board_text.insert(tk.END, "\n".join(lines))

    def _set_status(self, text):
        self._status_var.set(text)
        if self._logger:
            try:
                self._logger(f"Chess: {text}")
            except Exception:
                pass
