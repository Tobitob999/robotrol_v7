import json
import os

from chess.board_detector import detect_board
from chess.piece_detector import detect_pieces
from chess.state import build_state


def detect_chess_state(image, config: dict, color_order: str = "rgb"):
    board_cfg = config.get("board", {})
    piece_cfg = config.get("piece_detection", {})
    pattern = board_cfg.get("pattern_size", [7, 7])
    square_px = int(board_cfg.get("square_size_px", 60))
    size = int(board_cfg.get("board_size_squares", 8))
    white_bottom = bool(board_cfg.get("white_bottom", True))

    detection = detect_board(
        image=image,
        pattern_size=(int(pattern[0]), int(pattern[1])),
        square_size_px=square_px,
        board_size_squares=size,
        color_order=color_order,
    )
    if not detection.found:
        return None, None

    warp = detection.warp_image
    if warp is None:
        return None, None

    piece_det = detect_pieces(
        warp_image=warp,
        board_size_squares=size,
        square_size_px=square_px,
        cfg=piece_cfg,
        color_order=color_order,
    )
    state = build_state(piece_det.board, white_bottom, piece_det.confidence)

    debug = config.get("debug", {})
    warp_path = debug.get("save_warp_path")
    if warp_path:
        try:
            import cv2
            cv2.imwrite(warp_path, warp)
        except Exception:
            pass

    return state, detection


def load_config(base_dir: str):
    path = os.path.join(base_dir, "configs", "chess.json")
    with open(path, "r", encoding="utf-8-sig") as f:
        return json.loads(f.read())
