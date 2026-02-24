from chess.types import PieceDetection
from chess.piece_classifier import classify_piece


def _to_gray(image, color_order: str):
    try:
        import cv2
    except Exception:
        cv2 = None
    if image is None:
        return None
    if len(image.shape) == 2:
        return image
    if cv2 is None:
        return None
    if color_order == "rgb":
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def detect_pieces(warp_image, board_size_squares: int, square_size_px: int, cfg: dict, color_order: str = "rgb"):
    try:
        import numpy as np
    except Exception:
        return PieceDetection(board=[[]], confidence=0.0)

    gray = _to_gray(warp_image, color_order)
    if gray is None:
        return PieceDetection(board=[[]], confidence=0.0)

    occ_thresh = float(cfg.get("occupancy_threshold", 18.0))
    min_ratio = float(cfg.get("min_piece_ratio", 0.08))
    max_ratio = float(cfg.get("max_piece_ratio", 0.6))
    light_char = str(cfg.get("light_piece_char", "P"))
    dark_char = str(cfg.get("dark_piece_char", "p"))

    size = int(board_size_squares)
    sq = int(square_size_px)
    margin = int(max(1, sq * 0.1))

    means = [[0.0 for _ in range(size)] for _ in range(size)]
    for r in range(size):
        for c in range(size):
            y0 = r * sq + margin
            y1 = (r + 1) * sq - margin
            x0 = c * sq + margin
            x1 = (c + 1) * sq - margin
            roi = gray[y0:y1, x0:x1]
            means[r][c] = float(np.mean(roi))

    white_vals = []
    black_vals = []
    for r in range(size):
        for c in range(size):
            if (r + c) % 2 == 0:
                white_vals.append(means[r][c])
            else:
                black_vals.append(means[r][c])

    white_mean = float(np.median(white_vals)) if white_vals else 0.0
    black_mean = float(np.median(black_vals)) if black_vals else 0.0

    board = [["." for _ in range(size)] for _ in range(size)]
    confidence_sum = 0.0
    confidence_count = 0

    for r in range(size):
        for c in range(size):
            y0 = r * sq + margin
            y1 = (r + 1) * sq - margin
            x0 = c * sq + margin
            x1 = (c + 1) * sq - margin
            roi = gray[y0:y1, x0:x1]
            baseline = white_mean if (r + c) % 2 == 0 else black_mean
            diff = np.abs(roi.astype(float) - baseline)
            mask = diff > occ_thresh
            ratio = float(np.sum(mask)) / float(roi.size) if roi.size else 0.0
            confidence_sum += min(ratio / max_ratio, 1.0) if max_ratio > 0 else 0.0
            confidence_count += 1
            if ratio < min_ratio or ratio > max_ratio:
                continue
            masked_vals = roi[mask]
            if masked_vals.size == 0:
                continue
            piece_mean = float(np.mean(masked_vals))
            color = "dark" if piece_mean < baseline else "light"
            try:
                piece_char = classify_piece(roi, color, cfg)
            except Exception:
                piece_char = dark_char if color == "dark" else light_char
            board[r][c] = piece_char

    confidence = confidence_sum / confidence_count if confidence_count else 0.0
    return PieceDetection(board=board, confidence=confidence)
