from typing import List, Tuple

from chess.types import BoardDetection


def _as_gray(image, color_order: str):
    import cv2
    if len(image.shape) == 2:
        return image
    if color_order == "rgb":
        return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def _estimate_outer_corners(corners, rows: int, cols: int):
    c = corners.reshape(rows, cols, 2)
    dx1 = c[0, 1] - c[0, 0]
    dx2 = c[rows - 1, cols - 1] - c[rows - 1, cols - 2]
    dy1 = c[1, 0] - c[0, 0]
    dy2 = c[rows - 1, cols - 1] - c[rows - 2, cols - 1]
    dx = 0.5 * (dx1 + dx2)
    dy = 0.5 * (dy1 + dy2)

    top_left = c[0, 0] - 0.5 * dx - 0.5 * dy
    top_right = c[0, cols - 1] + 0.5 * dx - 0.5 * dy
    bottom_right = c[rows - 1, cols - 1] + 0.5 * dx + 0.5 * dy
    bottom_left = c[rows - 1, 0] - 0.5 * dx + 0.5 * dy
    return [
        (float(top_left[0]), float(top_left[1])),
        (float(top_right[0]), float(top_right[1])),
        (float(bottom_right[0]), float(bottom_right[1])),
        (float(bottom_left[0]), float(bottom_left[1])),
    ]


def detect_board(image, pattern_size: Tuple[int, int], square_size_px: int, board_size_squares: int, color_order: str = "rgb"):
    try:
        import cv2
        import numpy as np
    except Exception:
        return BoardDetection(found=False, homography=None, warp_image=None, corners=None)

    if image is None:
        return BoardDetection(found=False, homography=None, warp_image=None, corners=None)

    rows, cols = int(pattern_size[1]), int(pattern_size[0])
    gray = _as_gray(image, color_order)

    try:
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags=flags)
    except Exception:
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)

    if not found or corners is None:
        return BoardDetection(found=False, homography=None, warp_image=None, corners=None)

    corners = corners.reshape(-1, 2)
    outer = _estimate_outer_corners(corners, rows, cols)

    size = int(board_size_squares * square_size_px)
    dst = [(0.0, 0.0), (float(size), 0.0), (float(size), float(size)), (0.0, float(size))]
    src_pts = np.array(outer, dtype=np.float32)
    dst_pts = np.array(dst, dtype=np.float32)

    H = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warp = cv2.warpPerspective(image, H, (size, size))

    homography = H.tolist()
    return BoardDetection(found=True, homography=homography, warp_image=warp, corners=outer)
