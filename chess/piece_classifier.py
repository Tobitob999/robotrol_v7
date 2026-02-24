import os


_TEMPLATE_CACHE = {}


def _load_template(path):
    if path in _TEMPLATE_CACHE:
        return _TEMPLATE_CACHE[path]
    try:
        import cv2
    except Exception:
        _TEMPLATE_CACHE[path] = None
        return None
    if not os.path.isfile(path):
        _TEMPLATE_CACHE[path] = None
        return None
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    _TEMPLATE_CACHE[path] = img
    return img


def classify_piece(roi_gray, color: str, cfg: dict):
    mode = (cfg.get("piece_mode") or "tnt").lower()
    light_char = str(cfg.get("light_piece_char", "P"))
    dark_char = str(cfg.get("dark_piece_char", "p"))

    if mode != "template":
        return light_char if color == "light" else dark_char

    templates = cfg.get("templates", {})
    if not templates:
        return light_char if color == "light" else dark_char

    try:
        import cv2
        import numpy as np
    except Exception:
        return light_char if color == "light" else dark_char

    best_score = -1.0
    best_piece = None
    for piece, path in templates.items():
        tmpl = _load_template(path)
        if tmpl is None:
            continue
        if roi_gray.shape[0] < tmpl.shape[0] or roi_gray.shape[1] < tmpl.shape[1]:
            continue
        res = cv2.matchTemplate(roi_gray, tmpl, cv2.TM_CCOEFF_NORMED)
        score = float(np.max(res))
        if score > best_score:
            best_score = score
            best_piece = str(piece)

    threshold = float(cfg.get("template_threshold", 0.55))
    if best_piece is None or best_score < threshold:
        return light_char if color == "light" else dark_char

    if color == "light":
        return best_piece.upper()
    return best_piece.lower()
