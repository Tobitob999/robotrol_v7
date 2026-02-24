import cv2
import numpy as np

from control.transforms import make_transform, matmul


def _save_debug_image(path, image):
    if not path:
        return
    try:
        cv2.imwrite(path, image)
    except Exception:
        pass


def _get_color_order(camera, cfg):
    order = None
    if camera is not None:
        order = getattr(camera, "color_order", None)
    if order:
        return order
    return cfg.get("color_order", "bgr")


def _to_bgr(image, color_order):
    if image is None:
        return None
    if len(image.shape) != 3:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if color_order == "rgb":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def _build_mask(image, tnt_cfg: dict, camera=None):
    if image is None:
        return None, None, False
    order = _get_color_order(camera, tnt_cfg)
    bgr = _to_bgr(image, order)
    if bgr is None:
        return None, None, False
    debug = bool(tnt_cfg.get("debug_save", False))
    if debug:
        _save_debug_image(tnt_cfg.get("debug_input_path"), bgr)

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    ranges = tnt_cfg.get("hsv_red_ranges", [])
    mask = None
    for r in ranges:
        if len(r) != 6:
            continue
        lower = np.array(r[0:3], dtype=np.uint8)
        upper = np.array(r[3:6], dtype=np.uint8)
        part = cv2.inRange(hsv, lower, upper)
        mask = part if mask is None else cv2.bitwise_or(mask, part)

    if mask is None:
        return bgr, None, debug

    ksize = int(tnt_cfg.get("morph_kernel", 5))
    ksize = max(3, ksize | 1)
    kernel = np.ones((ksize, ksize), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if debug:
        _save_debug_image(tnt_cfg.get("debug_mask_path"), mask)
    return bgr, mask, debug


def detect_tnt_contour(image, tnt_cfg: dict, camera=None):
    bgr, mask, debug = _build_mask(image, tnt_cfg, camera=camera)
    if mask is None:
        return None, bgr

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        if debug and bgr is not None:
            _save_debug_image(tnt_cfg.get("debug_contour_path"), bgr)
        return None, bgr

    min_area = float(tnt_cfg.get("min_area_px", 1500))
    best = None
    best_area = 0.0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        if area > best_area:
            best_area = area
            best = cnt
    if debug and bgr is not None:
        contour_img = bgr.copy()
        cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 2)
        if best is not None:
            cv2.drawContours(contour_img, [best], -1, (0, 0, 255), 2)
        _save_debug_image(tnt_cfg.get("debug_contour_path"), contour_img)
    return best, bgr


def detect_tnt_pose(image, camera_cfg: dict, tnt_cfg: dict, camera=None):
    if image is None:
        return None
    best, _ = detect_tnt_contour(image, tnt_cfg, camera=camera)
    if best is None:
        return None

    rect = cv2.minAreaRect(best)
    box = cv2.boxPoints(rect)
    box = np.array(box, dtype=np.float32)

    edge = float(tnt_cfg.get("edge_length_mm", 50.0))
    obj_pts = np.array([
        [-edge / 2.0, edge / 2.0, 0.0],
        [edge / 2.0, edge / 2.0, 0.0],
        [edge / 2.0, -edge / 2.0, 0.0],
        [-edge / 2.0, -edge / 2.0, 0.0],
    ], dtype=np.float32)

    intr = camera_cfg.get("intrinsics", {})
    fx = float(intr.get("fx", 800.0))
    fy = float(intr.get("fy", 800.0))
    cx = float(intr.get("cx", 0.0))
    cy = float(intr.get("cy", 0.0))
    cfg_size = camera_cfg.get("image_size")
    if isinstance(cfg_size, (list, tuple)) and len(cfg_size) >= 2:
        try:
            w_cfg = float(cfg_size[0])
            h_cfg = float(cfg_size[1])
            if cx > 0.0 and cy > 0.0:
                if abs(cx - w_cfg / 2.0) > 0.15 * w_cfg or abs(cy - h_cfg / 2.0) > 0.15 * h_cfg:
                    cfg_size = [cx * 2.0, cy * 2.0]
        except Exception:
            cfg_size = None
    else:
        if cx > 0.0 and cy > 0.0:
            cfg_size = [cx * 2.0, cy * 2.0]
        else:
            cfg_size = None
    try:
        h, w = image.shape[:2]
    except Exception:
        h, w = None, None
    if cfg_size and w and h:
        sx = float(w) / float(cfg_size[0]) if cfg_size[0] else 1.0
        sy = float(h) / float(cfg_size[1]) if cfg_size[1] else 1.0
        fx *= sx
        fy *= sy
        cx *= sx
        cy *= sy
    camera_matrix = np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    dist = np.array(camera_cfg.get("distortion", [0, 0, 0, 0, 0]), dtype=np.float32)

    ok, rvec, tvec = cv2.solvePnP(obj_pts, box, camera_matrix, dist)
    if not ok:
        return None

    R_cam, _ = cv2.Rodrigues(rvec)
    cam_T_face = make_transform(R_cam.tolist(), tvec.reshape(-1).tolist())
    face_offset = float(tnt_cfg.get("face_offset_mm", edge / 2.0))
    face_T_obj = make_transform(
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [0.0, 0.0, -face_offset],
    )
    cam_T_obj = matmul(cam_T_face, face_T_obj)
    return cam_T_obj
