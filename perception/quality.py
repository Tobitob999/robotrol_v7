def reprojection_error(object_points, image_points, rvec, tvec, camera_matrix, dist_coeffs):
    try:
        import cv2
        import numpy as np
    except Exception:
        return None
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    projected = projected.reshape(-1, 2)
    image_points = image_points.reshape(-1, 2)
    diff = projected - image_points
    err = (diff[:, 0] ** 2 + diff[:, 1] ** 2) ** 0.5
    return float(err.mean())


def confidence_from_reprojection(error_px, max_error_px):
    if error_px is None:
        return 0.0
    if max_error_px <= 0:
        return 0.0
    conf = 1.0 - (error_px / max_error_px)
    if conf < 0.0:
        conf = 0.0
    if conf > 1.0:
        conf = 1.0
    return conf
