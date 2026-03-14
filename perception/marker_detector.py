from dataclasses import dataclass


@dataclass
class MarkerObservation:
    marker_id: int
    corners: list


class ArucoMarkerDetector:
    def __init__(self, config: dict):
        self.dictionary_name = config.get("dictionary", "DICT_4X4_50")
        self._cv2 = None

    def _load_cv2(self):
        if self._cv2 is None:
            import cv2
            self._cv2 = cv2

    def _dictionary(self):
        self._load_cv2()
        if not hasattr(self._cv2, "aruco"):
            raise RuntimeError("cv2.aruco is not available")
        aruco = self._cv2.aruco
        name = self.dictionary_name
        return aruco.getPredefinedDictionary(getattr(aruco, name))

    def detect(self, image):
        self._load_cv2()
        if image is None:
            raise RuntimeError("No image data provided")
        aruco = self._cv2.aruco
        corners, ids, _ = aruco.detectMarkers(image, self._dictionary())
        results = []
        if ids is None:
            return results
        for marker_id, marker_corners in zip(ids.flatten().tolist(), corners):
            results.append(MarkerObservation(marker_id=marker_id, corners=marker_corners))
        return results
